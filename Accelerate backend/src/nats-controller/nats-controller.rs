use super::{NATSReceipt, PICOState};
use crate::Connection::{Connected, Disconnected};
use crate::nats::connection::Connection;
use async_nats::{Client, Error, Subscriber, connect};
use futures_util::StreamExt;
use nanoid::nanoid;
use serde::Deserialize;
use std::convert::Into;
use std::option::Option;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::sync::Mutex;
use tokio::sync::watch::{Receiver, Sender, channel};

const RECEIPT_SUBJECT: &str = "machine/receipt";
const CALIBRATION_SUBJECT: &str = "machine/calibration";
const STATE_SUBJECT: &str = "machine/state";
const CONFIGURATION_SUBJECT: &str = "machine/configuration";
const CALIBRATION_TIMEOUT: Duration = Duration::new(45, 0);

pub struct NATSController {
    id: String,
    client: Client,
    receipts: Mutex<Subscriber>,
    configuration: Sender<Option<[f64; 6]>>,
    state: Sender<Option<PICOState>>,
    connection: Sender<Connection>,
}

impl NATSController {

    pub async fn connect(address: &str) -> Result<Self, Error> {
        let (connection, _) = channel(Disconnected);
        let client = connect(address).await?;
        let (state, configuration) = tokio::try_join!(
            NATSController::request::<PICOState>(&client, STATE_SUBJECT),
            NATSController::request::<[f64; 6]>(&client, CONFIGURATION_SUBJECT),
        )?;
        let (configuration, _) = channel(Some(configuration));
        let (state, _) = channel(Some(state));
        let id = nanoid!();
        let receipts = Mutex::from(client.subscribe(RECEIPT_SUBJECT).await?);
        client.flush().await?;
        connection.send_replace(Connected);
        tracing::info!(controller = %id, address = %address, "controller connected to NATS");
        let controller = NATSController {
            id,
            client,
            receipts,
            configuration,
            state,
            connection,
        };
        Ok(controller)
    }

    pub async fn disconnect(&self) -> Result<(), Error> {
        self.client.drain().await?;
        self.connection.send_replace(Connection::Disconnected);
        tracing::info!(controller = %self.id, "controller disconnected from NATS");
        Ok(())
    }

    pub fn connection(&self) -> Receiver<Connection> {
        self.connection.subscribe()
    }

    pub fn state(&self) -> Receiver<Option<PICOState>> {
        self.state.subscribe()
    }

    pub fn configuration(&self) -> Receiver<Option<[f64; 6]>> {
        self.configuration.subscribe()
    }

    pub async fn request<T: for<'de> Deserialize<'de>>(
        client: &Client,
        subject: &str,
    ) -> Result<T, Error> {
        let response = client.request(subject.to_owned(), "".into()).await?;
        let data = serde_json::from_slice::<T>(&response.payload)?;
        Ok(data)
    }

    pub async fn calibrate(&self, correction: [f64; 6]) -> Result<Option<NATSReceipt>, Error> {
        let mut receipts = self.receipts.try_lock()?;
        let id = nanoid!();
        let data = serde_json::json!({
            "id": id,
            "correction": correction,
        });
        let payload = serde_json::to_vec(&data)?;
        self.client
            .publish(CALIBRATION_SUBJECT, payload.into())
            .await?;
        self.state.send_replace(Some(PICOState::Busy));
        if let Err(error) = self.client.flush().await {
            self.state.send_replace(Some(PICOState::Error));
            return Err(error.into());
        }
        tracing::info!(controller = %self.id, %id, ?correction, "calibration command sent");
        let receipt = self.receipt(&mut receipts, &id).await;
        if receipt.as_ref().is_some_and(|receipt| receipt.ok) {
            self.configuration.send_modify(|configuration| {
                if let Some(configuration) = configuration {
                    for (angle, delta) in configuration.iter_mut().zip(correction) {
                        *angle += delta;
                    }
                }
            });
        }
        self.state.send_replace(Some(if receipt.is_some() {
            PICOState::Idle
        } else {
            PICOState::Error
        }));
        Ok(receipt)
    }

    async fn receipt(&self, receipts: &mut Subscriber, id: &str) -> Option<NATSReceipt> {
        let receive = async {
            while let Some(message) = receipts.next().await {
                let data = serde_json::from_slice::<serde_json::Value>(&message.payload).ok()?;
                if data["id"] == id {
                    let mut receipt: NATSReceipt = serde_json::from_value(data).ok()?;
                    receipt.timestamp = Some(SystemTime::now().duration_since(UNIX_EPOCH).ok()?);
                    tracing::info!(
                        controller = %self.id,
                        receipt = %receipt.id,
                        ok = receipt.ok,
                        error = ?receipt.error,
                        timestamp = ?receipt.timestamp,
                        "receipt received"
                    );
                    return Some(receipt);
                }
            }
            tracing::warn!(%id, "receipt subscription closed");
            return None;
        };
        match tokio::time::timeout(CALIBRATION_TIMEOUT, receive).await {
            Err(error) => {
                self.connection.send_replace(Disconnected);
                tracing::error!(%id, %error, "no receipt received");
                None
            },
            Ok(receipt) => receipt
        }
    }
}
