use HSSE::{NATSReceipt, PICOState, logger};
use async_nats::{Client, Error, Message};
use futures_util::StreamExt;
use serde::{Deserialize, Serialize};
use std::time::Duration;
use tokio::time::Instant;

const CALIBRATION_DURATION: Duration = Duration::from_secs(2);

#[derive(Deserialize)]
struct Calibration {
    id: String,
    correction: [f64; 6],
}

async fn publish<T: Serialize>(client: &Client, subject: &str, data: &T) -> Result<(), Error> {
    client
        .publish(subject.to_owned(), serde_json::to_vec(data)?.into())
        .await?;
    Ok(())
}

async fn reply<T: Serialize>(client: &Client, message: Message, data: &T) -> Result<(), Error> {
    if let Some(subject) = message.reply {
        publish(client, subject.as_str(), data).await?;
    }
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Error> {
    logger::init()?;
    let address = std::env::var("NATS_URL").unwrap_or_else(|_| "nats://127.0.0.1:4222".into());
    let client = async_nats::connect(&address).await?;
    let mut states = client.subscribe("machine/state").await?;
    let mut configurations = client.subscribe("machine/configuration").await?;
    let mut calibrations = client.subscribe("machine/calibration").await?;
    client.flush().await?;
    let mut configuration = [0.0_f64; 6];
    let mut state = PICOState::Idle;
    let mut pending: Option<(Calibration, [f64; 6])> = None;
    let mut completion = Instant::now();
    tracing::info!(%address, "Pico simulator ready");

    loop {
        tokio::select! {
            Some(message) = states.next() => {
                reply(&client, message, &state).await?;
            }
            Some(message) = configurations.next() => {
                reply(&client, message, &configuration).await?;
            }
            Some(message) = calibrations.next() => {
                let command = match serde_json::from_slice::<Calibration>(&message.payload) {
                    Ok(command) => command,
                    Err(error) => {
                        tracing::warn!(%error, "invalid calibration command ignored");
                        continue;
                    }
                };
                if pending.is_some() {
                    let receipt = NATSReceipt {
                        id: command.id,
                        ok: false,
                        error: Some("calibration already in progress".into()),
                        timestamp: None,
                    };
                    publish(&client, "machine/receipt", &receipt).await?;
                    continue;
                }
                let next = std::array::from_fn::<_, 6, _>(|i| configuration[i] + command.correction[i]);
                let valid = next.iter().all(|angle| angle.is_finite());
                if valid {
                    state = PICOState::Busy;
                    publish(&client, "machine/state", &state).await?;
                    client.flush().await?;
                    completion = Instant::now() + CALIBRATION_DURATION;
                    tracing::info!(id = %command.id, "calibration started; state Busy");
                    pending = Some((command, next));
                    continue;
                }
                let receipt = NATSReceipt {
                    id: command.id,
                    ok: valid,
                    error: (!valid).then(|| "non-finite joint angle".into()),
                    timestamp: None,
                };
                publish(&client, "machine/receipt", &receipt).await?;
                tracing::info!(id = %receipt.id, ok = receipt.ok, ?configuration, "calibration simulated");
            }
            _ = tokio::time::sleep_until(completion), if pending.is_some() => {
                let (command, next) = pending.take().expect("pending calibration");
                configuration = next;
                publish(&client, "machine/configuration", &configuration).await?;
                state = PICOState::Idle;
                publish(&client, "machine/state", &state).await?;
                let receipt = NATSReceipt { id: command.id, ok: true, error: None, timestamp: None };
                publish(&client, "machine/receipt", &receipt).await?;
                tracing::info!(id = %receipt.id, ?configuration, "calibration completed; state Idle");
            }
            result = tokio::signal::ctrl_c() => {
                result?;
                break;
            }
            else => break,
        }
    }
    client.drain().await?;
    Ok(())
}
