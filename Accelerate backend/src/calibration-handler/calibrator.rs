use crate::{CalibrationError, Chain, SharedNATS};
use nalgebra::{Translation3, Vector3};
use nanoid::nanoid;
use tokio::sync::{mpsc, oneshot};

const QUEUE_CAPACITY: usize = 32;

struct QueuedTask {
    id: String,
    target: Vector3<f64>,
    reply: oneshot::Sender<Result<[f64; 6], CalibrationError>>,
}

pub struct Task {
    pub id: String,
    pub result: oneshot::Receiver<Result<[f64; 6], CalibrationError>>,
}

#[derive(Clone)]
pub struct Calibrator {
    queue: mpsc::Sender<QueuedTask>,
}

impl Calibrator {

    pub fn create(nats: SharedNATS, chain: Chain) -> Self {
        let (queue, mut tasks) = mpsc::channel::<QueuedTask>(QUEUE_CAPACITY);
        tokio::spawn(async move {
            while let Some(task) = tasks.recv().await {
                tracing::info!(task = %task.id, target = ?task.target, "calibration started");
                let result = execute(&nats, &chain, task.target).await;
                match &result {
                    Ok(correction) => tracing::info!(task = %task.id, ?correction, "calibration finished"),
                    Err(error) => tracing::error!(task = %task.id, ?error, "calibration failed"),
                }
                let _ = task.reply.send(result);
            }
        });
        Self { queue }
    }

    pub async fn add(&self, target: Vector3<f64>) -> Result<Task, CalibrationError> {
        let id = nanoid!();
        let (reply, result) = oneshot::channel();
        self.queue
            .try_send(QueuedTask {
                id: id.clone(),
                target,
                reply,
            })
            .map_err(|error| match error {
                mpsc::error::TrySendError::Full(_) => CalibrationError::QueueFull,
                mpsc::error::TrySendError::Closed(_) => CalibrationError::QueueClosed,
            })?;
        tracing::info!(task = %id, target = ?target, "calibration queued");
        Ok(Task { id, result })
    }
}

async fn execute(
    nats: &SharedNATS,
    chain: &Chain,
    target: Vector3<f64>,
) -> Result<[f64; 6], CalibrationError> {
    let current = (*nats.configuration().borrow())
        .ok_or(CalibrationError::InvalidConfiguration)?;
    let correction = correction(chain, &current, target)?;
    let receipt = nats.calibrate(correction)
        .await
        .map_err(CalibrationError::NATSError)?
        .ok_or(CalibrationError::NoReceipt)?;
    if !receipt.ok {
        return Err(CalibrationError::Rejected(receipt.error.unwrap()));
    }
    Ok(correction)
}

fn correction(
    chain: &Chain,
    current: &[f64; 6],
    target: Vector3<f64>,
) -> Result<[f64; 6], CalibrationError> {
    let mut pose = chain.pose(current);
    pose.translation = Translation3::from(target);
    let configuration = chain
        .inverse(&pose, current)
        .ok_or(CalibrationError::Unreachable)?;
    Ok(std::array::from_fn(|i| configuration[i] - current[i]))
}
