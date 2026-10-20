#[derive(Debug)]
pub enum CalibrationError {
    InvalidConfiguration,
    Unreachable,
    NATSError(async_nats::Error),
    SQLError(rusqlite::Error),
    NoReceipt,
    Rejected(String),
    QueueClosed,
    QueueFull,
}
