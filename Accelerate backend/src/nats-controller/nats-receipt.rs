use std::time::Duration;
use serde::{Deserialize, Serialize};

#[derive(Clone, Serialize, Deserialize)]
pub struct NATSReceipt {
    pub id: String,
    pub ok: bool,
    pub error: Option<String>,
    pub timestamp: Option<Duration>,
}
