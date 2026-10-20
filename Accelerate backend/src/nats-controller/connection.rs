use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub enum Connection {
    Connected,
    Disconnected
}
