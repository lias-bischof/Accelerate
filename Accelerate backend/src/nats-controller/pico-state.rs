use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum PICOState {
    Idle,
    Busy,
    Error,
    Homing
}
