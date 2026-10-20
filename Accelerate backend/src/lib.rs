mod kinematics;
mod sqlite;
#[path = "nats-controller/mod.rs"]
mod nats;
pub mod logger;
#[path = "rest-controller/mod.rs"]
pub mod rest;

#[path = "calibration-handler/mod.rs"]
mod calibration;

use std::sync::Arc;
use tokio::sync::Mutex;
pub use calibration::{Calibrator, CalibrationError, Task};
pub use nats::{Connection, NATSController, PICOState, NATSReceipt};
pub type SharedNATS = std::sync::Arc<NATSController>;
pub type SharedSQL = Arc<Mutex<SQLController>>;
pub use kinematics::{Chain, Joint};
pub use sqlite::SQLController;
