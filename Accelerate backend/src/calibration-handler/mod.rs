mod calibrator;
#[path = "calibration-error.rs"]
mod error;

pub use calibrator::{Calibrator, Task};
pub use error::CalibrationError;
