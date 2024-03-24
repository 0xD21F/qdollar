use std::error::Error;
use std::fmt;

#[derive(Debug, PartialEq)]
pub enum QDollarError {
    NoRegisteredGestures,
}

impl fmt::Display for QDollarError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            QDollarError::NoRegisteredGestures => write!(f, "No gestures registered for recognition"),
        }
    }
}

impl Error for QDollarError {}
