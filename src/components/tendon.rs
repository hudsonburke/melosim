use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TendonParams {
    pub spring_length: f64,
    pub width: f64,
}
