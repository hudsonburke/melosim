use crate::components::body::Site;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Path {
    pub points: Vec<Site>,
}
