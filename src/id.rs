use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EntityID(u32);

impl EntityID {
    pub fn new(id: u32) -> Self {
        Self(id)
    }

    pub fn inner(&self) -> u32 {
        self.0
    }

    pub fn next(&self) -> Self {
        Self(self.0 + 1)
    }
}
