/// Dense entity ID used throughout melosim.
/// Indexes directly into `Vec<Option<T>>` component storage.
/// Zero hash lookups — `storage[id.0 as usize]` is a single load.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub struct EntityID(pub u32);

impl EntityID {
    pub fn as_usize(&self) -> usize {
        self.0 as usize
    }
}

impl From<u32> for EntityID {
    fn from(v: u32) -> Self {
        Self(v)
    }
}
