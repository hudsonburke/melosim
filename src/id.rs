use slotmap::new_key_type;

new_key_type! {
    /// Single key type for all entities in the dynamic Build World.
    /// An entity can have components across multiple type-indexed SlotMaps.
    /// Generational safety protects against use-after-delete during construction.
    pub struct EntityKey;
}

/// Dense entity ID used in FlatWorld (the frozen simulation snapshot).
/// Indexes directly into parallel `Vec<Option<T>>` arrays.
/// Zero hash lookups — `flat.inertials[id.0 as usize]` is a single load.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
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
