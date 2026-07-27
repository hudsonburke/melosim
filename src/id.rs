use slotmap::new_key_type;

new_key_type! {
    /// Single key type for all entities.
    /// An entity can have components across multiple type-indexed SlotMaps.
    pub struct EntityKey;
}
