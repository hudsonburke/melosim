#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Geometry {
    pub id: EntityID,
    pub body: EntityID,
    pub mesh: String,
    pub role: GeometryRole,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum GeometryRole {
    Collision,
    Visualization,
    Simulation,
}
