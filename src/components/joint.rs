// A joint is now represented as an intermediate node in the ChildOf hierarchy.
//
// Joint entity structure:
//   - ChildOf { parent: parent_frame }  (the joint is a child of its parent frame)
//   - Position / Rotation (static offset from parent, optional)
//   - Children containing coordinate entities
//
// The child frame is a child of the joint entity via ChildOf:
//   child_frame ChildOf → joint
//
// Joint kind is inferred from the coordinate/effect configuration:
//   - 0 coordinates → WeldJoint
//   - 1 rotation coordinate → PinJoint
//   - 1 translation coordinate → SlideJoint
//   - 2 rotation coordinates → UniversalJoint
//   - 3 rotation coordinates → BallJoint
//   - 6 coordinates (3 rotation + 3 translation) → FreeJoint
//   - other → CustomJoint
