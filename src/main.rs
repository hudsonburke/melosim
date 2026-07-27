use melosim::components::*;
use melosim::id::EntityKey;
use melosim::math::{Transform, Vec3};
use melosim::system::SystemRegistry;
use melosim::validate;
use melosim::world::World;

// ── Example: Custom joint from a downstream crate ─────

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct PrismaticJoint {
    body_a: EntityKey,
    body_b: EntityKey,
    limits: Option<JointLimits>,
    axis: [f64; 3],
}

#[allow(dead_code)]
fn prismatic_system(_world: &mut World) {}

// ── Main ──────────────────────────────────────────────

fn main() {
    // ── Phase 1: Build World (extensible, dynamic) ──
    let mut world = World::new();

    // ── Bodies ──
    let ground = world.insert(InertialProperties {
        mass: 0.0,
        com: [0.0, 0.0, 0.0],
        inertia: [0.0; 6],
    });
    world.insert(Frame {
        parent: ground,
        transform: Transform::default(),
    });

    let pelvis = world.insert(InertialProperties {
        mass: 11.78,
        com: [0.0, 0.0, 0.0],
        inertia: [0.18, 0.22, 0.20, 0.0, 0.0, 0.0],
    });
    world.insert(Frame {
        parent: ground,
        transform: Transform::default(),
    });

    let femur = world.insert(InertialProperties {
        mass: 9.3,
        com: [0.0, -0.17, 0.0],
        inertia: [0.12, 0.12, 0.02, 0.0, 0.0, 0.0],
    });
    world.insert(Frame {
        parent: pelvis,
        transform: Transform::default(),
    });

    // ── Simple joints ──
    let _pelvis_free = world.insert(FreeJoint {
        body_a: ground,
        body_b: pelvis,
        limits: None,
    });

    let _hip = world.insert(HingeJoint {
        body_a: pelvis,
        body_b: femur,
        limits: Some(JointLimits { lower: -2.0, upper: 0.5 }),
        axis: [1.0, 0.0, 0.0],
    });

    // ── UniversalJoint (e.g., lumbar spine) ──
    let _lumbar = world.insert(UniversalJoint {
        body_a: pelvis,
        body_b: femur,
        limits: Some(JointLimits { lower: -0.5, upper: 0.5 }),
        axis1: [1.0, 0.0, 0.0],
        axis2: [0.0, 1.0, 0.0],
    });

    // ── CustomJoint (e.g., knee with coupled motion) ──
    // 1. Create coordinate entities
    let knee_flexion = world.insert(JointCoordinate {
        name: "knee_flexion".into(),
        range_min: -2.0,
        range_max: 0.0,
        default_value: 0.0,
        stiffness: 0.0,
        damping: 0.0,
        clamped: true,
        locked: false,
        prescribed_function: None,
    });

    // 2. Create the CustomJoint referencing those coordinates
    let knee = world.insert(CustomJoint {
        body_a: femur,
        body_b: pelvis,
        limits: None,
        coordinates: vec![knee_flexion],
    });

    // 3. Create CoordinateEffects mapping coordinates to transform components
    let flex_effect = world.insert(CoordinateEffect {
        coordinate: knee_flexion,
        joint: knee,
        component: TransformComponent::RotationY,
        function: JointFunction::Linear {
            slope: -1.0,
            intercept: 0.0,
        },
    });

    let ap_translate = world.insert(CoordinateEffect {
        coordinate: knee_flexion,
        joint: knee,
        component: TransformComponent::TranslationX,
        function: JointFunction::Polynomial {
            coefficients: vec![0.002, -0.015, 0.0, 0.0],
        },
    });

    // 4. Create SpatialTransform grouping the effects
    let _knee_transform = world.insert(SpatialTransform {
        joint: knee,
        effects: vec![flex_effect, ap_translate],
    });

    // ── Site (muscle attachment point) ──
    let _asis = world.insert(Site {
        parent: pelvis,
        offset: Vec3::new(0.01, 0.02, 0.13),
    });

    // ── Register and run validation systems ──
    let mut registry = SystemRegistry::new();
    registry.add("validate_hinge", validate::validate_hinge);
    registry.add("validate_slide", validate::validate_slide);
    registry.add("validate_ball", validate::validate_ball);
    registry.add("validate_free", validate::validate_free);
    registry.add("validate_fixed", validate::validate_fixed);
    registry.add("validate_universal", validate::validate_universal);
    registry.add("validate_custom", validate::validate_custom);
    registry.add("validate_coordinate", validate::validate_coordinate);
    registry.add("validate_coordinate_effect", validate::validate_coordinate_effect);
    registry.add("validate_spatial_transform", validate::validate_spatial_transform);
    registry.add("validate_frame", validate::validate_frame);
    registry.add("validate_site", validate::validate_site);
    registry.add("print_errors", validate::print_errors);
    registry.run(&mut world);

    println!("\nBuild World:\n  {:?}", world);
    println!("  component count: {}", world.components.len());

    // ── Phase 2: Freeze → FlatWorld (dense, GPU-ready) ──
    let flat = world.freeze();

    println!("\nFlatWorld snapshot:");
    println!("  {:?}", flat);

    // Demonstrate dense indexing on new types
    if let Some(knee_cj) = &flat.custom_joints[0] {
        println!("\nCustomJoint at index 0:");
        println!("  body_a={:?}, coordinates: {} DOFs", knee_cj.body_a, knee_cj.coordinates.len());
    }
    if let Some(coord) = &flat.coordinates[0] {
        println!("  Coordinate '{}' range [{}, {}]", coord.name, coord.range_min, coord.range_max);
    }
    if let Some(effect) = &flat.coordinate_effects[0] {
        println!("  Effect: {:?} via {:?}", effect.component, effect.function);
    }

    // ── Custom type example (downstream) ──
    world.insert(PrismaticJoint {
        body_a: pelvis,
        body_b: femur,
        limits: None,
        axis: [0.0, 1.0, 0.0],
    });

    let flat2 = world.freeze();
    println!("\nFlatWorld with custom prismatic:");
    println!("  {:?}", flat2);
    println!("  extensions: {} types registered", flat2.extensions.len());
}
