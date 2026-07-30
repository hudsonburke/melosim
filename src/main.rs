use melosim::components::*;
// use melosim::id::EntityID;
use melosim::math::{Transform, Vec3};
use melosim::systems;
use melosim::world::World;

// ── Main ──

fn main() {
    let mut world = World::new();

    // ── Bodies ──
    let ground = world.spawn();
    world.attach(
        ground,
        InertialProperties {
            mass: 0.0,
            com: [0.0, 0.0, 0.0],
            inertia: [0.0; 6],
        },
    );
    world.attach(
        ground,
        Name {
            value: "ground".into(),
        },
    );
    let ground_frame = world.spawn();
    world.attach(
        ground_frame,
        Frame {
            parent: ground,
            transform: Transform::default(),
        },
    );

    let pelvis = world.spawn();
    world.attach(
        pelvis,
        InertialProperties {
            mass: 11.78,
            com: [0.0, 0.0, 0.0],
            inertia: [0.18, 0.22, 0.20, 0.0, 0.0, 0.0],
        },
    );
    world.attach(
        pelvis,
        Name {
            value: "pelvis".into(),
        },
    );
    let pelvis_frame = world.spawn();
    world.attach(
        pelvis_frame,
        Frame {
            parent: ground,
            transform: Transform::default(),
        },
    );

    let femur = world.spawn();
    world.attach(
        femur,
        InertialProperties {
            mass: 9.3,
            com: [0.0, -0.17, 0.0],
            inertia: [0.12, 0.12, 0.02, 0.0, 0.0, 0.0],
        },
    );
    world.attach(
        femur,
        Name {
            value: "femur".into(),
        },
    );
    let femur_frame = world.spawn();
    world.attach(
        femur_frame,
        Frame {
            parent: pelvis,
            transform: Transform::default(),
        },
    );

    // ── Simple joints ──
    let pelvis_free = world.spawn();
    world.attach(
        pelvis_free,
        FreeJoint {
            body_a: ground,
            body_b: pelvis,
            limits: None,
        },
    );

    let hip = world.spawn();
    world.attach(
        hip,
        HingeJoint {
            body_a: pelvis,
            body_b: femur,
            limits: Some(JointLimits {
                lower: -2.0,
                upper: 0.5,
            }),
            axis: [1.0, 0.0, 0.0],
        },
    );

    // ── UniversalJoint (e.g., lumbar spine) ──
    let lumbar = world.spawn();
    world.attach(
        lumbar,
        UniversalJoint {
            body_a: pelvis,
            body_b: femur,
            limits: Some(JointLimits {
                lower: -0.5,
                upper: 0.5,
            }),
            axis1: [1.0, 0.0, 0.0],
            axis2: [0.0, 1.0, 0.0],
        },
    );

    // ── CustomJoint (e.g., knee with coupled motion) ──
    // 1. Create coordinate entities
    let knee_flexion = world.spawn();
    world.attach(
        knee_flexion,
        JointCoordinate {
            range_min: -2.0,
            range_max: 0.0,
            default_value: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            clamped: true,
            locked: false,
            prescribed_function: None,
        },
    );
    world.attach(
        knee_flexion,
        Name {
            value: "knee_flexion".into(),
        },
    );

    // 2. Create the CustomJoint referencing those coordinates
    let knee = world.spawn();
    world.attach(
        knee,
        CustomJoint {
            body_a: femur,
            body_b: pelvis,
            limits: None,
            coordinates: vec![knee_flexion],
        },
    );

    // 3. Create CoordinateEffects mapping coordinates to transform components
    let flex_effect = world.spawn();
    world.attach(
        flex_effect,
        CoordinateEffect {
            coordinate: knee_flexion,
            joint: knee,
            component: TransformComponent::RotationY,
            function: JointFunction::Linear {
                slope: -1.0,
                intercept: 0.0,
            },
        },
    );

    let ap_translate = world.spawn();
    world.attach(
        ap_translate,
        CoordinateEffect {
            coordinate: knee_flexion,
            joint: knee,
            component: TransformComponent::TranslationX,
            function: JointFunction::Polynomial {
                coefficients: vec![0.002, -0.015, 0.0, 0.0],
            },
        },
    );

    // 4. Create SpatialTransform grouping the effects
    let knee_transform = world.spawn();
    world.attach(
        knee_transform,
        SpatialTransform {
            joint: knee,
            effects: vec![flex_effect, ap_translate],
        },
    );

    // ── Site (muscle attachment point) ──
    let asis = world.spawn();
    world.attach(
        asis,
        Site {
            parent: pelvis,
            offset: Vec3::new(0.01, 0.02, 0.13),
        },
    );

    // ── Run systems (validation, etc.) ──
    systems::run_systems(&mut world);
    systems::print_errors(&mut world);

    println!("\nBuild World:\n  {:?}", world);
    println!("  component count: {}", world.components.len());
}
