use melosim::components::*;
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

    // ── Set up hierarchy: pelvis is child of ground ──
    world.set_parent(pelvis, ground);

    // ── Simple joints using convenience builders ──
    // add_free: creates joint entity as intermediate node
    let _pelvis_free = world.add_free(ground, pelvis);

    let _hip = world.add_hinge(
        pelvis,
        femur,
        [1.0, 0.0, 0.0],
        Some((-2.0, 0.5)),
    );

    // ── UniversalJoint (e.g., lumbar spine) ──
    let _lumbar = world.add_universal(
        pelvis,
        femur,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        Some((-0.5, 0.5)),
    );

    // ── CustomJoint (e.g., knee with coupled motion) ──
    // 1. Create coordinate entity
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

    // 2. Create the Joint as intermediate node (coordinates set as children)
    let _knee = world.add_custom(
        femur,
        pelvis,
        vec![knee_flexion],
    );

    // 3. Create CoordinateEffects as children of coordinates
    let flex_effect = world.spawn();
    world.set_parent(flex_effect, knee_flexion);
    world.attach(
        flex_effect,
        CoordinateEffect {
            component: TransformComponent::RotationY,
            function: JointFunction::Linear {
                slope: -1.0,
                intercept: 0.0,
            },
        },
    );

    let ap_translate = world.spawn();
    world.set_parent(ap_translate, knee_flexion);
    world.attach(
        ap_translate,
        CoordinateEffect {
            component: TransformComponent::TranslationX,
            function: JointFunction::Polynomial {
                coefficients: vec![0.002, -0.015, 0.0, 0.0],
            },
        },
    );

    // ── Site (muscle attachment point) ──
    // Sites are now just entities with ChildOf + Position (no marker needed)
    let _asis = world.spawn();
    world.set_parent(_asis, pelvis);
    world.attach(_asis, Position::new(0.01, 0.02, 0.13));

    // ── Run systems (validation, etc.) ──
    systems::run_systems(&mut world);
    systems::print_errors(&mut world);

    println!("\nBuild World:\n  {:?}", world);
    println!("  component count: {}", world.components.len());
}
