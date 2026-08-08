use melosim::components::*;
use melosim::world::World;
use bevy_ecs::prelude::Entity;

// ── Main ──

fn main() {
    let mut world = World::new();

    // ── Bodies ──
    let ground = world.spawn(()).id();
    world.entity_mut(ground).insert(InertialProperties {
        mass: 0.0,
        com: [0.0, 0.0, 0.0],
        inertia: [0.0; 6],
    });
    world.entity_mut(ground).insert(Name {
        value: "ground".into(),
    });

    let pelvis = world.spawn(()).id();
    world.entity_mut(pelvis).insert(InertialProperties {
        mass: 11.78,
        com: [0.0, 0.0, 0.0],
        inertia: [0.18, 0.22, 0.20, 0.0, 0.0, 0.0],
    });
    world.entity_mut(pelvis).insert(Name {
        value: "pelvis".into(),
    });

    let femur = world.spawn(()).id();
    world.entity_mut(femur).insert(InertialProperties {
        mass: 9.3,
        com: [0.0, -0.17, 0.0],
        inertia: [0.12, 0.12, 0.02, 0.0, 0.0, 0.0],
    });
    world.entity_mut(femur).insert(Name {
        value: "femur".into(),
    });

    // ── Set up hierarchy: pelvis is child of ground ──
    world.entity_mut(pelvis).insert(ChildOf { parent: ground });

    // ── Simple joints using convenience builders ──
    let _pelvis_free = add_free(&mut world, ground, pelvis);

    let _hip = add_hinge(
        &mut world,
        pelvis,
        femur,
        [1.0, 0.0, 0.0],
        Some((-2.0, 0.5)),
    );

    // ── UniversalJoint ──
    let _lumbar = add_universal(
        &mut world,
        pelvis,
        femur,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        Some((-0.5, 0.5)),
    );

    // ── CustomJoint (knee with coupled motion) ──
    let knee_flexion = world.spawn(()).id();
    world.entity_mut(knee_flexion).insert(JointCoordinate {
        range_min: -2.0,
        range_max: 0.0,
        default_value: 0.0,
        stiffness: 0.0,
        damping: 0.0,
        clamped: true,
        locked: false,
        prescribed_function: None,
    });
    world.entity_mut(knee_flexion).insert(Name {
        value: "knee_flexion".into(),
    });

    let _knee = add_custom(
        &mut world,
        femur,
        pelvis,
        vec![knee_flexion],
        None,
    );

    let flex_effect = world.spawn(()).id();
    world.entity_mut(flex_effect).insert(ChildOf { parent: knee_flexion });
    world.entity_mut(flex_effect).insert(CoordinateEffect {
        component: TransformComponent::RotationY,
        function: JointFunction::Linear {
            slope: -1.0,
            intercept: 0.0,
        },
    });

    let ap_translate = world.spawn(()).id();
    world.entity_mut(ap_translate).insert(ChildOf { parent: knee_flexion });
    world.entity_mut(ap_translate).insert(CoordinateEffect {
        component: TransformComponent::TranslationX,
        function: JointFunction::Polynomial {
            coefficients: vec![0.002, -0.015, 0.0, 0.0],
        },
    });

    // ── Site (muscle attachment point) ──
    let _asis = world.spawn(()).id();
    world.entity_mut(_asis).insert(ChildOf { parent: pelvis });
    world.entity_mut(_asis).insert(Position::new(0.01, 0.02, 0.13));

    // ── Run systems (validation, etc.) ──
    melosim::systems::run_systems(&mut world);
    melosim::systems::print_errors(&mut world);

    // ── Debug summary ──
    let bodies = world.query::<&InertialProperties>().iter(&world).count();
    let coords = world.query::<&JointCoordinate>().iter(&world).count();
    let muscles = world.query::<&Muscle>().iter(&world).count();
    let effects = world.query::<&CoordinateEffect>().iter(&world).count();
    println!(
        "\nBuild World:\n  Bodies: {}, Coordinates: {}, Muscles: {}, Effects: {}",
        bodies, coords, muscles, effects
    );
}

// ── Joint convenience builders (inlined from WorldExt) ──

fn add_hinge(
    world: &mut World,
    parent_frame: Entity,
    child_frame: Entity,
    axis: [f64; 3],
    limits: Option<(f64, f64)>,
) -> Entity {
    let joint = world.spawn(()).id();
    world.entity_mut(joint).insert(ChildOf { parent: parent_frame });
    world.entity_mut(child_frame).insert(ChildOf { parent: joint });

    let coord = world.spawn(()).id();
    world.entity_mut(coord).insert(ChildOf { parent: joint });
    world.entity_mut(coord).insert(JointCoordinate {
        range_min: limits.map_or(-1e10, |l| l.0),
        range_max: limits.map_or(1e10, |l| l.1),
        default_value: 0.0,
        stiffness: 0.0,
        damping: 0.0,
        clamped: limits.is_some(),
        locked: false,
        prescribed_function: None,
    });

    let effect = world.spawn(()).id();
    world.entity_mut(effect).insert(ChildOf { parent: coord });
    world.entity_mut(effect).insert(CoordinateEffect {
        component: TransformComponent::RotationAboutAxis(axis),
        function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
    });

    joint
}

fn add_free(world: &mut World, parent_frame: Entity, child_frame: Entity) -> Entity {
    let joint = world.spawn(()).id();
    world.entity_mut(joint).insert(ChildOf { parent: parent_frame });
    world.entity_mut(child_frame).insert(ChildOf { parent: joint });

    let rot_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for axis in &rot_axes {
        let coord = world.spawn(()).id();
        world.entity_mut(coord).insert(ChildOf { parent: joint });
        world.entity_mut(coord).insert(JointCoordinate {
            range_min: -1e10, range_max: 1e10, default_value: 0.0,
            stiffness: 0.0, damping: 0.0, clamped: false, locked: false, prescribed_function: None,
        });
        let effect = world.spawn(()).id();
        world.entity_mut(effect).insert(ChildOf { parent: coord });
        world.entity_mut(effect).insert(CoordinateEffect {
            component: TransformComponent::RotationAboutAxis(*axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });
    }
    let trans_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for axis in &trans_axes {
        let coord = world.spawn(()).id();
        world.entity_mut(coord).insert(ChildOf { parent: joint });
        world.entity_mut(coord).insert(JointCoordinate {
            range_min: -1e10, range_max: 1e10, default_value: 0.0,
            stiffness: 0.0, damping: 0.0, clamped: false, locked: false, prescribed_function: None,
        });
        let effect = world.spawn(()).id();
        world.entity_mut(effect).insert(ChildOf { parent: coord });
        world.entity_mut(effect).insert(CoordinateEffect {
            component: TransformComponent::TranslationAlongAxis(*axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });
    }
    joint
}

fn add_universal(
    world: &mut World,
    parent_frame: Entity,
    child_frame: Entity,
    axis1: [f64; 3],
    axis2: [f64; 3],
    limits: Option<(f64, f64)>,
) -> Entity {
    let joint = world.spawn(()).id();
    world.entity_mut(joint).insert(ChildOf { parent: parent_frame });
    world.entity_mut(child_frame).insert(ChildOf { parent: joint });

    for axis in &[axis1, axis2] {
        let coord = world.spawn(()).id();
        world.entity_mut(coord).insert(ChildOf { parent: joint });
        world.entity_mut(coord).insert(JointCoordinate {
            range_min: limits.map_or(-1e10, |l| l.0),
            range_max: limits.map_or(1e10, |l| l.1),
            default_value: 0.0,
            stiffness: 0.0, damping: 0.0,
            clamped: limits.is_some(), locked: false, prescribed_function: None,
        });
        let effect = world.spawn(()).id();
        world.entity_mut(effect).insert(ChildOf { parent: coord });
        world.entity_mut(effect).insert(CoordinateEffect {
            component: TransformComponent::RotationAboutAxis(*axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });
    }
    joint
}

fn add_custom(
    world: &mut World,
    parent_frame: Entity,
    child_frame: Entity,
    coordinates: Vec<Entity>,
    _limits: Option<(f64, f64)>,
) -> Entity {
    let joint = world.spawn(()).id();
    world.entity_mut(joint).insert(ChildOf { parent: parent_frame });
    world.entity_mut(child_frame).insert(ChildOf { parent: joint });
    for coord in &coordinates {
        world.entity_mut(*coord).insert(ChildOf { parent: joint });
    }
    joint
}
