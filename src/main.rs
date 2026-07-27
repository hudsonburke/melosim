use melosim::math::{Transform, Vec3};
use melosim::components::*;
use melosim::world::World;

fn main() {
    let mut world = World::new();

    // Create ground
    let _ground = world.insert(InertialProperties {
        mass: 0.0,
        com: [0.0, 0.0, 0.0],
        inertia: [0.0; 6],
    });

    // Create pelvis
    let pelvis = world.insert(InertialProperties {
        mass: 11.78,
        com: [0.0, 0.0, 0.0],
        inertia: [0.18, 0.22, 0.20, 0.0, 0.0, 0.0],
    });
    world.insert(Frame {
        parent: _ground,
        transform: Transform::default(),
    });

    // Create femur
    let femur = world.insert(InertialProperties {
        mass: 9.3,
        com: [0.0, -0.17, 0.0],
        inertia: [0.12, 0.12, 0.02, 0.0, 0.0, 0.0],
    });

    // Create hip joint
    let _hip = world.insert(Joint {
        body_a: pelvis,
        body_b: femur,
        joint_type: JointType::Ball,
        limits: Some(JointLimits {
            lower: -2.0,
            upper: 2.0,
        }),
    });

    // Create a site for ASIS landmark
    let asis = world.insert(Site {
        parent: pelvis,
        offset: Vec3::new(0.01, 0.02, 0.13),
    });

    // Attach landmark role to the site
    let _landmark = world.insert(Landmark {
        site: asis,
        name: "ASIS".to_string(),
    });

    // Create a muscle
    let _muscle = world.insert(HillTypeMuscleParams {
        max_force: 2000.0,
        optimal_fiber_length: 0.11,
        tendon_slack_length: 0.13,
        pcsa: 30.0,
        pennation_angle: 0.1,
    });

    // Validate
    let errors = world.validate();
    if errors.is_empty() {
        println!("World is valid");
    } else {
        for e in &errors {
            println!("ERROR: {}", e);
        }
    }

    // Print summary
    println!("{:?}", world);
}
