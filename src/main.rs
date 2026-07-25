fn main() {
    let mut world = World::new();

    // Create ground
    let _ground = world.add_body(0.0, [0.0, 0.0, 0.0], [0.0; 6]);

    // Create pelvis
    let pelvis = world.add_body(11.78, [0.0, 0.0, 0.0], [0.18, 0.22, 0.20, 0.0, 0.0, 0.0]);
    world.add_transform(pelvis, Transform::default());

    // Create femur
    let femur = world.add_body(9.3, [0.0, -0.17, 0.0], [0.12, 0.12, 0.02, 0.0, 0.0, 0.0]);

    // Create hip joint
    let _hip = world.add_joint(
        pelvis,
        femur,
        JointType::Ball,
        Some(JointLimits {
            lower: -2.0,
            upper: 2.0,
        }),
    );

    // Create a site for ASIS landmark
    let asis = world.add_site(pelvis, Vec3::new(0.01, 0.02, 0.13));

    // Attach landmark role to the site
    let mut landmarks = vec![];
    landmarks.push(Landmark {
        site: asis,
        name: "ASIS".to_string(),
    });

    // Create a muscle
    let _muscle_id = world.add_muscle(
        "iliopsoas".to_string(),
        vec![
            MusclePoint {
                body: pelvis,
                offset: Vec3::new(0.0, 0.0, 0.1),
            },
            MusclePoint {
                body: femur,
                offset: Vec3::new(0.0, -0.2, 0.0),
            },
        ],
        2000.0,
        0.11,
        0.13,
        30.0,
        0.1,
    );

    // Validate
    let errors = world.validate();
    if errors.is_empty() {
        println!("World is valid");
    } else {
        for e in &errors {
            println!("ERROR: {}", e);
        }
    }

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&world).unwrap();
    println!("World JSON ({} bytes)", json.len());
}
