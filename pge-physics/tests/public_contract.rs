use pge_physics::{
    BodyDesc, BodyId, BodyMode, BoundedKinematicTarget, ColliderDesc, ColliderId, ColliderShape,
    JointBreakCause, JointBreakThresholds, JointDesc, JointId, JointKindDesc, KinematicTargetMode,
    MassPropertiesDesc, PhysicsConfig, PhysicsError, PhysicsEventKind, PhysicsWorld, Pose,
};

fn add_articulation_body(
    world: &mut PhysicsWorld,
    id: &str,
    position: [f32; 3],
    mode: BodyMode,
    linear_velocity_mps: [f32; 3],
    angular_velocity_rps: [f32; 3],
) -> BodyId {
    let id = BodyId::new(id);
    world
        .create_body(
            id.clone(),
            BodyDesc {
                mode,
                pose: Pose {
                    translation: position,
                    ..Pose::default()
                },
                linear_velocity_mps,
                angular_velocity_rps,
                mass: (mode == BodyMode::Dynamic).then_some(MassPropertiesDesc {
                    mass_kg: 1.0,
                    principal_inertia_kg_m2: [0.1; 3],
                    ..MassPropertiesDesc::default()
                }),
                ..BodyDesc::default()
            },
        )
        .unwrap();
    id
}

#[test]
fn external_consumer_uses_only_pge_owned_physics_contracts() {
    let mut world = PhysicsWorld::new(PhysicsConfig {
        fixed_dt_sec: 1.0 / 120.0,
        substeps: 2,
        ..PhysicsConfig::default()
    })
    .unwrap();
    let floor = BodyId::new("floor");
    world
        .create_body(
            floor.clone(),
            BodyDesc {
                mode: BodyMode::Static,
                pose: Pose {
                    translation: [0.0, 0.0, -0.05],
                    ..Pose::default()
                },
                ..BodyDesc::default()
            },
        )
        .unwrap();
    world
        .create_collider(
            ColliderId::new("floor:collider"),
            &floor,
            ColliderDesc::new(ColliderShape::Box {
                size: [2.0, 2.0, 0.1],
            }),
        )
        .unwrap();
    let ball = BodyId::new("ball");
    world
        .create_body(
            ball.clone(),
            BodyDesc {
                pose: Pose {
                    translation: [0.0, 0.0, 0.3],
                    ..Pose::default()
                },
                ..BodyDesc::default()
            },
        )
        .unwrap();
    let ball_collider = ColliderId::new("ball:collider");
    world
        .create_collider(
            ball_collider.clone(),
            &ball,
            ColliderDesc::new(ColliderShape::Sphere { radius: 0.05 }),
        )
        .unwrap();

    let checkpoint = world.checkpoint();
    let mut saw_contact = false;
    for _ in 0..120 {
        saw_contact |= world
            .step()
            .events
            .iter()
            .any(|event| event.kind == PhysicsEventKind::ContactStarted);
    }
    assert!(saw_contact);
    assert_eq!(world.body_snapshot(&ball).unwrap().id, ball);
    assert_eq!(
        world
            .cast_ray([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 2.0, true)
            .unwrap()
            .unwrap()
            .collider,
        ball_collider
    );
    assert_eq!(world.snapshot().bodies.len(), 2);

    world.restore(&checkpoint).unwrap();
    assert_eq!(world.snapshot().step_index, 0);
    world.remove_body(&ball).unwrap();
    assert_eq!(world.snapshot().bodies.len(), 1);
}

#[test]
fn coupled_pose_target_uses_angular_cap_to_slow_translation() {
    fn world_with_kinematic_body(id: &BodyId) -> PhysicsWorld {
        let mut world = PhysicsWorld::new(PhysicsConfig {
            gravity_mps2: [0.0; 3],
            fixed_dt_sec: 0.1,
            substeps: 1,
        })
        .unwrap();
        world
            .create_body(
                id.clone(),
                BodyDesc {
                    mode: BodyMode::KinematicPosition,
                    ..BodyDesc::default()
                },
            )
            .unwrap();
        world
    }

    let target = BoundedKinematicTarget {
        pose: Pose {
            translation: [1.0, 0.0, 0.0],
            rotation_xyzw: [
                0.0,
                0.0,
                std::f32::consts::FRAC_1_SQRT_2,
                std::f32::consts::FRAC_1_SQRT_2,
            ],
        },
        maximum_linear_speed_mps: 1.0,
        maximum_angular_speed_rps: 0.2,
        maximum_linear_acceleration_mps2: 1_000.0,
        maximum_angular_acceleration_rps2: 1_000.0,
    };

    let independent_id = BodyId::new("independent");
    let mut independent = world_with_kinematic_body(&independent_id);
    independent
        .set_bounded_kinematic_target(&independent_id, target)
        .unwrap();
    independent.step();
    let independent_pose = independent.body_snapshot(&independent_id).unwrap().pose;

    let coupled_id = BodyId::new("coupled");
    let mut coupled = world_with_kinematic_body(&coupled_id);
    coupled
        .set_bounded_kinematic_target_with_mode(
            &coupled_id,
            target,
            KinematicTargetMode::CoupledPose,
        )
        .unwrap();
    coupled.step();
    let coupled_pose = coupled.body_snapshot(&coupled_id).unwrap().pose;
    let coupled_angle = 2.0 * coupled_pose.rotation_xyzw[2].asin();
    let coupled_translation_fraction = coupled_pose.translation[0];
    let coupled_rotation_fraction = coupled_angle / std::f32::consts::FRAC_PI_2;

    assert!((independent_pose.translation[0] - 0.1).abs() < 1.0e-5);
    assert!(coupled_pose.translation[0] < independent_pose.translation[0] * 0.2);
    assert!((coupled_translation_fraction - coupled_rotation_fraction).abs() < 1.0e-5);
    assert!((coupled_angle - 0.02).abs() < 1.0e-5);
}

#[test]
fn multibody_chain_is_publicly_observable_and_checkpoint_stable() {
    let mut world = PhysicsWorld::new(PhysicsConfig {
        gravity_mps2: [0.0; 3],
        fixed_dt_sec: 1.0 / 120.0,
        substeps: 2,
    })
    .unwrap();
    let base = add_articulation_body(
        &mut world,
        "base",
        [0.0, 0.0, 0.0],
        BodyMode::Static,
        [0.0; 3],
        [0.0; 3],
    );
    let shoulder = add_articulation_body(
        &mut world,
        "shoulder",
        [1.0, 0.0, 0.0],
        BodyMode::Dynamic,
        [0.0; 3],
        [0.0; 3],
    );
    let wrist = add_articulation_body(
        &mut world,
        "wrist",
        [2.0, 0.0, 0.0],
        BodyMode::Dynamic,
        [0.0; 3],
        [0.0; 3],
    );
    let shoulder_joint = JointId::new("shoulder-joint");
    let wrist_joint = JointId::new("wrist-joint");
    let fixed_link = |body1: BodyId, body2: BodyId| JointDesc {
        body1,
        body2,
        local_frame1: Pose {
            translation: [0.5, 0.0, 0.0],
            ..Pose::default()
        },
        local_frame2: Pose {
            translation: [-0.5, 0.0, 0.0],
            ..Pose::default()
        },
        kind: JointKindDesc::Fixed,
        contacts_enabled: false,
    };
    world
        .create_multibody_joint(shoulder_joint.clone(), fixed_link(base, shoulder.clone()))
        .unwrap();
    world
        .create_multibody_joint(wrist_joint.clone(), fixed_link(shoulder, wrist.clone()))
        .unwrap();
    let checkpoint = world.checkpoint();
    world.step();
    let first = world.snapshot();
    assert_eq!(first.joints.len(), 2);
    assert!(first
        .joints
        .iter()
        .flat_map(|joint| joint.constraint_error)
        .all(f32::is_finite));
    assert!((world.body_snapshot(&wrist).unwrap().pose.translation[0] - 2.0).abs() < 1.0e-3);

    world.restore(&checkpoint).unwrap();
    world.step();
    assert_eq!(world.snapshot(), first);
    world.remove_joint(&shoulder_joint).unwrap();
    assert_eq!(world.snapshot().joints.len(), 1);
    assert_eq!(world.joint_snapshot(&wrist_joint).unwrap().id, wrist_joint);
}

#[test]
fn joint_friction_is_a_bounded_coulomb_impulse() {
    let mut world = PhysicsWorld::new(PhysicsConfig {
        gravity_mps2: [0.0; 3],
        fixed_dt_sec: 0.1,
        substeps: 1,
    })
    .unwrap();
    let base = add_articulation_body(
        &mut world,
        "friction-base",
        [0.0; 3],
        BodyMode::Static,
        [0.0; 3],
        [0.0; 3],
    );
    let child = add_articulation_body(
        &mut world,
        "friction-child",
        [0.0; 3],
        BodyMode::Dynamic,
        [0.0; 3],
        [0.0, 0.0, 2.0],
    );
    let joint = JointId::new("friction-joint");
    world
        .create_joint(
            joint.clone(),
            JointDesc {
                body1: base,
                body2: child.clone(),
                local_frame1: Pose::default(),
                local_frame2: Pose::default(),
                kind: JointKindDesc::Revolute {
                    axis: [0.0, 0.0, 1.0],
                    limits: None,
                    motor: None,
                },
                contacts_enabled: false,
            },
        )
        .unwrap();
    world.set_joint_friction(&joint, 1.0).unwrap();
    world.step();
    let snapshot = world.joint_snapshot(&joint).unwrap();
    assert_eq!(snapshot.friction_maximum_effort, Some(1.0));
    assert!(snapshot.friction_applied_impulse[5] < 0.0);
    assert!(snapshot.friction_applied_impulse[5].abs() <= 0.1 + 1.0e-6);
    let resulting_speed = world.body_snapshot(&child).unwrap().angular_velocity_rps[2];
    assert!((0.0..2.0).contains(&resulting_speed));
}

#[test]
fn break_thresholds_emit_stable_events_and_replay_after_restore() {
    let mut world = PhysicsWorld::new(PhysicsConfig {
        gravity_mps2: [0.0; 3],
        fixed_dt_sec: 0.1,
        substeps: 1,
    })
    .unwrap();
    for prefix in ["a", "b"] {
        let base = add_articulation_body(
            &mut world,
            &format!("{prefix}-base"),
            [0.0; 3],
            BodyMode::Static,
            [0.0; 3],
            [0.0; 3],
        );
        let child = add_articulation_body(
            &mut world,
            &format!("{prefix}-child"),
            [0.0; 3],
            BodyMode::Dynamic,
            [5.0, 0.0, 0.0],
            [0.0; 3],
        );
        let joint = JointId::new(format!("{prefix}-joint"));
        world
            .create_joint(
                joint.clone(),
                JointDesc {
                    body1: base,
                    body2: child,
                    local_frame1: Pose::default(),
                    local_frame2: Pose::default(),
                    kind: JointKindDesc::Fixed,
                    contacts_enabled: false,
                },
            )
            .unwrap();
        world
            .set_joint_break_thresholds(
                &joint,
                JointBreakThresholds {
                    maximum_force_n: None,
                    maximum_torque_nm: None,
                    maximum_linear_impulse_ns: Some(0.0),
                    maximum_angular_impulse_nms: None,
                },
            )
            .unwrap();
    }
    let checkpoint = world.checkpoint();
    let first = world.step().joint_breaks;
    assert_eq!(first.len(), 2);
    assert_eq!(first[0].joint, JointId::new("a-joint"));
    assert_eq!(first[1].joint, JointId::new("b-joint"));
    assert_eq!(first[0].cause, JointBreakCause::LinearImpulse);
    assert_eq!(first[0].step_index, 1);
    assert_eq!(first[0].sequence, 0);
    assert_eq!(first[1].sequence, 1);
    assert_eq!(
        world.joint_snapshot(&first[0].joint),
        Err(PhysicsError::UnknownJoint(first[0].joint.clone()))
    );

    world.restore(&checkpoint).unwrap();
    assert_eq!(world.step().joint_breaks, first);
}
