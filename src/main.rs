use bevy::prelude::*;
use bevy_rapier2d::{prelude::*, rapier::prelude::{JointAxis, JointAxesMask}};

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            title: "Player Movement Example".to_string(),
            width: 1280.0,
            height: 1000.0,
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_startup_system(spawn_rocket)
        .add_system(motor_control)
        .add_system(engine_control)
        .add_system(landing_leg_control)
        .add_system(print_leg_position)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        .run();
}

// The float value is the player movement speed in 'pixels/second'.
#[derive(Component)]
struct Player(f32);

#[derive(Component)]
struct ColdGasThruster {
    pub force: f32
}

#[derive(Component)]
struct MainEngine {
    pub force: f32
}


#[derive(Component)]
struct LandingLeg;


#[derive(Component, Debug)]
enum Side {
    Left,
    Right,
}


fn spawn_rocket(mut commands: Commands) {
    commands.spawn().insert_bundle(Camera2dBundle::default());

    let rocket_height = 600.0;
    let rocket_radius = 50.0;

    // Spawn entity with `Player` struct as a component for access in movement query.
    let rocket_id = commands
        .spawn()
        .insert_bundle(SpriteBundle {
            sprite: Sprite {
                color: Color::rgb(0.0, 0.0, 0.0),
                custom_size: Some(Vec2::new(rocket_radius * 2.0, rocket_height)),
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(rocket_radius, rocket_height / 2.0))
        .insert(CollisionGroups::new(0b0001, 0b1110))
        .insert(Ccd::enabled()).id();
    
    add_cold_gas_thruster(
        &mut commands,
        rocket_id,
        Vec2::new(rocket_radius, rocket_height * 0.3),
        Side::Right,
    );

    add_cold_gas_thruster(
        &mut commands,
        rocket_id,
        Vec2::new(-rocket_radius, rocket_height * 0.3),
        Side::Left,
    );

    add_main_engine(
        &mut commands,
        rocket_id,
        Vec2::new(0.0, -rocket_height / 2.0),
    );

    add_landing_legs(
        &mut commands,
        rocket_id,
        -rocket_height / 2.0,
        2.0 * rocket_radius,
    );

    let ground_size = 500.0;
    let ground_height = 10.0;

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(
            0.0,
            -500.0,
            0.0,
        )))
        .insert(Collider::cuboid(ground_size, ground_height))
        .insert(Friction {
            coefficient: 0.8,
            combine_rule: CoefficientCombineRule::Average,
        })
        .insert(Ccd::enabled());
}

fn add_cold_gas_thruster(commands: &mut Commands, rocket_id: Entity, position: Vec2, side: Side) {
    let joint = FixedJointBuilder::new()
    .local_anchor1(Vec2::new(position.x, position.y));
    commands.spawn_bundle(TransformBundle::from(Transform::from_xyz(
        position.x,
        position.y,
        0.0,
    )))
        .insert(RigidBody::Dynamic)
        .insert(Collider::ball(8.0))
        .insert(CollisionGroups::new(0b0001, 0b1110))
        .insert(ExternalImpulse::default())
        .insert(ColdGasThruster{force: 5.0})
        .insert(side)
        .insert(ImpulseJoint::new(rocket_id, joint));
}

fn add_main_engine(commands: &mut Commands, rocket_id: Entity, position: Vec2) {
    let engine_height = 20.0;
    let engine_width = 40.0;
    let joint = FixedJointBuilder::new()
    .local_anchor1(Vec2::new(position.x, position.y - engine_height / 2.0));
    commands.spawn_bundle(TransformBundle::from(Transform::from_xyz(
        position.x,
        position.y - engine_height / 2.0,
        0.0,
    )))
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(engine_width / 2.0, engine_height / 2.0))
        .insert(ExternalImpulse::default())
        .insert(MainEngine{force: 20.0})
        .insert(ImpulseJoint::new(rocket_id, joint))
        .insert(CollisionGroups::new(0b0001, 0b1110))
        .insert(Ccd::enabled());
}

fn add_landing_legs(commands: &mut Commands, rocket_id: Entity, y_pos: f32, separation: f32) {
    let leg_length = 100.0;
    let leg_width = 20.0;
    let pi = std::f32::consts::PI;
    let motor_pos = pi * 0.99;
    let leg_angle = pi * 0.99;
    let joint_mask = JointAxesMask::X | JointAxesMask::Y;
    let joint = GenericJointBuilder::new(joint_mask)
        .local_axis1(Vec2::Y)
        .local_axis2(Vec2::Y)
        .local_anchor1(Vec2::new(-separation / 2.0, y_pos))
        .local_anchor2(Vec2::new(0.0, leg_length / 2.0))
        // .local_basis1(pi)
        .local_basis2(pi * 0.75)
        .set_motor(JointAxis::X, 0.0, 0.0, 10e9, 1.)
        .set_motor(JointAxis::X, 0.0, 0.0, 10e9, 1.)
        .motor_velocity(JointAxis::AngX, 0.0, 0.0);

    commands.spawn_bundle(TransformBundle::from(Transform::from_xyz(
            -separation / 2.0,
            y_pos,
            0.0,
        ).with_rotation(Quat::from_rotation_z(leg_angle))))
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(leg_width / 2.0, leg_length / 2.0))
        .insert(ImpulseJoint::new(rocket_id, joint))
        .insert(CollisionGroups::new(0b0001, 0b1110))
        .insert(LandingLeg)
        .insert(Side::Left)
        .insert(Ccd::enabled());

    let joint = GenericJointBuilder::new(joint_mask)
        .local_axis1(Vec2::Y)
        .local_axis2(Vec2::Y)
        .local_anchor1(Vec2::new(separation / 2.0, y_pos))
        .local_anchor2(Vec2::new(0.0, leg_length / 2.0))
        .local_basis2(-pi * 1.75 )
        .set_motor(JointAxis::X, 0.0, 0.0, 10e9, 1.)
        .set_motor(JointAxis::X, 0.0, 0.0, 10e9, 1.)
        .motor_velocity(JointAxis::AngX, 0.0, 0.0);

    commands.spawn_bundle(TransformBundle::from(Transform::from_xyz(
            separation / 2.0,
            y_pos,
            0.0,
        ).with_rotation(Quat::from_rotation_z(-leg_angle))))
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(leg_width / 2.0, leg_length / 2.0))
        .insert(ImpulseJoint::new(rocket_id, joint))
        .insert(CollisionGroups::new(0b0001, 0b1110))
        .insert(LandingLeg)
        .insert(Side::Right)
        .insert(Ccd::enabled());
}

fn motor_control(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&ColdGasThruster, &mut ExternalImpulse, &Side, &Transform)>,
) {
    let use_left_thruster = keyboard_input.pressed(KeyCode::D) || keyboard_input.pressed(KeyCode::Right);
    let use_right_thruster = keyboard_input.pressed(KeyCode::A) || keyboard_input.pressed(KeyCode::Left);
    for (thruster, mut ext_imp, side, tf) in query.iter_mut() {
        match side {
            Side::Left => {
                let right_dir = tf.right();
                ext_imp.impulse = if use_left_thruster {Vec2::new(right_dir.x, right_dir.y) * thruster.force} else {Vec2::ZERO}
            },
            Side::Right => {
                let left_dir = tf.left();
                ext_imp.impulse = if use_right_thruster {Vec2::new(left_dir.x, left_dir.y) * thruster.force} else {Vec2::ZERO}
            },
        }   
    }
}

fn engine_control(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&MainEngine, &mut ExternalImpulse, &Transform)>,
) {
    let up = keyboard_input.pressed(KeyCode::W) || keyboard_input.pressed(KeyCode::Up);
    for (thruster, mut ext_imp, tf) in query.iter_mut() {
        let up_dir = tf.up();
        ext_imp.impulse = if up {Vec2::new(up_dir.x, up_dir.y) * thruster.force} else {Vec2::ZERO}
    }
}

fn landing_leg_control(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&mut ImpulseJoint, &Side), With<LandingLeg>>,
) {
    let unfold = keyboard_input.pressed(KeyCode::Space);
    let pi = std::f32::consts::PI;
    for (mut joint, side) in query.iter_mut() {
        if unfold {
            match side {
                Side::Left => joint.data.set_motor(
                    JointAxis::AngX,
                    0.0,
                    0.0,
                    10000.0,
                    1000.0),
                Side::Right => joint.data.set_motor(
                    JointAxis::AngX,
                    0.0,
                    0.0,
                    10000.0,
                    1000.0),
            };
        }
    }
}

fn print_leg_position(
    mut query: Query<(&mut ImpulseJoint, &Side, &Transform), With<LandingLeg>>,
) {
    for (joint, side, tf) in query.iter_mut() {    
        match side {  
            Side::Left => println!("{:?} {:?}", side, tf.rotation.to_axis_angle()),
            _ => ()
        };
    }
}
