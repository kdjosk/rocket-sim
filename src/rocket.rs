use bevy::prelude::*;
use bevy_rapier2d::{prelude::*, rapier::prelude::{JointAxis, JointAxesMask}};

pub struct TwoStageRocket<'a> {
    first_stage: Stage<'a>,
    second_stage: Stage<'a>,
    landing_leg: LandingLeg,
}

pub struct Stage<'a> {
    height: f32,
    diameter: f32,
    engines: &'a [Engine],
}

pub struct Engine {
    isp_asl: f32,
    isp_vac: f32,
    thrust_asl: f32,
    thrust_vac: f32,
}

#[derive(Component)]
pub struct TrackedByCamera; 

#[derive(Component)]
pub struct ColdGasThruster {
    pub force: f32
}

#[derive(Component)]
pub struct MainEngine {
    pub force: f32
}

#[derive(Component, Clone)]
pub struct LandingLeg {
    length: f32,
    mass: f32,
    width: f32,
}

pub struct Ground {
    height: f32,
    width: f32,
}

#[derive(Component, Debug)]
pub enum Side {
    Left,
    Right,
}

pub const PX_PER_METER: f32 = 10.0;

impl Engine {
    pub const MERLIN_1D: Self = Self {
        isp_asl: 282.0,
        isp_vac: 311.0,
        thrust_asl: 854_000.0,
        thrust_vac: 931_000.0,
    };

    pub const MERLIN_1D_VAC: Self = Self {
        isp_asl: 150.0,
        isp_vac: 348.0,
        thrust_asl: 400_000.0,
        thrust_vac: 981_000.0
    };
}

impl TwoStageRocket<'static> {
    // https://forum.nasaspaceflight.com/index.php?topic=41947.0
    pub const FALCON_9_V_1_2: Self = Self {
        first_stage: Stage {
                height: 42.6 * PX_PER_METER,
                diameter: 3.66 * PX_PER_METER,
                engines: &[
                    Engine::MERLIN_1D,
                    Engine::MERLIN_1D,
                    Engine::MERLIN_1D,
                    Engine::MERLIN_1D,
                    Engine::MERLIN_1D,
                    Engine::MERLIN_1D,
                    Engine::MERLIN_1D,
                    Engine::MERLIN_1D,
                    Engine::MERLIN_1D,
                ]
            },
        second_stage: Stage {
                height: 27.4 * PX_PER_METER,
                diameter: 3.66 * PX_PER_METER,
                engines: &[
                    Engine::MERLIN_1D_VAC,
                ]
            },
        landing_leg: LandingLeg { length: 10.0 * PX_PER_METER, width: 0.3 * PX_PER_METER, mass: 600.0 }
    };

    fn height(self) -> f32 {
        self.first_stage.height + self.second_stage.height
    }
}


static ROCKET_PART_GROUP: u32 = 0b0001;


fn spawn_stage(commands: &mut Commands, stage_height: f32, stage_diameter: f32) -> Entity {
    return commands
    .spawn()
    .insert_bundle(SpriteBundle {
        sprite: Sprite {
            color: Color::rgb(1.0, 1.0, 1.0),
            custom_size: Some(Vec2::new(stage_diameter, stage_height)),
            ..Default::default()
        },
        ..Default::default()
    })
    .insert(RigidBody::Dynamic)
    .insert(Collider::cuboid(stage_diameter / 2.0, stage_height / 2.0))
    .insert(CollisionGroups::new(ROCKET_PART_GROUP, !ROCKET_PART_GROUP))
    .insert(Ccd::enabled()).id();
}


pub fn spawn_falcon9_rocket(mut commands: Commands) {
    let rocket = &TwoStageRocket::FALCON_9_V_1_2;

    let first_stage = spawn_stage(&mut commands, rocket.first_stage.height, rocket.first_stage.diameter);

    commands.entity(first_stage).insert(TrackedByCamera);

    commands.spawn_bundle(Camera2dBundle::default());
    
    add_cold_gas_thruster(
        &mut commands,
        first_stage,
        Vec2::new(rocket.first_stage.diameter / 2.0, rocket.first_stage.height * 0.3),
        Side::Right,
    );

    add_cold_gas_thruster(
        &mut commands,
        first_stage,
        Vec2::new(-rocket.first_stage.diameter / 2.0, rocket.first_stage.height * 0.3),
        Side::Left,
    );

    add_main_engine(
        &mut commands,
        first_stage,
        Vec2::new(0.0, -rocket.first_stage.height / 2.0),
    );

    add_landing_legs(
        &mut commands,
        first_stage,
        -rocket.first_stage.height / 2.0,
        rocket.first_stage.diameter,
        &rocket.landing_leg,
    );

    add_ground(
        &mut commands,
        &Ground { height: 2.0 * PX_PER_METER, width: 50.0 * PX_PER_METER },
    )


}


fn add_ground(commands: &mut Commands, ground: &Ground) {

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(
            0.0,
            -500.0,
            0.0,
        )))
        .insert(Collider::cuboid(ground.width, ground.height))
        .insert(Friction {
            coefficient: 1.0,
            combine_rule: CoefficientCombineRule::Max,
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
        1.0,
    )))
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(engine_width / 2.0, engine_height / 2.0))
        .insert(ExternalImpulse::default())
        .insert(MainEngine{force: 20.0})
        .insert(ImpulseJoint::new(rocket_id, joint))
        .insert(CollisionGroups::new(0b0001, 0b1110))
        .insert(Ccd::enabled());
}

fn add_landing_legs(commands: &mut Commands, rocket_id: Entity, y_pos: f32, separation: f32, leg: &LandingLeg) {
    let pi = std::f32::consts::PI;
    let motor_pos = pi * 0.99;
    let leg_angle = pi * 0.99;
    let joint_mask = JointAxesMask::X | JointAxesMask::Y;
    let joint = GenericJointBuilder::new(joint_mask)
        .local_axis1(Vec2::Y)
        .local_axis2(Vec2::Y)
        .local_anchor1(Vec2::new(-separation / 2.0, y_pos))
        .local_anchor2(Vec2::new(0.0, leg.length / 2.0))
        .local_basis2(pi * 0.75)
        .set_motor(JointAxis::X, 0.0, 0.0, 10e9, 1.)
        .set_motor(JointAxis::Y, 0.0, 0.0, 10e9, 1.)
        .motor_velocity(JointAxis::AngX, 0.0, 0.0);

    let tf = Transform::from_xyz(
        -separation / 2.0,
        y_pos,
        1.0,
    ).with_rotation(Quat::from_rotation_z(leg_angle));
    commands.spawn_bundle(TransformBundle::from(tf))
        .insert_bundle(SpriteBundle {
            sprite: Sprite {
                color: Color::rgb(0.0, 0.0, 0.0),
                custom_size: Some(Vec2::new(leg.width, leg.length)),
                ..Default::default()
            },
            transform: tf,
            ..Default::default()
        })
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(leg.width / 2.0, leg.length / 2.0))
        .insert(ImpulseJoint::new(rocket_id, joint))
        .insert(CollisionGroups::new(0b0001, 0b1110))
        .insert(leg.clone())
        .insert(Side::Left)
        .insert(Ccd::enabled());

    let joint = GenericJointBuilder::new(joint_mask)
        .local_axis1(Vec2::Y)
        .local_axis2(Vec2::Y)
        .local_anchor1(Vec2::new(separation / 2.0, y_pos))
        .local_anchor2(Vec2::new(0.0, leg.length / 2.0))
        .local_basis2(-pi * 1.75 )
        .set_motor(JointAxis::X, 0.0, 0.0, 10e9, 1.)
        .set_motor(JointAxis::Y, 0.0, 0.0, 10e9, 1.)
        .motor_velocity(JointAxis::AngX, 0.0, 0.0);
    
    let tf = Transform::from_xyz(
        separation / 2.0,
        y_pos,
        0.0,
    ).with_rotation(Quat::from_rotation_z(-leg_angle));

    commands.spawn_bundle(TransformBundle::from(tf))
        .insert_bundle(SpriteBundle {
            sprite: Sprite {
                color: Color::rgb(0.0, 0.0, 0.0),
                custom_size: Some(Vec2::new(leg.width, leg.length)),
                ..Default::default()
            },
            transform: tf,
            ..Default::default()
        })
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(leg.width / 2.0, leg.length / 2.0))
        .insert(ImpulseJoint::new(rocket_id, joint))
        .insert(CollisionGroups::new(0b0001, 0b1110))
        .insert(leg.clone())
        .insert(Side::Right)
        .insert(Ccd::enabled());
}

pub fn motor_control(
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

pub fn engine_control(
    keyboard_input: Res<Input<KeyCode>>,
    mut query: Query<(&MainEngine, &mut ExternalImpulse, &Transform)>,
) {
    let up = keyboard_input.pressed(KeyCode::W) || keyboard_input.pressed(KeyCode::Up);
    for (thruster, mut ext_imp, tf) in query.iter_mut() {
        let up_dir = tf.up();
        ext_imp.impulse = if up {Vec2::new(up_dir.x, up_dir.y) * thruster.force} else {Vec2::ZERO}
    }
}

pub fn landing_leg_control(
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
