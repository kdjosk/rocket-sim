use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use rocket::TrackedByCamera;
mod rocket;

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            title: "Player Movement Example".to_string(),
            width: 1280.0,
            height: 1000.0,
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_startup_system(rocket::spawn_falcon9_rocket)
        .add_system(rocket::motor_control)
        .add_system(rocket::engine_control)
        .add_system(rocket::landing_leg_control)
        .add_system(update_camera)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(rocket::PX_PER_METER))
        .add_plugin(RapierDebugRenderPlugin::default())
        .run();
}

fn update_camera(mut query: Query<(&mut Transform, Option<&Camera>, Option<&TrackedByCamera>)>) {
    let mut obj_tf = Transform::default();
    for (tf, _, obj) in query.iter_mut() {
        if let Some(_) = obj {
            obj_tf = tf.clone();
        }
    }

    for (mut tf, camera, _) in query.iter_mut() {
        if let Some(_) = camera {
            tf.translation = obj_tf.translation;
        }
    }
}