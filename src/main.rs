use bevy::prelude::*;
mod rocket;

fn main() {
    App::new()
        .insert_resource(WindowDescriptor {
            title: "Player Movement Example".to_string(),
            width: 1280.0,
            height: 1000.0,
            ..Default::default()
        })
        .insert_resource(ClearColor(Color::rgb(
            0x87 as f32 / 255.0,
            0xCE as f32 / 255.0,
            0xEB as f32 / 255.0,
        )))
        .add_plugins(DefaultPlugins)
        .add_system(bevy::window::close_on_esc)
        .add_plugin(rocket::RocketSimPlugin)
        .run();
}

