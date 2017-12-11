use std::io::{Read, Write};
use std::boxed::FnBox;

use serde::{self, Serialize, Deserialize};
use serde_json::{Deserializer, Serializer, Error as JsonError};
use serde_json::de::IoRead as JsonRead;

use nalgebra::{Vector3, Isometry3, Translation3, UnitQuaternion};

// GFX
use gfx;
use app::App;
use ui::Slider;

use common::{Common, CommonReply};

pub struct Settings {
    pub speed: Slider,
    pub length: Slider,
}

#[derive(Serialize, Deserialize)]
pub struct SettingsState {
    pub speed: f32,
    pub length: f32,
    pub speed_slider_pos: Isometry3<f32>,
    pub length_slider_pos: Isometry3<f32>,
}

impl Settings {
    pub fn new() -> Self {
        Settings {
            speed: Slider::new(Isometry3::from_parts(
                    Translation3::new(0., 1.5, 0.),
                    UnitQuaternion::rotation_between(
                        &Vector3::new(0., 0., 1.),
                        &Vector3::new(1., 1., 1.),
                    ).unwrap(),
                ),
                0.15,
                0.50,
                0.20,
                1.,
            ),
            length: Slider::new(Isometry3::from_parts(
                    Translation3::new(0.5, 1.5, -0.5),
                    UnitQuaternion::rotation_between(
                        &Vector3::new(0., 0., 1.),
                        &Vector3::new(1., 1., 1.),
                    ).unwrap(),
                ),
                0.15,
                0.50,
                0.20,
                0.5,
            ),
        }
    }
}

impl<R: gfx::Resources + 'static, C: gfx::CommandBuffer<R> + 'static, W: Write, Re: Read> App<R, C, W, Re>
    for Settings {
    fn se_state(&self,
                serializer: &mut Serializer<W>)
                -> Result<<&mut Serializer<W> as serde::Serializer>::Ok, JsonError> {
        let state = SettingsState {
            speed: self.speed.value,
            length: self.length.value,
            speed_slider_pos: self.speed.position,
            length_slider_pos: self.length.position,
        };
        state.serialize(serializer)
    }

    fn de_state(&mut self, deserializer: &mut Deserializer<JsonRead<Re>>) -> Result<(), JsonError> {
        let state = SettingsState::deserialize(deserializer)?;
        self.speed.value = state.speed;
        self.length.value = state.length;
        self.speed.position = state.speed_slider_pos;
        self.length.position = state.length_slider_pos;
        Ok(())
    }

    fn update<'b>(&'b mut self,
                  common: &mut Common<R, C>)
                  -> Box<FnBox(&mut CommonReply<R, C>) + 'b> {
        self.speed.length = 0.2 + 0.6 * self.length.value;
        let speed = self.speed.update(&mut common.gurus.interact);
        let length = self.length.update(&mut common.gurus.interact);

        Box::new(move |r: &mut CommonReply<_, _>| {
            r.meta.physics_speed = speed(r);
            length(r);
        })
    }
}
