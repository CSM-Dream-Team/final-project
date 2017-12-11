use std::boxed::FnBox;

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
                    Translation3::new(0., 1.5, 0.),
                    UnitQuaternion::rotation_between(
                        &Vector3::new(0., 0.5, 1.),
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

impl<R: gfx::Resources, C: gfx::CommandBuffer<R> + 'static> App<R, C> for Settings {
    fn update<'a>(&'a mut self, common: &mut Common<R, C>) -> Box<FnBox(&mut CommonReply<R, C>) + 'a>
    {
        self.speed.length = 0.2 + 0.6 * self.length.value;
        let speed = self.speed.update(&mut common.gurus.interact);
        let length = self.length.update(&mut common.gurus.interact);

        Box::new(move |r: &mut CommonReply<_, _>| {
            r.meta.physics_speed = speed(r);
            length(r);
        })
    }
}
