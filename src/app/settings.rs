use std::boxed::FnBox;

use nalgebra::{Vector3, Isometry3, Translation3, UnitQuaternion};
use std::cell::Cell;
use std::rc::Rc;

// GFX
use gfx;
use app::App;
use ui::Slider;

use common::{Common, CommonReply};

pub struct Settings {
    pub speed: Slider,
    speed_value: Rc<Cell<f32>>,
}

impl Settings {
    pub fn new(value: Rc<Cell<f32>>) -> Self {
        let rot = UnitQuaternion::rotation_between(
            &Vector3::new(0., 0., 1.),
            &Vector3::new(1., 1., 1.),
        ).unwrap();
        Settings {
            speed: Slider::new(Isometry3::from_parts(
                    Translation3::new(0., 1.5, 0.),
                    rot,
                ),
                0.05,
                0.3,
                1.,
            ),
            speed_value: value,
        }
    }
}

impl<R: gfx::Resources, C: gfx::CommandBuffer<R> + 'static> App<R, C> for Settings {
    fn update<'a>(&'a mut self, common: &mut Common<R, C>) -> Box<FnBox(&mut CommonReply<R, C>) + 'a>
    {
        let speed = self.speed.update(&mut common.gurus.interact.primary);
        let speed_value = &self.speed_value;
        Box::new(move |r: &mut CommonReply<_, _>| {
            speed_value.set(speed(r));
        })
    }
}
