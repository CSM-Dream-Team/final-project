use std::boxed::FnBox;

use gfx;
use common::{Common, CommonReply};

// pub mod halo;
// pub mod home;
// pub mod lets_get_physical;
pub mod snowflakes;
// pub mod workshop;

pub const NEAR_PLANE: f64 = 0.1;
pub const FAR_PLANE: f64 = 1000.;

pub trait App<C: gfx::CommandBuffer<R>, R: gfx::Resources> {
    fn update<'a, 'b>(&'a mut self, common: &'b mut Common<R>) -> Box<FnBox(&'a CommonReply<R>)>;
}
