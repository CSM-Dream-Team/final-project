use std::boxed::FnBox;

use gfx;
use common::{Common, CommonReply};

pub mod halo;
// pub mod home;
pub mod lets_get_physical;
pub mod snowflakes;
// pub mod workshop;

pub trait App<R: gfx::Resources, C: gfx::CommandBuffer<R>> {
    fn update<'a>(&'a mut self, common: &mut Common<R, C>) -> Box<FnBox(&mut CommonReply<R, C>) + 'a>;
}
