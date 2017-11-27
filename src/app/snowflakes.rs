use std::boxed::FnBox;

// Flight
use flight::{PbrMesh, Error, load};
use flight::draw::DrawParams;
use flight::vr::VrMoment;

// GFX
use gfx;
use app::App;

use common::{Common, CommonReply};

pub struct Snowflakes<R: gfx::Resources> {
    snowman: PbrMesh<R>,
    snow_block: PbrMesh<R>,
}


impl<R: gfx::Resources> Snowflakes<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Self, Error> {
        Ok(Snowflakes {
            snowman: load::object_directory(factory, "assets/snowman/")?,
            snow_block: load::object_directory(factory, "assets/snow-block/")?,
        })
    }
}

impl<R: gfx::Resources, C: gfx::CommandBuffer<R>> App<R, C> for Snowflakes<R> {
    fn update<'a, 'b>(&'a mut self, common: &'b mut Common<R, C>) -> Box<FnBox(&'a mut CommonReply<R, C>)> {
        Box::new(|r| {})
    }
}
