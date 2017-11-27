
// Flight
use flight::{PbrMesh, Error, load};
use flight::draw::DrawParams;
use flight::vr::VrMoment;

// GFX
use gfx;
use app::App;

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

impl<C: gfx::CommandBuffer<R>, R: gfx::Resources> App<C, R> for Snowflakes<R> {
    fn draw(&mut self, ctx: &mut DrawParams<R, C>, vrm: &VrMoment) {}
}
