// Flight
use flight::{PbrMesh, Error, load};
use flight::draw::DrawParams;
use flight::vr::VrMoment;

// GFX
use gfx;
use app::App;

pub struct Home {}

impl Home {
    pub fn new() -> Home {
        Home {}
    }
}

impl<C: gfx::CommandBuffer<R>, R: gfx::Resources> App<C, R> for Home {
    fn draw(&mut self, ctx: &mut DrawParams<R, C>, vrm: &VrMoment) {

    }
}
