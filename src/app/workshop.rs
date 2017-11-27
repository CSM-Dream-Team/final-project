// Flight
use flight::{PbrMesh, Error, load};
use flight::draw::DrawParams;
use flight::vr::VrMoment;

// GFX
use gfx;
use app::App;

pub struct Workshop {}

impl Workshop {
    pub fn new() -> Workshop {
        Workshop {}
    }
}

impl<C: gfx::CommandBuffer<R>, R: gfx::Resources> App<C, R> for Workshop {
    fn draw(&mut self, ctx: &mut DrawParams<R, C>, vrm: &VrMoment) {

    }
}
