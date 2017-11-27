// Flight
use flight::{PbrMesh, Error, load};
use flight::draw::DrawParams;
use flight::vr::VrMoment;

// GFX
use gfx;
use app::App;

pub struct LetsGetPhysical {}

impl LetsGetPhysical {
    pub fn new() -> LetsGetPhysical {
        LetsGetPhysical {}
    }
}

impl<C: gfx::CommandBuffer<R>, R: gfx::Resources> App<C, R> for LetsGetPhysical {
    fn draw(&mut self, ctx: &mut DrawParams<R, C>, vrm: &VrMoment) {

    }
}
