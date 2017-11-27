// Flight
use flight::{PbrMesh, Error, load};
use flight::draw::DrawParams;
use flight::vr::VrMoment;

// GFX
use gfx;
use app::App;

pub struct Halo {}

impl Halo {
    pub fn new() -> Halo {
        Halo {}
    }
}

impl<C: gfx::CommandBuffer<R>, R: gfx::Resources> App<C, R> for Halo {
    fn draw(&mut self, ctx: &mut DrawParams<R, C>, vrm: &VrMoment) {

    }
}