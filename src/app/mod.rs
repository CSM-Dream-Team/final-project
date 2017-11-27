use gfx;
use flight::draw::DrawParams;
use flight::vr::VrMoment;

pub mod halo;
pub mod home;
pub mod lets_get_physical;
pub mod snowflakes;
pub mod workshop;

pub const NEAR_PLANE: f64 = 0.1;
pub const FAR_PLANE: f64 = 1000.;

pub trait App<C: gfx::CommandBuffer<R>, R: gfx::Resources> {
    fn draw(&mut self, ctx: &mut DrawParams<R, C>, vrm: &VrMoment);
}
