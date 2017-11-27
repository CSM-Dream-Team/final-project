use gfx;
use flight::draw::{Painter, PbrStyle, SolidStyle, UnishadeStyle};

pub mod gurus;

pub struct Meshes {

}

pub struct Painters<R: gfx::Resources> {
    solid: Painter<R, SolidStyle<R>>,
    pbr: Painter<R, PbrStyle<R>>,
    unishade: Painter<R, UnishadeStyle<R>>,
}
