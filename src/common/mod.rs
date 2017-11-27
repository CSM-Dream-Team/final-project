pub mod gurus;

use gfx;

use flight::PbrMesh;
use flight::draw::{Painter, PbrStyle, SolidStyle, UnishadeStyle};
use flight::mesh::*;

use self::gurus::*;

pub struct Meshes<R: gfx::Resources> {
    // UI Elements
    controller: PbrMesh<R>,
    grid_lines: Mesh<R, VertC, ()>,

    // Rays
    red_ray: Mesh<R, VertC, ()>,
    blue_ray: Mesh<R, VertC, ()>,

    // Shapes
    cube: PbrMesh<R>,
}

pub struct Painters<R: gfx::Resources> {
    solid: Painter<R, SolidStyle<R>>,
    pbr: Painter<R, PbrStyle<R>>,
    unishade: Painter<R, UnishadeStyle<R>>,
}

pub struct Gurus {
    interact: interact::InteractGuru,
    physics: physics::PhysicsGuru,
}

pub struct Common<R: gfx::Resources> {
    gurus: Gurus,
    painters: Painters<R>,
    meshes: Meshes<R>,
}
