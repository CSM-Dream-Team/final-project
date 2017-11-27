pub mod gurus;

use std::path::Path;

use gfx;
use nalgebra::Vector3;

use flight::{Error, load, PbrMesh, Texture};
use flight::draw::{Painter, PbrMaterial, PbrStyle, SolidStyle, UnishadeStyle};
use flight::mesh::*;

use self::gurus::*;
use geo::*;

pub struct Meshes<R: gfx::Resources> {
    // UI Elements
    controller: PbrMesh<R>,
    grid_lines: Mesh<R, VertC, ()>,

    // Rays
    red_ray: Mesh<R, VertC, ()>,
    blue_ray: Mesh<R, VertC, ()>,
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

pub struct GuruReply {
    interact: interact::InteractionReply,
    physics: physics::PhysicsReply,
}

pub struct Common<R: gfx::Resources> {
    gurus: Gurus,
    painters: Painters<R>,
    meshes: Meshes<R>,
}

pub struct CommonReply<R: gfx::Resources> {
    reply: GuruReply,
    painters: Painters<R>,
    meshes: Meshes<R>,
}

fn load_simple_object<P, R, F>(f: &mut F, path: P, albedo: [u8; 4])
    -> Result<Mesh<R, VertNTT, PbrMaterial<R>>, Error>
    where P: AsRef<Path>, R: gfx::Resources, F: gfx::Factory<R>
{
    use gfx::format::*;
    Ok(load::wavefront_file(path)
        ?
        .compute_tan()
        .with_material(PbrMaterial {
            normal: Texture::<_, (R8_G8_B8_A8, Unorm)>::uniform_value(f, albedo)?,
            albedo: Texture::<_, (R8_G8_B8_A8, Srgb)>::uniform_value(f, [0x60, 0x60, 0x60, 0xFF])?,
            metalness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x00)?,
            roughness: Texture::<_, (R8, Unorm)>::uniform_value(f, 0x20)?,
        })
        .upload(f))
}

impl<R: gfx::Resources> Meshes<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Meshes<R> {
        Meshes {
            controller: load_simple_object(factory,
                                           "assets/controller.obj",
                                           [0x80, 0x80, 0xFF, 0xFF]).unwrap(),
            grid_lines: grid_lines(8, Vector3::new(8., 8., 8.)).upload(factory),
            red_ray: make_ray([1., 0., 0.]).upload(factory),
            blue_ray: make_ray([0., 0., 1.]).upload(factory),
        }
    }
}

impl<R: gfx::Resources> Painters<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Painters<R>, Error> {
        let mut solid = Painter::new(factory)?;
        solid.setup(factory, Primitive::LineList)?;
        solid.setup(factory, Primitive::TriangleList)?;

        let mut pbr: Painter<_, PbrStyle<_>> = Painter::new(factory)?;
        pbr.setup(factory, Primitive::TriangleList)?;

        let mut unishade = Painter::new(factory)?;
        unishade.setup(factory, Primitive::TriangleList)?;

        Ok(Painters {
            solid: solid,
            pbr: pbr,
            unishade: unishade,
        })
    }
}

impl<R: gfx::Resources> Common<R> {
    pub fn resolve(self) -> CommonReply<R> {
        CommonReply {
            reply: GuruReply {
                interact: self.gurus.interact.finish(),
                physics: self.gurus.physics.finish(),
            },
            painters: self.painters,
            meshes: self.meshes,
        }
    }
}
