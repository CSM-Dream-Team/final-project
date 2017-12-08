pub mod gurus;

use std::path::Path;

use gfx;
use nalgebra::{Vector3, Point2};

use flight::{Error, load, PbrMesh, Texture};
use flight::draw::{DrawParams, Painter, PbrMaterial, PbrStyle, SolidStyle, UnishadeStyle};
use flight::mesh::*;

use self::gurus::*;
use geo::*;

pub struct Meshes<R: gfx::Resources> {
    // UI Elements
    pub controller: PbrMesh<R>,
    pub grid_lines: Mesh<R, VertC, ()>,
    pub floor: PbrMesh<R>,
    pub slider_control: PbrMesh<R>,
    pub slider_frame: PbrMesh<R>,

    // Rays
    pub red_ray: Mesh<R, VertC, ()>,
    pub blue_ray: Mesh<R, VertC, ()>,
}

pub struct Painters<R: gfx::Resources> {
    pub solid: Painter<R, SolidStyle<R>>,
    pub pbr: Painter<R, PbrStyle<R>>,
    pub unishade: Painter<R, UnishadeStyle<R>>,
}

pub struct Gurus {
    pub interact: interact::InteractGuru,
    pub physics: physics::PhysicsGuru,
}

pub struct GuruReply {
    pub interact: interact::InteractionReply,
    pub physics: physics::PhysicsReply,
}

pub struct Meta {
    pub physics_speed: f32,
}

pub struct Common<R: gfx::Resources, C: gfx::CommandBuffer<R>> {
    pub draw_params: DrawParams<R, C>,
    pub gurus: Gurus,
    pub painters: Painters<R>,
    pub meshes: Meshes<R>,
    pub meta: Meta,
}

pub struct CommonReply<R: gfx::Resources, C: gfx::CommandBuffer<R>> {
    pub draw_params: DrawParams<R, C>,
    pub reply: GuruReply,
    pub painters: Painters<R>,
    pub meshes: Meshes<R>,
    pub meta: Meta,
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
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Meshes<R>, Error> {
        use gfx::format::*;
        Ok(Meshes {
            controller: load_simple_object(
                factory,
                "assets/controller.obj",
                [0x80, 0x80, 0xFF, 0xFF])?,
            grid_lines: grid_lines(8, Vector3::new(8., 8., 8.)).upload(factory),
            floor: plane(5.)
                .with_tex(Point2::new(0., 0.))
                .compute_tan()
                .with_material(PbrMaterial {
                    normal: Texture::<_, (R8_G8_B8_A8, Unorm)>::uniform_value(factory, [0x80, 0x80, 0xFF, 0xFF])?,
                    albedo: Texture::<_, (R8_G8_B8_A8, Srgb)>::uniform_value(factory, [0xA0, 0xA0, 0xA0, 0xFF])?,
                    metalness: Texture::<_, (R8, Unorm)>::uniform_value(factory, 0xFF)?,
                    roughness: Texture::<_, (R8, Unorm)>::uniform_value(factory, 0x40)?,
                })
                .upload(factory),
            slider_control: load_simple_object(
                factory,
                "assets/slider/control.obj",
                [0x80, 0x80, 0xFF, 0xFF])?,
            slider_frame: load_simple_object(
                factory,
                "assets/slider/frame.obj",
                [0xFF, 0x80, 0x80, 0xFF])?,
            red_ray: make_ray([1., 0., 0.]).upload(factory),
            blue_ray: make_ray([0., 0., 1.]).upload(factory),
        })
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

impl<R: gfx::Resources, C: gfx::CommandBuffer<R>> Common<R, C> {
    pub fn resolve(self, dt: f32) -> CommonReply<R, C> {
        CommonReply {
            draw_params: self.draw_params,
            reply: GuruReply {
                interact: self.gurus.interact.resolve(),
                physics: self.gurus.physics.resolve(dt),
            },
            painters: self.painters,
            meshes: self.meshes,
            meta: self.meta,
        }
    }
}
