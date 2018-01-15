pub mod gurus;

use std::collections::HashMap;
use std::path::Path;

use gfx;
use nalgebra::{Vector3, Point2};

use flight::{Error, load, UberMesh, Texture};
use flight::draw::{DrawParams, Painter, UberMaterial, UberStyle, SolidStyle, UnishadeStyle};
use flight::mesh::*;

use self::gurus::*;
use geo::*;

pub struct Meshes<R: gfx::Resources> {
    // UI Elements
    pub controller: UberMesh<R>,
    pub wire_box: Mesh<R, VertC, ()>,
    pub floor: UberMesh<R>,
    pub slider_control: UberMesh<R>,
    pub slider_frame: UberMesh<R>,

    // Rays
    pub red_ray: Mesh<R, VertC, ()>,
    pub blue_ray: Mesh<R, VertC, ()>,
}

pub struct Painters<R: gfx::Resources> {
    pub solid: Painter<R, SolidStyle<R>>,
    pub uber: Painter<R, UberStyle<R>>,
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
    pub active_apps: HashMap<String, bool>,
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

pub fn simple_material<R, F>(
    f: &mut F,
    albedo: [f32; 3],
    metalness: f32,
    roughness: f32,
    flatness: f32,
)
    -> Result<UberMaterial<R>, Error>
    where R: gfx::Resources, F: gfx::Factory<R>
{
    fn f2unorm(v: f32) -> u8 {
        (v * 256.).round().min(255.).max(0.) as u8
    }

    let albedo = [f2unorm(albedo[0]), f2unorm(albedo[1]), f2unorm(albedo[2]), 255];
    let knobs = [f2unorm(metalness), f2unorm(roughness), f2unorm(flatness), 0];
    use gfx::format::*;
    Ok(UberMaterial {
        albedo: Texture::<_, (R8_G8_B8_A8, Srgb)>::uniform_value(f, albedo)?,
        normal: Texture::<_, (R8_G8_B8_A8, Unorm)>::uniform_value(f, [0x80, 0x80, 0xFF, 0xFF])?,
        knobs: Texture::<_, (R8_G8_B8_A8, Unorm)>::uniform_value(f, knobs)?,
    })
}

pub fn open_simple_object<P, R, F>(
    f: &mut F,
    path: P,
    albedo: [f32; 3],
    metalness: f32,
    roughness: f32,
    flatness: f32,
)
    -> Result<UberMesh<R>, Error>
    where P: AsRef<Path>, R: gfx::Resources, F: gfx::Factory<R>
{
    Ok(load::open_wavefront(path)?.compute_tan().with_material(
        simple_material(f, albedo, metalness, roughness, flatness)?
    ).upload(f))
}

pub fn open_object_directory<P, R, F>(f: &mut F, path: P)
    -> Result<UberMesh<R>, Error>
    where P: AsRef<Path>, R: gfx::Resources, F: gfx::Factory<R>
{
    let path = path.as_ref();
    load::open_uber_mesh(
        f, 
        path.join("model.obj"),
        path.join("albedo.png"),
        path.join("normal.png"),
        path.join("knobs.png"))
}

impl<R: gfx::Resources> Meshes<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Meshes<R>, Error> {
        Ok(Meshes {
            controller: open_simple_object(
                factory,
                "assets/controller.obj",
                [0.6, 0.6, 0.6],
                1.,
                0.2,
                0.)?,
            wire_box: grid_lines(1, Vector3::new(1., 1., 1.)).upload(factory),
            floor: plane(2.5)
                .with_tex(Point2::new(0., 0.))
                .compute_tan()
                .with_material(simple_material(factory, [0.8, 0.8, 0.8], 0., 0.6, 0.)?)
                .upload(factory),
            slider_control: open_simple_object(
                factory,
                "assets/slider/control.obj",
                [0.6, 0.6, 0.6],
                0.,
                0.2,
                0.)?,
            slider_frame: open_simple_object(
                factory,
                "assets/slider/frame.obj",
                [0.6, 0.6, 0.6],
                0.,
                0.2,
                0.)?,
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

        let mut uber: Painter<_, UberStyle<_>> = Painter::new(factory)?;
        uber.setup(factory, Primitive::TriangleList)?;

        let mut unishade = Painter::new(factory)?;
        unishade.setup(factory, Primitive::TriangleList)?;

        Ok(Painters {
            solid: solid,
            uber: uber,
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
