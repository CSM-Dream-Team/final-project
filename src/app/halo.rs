use std::boxed::FnBox;

use nalgebra::{self as na, Vector3, Similarity3, Isometry3, Translation3, UnitQuaternion};
use ncollide::shape::{Plane, Cylinder};
use nphysics3d::object::RigidBody;

// Flight
use flight::{PbrMesh, Error, load};

// GFX
use gfx;
use app::App;

use common::{Common, CommonReply};

pub struct Halo<R: gfx::Resources> {
    halo_mesh: PbrMesh<R>,
}

impl<R: gfx::Resources> Halo<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Self, Error> {
        Ok(Halo {
            halo_mesh: load::object_directory(factory, "assets/halo/")?,
        })
    }
}

impl<R: gfx::Resources, C: gfx::CommandBuffer<R>> App<R, C> for Halo<R> {
    fn update<'a>(&'a mut self, common: &mut Common<R, C>) -> Box<FnBox(&mut CommonReply<R, C>) + 'a>
    {
        // Draw controllers
        for cont in &[&common.gurus.interact.primary, &common.gurus.interact.secondary] {
            common.painters.pbr.draw(&mut common.draw_params, na::convert(cont.data.pose), &common.meshes.controller);
        }

        // Draw floor
        let floor = Plane::new(Vector3::y());
        common.gurus.interact.primary.laser(&na::one(), &floor);
        let mut floor_rb = RigidBody::new_static(floor, 0.1, 0.6);
        floor_rb.set_margin(0.00001);
        common.gurus.physics.body(floor_rb);
        common.painters.pbr.draw(&mut common.draw_params, na::one(), &common.meshes.floor);

        // Draw torus
        let torus = Cylinder::new(0.02, 0.5);
        let torus = common.gurus.interact.primary.pointing_laser(
            &Isometry3::from_parts(Translation3::new(0., 3., 0.), na::one()),
            &torus,
            true,
        );

        Box::new(move |r: &mut CommonReply<_, _>| {
            let _torus = torus(&r.reply.interact);
            r.painters.pbr.draw(&mut r.draw_params, na::convert(
                Similarity3::from_parts(
                    Translation3::new(0., 2.5, 0.),
                    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.),
                    0.5
                )
            ), &self.halo_mesh);
        })
    }
}
