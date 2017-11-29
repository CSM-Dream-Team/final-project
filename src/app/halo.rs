use std::boxed::FnBox;

use nalgebra::{self as na, Vector3, Similarity3, Isometry3, Translation3};
use ncollide::shape::{Plane, Cylinder};
use nphysics3d::object::RigidBody;

// GFX
use gfx;
use app::App;

use common::{Common, CommonReply};

pub struct Halo {
}


impl Halo {
    pub fn new() -> Self {
        Halo { }
    }
}

impl<R: gfx::Resources, C: gfx::CommandBuffer<R>> App<R, C> for Halo {
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
            let color = if let Some(_) = torus(&mut r.reply.interact) {
                &r.meshes.red_ray
            } else {
                &r.meshes.blue_ray
            };

            let toi = r.reply.interact.primary.laser_toi.max(0.01).min(20.);
            r.painters.solid.draw(&mut r.draw_params, na::convert(
                Similarity3::from_isometry(r.reply.interact.primary.data.pose, toi)
            ), color);

            let toi = r.reply.interact.secondary.laser_toi.max(0.01).min(20.);
            r.painters.solid.draw(&mut r.draw_params, na::convert(
                Similarity3::from_isometry(r.reply.interact.secondary.data.pose, toi)
            ), &r.meshes.blue_ray);
        })
    }
}
