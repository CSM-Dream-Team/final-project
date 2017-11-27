use std::boxed::FnBox;

use nalgebra::{self as na, Isometry3, Vector3};
use ncollide::shape::{Cuboid, Plane};
use nphysics3d::object::{RigidBody};

// Flight
use flight::{PbrMesh, Error, load};

// GFX
use gfx;
use app::App;

use common::{Common, CommonReply};

pub struct Snowflakes<R: gfx::Resources> {
    blocks: Vec<Isometry3<f32>>,
    snowman: PbrMesh<R>,
    snow_block: PbrMesh<R>,
}


impl<R: gfx::Resources> Snowflakes<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Self, Error> {
        Ok(Snowflakes {
            blocks: Vec::new(),
            snowman: load::object_directory(factory, "assets/snowman/")?,
            snow_block: load::object_directory(factory, "assets/snow-block/")?,
        })
    }
}

impl<R: gfx::Resources, C: gfx::CommandBuffer<R>> App<R, C> for Snowflakes<R> {
    fn update<'a>(&'a mut self, common: &mut Common<R, C>) -> Box<FnBox(&mut CommonReply<R, C>) + 'a>
    {
        // Add blocks
        let p = &common.gurus.interact.primary.data;
        if p.trigger > 0.5 && p.trigger - p.trigger_delta < 0.5 {
            self.blocks.push(p.pose);
        }

        let physics = &mut common.gurus.physics;

        let floor = Plane::new(Vector3::new(0., 1., 0.));
        let mut floor_rb = RigidBody::new_static(floor, 0.1, 0.6);
        floor_rb.set_margin(0.00001);
        physics.body(floor_rb);

        let futures: Vec<_> = self.blocks
            .iter_mut()
            .map(|b| {
                let block = Cuboid::new(Vector3::new(0.15, 0.15, 0.3));
                let mut block = RigidBody::new_dynamic(block, 100., 0.0, 0.8);
                block.set_margin(0.00001);
                block.set_transformation(*b);
                (physics.body(block), b)
            })
            .collect();

        let snow_block = &self.snow_block;
        Box::new(move |r: &mut CommonReply<_, _>| for (f, block) in futures {
            let body = f(&r.reply.physics);
            *block = *body.position();
            r.painters.pbr.draw(&mut r.draw_params, na::convert(*block), snow_block);
        })
    }
}
