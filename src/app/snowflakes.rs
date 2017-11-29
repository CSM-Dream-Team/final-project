use std::boxed::FnBox;

use nalgebra::{self as na, Vector3};
use ncollide::shape::{Cuboid};
use nphysics3d::object::{RigidBody};

// Flight
use flight::{PbrMesh, Error, load};

// GFX
use gfx;
use app::App;

use common::{Common, CommonReply};

pub struct Snowflakes<R: gfx::Resources> {
    blocks: Vec<RigidBody<f32>>,
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
            let block = Cuboid::new(Vector3::new(0.15, 0.15, 0.3));
            let mut block = RigidBody::new_dynamic(block, 100., 0.0, 0.8);
            block.set_margin(0.00001);
            block.set_transformation(p.pose);
            block.set_lin_vel(p.lin_vel);
            block.set_ang_vel(p.ang_vel);
            self.blocks.push(block);
        }

        let physics = &mut common.gurus.physics;
        let futures: Vec<_> = self.blocks
            .iter_mut()
            .map(|b| {

                (physics.body(b.clone()), b)
            })
            .collect();

        let snow_block = &self.snow_block;
        Box::new(move |r: &mut CommonReply<_, _>| for (f, block) in futures {
            let body = f(&r.reply.physics);
            *block = body;
            r.painters.pbr.draw(&mut r.draw_params, na::convert(*block.position()), snow_block);
        })
    }
}
