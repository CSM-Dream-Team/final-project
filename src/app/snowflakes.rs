use std::boxed::FnBox;

use nalgebra::{self as na, Vector3};
use ncollide::shape::Cuboid;
use nphysics3d::object::RigidBody;

// Flight
use flight::{PbrMesh, Error, load};

// GFX
use gfx;
use app::App;

use common::{Common, CommonReply};
use common::gurus::interact::GrabableState;

pub struct Snowblock {
    body: RigidBody<f32>,
    grabbed: GrabableState,
}

pub struct Snowflakes<R: gfx::Resources> {
    blocks: Vec<Snowblock>,
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
    fn update<'a>(
        &'a mut self,
        common: &mut Common<R, C>)
        -> Box<FnBox(&mut CommonReply<R, C>) + 'a>
    {
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
                self.blocks.push(Snowblock {
                    body: block,
                    grabbed: GrabableState::new(),
                });
            }
        }

        let physics = &mut common.gurus.physics;
        let interact = &mut common.gurus.interact;
        let futures: Vec<_> = self.blocks
            .iter_mut()
            .map(|s| {(
                physics.body(s.body.clone()),
                s.grabbed.update(interact, s.body.position(), s.body.shape().as_ref()),
                s,
            )})
            .collect();

        let snow_block = &self.snow_block;
        Box::new(move |r: &mut CommonReply<_, _>| for (phys_f, grabbed_f, block) in futures {
            grabbed_f(&r.reply.interact);

            let pos = match block.grabbed.offset {
                Some(o) => {
                    let position = r.reply.interact.primary.data.pose * o;
                    block.body.set_transformation(position);
                    position
                }
                None => *block.body.position(),
            };

            let body = phys_f(&r.reply.physics);
            block.body = body;
            r.painters.pbr.draw(&mut r.draw_params, na::convert(pos), snow_block);
        })
    }
}
