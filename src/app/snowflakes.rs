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
use common::gurus::interact::{GrabableState, GrabbablePhysicsState};

pub struct Snowblock(GrabbablePhysicsState);

impl Snowblock {
    fn update<'a, R: gfx::Resources, C: gfx::CommandBuffer<R> + 'static>(
        &'a mut self,
        common: &mut Common<R, C>)
        -> impl FnOnce(&mut CommonReply<R, C>, &PbrMesh<R>) + 'a
    {
        let gp = self.0.update(
            &mut common.gurus.interact.primary,
            &mut common.gurus.physics,
        );
        move |reply, mesh| {
            let pos = gp(reply);
            reply.painters.pbr.draw(&mut reply.draw_params, na::convert(pos), mesh);
        }
    }
}

pub struct Snowflakes<R: gfx::Resources> {
    blocks: Vec<Snowblock>,
    new_blocks: Vec<Snowblock>,
    snowman: PbrMesh<R>,
    snow_block: PbrMesh<R>,
}

impl<R: gfx::Resources> Snowflakes<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Self, Error> {
        Ok(Snowflakes {
            blocks: Vec::new(),
            new_blocks: Vec::new(),
            snowman: load::object_directory(factory, "assets/snowman/")?,
            snow_block: load::object_directory(factory, "assets/snow-block/")?,
        })
    }
}

impl<R: gfx::Resources + 'static, C: gfx::CommandBuffer<R> + 'static> App<R, C> for Snowflakes<R> {
    fn update<'a>(
        &'a mut self,
        common: &mut Common<R, C>)
        -> Box<FnBox(&mut CommonReply<R, C>) + 'a>
    {
        self.blocks.append(&mut self.new_blocks);

        let block_shape = Cuboid::new(Vector3::new(0.15, 0.15, 0.3));
        let add_future = GrabableState::default().update(
            &mut common.gurus.interact.primary,
            common.gurus.interact.secondary.data.pose,
            &block_shape,
        );

        let futures: Vec<_> = self.blocks
            .iter_mut()
            .map(|s| s.update(common))
            .collect();

        let snow_block = &self.snow_block;
        let new_blocks = &mut self.new_blocks;
        Box::new(move |r: &mut CommonReply<R, C>| {
            use self::GrabableState::*;
            match add_future(&r.reply.interact) {
                g @ Held { .. } => new_blocks.push({
                    let mut body = RigidBody::new_dynamic(block_shape, 100., 0.0, 0.8);
                    body.set_margin(0.00001);
                    Snowblock(GrabbablePhysicsState::new(body, g, &r.reply.interact.primary.data))
                }),
                _ => (),
            }
            for block in futures { block(r, snow_block); }
            r.painters.pbr.draw(&mut r.draw_params, na::convert(
                r.reply.interact.secondary.data.pose
            ), snow_block);
        })
    }
}
