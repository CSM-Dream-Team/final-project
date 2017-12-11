use std::io::{Read, Write};
use std::boxed::FnBox;

use serde::{self, Serialize, Deserialize};
use serde_json::{Deserializer, Serializer, Error as JsonError};
use serde_json::de::IoRead as JsonRead;

use nalgebra::{self as na, Vector3, Isometry3};
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
    fn update<'a, R: gfx::Resources, C: gfx::CommandBuffer<R> + 'static>
        (&'a mut self,
         common: &mut Common<R, C>)
         -> impl FnOnce(&mut CommonReply<R, C>, &PbrMesh<R>) + 'a {
        let gp = self.0.update(&mut common.gurus.interact.primary,
                               &mut common.gurus.physics);
        move |reply, mesh| {
            let pos = gp(reply);
            reply.painters.pbr.draw(&mut reply.draw_params, na::convert(pos), mesh);
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct SnowflakeState {
    block_locations: Vec<Isometry3<f32>>,
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

impl<R: gfx::Resources + 'static, C: gfx::CommandBuffer<R> + 'static, W: Write, Re: Read> App<R, C, W, Re>
    for Snowflakes<R> {
    fn se_state(&self,
                serializer: &mut Serializer<W>)
                -> Result<<&mut Serializer<W> as serde::Serializer>::Ok, JsonError> {
        let block_locations = self.blocks.iter().map(|b| *b.0.body.position()).collect();
        let state = SnowflakeState { block_locations: block_locations };
        state.serialize(serializer)
    }

    fn de_state(&mut self, deserializer: &mut Deserializer<JsonRead<Re>>) -> Result<(), JsonError> {
        // Clear all of the current state
        self.blocks.clear();

        // Read in the new block locations
        let state = SnowflakeState::deserialize(deserializer)?;
        self.new_blocks = state.block_locations.iter().map(|l| {
            let block_shape = Cuboid::new(Vector3::new(0.15, 0.15, 0.3));
            let mut body = RigidBody::new_dynamic(block_shape, 100., 0.0, 0.8);
            body.set_transformation(*l);
            Snowblock(GrabbablePhysicsState::new_free(body))
        }).collect();
        Ok(())
    }

    fn update<'b>(&'b mut self,
                  common: &mut Common<R, C>)
                  -> Box<FnBox(&mut CommonReply<R, C>) + 'b> {
        self.blocks.append(&mut self.new_blocks);

        let block_shape = Cuboid::new(Vector3::new(0.15, 0.15, 0.3));
        let add_future = GrabableState::default().update(&mut common.gurus.interact.primary,
                                                         common.gurus.interact.secondary.data.pose,
                                                         &block_shape);

        let futures: Vec<_> = self.blocks
            .iter_mut()
            .map(|s| s.update(common))
            .collect();

        let snow_block = &self.snow_block;
        let new_blocks = &mut self.new_blocks;
        Box::new(move |r: &mut CommonReply<R, C>| {
            use self::GrabableState::*;
            match add_future(&r.reply.interact) {
                g @ Held { .. } => {
                    new_blocks.push({
                        let mut body = RigidBody::new_dynamic(block_shape, 100., 0.0, 0.8);
                        body.set_margin(0.00001);
                        Snowblock(GrabbablePhysicsState::new(body,
                                                             g,
                                                             &r.reply.interact.primary.data))
                    })
                }
                _ => (),
            }
            for block in futures {
                block(r, snow_block);
            }
            r.painters.pbr.draw(&mut r.draw_params,
                                na::convert(r.reply.interact.secondary.data.pose),
                                snow_block);
        })
    }
}
