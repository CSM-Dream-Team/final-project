use std::io::{Read, Write};
use std::boxed::FnBox;

use serde::{self, Serialize, Deserialize};
use serde_json::{Deserializer, Serializer, Error as JsonError};
use serde_json::de::IoRead as JsonRead;

use nalgebra::{self as na, Vector3, Isometry3, Translation3};
use ncollide::shape::{ShapeHandle, Compound, Cuboid, Ball};
use nphysics3d::object::RigidBody;

// Flight
use flight::{PbrMesh, Error, load};

// GFX
use gfx;
use app::App;

use common::{Common, CommonReply, Meta};
use common::gurus::interact::{GrabableState, GrabbablePhysicsState};

pub struct Snowblock(GrabbablePhysicsState);

impl Snowblock {
    fn update<'a, R: gfx::Resources, C: gfx::CommandBuffer<R> + 'static>
        (&'a mut self,
         common: &mut Common<R, C>)
         -> impl FnOnce(&mut CommonReply<R, C>, &PbrMesh<R>) + 'a {
        let gp = self.0.update(&mut common.gurus.interact,
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
    remove_blocks: Vec<usize>,
    snowman: PbrMesh<R>,
    snow_block: PbrMesh<R>,
}

impl<R: gfx::Resources> Snowflakes<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Self, Error> {
        Ok(Snowflakes {
            blocks: Vec::new(),
            new_blocks: Vec::new(),
            remove_blocks: Vec::new(),
            snowman: load::object_directory(factory, "assets/snowman/")?,
            snow_block: load::object_directory(factory, "assets/snow-block/")?,
        })
    }
}

impl<R: gfx::Resources + 'static, C: gfx::CommandBuffer<R> + 'static, W: Write, Re: Read> App<R, C, W, Re>
    for Snowflakes<R> {
    fn se_state(&self,
                serializer: &mut Serializer<W>, _: &mut Meta)
                -> Result<<&mut Serializer<W> as serde::Serializer>::Ok, JsonError> {
        let block_locations = self.blocks.iter().map(|b| *b.0.body.position()).collect();
        let state = SnowflakeState { block_locations: block_locations };
        state.serialize(serializer)
    }

    fn de_state(&mut self, deserializer: &mut Deserializer<JsonRead<Re>>, _: &mut Meta) -> Result<(), JsonError> {
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
        // Add the old blocks
        self.blocks.append(&mut self.new_blocks);

        {
            // Remove blocks that have been thrown off the platform.
            for remove in self.remove_blocks.iter() {
                self.blocks.remove(*remove);
            }
            self.remove_blocks.clear();
        }

        // Snowmen
        let snowman_shapes = vec![
            (Isometry3::new(Vector3::new(0., 0.22, 0.), na::zero()), ShapeHandle::new(Ball::new(0.26))),
            (Isometry3::new(Vector3::new(0., 0.60, 0.), na::zero()), ShapeHandle::new(Ball::new(0.20))),
            (Isometry3::new(Vector3::new(0., 0.85, 0.), na::zero()), ShapeHandle::new(Ball::new(0.15))),
        ];
        let snowman_shape = Compound::new(snowman_shapes);
        let snowmen_locations = vec![Translation3::new(2., 0., 2.),
                                     Translation3::new(-2., 0., 2.),
                                     Translation3::new(-2., 0., -2.),
                                     Translation3::new(2., 0., -2.)];
        for loc in snowmen_locations {
            common.painters.pbr.draw(&mut common.draw_params, na::convert(loc), &self.snowman);
            let mut body = RigidBody::new_static(snowman_shape.clone(), 0.0, 0.8);
            body.set_translation(loc);
            common.gurus.physics.body(body);
        }

        // Setup blocks & futures
        let block_shape = Cuboid::new(Vector3::new(0.15, 0.15, 0.3));
        let add_future = GrabableState::default().update(&mut common.gurus.interact.primary,
                                                         common.gurus.interact.secondary.data.pose,
                                                         &block_shape);

        let remove_blocks = &mut self.remove_blocks;
        let futures: Vec<_> = self.blocks
            .iter_mut()
            .enumerate()
            .map(|(i, s)| {
                // TODO: this doesn't work with alternate gravity
                if s.0.body.position().translation.vector.y < -10. {
                    remove_blocks.push(i);
                }
                s.update(common)
            })
            .collect();

        // Render snow blocks
        let snow_block = &self.snow_block;
        let new_blocks = &mut self.new_blocks;
        Box::new(move |r: &mut CommonReply<R, C>| {
            use self::GrabableState::*;
            match add_future(&r.reply.interact) {
                Held { .. } => new_blocks.push({
                    let mut body = RigidBody::new_dynamic(block_shape, 100., 0.0, 0.8);
                    body.set_margin(0.00001);
                    Snowblock(GrabbablePhysicsState::new_free(body))
                }),
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
