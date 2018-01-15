use std::collections::HashMap;
use std::io::{Read, Write};
use std::boxed::FnBox;

use serde::{self, Serialize, Deserialize};
use serde_json::{Deserializer, Serializer, Error as JsonError};
use serde_json::de::IoRead as JsonRead;

use nalgebra::{self as na, Vector3, Similarity3, Isometry3, Translation3, UnitQuaternion};
use ncollide::shape::{Cuboid, Cylinder};
use nphysics3d::object::RigidBody;

// Flight
use flight::{UberMesh, Error};

// GFX
use gfx;
use app::App;

use common::{open_object_directory, Common, CommonReply, Meta};
use interact::ControllerIndex;

pub struct Halo<R: gfx::Resources> {
    halo_mesh: UberMesh<R>,
}

#[derive(Serialize, Deserialize)]
pub struct HaloState {
    pub active_apps: HashMap<String, bool>,
}

impl<R: gfx::Resources> Halo<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Self, Error> {
        Ok(Halo { halo_mesh: open_object_directory(factory, "assets/halo/")? })
    }
}

impl<R: gfx::Resources + 'static, C: gfx::CommandBuffer<R> + 'static, W: Write, Re: Read> App<R, C, W, Re> for Halo<R> {
    fn se_state(&self, serializer: &mut Serializer<W>, meta: &mut Meta) -> Result<<&mut Serializer<W> as serde::Serializer>::Ok, JsonError> {
        let state = HaloState {
            active_apps: meta.active_apps.clone(),
        };
        state.serialize(serializer)
    }

    fn de_state(&mut self, deserializer: &mut Deserializer<JsonRead<Re>>, meta: &mut Meta) -> Result<(), JsonError> {
        let state = HaloState::deserialize(deserializer)?;
        meta.active_apps = state.active_apps;
        Ok(())
    }

    fn update<'b>(&'b mut self,
                  common: &mut Common<R, C>)
                  -> Box<FnBox(&mut CommonReply<R, C>) + 'b> {
        // Draw controllers
        for cont in &[&common.gurus.interact.primary, &common.gurus.interact.secondary] {
            common.painters.uber.draw(&mut common.draw_params,
                                     na::convert(cont.data.pose),
                                     &common.meshes.controller);
        }

        // Draw floor
        let floor_shape = Cuboid::new(Vector3::new(2.5, 1., 2.5));
        let floor_pos = Isometry3::from_parts(Translation3::new(0., -1., 0.), na::one());
        common.gurus.interact.primary.laser(&floor_pos, &floor_shape);
        common.gurus.interact.secondary.laser(&floor_pos, &floor_shape);

        let mut floor_rb = RigidBody::new_static(floor_shape, 0.1, 0.6);
        floor_rb.set_transformation(floor_pos);
        floor_rb.set_margin(0.00001);
        common.gurus.physics.body(floor_rb);
        common.painters.uber.draw(&mut common.draw_params, na::one(), &common.meshes.floor);

        // Draw torus
        let torus = Cylinder::new(0.02, 0.5);
        let torus = common.gurus
            .interact
            .primary
            .pointing_laser(&Isometry3::from_parts(Translation3::new(0., 3., 0.), na::one()),
                            &torus,
                            true);

        // Setup toggle futures
        let toggles = vec![
            ("lets_get_physical", Translation3::new(2., 1., 0.)),
            ("snowflakes", Translation3::new(0., 1., 2.)),
        ];
        let toggle_box_shape = Cuboid::new(Vector3::new(0.25, 0.25, 0.25));
        let toggle_futures: Vec<_> = toggles.into_iter().flat_map(|(app, trans)| {
            // Draw the toggles
            common.painters.solid.draw(&mut common.draw_params,
                                       na::convert(Similarity3::from_parts(trans, na::one(), 0.5)),
                                       &common.meshes.wire_box);

            // Return the future
            (&[ControllerIndex::primary(), ControllerIndex::secondary()]).into_iter().map(|&idx| {
                (app, idx,
                 idx.guru(&mut common.gurus.interact).pointing_laser(&Isometry3::from_parts(trans, na::one()),
                                                                     &toggle_box_shape, true))
            }).collect::<Vec<_>>()
        }).collect();

        Box::new(move |r: &mut CommonReply<_, _>| {
            let _torus = torus(&r.reply.interact);

            // Do the toggles
            for (app, i, f) in toggle_futures.into_iter() {
                let con = i.reply(&r.reply.interact);
                if f(&r.reply.interact).is_some() && con.data.trigger > 0.5 &&
                    con.data.trigger - con.data.trigger_delta < 0.5 {
                    match r.meta.active_apps.get_mut(app) {
                        Some(v) => {
                            *v = !*v;
                            *v
                        },
                        None => false,
                    };
                }
            }

            // Draw the halo
            r.painters.uber.draw(&mut r.draw_params, na::convert(
                Similarity3::from_parts(
                    Translation3::new(0., 2.5, 0.),
                    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.),
                    0.5
                )
            ), &self.halo_mesh);

            for con in &[&r.reply.interact.primary, &r.reply.interact.secondary] {
                r.painters.solid.draw(&mut r.draw_params, na::convert(
                    Similarity3::from_isometry(
                        con.data.pose,
                        con.laser_toi.max(0.01).min(10.),
                    )
                ), if con.laser_toi == ::std::f32::INFINITY { &r.meshes.blue_ray } else { &r.meshes.red_ray });

            }
        })
    }
}
