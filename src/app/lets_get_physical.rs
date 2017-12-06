use std::boxed::FnBox;

use nalgebra::{self as na, Vector3, Isometry3, Translation3, Similarity3, UnitQuaternion};
use ncollide::shape::{ShapeHandle, Compound, Cuboid, Cylinder};
use nphysics3d::object::RigidBody;

// Flight
use flight::{PbrMesh, Error, load};

// GFX
use gfx;
use app::App;

use common::{Common, CommonReply};
use common::gurus::interact::{GrabbablePhysicsState};

pub struct LetsGetPhysical<R: gfx::Resources> {
    mjolnir: PbrMesh<R>,
    grabbable_state: GrabbablePhysicsState,
}

impl<R: gfx::Resources> LetsGetPhysical<R> {
    pub fn new<F: gfx::Factory<R>>(factory: &mut F) -> Result<Self, Error> {
        let shapes = vec! [
            (
                Isometry3::new(Vector3::new(0.,-3.,0.) * 0.08, na::zero()),
                ShapeHandle::new(Cuboid::new(Vector3::new(2.0, 1.5 , 1.5) * 0.08))
            ),
            (
                Isometry3::new(Vector3::new(0.,1.25,0.) * 0.08, na::zero()),
                ShapeHandle::new(Cylinder::new(3.25 * 0.08, 0.5 * 0.08))
            ),
        ];
        let compound = Compound::new(shapes);
        let mut mjolnir_body = RigidBody::new_dynamic(compound, 2330., 0.35, 0.47);
        mjolnir_body.set_transformation(Isometry3::new(Vector3::new(0., 0.5, 0.), na::zero()));

        Ok(LetsGetPhysical {
            mjolnir: load::object_directory(factory, "assets/hammer/")?,
            grabbable_state: GrabbablePhysicsState::new_free(mjolnir_body),
        })
    }
}

impl<R: gfx::Resources + 'static, C: gfx::CommandBuffer<R> + 'static> App<R, C> for LetsGetPhysical<R> {
    fn update<'a>(
        &'a mut self,
        common: &mut Common<R, C>)
        -> Box<FnBox(&mut CommonReply<R, C>) + 'a>
    {
        let gp = self.grabbable_state.update(
            &mut common.gurus.interact.primary,
            &mut common.gurus.physics,
        );

        let mjolnir = &self.mjolnir;
        Box::new(move |r: &mut CommonReply<R, C>| {
            let pos = gp(r);
            r.painters.pbr.draw(&mut r.draw_params, na::convert(pos), mjolnir);
        })
    }
}
