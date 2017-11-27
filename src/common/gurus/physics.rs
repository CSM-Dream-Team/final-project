use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nalgebra::{Vector3, Isometry3};
use std::rc::Rc;
use std::cell::RefCell;

pub struct PhysicsGuru {
    world: World<f32>,
}

impl PhysicsGuru {
    pub fn new(gravity: Vector3<f32>) -> PhysicsGuru {
        let mut world = World::new();
        world.set_gravity(gravity);
        PhysicsGuru {
            world: world,
        }
    }

    pub fn body(&mut self, body: RigidBody<f32>)
        -> impl FnOnce(&PhysicsReply)
        -> RigidBody<f32>
    {
        let body = self.world.add_rigid_body(body);
        move |_| match Rc::try_unwrap(body) {
            Ok(b) => b.into_inner(),
            Err(_) => panic!("Guru has not been resolved."),
        }
    }

    pub fn resolve(mut self, dt: f32) -> PhysicsReply {
        self.world.step(dt);
        PhysicsReply { _me: (), }
    }
}

pub struct PhysicsReply {
    _me: (),
}
