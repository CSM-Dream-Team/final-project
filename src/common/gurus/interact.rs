use nalgebra::{self as na, Matrix3, Point3, Vector3, Isometry3};
use ncollide::shape::Shape;
use ncollide::query::{PointQuery, RayCast, RayIntersection, Ray};
use nphysics3d::object::RigidBody;
use nphysics3d::volumetric::InertiaTensor;
use gfx;
use flight::vr::{Trackable, MappedController};
use common::{CommonReply};
use super::physics::PhysicsGuru;
use std::collections::BinaryHeap;
use std::cmp::{Ord, PartialOrd, PartialEq, Ordering};
use std::f32::INFINITY;

pub struct RayHit(pub RayIntersection<Vector3<f32>>);

/// Answers queries about user interactions.
pub struct InteractGuru {
    pub primary: ControllerGuru,
    pub secondary: ControllerGuru,
}

impl InteractGuru {
    /// Create a new `InteractGuru` that checkes aganst the given controllers.
    pub fn new(primary: &MappedController, secondary: &MappedController) -> InteractGuru {
        InteractGuru {
            primary: ControllerGuru {
                data: MappedController { ..*primary },
                pointed_queries: BinaryHeap::new(),
                pointed_data: vec![None],
                point_blocked: false,
                touch_blocked: false,
                laser_toi: INFINITY,
                index: 0,
            },
            secondary: ControllerGuru {
                data: MappedController { ..*secondary },
                pointed_queries: BinaryHeap::new(),
                pointed_data: vec![None],
                point_blocked: false,
                touch_blocked: false,
                laser_toi: INFINITY,
                index: 1,
            },
        }
    }

    // Complete this guru's calculations, enabling it to answer all waiting
    // questions.
    pub fn resolve(self) -> InteractionReply {
        InteractionReply {
            primary: self.primary.resolve(),
            secondary: self.secondary.resolve(),
        }
    }
}

struct InteractQuery {
    t: f32,
    i: usize,
    stop: bool,
}

impl PartialEq for InteractQuery {
    fn eq(&self, other: &InteractQuery) -> bool {
        self.i.eq(&other.i)
    }
}

impl Eq for InteractQuery {}

impl Ord for InteractQuery {
    fn cmp(&self, other: &InteractQuery) -> Ordering {
        self.partial_cmp(other).expect("TOI can't be NaN")
    }
}

impl PartialOrd for InteractQuery {
    fn partial_cmp(&self, other: &InteractQuery) -> Option<Ordering> {
        other.t.partial_cmp(&self.t)
    }
}

/// Answers queries about VR controller interactions.
pub struct ControllerGuru {
    /// The current state of the controller.
    pub data: MappedController,
    laser_toi: f32,
    pointed_queries: BinaryHeap<InteractQuery>,
    pointed_data: Vec<Option<RayHit>>,
    point_blocked: bool,
    touch_blocked: bool,
    index: usize,
}

impl ControllerGuru {
    /// Update the endpoint of the controller's visual laser line by providing a
    /// terminating surface.
    pub fn laser(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &Shape<Point3<f32>, Isometry3<f32>>,
    ) {
        let ray = Ray::new(self.data.origin(), self.data.pointing());
        if let Some(t) = shape.toi_with_ray(pos, &ray, true) {
            self.laser_toi(t);
        }
    }

    /// Update the length of controller's visual laser line.
    pub fn laser_toi(&mut self, toi: f32) {
        self.laser_toi = self.laser_toi.min(toi);
    }

    /// Will get the reply for this controller from the complete interaction reply
    pub fn controller_reply(&self) -> impl FnOnce(&InteractionReply) -> &ControllerReply {
        let controller_index = self.index;
        move |reply| [&reply.primary, &reply.secondary][controller_index]
    }

    fn pointing_partial(
        &mut self,
        hit: Option<RayHit>,
        stops: bool,
    )
        -> impl FnOnce(&InteractionReply)
        -> Option<&RayHit>
    {
        let mut index = 0;
        if let Some(hit) = hit {
            index = self.pointed_data.len();
            self.pointed_queries.push(InteractQuery {
                t: hit.0.toi,
                i: index,
                stop: stops,
            });
            self.pointed_data.push(Some(hit));
        }
        let con = self.controller_reply();
        move |reply| con(reply)
            .results
            .get(index)
            .map(|r| r.as_ref())
            .unwrap_or(None)
    }

    /// Check if the controller is pointing at the given shape. The shape can
    /// optionally block line of sight, stopping more distant interactions from
    /// triggering.
    ///
    /// Note that this function returns an in-progress answer which can only be
    /// completed once the `ControllerGuru` has finished.
    pub fn pointing(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &Shape<Point3<f32>, Isometry3<f32>>,
        stops: bool,
    )
        -> impl FnOnce(&InteractionReply)
        -> Option<&RayHit>
    {
        let hit = if !self.point_blocked {
            let ray = Ray::new(self.data.origin(), self.data.pointing());
            shape.toi_and_normal_with_ray(pos, &ray, true).map(|h| RayHit(h))
        } else {
            None
        };
        self.pointing_partial(hit, stops)
    }

    /// Check if the controller is pointing at the given shape and update the
    /// visual laser line to terminate on its surface. The shape can optionally
    /// block line of sight, stopping more distant interactions from triggering.
    ///
    /// Note that this function returns an in-progress answer which can only be
    /// completed once the `ControllerGuru` has finished.
    pub fn pointing_laser(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &Shape<Point3<f32>, Isometry3<f32>>,
        stops: bool,
    )
        -> impl FnOnce(&InteractionReply)
        -> Option<&RayHit>
    {
        let ray = Ray::new(self.data.origin(), self.data.pointing());
        let hit = if !self.point_blocked {
            let hit = shape.toi_and_normal_with_ray(pos, &ray, true);
            if let Some(ref hit) = hit {
                self.laser_toi(hit.toi);
            }
            hit.map(|h| RayHit(h))
        } else {
            if let Some(t) = shape.toi_with_ray(pos, &ray, true) {
                self.laser_toi(t);
            }
            None
        };
        self.pointing_partial(hit, stops)
    }

    /// Block the controller from pointing at anything. This will force all
    /// calls to `pointing` and`pointing_laser` (past and future) to return
    /// `None`.
    pub fn block_pointing(&mut self) {
        self.point_blocked = true;
        self.pointed_queries.clear();
        self.pointed_data.clear();
    }

    /// Blocks the controller from touching anything. This will force all calls
    /// to `touch` (past and future) to return `false`.
    pub fn block_touch(&mut self) {
        self.touch_blocked = true;
    }

    /// Blocks the controller from touching or pointing at anything.
    pub fn block(&mut self) {
        self.block_pointing();
        self.block_touch();
    }

    /// Check if the controller has not been blocked and is touching the given
    /// shape.
    pub fn touched(
        &self,
        pos: &Isometry3<f32>,
        shape: &Shape<Point3<f32>, Isometry3<f32>>,
    )
        -> impl FnOnce(&InteractionReply)
        -> bool
    {
        let touched = !self.touch_blocked && shape.contains_point(pos, &self.data.origin());
        let con = self.controller_reply();
        move |reply| touched && con(reply).can_touch
    }

    /// Complete this guru's calculations, enabling it to answer all waiting
    /// questions.
    pub fn resolve(mut self) -> ControllerReply {
        while let Some(InteractQuery { stop: false, .. }) = self.pointed_queries.pop() {}
        for q in self.pointed_queries {
            self.pointed_data[q.i] = None;
        }
        ControllerReply {
            results: self.pointed_data,
            laser_toi: self.laser_toi,
            data: self.data,
            can_touch: !self.touch_blocked,
        }
    }
}

/// Enables the completion of interaction questions.
pub struct InteractionReply {
    pub primary: ControllerReply,
    pub secondary: ControllerReply,
}

/// Enables the completion of controller interaction questions.
pub struct ControllerReply {
    results: Vec<Option<RayHit>>,
    /// The length of the visual laser line.
    pub laser_toi: f32,
    /// The current state of the controller.
    pub data: MappedController,
    can_touch: bool,
}

/// Represents something being grabbed
#[derive(Clone)]
pub enum GrabableState {
    Held {
        offset: Isometry3<f32>,
        lin_vel: Vector3<f32>,
        ang_vel: Vector3<f32>,
    },
    Pointed,
    Free,
}

impl Default for GrabableState {
    fn default() -> Self {
        GrabableState::Free
    }
}

impl GrabableState {
    pub fn update(
            &self,
            interact: &mut ControllerGuru,
            pos: Isometry3<f32>,
            shape: &Shape<Point3<f32>, Isometry3<f32>>)
            -> impl FnOnce(&InteractionReply)
            -> GrabableState
    {
        use self::GrabableState::*;

        let down = interact.data.trigger > 0.5;
        let pressed = down && interact.data.trigger - interact.data.trigger_delta < 0.5;
        let (persist, touched) = if let (&Held { offset, .. }, true) = (self, down) {
            (Some(offset), None)
        } else {
            (None, match (self, pressed) {
                (&Free, true) | (&Pointed, true) => Some(interact.touched(&pos, shape)),
                (&Held { .. }, _) => {
                    interact.block();
                    None
                }
                _ => None,
            })
        };
        let pointing = interact.pointing_laser(&pos, shape, true);
        let con = interact.controller_reply();

        move |reply| {
            let con = con(reply);
            let ang_vel = con.data.ang_vel;
            let mut lin_vel = con.data.lin_vel;
            if let Some(offset) = persist {
                let com = *self.body.center_of_mass();
                lin_vel += ang_vel.cross(&(con.data.origin() - com));
                Held { offset, lin_vel, ang_vel }
            } else if touched.map(|t| t(reply)).unwrap_or(false) {
                let offset = con.data.pose.inverse() * pos;
                lin_vel += ang_vel.cross(&offset.translation.vector);
                Held { offset, lin_vel, ang_vel }
            } else if pointing(reply).is_some() {
                Pointed
            } else {
                Free
            }
        }
    }
}

/// Represents something being grabbed
#[derive(Clone)]
pub struct GrabbablePhysicsState {
    pub grab: GrabableState,
    pub body: RigidBody<f32>,
}

fn displace_tensor(disp: Vector3<f32>, mass: f32, tensor: &mut Matrix3<f32>) {
    let d_sq = na::norm_squared(&disp);
    for i in 0..3 {
        for j in 0..3 {
            let rr = disp[i] * disp[j];
            tensor[(i, j)] += mass * (if i == j { d_sq } else { 0. } - rr)
        }
    }
}

impl GrabbablePhysicsState {
    fn ballerina_factor(&self, displace: Vector3<f32>, spin: Vector3<f32>) -> Vector3<f32> {
        if let Some(mass) = self.body.mass() {
            let inv_cm_tensor = self.body.inv_inertia();
            let cm_tensor = inv_cm_tensor.try_inverse().unwrap_or(na::zero());
            let dir = spin.normalize();
            let cm_inertia = dir.dot(&(cm_tensor * dir));
            let hand_inertia = cm_inertia +  mass * na::norm_squared(&dir.cross(&displace));
            spin * hand_inertia / cm_inertia
        } else {
            spin
        }
    }

    pub fn new_free(body: RigidBody<f32>) -> Self {
        GrabbablePhysicsState { grab: Default::default(), body }
    }

    pub fn new(mut body: RigidBody<f32>, grab: GrabableState, con: &MappedController) -> Self {
        if let GrabableState::Held { offset, lin_vel, ang_vel } = grab {
            body.set_transformation(con.pose * offset);
            body.set_lin_vel(lin_vel);
            body.set_ang_vel(ang_vel);
        }
        GrabbablePhysicsState { grab, body }
    }

    pub fn update<'a, R: gfx::Resources, C: gfx::CommandBuffer<R>>(
        &'a mut self,
        interact: &mut ControllerGuru,
        physics: &mut PhysicsGuru)
        -> impl FnOnce(&mut CommonReply<R, C>)
        -> Isometry3<f32> + 'a
    {
        let phys = physics.body(self.body.clone());
        let grab = self.grab.update(
            interact,
            *self.body.position(),
            self.body.shape().as_ref());
        let con = interact.controller_reply();

        move |reply| {
            let con = con(&reply.reply.interact);
            let next_grab = grab(&reply.reply.interact);
            self.body = phys(&reply.reply.physics);

            use self::GrabableState::*;
            let pos = match (&next_grab, &self.grab) {
                (&Held { offset, lin_vel, ang_vel }, _) => {
                    let position = con.data.pose * offset;
                    self.body.set_transformation(position);
                    self.body.set_lin_vel(lin_vel);
                    self.body.set_ang_vel(ang_vel);

                    position
                },
                (_, &Held { offset, ang_vel, .. }) => {
                    // Just released, effective moment of interia changes
                    let pos = *self.body.position();
                    let com = *self.body.center_of_mass();
                    let ang_vel = self.ballerina_factor(
                        con.data.origin() - com,
                        ang_vel,
                    );
                    self.body.set_ang_vel(ang_vel);

                    pos
                },
                _ => *self.body.position(),
            };
            self.grab = next_grab;
            pos
        }
    }
}
