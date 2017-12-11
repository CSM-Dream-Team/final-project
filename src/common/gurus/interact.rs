use nalgebra::{Point3, Vector3, Isometry3, Translation3};
use ncollide::shape::Shape;
use ncollide::query::{PointQuery, RayCast, RayIntersection, Ray};
use nphysics3d::object::RigidBody;
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
    pub dt: f64,
}

impl InteractGuru {
    /// Create a new `InteractGuru` that checkes aganst the given controllers.
    pub fn new(primary: &MappedController, secondary: &MappedController, dt: f64) -> InteractGuru {
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
            dt,
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

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct ControllerIndex(u8);

impl ControllerIndex {
    pub fn primary() -> Self { ControllerIndex(0) }
    pub fn secondary() -> Self { ControllerIndex(1) }

    pub fn guru(self, guru: &mut InteractGuru) -> &mut ControllerGuru {
        [&mut guru.primary, &mut guru.secondary][self.0 as usize]
    }

    pub fn reply(self, reply: &InteractionReply) -> &ControllerReply {
        [&reply.primary, &reply.secondary][self.0 as usize]
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
    index: u8,
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
        if toi >= 0. { self.laser_toi = self.laser_toi.min(toi); }
    }

    /// Use to get reply object
    pub fn index(&self) -> ControllerIndex {
        ControllerIndex(self.index)
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
        let ind = self.index();
        move |reply| ind.reply(reply)
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
        let ind = self.index();
        move |reply| touched && ind.reply(reply).can_touch
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

pub const YANK_DIFFICULTY: f32 = 1.0;

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum MoveableIntention {
    Move,
    Manipulate,
    Free,
}

#[derive(Debug, Clone)]
pub enum Moveable {
    Grabbed {
        index: ControllerIndex,
    },
    Yanked {
        progress: f32,
        index: ControllerIndex,
    },
    Free,

}

impl Default for Moveable {
    fn default() -> Self {
        Moveable::Free
    }
}

#[derive(Debug, Clone)]
pub struct MoveData {
    pub intent: MoveableIntention,
    pub fixed: Option<Fixed>,
}

#[derive(Debug, Clone)]
pub struct Fixed {
    pub by: ControllerIndex,
    pub pos: Isometry3<f32>,
    pub inv_offset: Isometry3<f32>,
    pub lin_vel: Vector3<f32>,
    pub ang_vel: Vector3<f32>,
}

impl Fixed {
    pub fn location(&self) -> Point3<f32> {
        self.pos * Point3::origin()
    }
}

impl Moveable {
    pub fn update<'a>(
        &'a mut self,
        interact: &mut InteractGuru,
        pos: Isometry3<f32>,
        shape: &Shape<Point3<f32>, Isometry3<f32>>,
        inv_yank_offset: Isometry3<f32>,
        yank_speed: f32,
    )
        -> impl FnOnce(&InteractionReply)
        -> MoveData + 'a
    {
        use self::Moveable::*;
        use self::MoveableIntention as Mi;

        let solid = match self {
            &mut Free => true,
            &mut Yanked { index, ..} | &mut Grabbed { index, .. } => {
                index.guru(interact).block();
                false
            },
        };
        let cons: Vec<_> = (&[ControllerIndex::primary(), ControllerIndex::secondary()])
            .into_iter()
            .map(|&con| {
                let guru = con.guru(interact);
                (con, guru.pointing_laser(&pos, shape, solid), guru.touched(&pos, shape))
            }).collect();

        let d_yank = interact.dt as f32 / yank_speed;

        move |reply| {
            match self {
                &mut Free => {
                    for (ind, pointed, touched) in cons
                        .into_iter()
                        .map(|(i, p, t)| (i, p(reply), t(reply)))
                    {
                        let con = ind.reply(reply);
                        if let (Some(_), true) = (pointed, con.data.menu) {
                            *self = Yanked {
                                index: ind,
                                progress: 0.,
                            };
                            break
                        }
                        if con.data.trigger > 0.5 {
                            if touched {
                                *self = Grabbed {
                                    index: ind,
                                };
                                break
                            }
                        }
                    }
                },
                &mut Yanked { progress, index } => {
                    if progress + d_yank > 1. && !index.reply(reply).data.menu {
                        *self = Free;
                    } else {
                        *self = Yanked {
                            progress: (progress + d_yank).min(1.),
                            index: index,
                        };
                    }
                },
                &mut Grabbed { index, .. } => {
                    if index.reply(reply).data.trigger < 0.5 {
                        *self = Free;
                    }
                },
            };
            match self {
                &mut Grabbed { index } => {
                    let con = index.reply(reply);
                    let inv_offset = con.data.pose.inverse() * pos; // TODO: Do we have to invert?
                    let ang_vel = con.data.ang_vel;
                    let mut lin_vel = con.data.lin_vel;
                    lin_vel += ang_vel.cross(&inv_offset.translation.vector);

                    return MoveData {
                        intent: Mi::Manipulate,
                        fixed: Some(Fixed {
                            by: index,
                            pos: con.data.pose_delta * pos,
                            inv_offset: inv_offset,
                            lin_vel: lin_vel,
                            ang_vel: ang_vel,
                        }),
                    };
                },
                &mut Yanked { progress, index, .. } => {
                    let con = index.reply(reply);

                    let (next, lin_vel, ang_vel) = if progress < 1. {
                        let mut dp = d_yank / (1. - progress + d_yank);
                        dp = dp.min(1.).max(0.);
                        let dest = con.data.pose() * inv_yank_offset;
                        let next = Isometry3::from_parts(
                            Translation3::from_vector(
                                pos.translation.vector * (1. - dp) + dest.translation.vector * dp
                            ),
                            pos.rotation.slerp(&dest.rotation, dp),
                        );
                        let lin_vel = (next.translation.vector - pos.translation.vector) / con.data.dt as f32;
                        let ang_vel = (next.rotation * pos.rotation.inverse()).scaled_axis() / con.data.dt as f32;
                        (next, lin_vel, ang_vel)
                    } else {
                        let ang_vel = con.data.ang_vel;
                        let mut lin_vel = con.data.lin_vel;
                        lin_vel += ang_vel.cross(&inv_yank_offset.translation.vector);
                        (con.data.pose() * inv_yank_offset, lin_vel, ang_vel)
                    };

                    return MoveData {
                        intent: Mi::Move,
                        fixed: Some(Fixed {
                            by: index,
                            pos: next,
                            inv_offset: con.data.pose.inverse() * pos,
                            lin_vel: lin_vel,
                            ang_vel: ang_vel,
                        }),
                    };
                },
                _ => (),
            }
            MoveData {
                intent: Mi::Free,
                fixed: None,
            }
        }
    }
}

/// Represents something being grabbed
#[derive(Clone)]
#[deprecated(note="Please use 'Moveable'")]
pub enum GrabableState {
    Held {
        offset: Isometry3<f32>,
        lin_vel: Vector3<f32>,
        ang_vel: Vector3<f32>,
    },
    Pointed,
    Free,
}

#[allow(deprecated)]
impl Default for GrabableState {
    fn default() -> Self {
        GrabableState::Free
    }
}

#[allow(deprecated)]
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
        let ind = interact.index();

        move |reply| {
            let con = ind.reply(reply);
            let ang_vel = con.data.ang_vel;
            let mut lin_vel = con.data.lin_vel;
            if let Some(offset) = persist {
                lin_vel += ang_vel.cross(&offset.translation.vector);
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
    pub mov: Moveable,
    pub body: RigidBody<f32>,
}

impl GrabbablePhysicsState {
    pub fn new_free(body: RigidBody<f32>) -> Self {
        GrabbablePhysicsState { mov: Default::default(), body }
    }

    pub fn update<'a, R: gfx::Resources, C: gfx::CommandBuffer<R>>(
        &'a mut self,
        interact: &mut InteractGuru,
        physics: &mut PhysicsGuru,
        inv_yank_offset: Isometry3<f32>,
        yank_speed: f32,
    )
        -> impl FnOnce(&mut CommonReply<R, C>)
        -> Isometry3<f32> + 'a
    {
        let phys = physics.body(self.body.clone());
        let mov = self.mov.update(
            interact,
            *self.body.position(),
            self.body.shape().as_ref(),
            inv_yank_offset,
            yank_speed,
        );

        let body = &mut self.body;
        move |reply| {
            let mov_data = mov(&reply.reply.interact);
            *body = phys(&reply.reply.physics);
            match mov_data.fixed {
                Some(Fixed { pos, lin_vel, ang_vel, .. }) => {
                    body.set_transformation(pos);
                    body.set_lin_vel(lin_vel);
                    body.set_ang_vel(ang_vel);
                    pos
                }
                None => *body.position(),
            }
        }
    }
}
