use ncollide::query::{PointQuery, RayCast, RayIntersection, Ray};
use nalgebra::{Point3, Vector3, Isometry3};
use flight::vr::{Trackable, MappedController};
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
                data: MappedController {
                    .. *primary
                },
                pointed_queries: BinaryHeap::new(),
                pointed_data: vec![None],
                blocked: false,
                laser_toi: INFINITY,
                index: 0,
            },
            secondary: ControllerGuru {
                data: MappedController {
                    .. *secondary
                },
                pointed_queries: BinaryHeap::new(),
                pointed_data: vec![None],
                blocked: false,
                laser_toi: INFINITY,
                index: 1,
            },
        }
    }
    
    // Complete this guru's calculations, enabling it to answer all waiting
    // questions.
    pub fn finish(self) -> InteractionReply {
        InteractionReply {
            primary: self.primary.finish(),
            secondary: self.secondary.finish(),
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
    blocked: bool,
    index: usize,
}

impl ControllerGuru {
    /// Update the endpoint of the controller's visual laser line by providing a
    /// terminating surface.
    pub fn laser<S: RayCast<Point3<f32>, Isometry3<f32>>>(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &S,
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

    fn pointing_partial(
        &mut self, 
        hit: Option<RayHit>, 
        stops: bool
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
        let controller_index = self.index;
        move |reply| ([&reply.primary, &reply.secondary][controller_index])
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
    pub fn pointing<S: RayCast<Point3<f32>, Isometry3<f32>>>(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &S,
        stops: bool,
    )
        -> impl FnOnce(&InteractionReply)
        -> Option<&RayHit>
    {
        let hit = if !self.blocked {
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
    pub fn pointing_laser<S: RayCast<Point3<f32>, Isometry3<f32>>>(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &S,
        stops: bool,
    )
        -> impl FnOnce(&InteractionReply)
        -> Option<&RayHit>
    {
        let ray = Ray::new(self.data.origin(), self.data.pointing());
        let hit = if !self.blocked {
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
        self.blocked = true;
        self.pointed_queries.clear();
        self.pointed_data.clear();
    }

    /// Check if the controller is touching the given shape.
    pub fn touched<S: PointQuery<Point3<f32>, Isometry3<f32>>>(
        &mut self,
        pos: &Isometry3<f32>,
        shape: &S,
    )
        -> bool
    {
        shape.contains_point(pos, &self.data.origin())
    }

    /// Complete this guru's calculations, enabling it to answer all waiting
    /// questions.
    pub fn finish(mut self) -> ControllerReply {
        while let Some(InteractQuery { stop: false, .. }) = self.pointed_queries.pop() {}
        for q in self.pointed_queries {
            self.pointed_data[q.i] = None;
        }
        ControllerReply {
            results: self.pointed_data,
            laser_toi: self.laser_toi,
            data: self.data,
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
}
