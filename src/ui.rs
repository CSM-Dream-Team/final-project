use flight::vr::Trackable;
use interact::{ControllerGuru, GrabableState};
use common::CommonReply;
use nalgebra::{Matrix4, Isometry3, Vector3, Translation3, Transform3};
use ncollide::shape::{Cuboid};
use gfx;

#[derive(Copy, Clone)]
enum SliderMode {
    Unheld,
    Moving,
    Sliding,
}

pub struct Slider {
    pub value: f32,
    pub position: Isometry3<f32>,
    pub thickness: f32,
    pub length: f32,
    grab: GrabableState,
    mode: SliderMode,
}

impl Slider {
    pub fn new(position: Isometry3<f32>, thickness: f32, length: f32, value: f32) -> Self {
        Slider {
            value: value,
            position,
            thickness,
            length,
            grab: Default::default(),
            mode: SliderMode::Unheld,
        }
    }

    pub fn update<'a, R, C>(
        &'a mut self,
        controller: &mut ControllerGuru,
    )
        -> impl FnOnce(&mut CommonReply<R, C>)
        -> f32 + 'a
        where R: gfx::Resources, C: gfx::CommandBuffer<R>
    {
        use self::SliderMode::*;
        use interact::GrabableState::*;

        let con = controller.controller_reply();
        let rad = self.thickness / 2.;
        let scaled = Vector3::new(rad, rad, self.length / 2.);
        let grab = self.grab.update(
            controller,
            self.position,
            &Cuboid::new(scaled)
        );
        let inv = self.position.inverse();

        move |reply| {
            self.grab = grab(&reply.reply.interact);
            let con = con(&reply.reply.interact);

            let next_pos = (inv * con.data.origin())[2];

            let slider_r_frac = 0.5 / 10.;
            let cap = 0.3 / 10. + slider_r_frac;
            let true_len = self.length * (1. - 2. * cap);
            let slider_r = slider_r_frac * self.length;

            let next_val = ((next_pos / true_len) + 0.5).max(0.).min(1.);
            let current_pos = (self.value - 0.5) * true_len;

            let scaling_transform = Transform3::from_matrix_unchecked(Matrix4::from_scaled_axis(scaled));
            reply.painters.pbr.draw(
                &mut reply.draw_params,
                self.position * scaling_transform,
                &reply.meshes.slider_frame,
            );
            reply.painters.pbr.draw(
                &mut reply.draw_params,
                self.position * Translation3::new(0., 0., current_pos) * scaling_transform,
                &reply.meshes.slider_frame,
            );

            self.mode = match (self.mode, &self.grab) {
                (Unheld, &Held { .. }) => {
                    if (current_pos - next_pos).abs() < slider_r {
                        Sliding
                    } else {
                        Moving
                    }
                },
                (Moving, &Held { offset, .. }) => {
                    self.position = con.data.pose() * offset;
                    Moving
                },
                (Sliding, &Held { .. }) => {
                    self.value = next_val;
                    Sliding
                },
                (_, &Free) | (_,  &Pointed) => Unheld,
            };
            self.value
        }
    }
}
