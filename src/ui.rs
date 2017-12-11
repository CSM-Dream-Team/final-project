use interact::{InteractGuru, Moveable, MoveableIntention};
use common::CommonReply;
use nalgebra::{Matrix4, Vector4, Isometry3, Vector3, Translation3, Transform3, UnitQuaternion};
use ncollide::shape::{Cuboid};
use gfx;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
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
    pub manip_length: f32,
    mov: Moveable,
    mode: SliderMode,
}

impl Slider {
    pub fn new(position: Isometry3<f32>, thickness: f32, length: f32, manip_length: f32, value: f32) -> Self {
        Slider {
            value: value,
            position,
            thickness,
            length,
            manip_length,
            mov: Default::default(),
            mode: SliderMode::Unheld,
        }
    }

    pub fn update<'a, R, C>(
        &'a mut self,
        interact: &mut InteractGuru,
    )
        -> impl FnOnce(&mut CommonReply<R, C>)
        -> f32 + 'a
        where R: gfx::Resources, C: gfx::CommandBuffer<R>
    {
        use self::SliderMode::*;

        let scaled = Vector3::new(self.thickness / 2., self.thickness / 2., self.length / 2.);
        let mov = self.mov.update(
            interact,
            self.position,
            &Cuboid::new(scaled),
            Isometry3::from_parts(
                Translation3::new(0., 0., -self.thickness / 2.),
                UnitQuaternion::rotation_between(
                    &Vector3::new(0., 0., 1.),
                    &Vector3::new(0., 1., 0.),
                ).unwrap(),
            ),
        );


        let mode = &mut self.mode;
        let value = &mut self.value;
        let position = &mut self.position;
        let manip_length = self.manip_length;
        let length = self.length;
        let thickness = self.thickness;
        move |reply| {
            let mov_data = mov(&reply.reply.interact);

            let cap = (0.6 / 10.) * length;
            let true_len = length - cap - manip_length;

            if let Some(fix) = mov_data.fixed {
                if mov_data.intent == MoveableIntention::Manipulate {

                    let next_pos = fix.inv_offset.inverse().translation.vector[2];
                    let slider_r = manip_length / 2.;
                    let next_val = ((next_pos / true_len) + 0.5).max(0.).min(1.);
                    let current_pos = (*value - 0.5) * true_len;

                    match (*mode, (current_pos - next_pos).abs() < slider_r) {
                        (Unheld, true) => *mode = Sliding,
                        (Unheld, false) => *mode = Moving,
                        (Sliding, _) => *value = next_val,
                        (Moving, _) => *position = fix.pos,
                    }
                } else {
                    *mode = Moving;
                    *position = fix.pos;
                }
            } else {
                *mode = Unheld;
            }

            reply.painters.pbr.draw(
                &mut reply.draw_params,
                (*position) * Transform3::from_matrix_unchecked(
                    Matrix4::from_diagonal(&Vector4::new(thickness / 2., thickness / 2., length / 10., 1.))
                ),
                &reply.meshes.slider_frame,
            );
            let slider_pos = (*value - 0.5) * true_len;
            reply.painters.pbr.draw(
                &mut reply.draw_params,
                (*position) * Translation3::new(0., 0., slider_pos) * Transform3::from_matrix_unchecked(
                    Matrix4::from_diagonal(&Vector4::new(thickness / 2., thickness / 2., manip_length / 2., 1.))
                ),
                &reply.meshes.slider_control,
            );

            *value
        }
    }
}
