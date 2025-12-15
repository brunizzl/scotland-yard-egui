use egui::{emath::RectTransform, *};

use crate::geo::{self, Pos3, Project3To2, ToScreen, Vec3};

const DEFAULT_AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

#[derive(serde::Deserialize, serde::Serialize, Clone)]
pub struct Camera3D {
    /// == 1.0 -> no change
    /// < 1.0  -> zoomed out
    /// > 1.0  -> zoomed in
    pub zoom: f32,
    zoom_speed: f32,

    direction: [Vec3; 3],

    /// offset of center independent of zoom + direction (e.g. in draw plane)
    /// note: this is in neighter coordinate system of `self.to_screen`, because
    /// it is used to add drag offset, which is a zoom-dependent translation.
    pub position: Pos2,

    to_screen: ToScreen,

    /// eighter lock all rotations (` == false`)
    /// or allow z-Axis (screen normal) rotations
    allow_rotation_2d: bool,
}

impl Default for Camera3D {
    fn default() -> Self {
        let default_rect = Rect::from_center_size(Pos2::ZERO, Vec2::splat(1.0));
        Self {
            zoom: 1.0,
            zoom_speed: 0.5,
            direction: DEFAULT_AXES,
            position: Pos2::new(0.0, 0.0),
            to_screen: ToScreen::new(
                Project3To2::new(&DEFAULT_AXES),
                RectTransform::identity(default_rect),
            ),
            allow_rotation_2d: true,
        }
    }
}

impl Camera3D {
    pub fn zoom(&self) -> f32 {
        self.zoom
    }

    /// position centered in front of camera in screen coordiantes
    pub fn in_front_of_cam(&self) -> Pos2 {
        self.to_screen.move_rect.to().center()
    }

    pub fn screen_to_intermediary(&self, screen_pos: Pos2) -> Pos2 {
        self.to_screen.move_rect.inverse().transform_pos(screen_pos)
    }

    pub fn update_to_screen(&mut self, screen: Rect) {
        //it is important to use the current zoom to compute the influence of self.position,
        //not the zoom of the last frame. thus we never read the last self.to_streen.
        let scale_rect = {
            let ratio = screen.aspect_ratio();
            //shapes are centered around zero, with extreme vertices having length 1.0
            let mut from_size = Vec2::splat(2.05 / self.zoom);
            if ratio < 1.0 {
                from_size.y /= ratio;
            } else {
                from_size.x *= ratio;
            };
            let from = Rect::from_center_size(Pos2::ZERO, from_size);
            RectTransform::from_to(from, screen)
        };
        let center = self.position.to_vec2() / scale_rect.scale();
        let from = scale_rect.from().translate(-center);
        let move_rect = RectTransform::from_to(from, screen);

        //something something "project" projects to camera coordinates,
        //so we need to invert the axe's rotation or something
        let project = Project3To2::from_transposed(&self.direction);
        self.to_screen = ToScreen::new(project, move_rect);
    }

    /// same effect as dragging the graph on the screen,
    /// only this is done in graph coordinates, not screen coordinates.
    pub fn shift_world_by(&mut self, shift: Vec3) {
        let shift_2d = self.to_screen.to_plane.project_pos(shift.to_pos3()).to_vec2();
        let scaled = shift_2d * self.to_screen.move_rect.scale();
        self.position += scaled;
        let screen = *self.to_screen.move_rect.to();
        self.update_to_screen(screen);
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label("Kamera: ");
            if ui.button("🏠").on_hover_text("setze Kamera zurück").clicked() {
                self.reset();
            }
            ui.add_space(3.0);
            ui.add(DragValue::new(&mut self.zoom_speed).range(0.05..=4.0).speed(0.01))
                .on_hover_text("Zoom-/Scroll-/Rotationsgeschwindigkeit");
            ui.add_space(3.0);
            if ui.add(Button::new("🔄").sense(Sense::drag())).dragged() {
                ui.ctx().request_repaint();
                self.rotate_z_2d(-0.01 * self.zoom_speed, Vec2::ZERO);
            }
            if ui.add(Button::new("🔃").sense(Sense::drag())).dragged() {
                ui.ctx().request_repaint();
                self.rotate_z_2d(0.01 * self.zoom_speed, Vec2::ZERO);
            }
            ui.add_space(3.0);
            ui.checkbox(&mut self.allow_rotation_2d, "")
                .on_hover_text("erlaube Rotieren mit Touchgesten für 2D Graphen");
        });
    }

    pub fn update_direction(&mut self, mut f: impl FnMut(Vec3) -> Vec3) {
        for axis in &mut self.direction {
            *axis = f(*axis);
        }
    }

    pub fn rotate_x(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_x(angle));
    }

    pub fn rotate_y(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_y(angle));
    }

    pub fn rotate_z(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_z(angle));
    }

    pub fn z_angle(&self) -> f32 {
        self.direction[2].angle_between(DEFAULT_AXES[2])
    }

    fn rotate_z_2d(&mut self, angle: f32, offset: Vec2) {
        self.rotate_z(angle);
        self.position = {
            let no_off = self.position - offset;
            let as_vec = Vec3::new(no_off.x, no_off.y, 0.0);
            let rotated = as_vec.rotate_z(angle);
            pos2(rotated.x, rotated.y) + offset
        };
    }

    fn zoom_delta(&self, info: &InputState) -> f32 {
        match info.multi_touch() {
            Some(touch) => touch.zoom_delta,
            None => info.zoom_delta().powf(self.zoom_speed), //speed is adjustable if done by ctrl + scrolling
        }
    }

    /// only z-rotation, zoom is centered around mouse pointer
    pub fn update_2d(&mut self, ui: &mut Ui, screen: Rect) {
        if ui.rect_contains_pointer(screen) {
            //Note: referencing ui inside lambda may cause deadlock
            ui.input(|info| {
                if info.pointer.button_down(PointerButton::Secondary) {
                    self.position += info.pointer.delta();
                }
                self.position += info.smooth_scroll_delta * self.zoom_speed;
                if let Some(drag) = info.multi_touch() {
                    self.position += drag.translation_delta;
                    if self.allow_rotation_2d {
                        let offset = info
                            .pointer
                            .interact_pos()
                            .map(|p| p - screen.center())
                            .unwrap_or(Vec2::ZERO);
                        self.rotate_z_2d(drag.rotation_delta, offset);
                    }
                }

                let zoom_delta = self.zoom_delta(info);
                self.zoom *= zoom_delta;
                if zoom_delta != 1.0
                    && let Some(ptr_pos) = info.pointer.latest_pos()
                {
                    //keep fixed point of zoom at mouse pointer
                    let mid_to_ptr = ptr_pos - screen.center();
                    let mut zoom_center = self.position.to_vec2() - mid_to_ptr;
                    zoom_center *= zoom_delta;
                    self.position = zoom_center.to_pos2() + mid_to_ptr;
                }
            });
        }
        self.update_to_screen(screen);
    }

    /// no translation, zoom is centered around screen middle    
    pub fn update_3d(&mut self, ui: &mut Ui, screen: Rect) {
        if ui.rect_contains_pointer(screen) {
            //Note: referencing ui inside lambda may cause deadlock
            ui.input(|info| {
                let mut drag_dist = Vec2::ZERO;
                if info.pointer.button_down(PointerButton::Secondary) {
                    drag_dist -= info.pointer.delta();
                }
                drag_dist -= info.smooth_scroll_delta * self.zoom_speed;
                if let Some(drag) = info.multi_touch() {
                    drag_dist -= drag.translation_delta;
                    self.rotate_z(drag.rotation_delta);
                }
                let drag_rot = drag_dist / self.to_screen.move_rect.scale();
                self.rotate_x(drag_rot.y);
                self.rotate_y(drag_rot.x);
                geo::gram_schmidt_3d(&mut self.direction);

                self.zoom *= self.zoom_delta(info);
            });
        }
        self.update_to_screen(screen);
    }

    pub fn reset_position(&mut self) {
        self.position = Pos2::ZERO;
    }

    pub fn reset_direction(&mut self) {
        self.direction = DEFAULT_AXES;
    }

    pub fn adjust_to_new_map(&mut self, new_shape: &crate::graph::Shape) {
        if new_shape.is_3d() {
            self.reset_position();
        } else {
            self.reset_direction();
        }
    }

    pub fn reset(&mut self) {
        *self = Self {
            zoom_speed: self.zoom_speed,
            allow_rotation_2d: self.allow_rotation_2d,
            ..Self::default()
        };
    }

    pub fn transform(&self, real_pos: Pos3) -> Pos2 {
        self.to_screen.apply(real_pos)
    }

    pub fn to_screen(&self) -> &ToScreen {
        &self.to_screen
    }

    pub fn screen_normal(&self) -> Vec3 {
        self.to_screen.to_plane.new_z
    }
}
