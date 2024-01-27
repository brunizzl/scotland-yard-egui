
use egui::{*, emath::RectTransform};

use crate::geo::{self, Pos3, Vec3, Project3To2, ToScreen};

const DEFAULT_AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

#[derive(serde::Deserialize, serde::Serialize)]
pub struct Camera3D {
    /// == 1.0 -> no change
    /// < 1.0  -> zoomed out
    /// > 1.0  -> zoomed in
    zoom: f32, 
    direction: [Vec3; 3],
    /// offset of center independent of zoom + direction (e.g. in draw plane)
    position: Pos2,

    to_screen: ToScreen,
}

impl Camera3D {
    pub fn zoom(&self) -> f32 {
        self.zoom
    }

    pub fn new() -> Self {
        let default_rect = Rect::from_center_size(Pos2::ZERO, Vec2::splat(1.0));
        Self { 
            zoom: 1.0, 
            direction: DEFAULT_AXES,
            position: Pos2::new(0.0, 0.0), 
            to_screen: ToScreen::new(Project3To2::new(&DEFAULT_AXES), RectTransform::identity(default_rect))
        }
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
            }
            else {
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
        let project = Project3To2::inverse_of(&self.direction);
        self.to_screen = ToScreen::new(project, move_rect);
    }

    pub fn update_direction(&mut self, mut f: impl FnMut(Vec3) -> Vec3) {
        for axis in &mut self.direction {
            *axis = f(*axis);
        }
    }

    pub fn rotate_x(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_x(angle))
    }

    pub fn rotate_y(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_y(angle))
    }

    pub fn rotate_z(&mut self, angle: f32) {
        self.update_direction(|v| v.rotate_z(angle))
    }

    /// only z-rotation, zoom is centered around mouse pointer
    pub fn update_2d(&mut self, ui: &mut Ui, screen: Rect) {  
        if ui.rect_contains_pointer(screen) { 
            //Note: referencing ui inside lambda may cause deadlock
            ui.input(|info| {
                if info.pointer.button_down(PointerButton::Secondary) {
                    self.position += info.pointer.delta();
                }
                self.position += info.scroll_delta;
                if let Some(drag) = info.multi_touch() {
                    self.position += drag.translation_delta;
                    self.rotate_z(drag.rotation_delta);
                }
    
                let zoom_delta = info.zoom_delta();
                self.zoom *= zoom_delta;
                if zoom_delta != 1.0 {
                    if let Some(ptr_pos) = info.pointer.latest_pos() {
                        //keep fixed point of zoom at mouse pointer
                        let mid_to_ptr = ptr_pos - screen.center();
                        let mut zoom_center = self.position.to_vec2() - mid_to_ptr;
                        zoom_center *= zoom_delta;
                        self.position = zoom_center.to_pos2() + mid_to_ptr;
                    }
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
                drag_dist -= info.scroll_delta;
                if let Some(drag) = info.multi_touch() {
                    drag_dist -= drag.translation_delta;
                    self.rotate_z(drag.rotation_delta);
                }
                let drag_rot = drag_dist / self.to_screen.move_rect.scale();
                self.rotate_x(drag_rot.y);
                self.rotate_y(drag_rot.x);
                geo::gram_schmidt_3d(&mut self.direction);
    
                let zoom_delta = info.zoom_delta();
                self.zoom *= zoom_delta;
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

    pub fn reset(&mut self) {
        *self = Self::new();
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
