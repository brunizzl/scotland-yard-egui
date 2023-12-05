
use egui::*;

use crate::{ graph::ConvexPolyhedron, app::*, geo::{Vec3, vec3, self} };

pub struct State {
    map: ConvexPolyhedron,
    camera_axes: [Vec3; 3],
}

impl State {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let start_axes = [Vec3::X, Vec3::Y, Vec3::Z];
        let len = 0.5;
        Self { map: ConvexPolyhedron::new_icosahedron(len), camera_axes: start_axes }
    }

    fn update_axes(&mut self, mut f: impl FnMut(Vec3) -> Vec3) {
        for axis in &mut self.camera_axes {
            *axis = f(*axis);
        }
    }

    fn rotate_axes_x(&mut self, angle: f32) {
        self.update_axes(|v| v.rotate_x(angle))
    }

    fn rotate_axes_y(&mut self, angle: f32) {
        self.update_axes(|v| v.rotate_y(angle))
    }

    #[allow(dead_code)]
    fn rotate_axes_z(&mut self, angle: f32) {
        self.update_axes(|v| v.rotate_z(angle))
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) { 
        ui.horizontal(|ui| { 
            ui.label("coming soon!");
        });
    }

    fn build_to_screen(&self, response: &Response) -> geo::ToScreen {

        let from = Rect::from_min_max(pos2(-1.05, -1.05), pos2(1.05, 1.05));

        let rect_len = f32::min(response.rect.height(), response.rect.width());
        let to_middle = (response.rect.width() - rect_len) / 2.0;
        let screen_min = response.rect.min + vec2(to_middle, 0.0);
        let to = Rect::from_min_size(screen_min, vec2(rect_len, rect_len));

        let to_screen = emath::RectTransform::from_to(from, to);

        //we actually really would love the rotation to be inverted, thus we invert it.
        //(because the implementation is a bit weird, 
        // as in draw_graph the camera is dragged, instead of the graph's axes)
        let axes = self.camera_axes.clone();
        let v1 = vec3(axes[0].x, axes[1].x, axes[2].x);
        let v2 = vec3(axes[0].y, axes[1].y, axes[2].y);
        let v3 = vec3(axes[0].z, axes[1].z, axes[2].z);
        let project = geo::Project3To2::new(v1, v2, v3);
        geo::ToScreen::new(project, to_screen)
    }

    pub fn draw_graph(&mut self, ui: &mut Ui) {

        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover());
        let to_screen = self.build_to_screen(&response);

        let background_id = response.id.with(0);
        let point_response = ui.interact(*to_screen.move_rect.to(), background_id, Sense::drag());
        self.rotate_axes_x(point_response.drag_delta().y * -0.004);
        self.rotate_axes_y(point_response.drag_delta().x * -0.004);
        geo::gram_schmidt_3d(&mut self.camera_axes);

        let grey_stroke = Stroke::new(1.0, GREY);
        self.map.draw_visible_edges(&to_screen, &painter, grey_stroke);
    }

}


