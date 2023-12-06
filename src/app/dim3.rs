
use egui::*;

use crate::{ graph::ConvexPolyhedron, app::*, geo::{Vec3, self} };


#[derive(Clone, Copy, PartialEq)]
enum Platonic { Tetrahedron, Cube, Octahedron, Dodecahedron, Icosahedron }

pub struct State {
    map: ConvexPolyhedron,
    map_shape: Platonic,
    map_axes: [Vec3; 3], //rotated by dragging picture
}

impl State {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let len = 0.5;
        Self { 
            map: ConvexPolyhedron::new_cube(len), 
            map_shape: Platonic::Cube, 
            map_axes: [Vec3::X, Vec3::Y, Vec3::Z], 
        }
    }

    fn update_axes(&mut self, mut f: impl FnMut(Vec3) -> Vec3) {
        for axis in &mut self.map_axes {
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

    pub fn recompute_graph(&mut self) {
        let scale = 0.5;
        self.map = match self.map_shape {
            Platonic::Cube => ConvexPolyhedron::new_cube(scale),
            Platonic::Dodecahedron => ConvexPolyhedron::new_dodecahedron(scale),
            Platonic::Icosahedron => ConvexPolyhedron::new_icosahedron(scale),
            Platonic::Octahedron => ConvexPolyhedron::new_octahedron(scale),
            Platonic::Tetrahedron => ConvexPolyhedron::new_tetrahedron(scale),
        };
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) { 
        ui.horizontal(|ui| { 
            ui.label("coming soon!");
        });
        ui.collapsing("Form", |ui| {
            let old_shape = self.map_shape;
            ui.radio_value(&mut self.map_shape, Platonic::Tetrahedron, "Tetraeder");
            ui.radio_value(&mut self.map_shape, Platonic::Octahedron, "Oktaeder");
            ui.radio_value(&mut self.map_shape, Platonic::Cube, "Würfel");
            ui.radio_value(&mut self.map_shape, Platonic::Icosahedron, "Ikosaeder");
            ui.radio_value(&mut self.map_shape, Platonic::Dodecahedron, "Dodekaeder");
            if self.map_shape != old_shape {
                self.recompute_graph();
            }
        });
    }

    fn build_to_screen(&self, response: &Response) -> geo::ToScreen {

        let from = Rect::from_min_max(pos2(-1.05, -1.05), pos2(1.05, 1.05));

        let rect_len = f32::min(response.rect.height(), response.rect.width());
        let to_middle = (response.rect.width() - rect_len) / 2.0;
        let screen_min = response.rect.min + vec2(to_middle, 0.0);
        let to = Rect::from_min_size(screen_min, vec2(rect_len, rect_len));

        let to_screen = emath::RectTransform::from_to(from, to);

        //something something project is to camera coordinates,
        //so we need to invert the axe's rotation or something
        let project = geo::Project3To2::new_transposed(&self.map_axes);
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
        geo::gram_schmidt_3d(&mut self.map_axes);

        let grey_stroke = Stroke::new(1.0, GREY);
        self.map.draw_visible_faces(&to_screen, &painter, grey_stroke);
    }

}


