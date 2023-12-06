
use egui::*;

use crate::{ graph::ConvexPolyhedron, app::*, geo::{Vec3, self} };


#[derive(Clone, Copy, PartialEq)]
enum Platonic { Tetrahedron, Cube, Octahedron, Dodecahedron, Icosahedron }

const DEFAULT_AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

pub struct State {
    map: ConvexPolyhedron,
    map_shape: Platonic,
    map_axes: [Vec3; 3], //rotated by dragging picture

    camera_2d: Camera2D,
}

impl State {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let scale = 1.0;
        Self { 
            map: ConvexPolyhedron::new_cube(scale), 
            map_shape: Platonic::Cube, 
            map_axes: DEFAULT_AXES, 

            camera_2d: Camera2D::new(),
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
        let scale = 1.0;
        self.map = match self.map_shape {
            Platonic::Cube => ConvexPolyhedron::new_cube(scale),
            Platonic::Dodecahedron => ConvexPolyhedron::new_dodecahedron(scale),
            Platonic::Icosahedron => ConvexPolyhedron::new_icosahedron(scale),
            Platonic::Octahedron => ConvexPolyhedron::new_octahedron(scale),
            Platonic::Tetrahedron => ConvexPolyhedron::new_tetrahedron(scale),
        };
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) { 
        if ui.button("🏠 Position").clicked() {
            self.camera_2d.reset();
            self.map_axes = DEFAULT_AXES;
        }
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
        let camera = self.camera_2d;
        let rect_min = vec2(-1.05, -1.05) / camera.zoom;
        let rect_max = vec2(1.05, 1.05) / camera.zoom;
        let from = Rect::from_min_max(rect_min.to_pos2(), rect_max.to_pos2());

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
        self.camera_2d.update(ui, &response); 

        //use offset tu update rotation -> state is remembered by self.map_axes
        // -> offset can then be set back to 0.
        self.rotate_axes_x(self.camera_2d.offset.y * -0.002 / self.camera_2d.zoom);
        self.rotate_axes_y(self.camera_2d.offset.x * -0.002 / self.camera_2d.zoom);
        geo::gram_schmidt_3d(&mut self.map_axes);
        self.camera_2d.offset = Vec2::ZERO;

        let to_screen = self.build_to_screen(&response);

        let grey_stroke = Stroke::new(1.0, GREY);
        self.map.draw_visible_faces(&to_screen, &painter, grey_stroke);
    }

}


