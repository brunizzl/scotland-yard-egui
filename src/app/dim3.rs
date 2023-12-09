
use egui::*;

use crate::{ graph::Embedding3D, app::*, geo::{Vec3, self} };


#[derive(Clone, Copy, PartialEq)]
enum MapShape { Tetrahedron, Octahedron, Icosahedron, DividedIcosahedron }

const DEFAULT_AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

pub struct State {
    map: Embedding3D,
    map_shape: MapShape,
    map_axes: [Vec3; 3], //rotated by dragging picture
    map_radius: usize,

    camera_2d: Camera2D,
}

impl State {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let s = 1.0;
        let r = 10;
        Self { 
            map: Embedding3D::new_subdivided_tetrahedron(s, r), 
            map_shape: MapShape::Tetrahedron, 
            map_axes: DEFAULT_AXES, 
            map_radius: r,

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
        let s = 1.0;
        let r = self.map_radius;
        self.map = match self.map_shape {
            MapShape::Icosahedron => Embedding3D::new_subdivided_icosahedron(s, r),
            MapShape::Octahedron => Embedding3D::new_subdivided_octahedron(s, r),
            MapShape::Tetrahedron => Embedding3D::new_subdivided_tetrahedron(s, r),
            MapShape::DividedIcosahedron => Embedding3D::new_subdivided_subdivided_icosahedron(s, r, 0),
        };
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) { 
        if ui.button("🏠 Position").clicked() {
            self.camera_2d.reset();
            self.map_axes = DEFAULT_AXES;
        }
        ui.collapsing("Form", |ui| {
            let old_shape = self.map_shape;
            ui.radio_value(&mut self.map_shape, MapShape::Tetrahedron, "Tetraeder");
            ui.radio_value(&mut self.map_shape, MapShape::Octahedron, "Oktaeder");
            ui.radio_value(&mut self.map_shape, MapShape::Icosahedron, "Ikosaeder");
            ui.radio_value(&mut self.map_shape, MapShape::DividedIcosahedron, "aufgepusteter\nIkosaeder");
            if self.map_shape != old_shape {
                self.recompute_graph();
            }
            if add_drag_value(ui, &mut self.map_radius, "Radius: ", 0, 1000) {
                self.recompute_graph();
            }
        });
    }

    fn build_to_screen(&self, response: &Response) -> geo::ToScreen {
        let to = response.rect;
        let aspect_ratio = to.width() / to.height();
        //shapes are centered around zero, with extreme vertices having length 1.0
        let from_size = if aspect_ratio < 1.0 {
            vec2(2.05, 2.05 / aspect_ratio)
        }
        else {
            vec2(2.05 * aspect_ratio, 2.05)
        };
        let from = Rect::from_center_size(Pos2::ZERO, from_size / self.camera_2d.zoom);

        let to_screen = emath::RectTransform::from_to(from, to);

        //something something "project" projects to camera coordinates,
        //so we need to invert the axe's rotation or something
        let project = geo::Project3To2::inverse_of(&self.map_axes);
        geo::ToScreen::new(project, to_screen)
    }

    pub fn draw_graph(&mut self, ui: &mut Ui) {

        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover()); 

        self.camera_2d.update_screen_centered(ui);
        let to_screen = self.build_to_screen(&response);
        let rot = -self.camera_2d.offset / to_screen.move_rect.scale();
        self.rotate_axes_x(rot.y);
        self.rotate_axes_y(rot.x);
        self.rotate_axes_z(-self.camera_2d.rotation);
        geo::gram_schmidt_3d(&mut self.map_axes);
        //use offset tu update rotation -> state is remembered by self.map_axes
        // -> offset can then be set back to 0.
        self.camera_2d.offset = Vec2::ZERO;
        self.camera_2d.rotation = 0.0;

        let scale = self.camera_2d.zoom / f32::max(self.map_radius as f32, 5.0);
        let grey_stroke = Stroke::new(scale * 10.0, GREY);
        self.map.draw_visible_edges(&to_screen, &painter, grey_stroke);
    }

}


