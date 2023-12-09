
use std::collections::VecDeque;

use egui::*;

use crate::{ graph::Embedding3D, app::*, geo::{Vec3, Pos3, self} };


#[derive(Clone, Copy, PartialEq)]
enum MapShape { Tetrahedron, Octahedron, Icosahedron, DividedIcosahedron }

const DEFAULT_AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

pub struct State {
    map: Embedding3D,
    map_shape: MapShape,
    map_axes: [Vec3; 3], //rotated by dragging picture
    map_radius: usize,

    camera_2d: Camera2D,

    /// lives in weird hybrid of 2D and 3D space.
    /// if on node: in 3D, if not on node: in 2D.
    /// he thus turns with the camera if he is on a node, else not.
    characters: Vec<Character>,
    tolerance: f32, //how close must a character be to a vertex to count as beeing on that vertex

    queue: VecDeque<usize>,
}

impl State {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut res = Self { 
            map: Embedding3D::new_subdivided_tetrahedron(1.0, 1), 
            map_shape: MapShape::Tetrahedron, 
            map_axes: DEFAULT_AXES, 
            map_radius: 6,

            camera_2d: Camera2D::new(),

            characters: vec![
                Character::new(false, Pos2::ZERO),
                Character::new(true, pos2(0.25, 0.0)),
            ],
            tolerance: 0.25,

            queue: VecDeque::new(),
        };
        res.recompute_graph();
        res
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
        self.tolerance = f32::min(0.25, 0.75 / self.map_radius as f32);
        let to_2d = geo::Project3To2::inverse_of(&self.map_axes);
        let potential = |v_pos: Pos3| -to_2d.signed_dist(v_pos.to_vec3()); 
        let (node_closest_to_cam, _) = self.map.find_local_minimum(potential, 0);
        for char in &mut self.characters {
            char.snap_to_node();
            char.nearest_node = node_closest_to_cam;
            char.update_3d(self.tolerance, &self.map, &to_2d, &mut self.queue);
        }
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
        draw_character_buttons(ui, &mut self.characters);
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

        for ch in &mut self.characters {
            ch.update_3d(self.tolerance, &self.map, &to_screen.to_plane, &mut self.queue);
            let node_pos = to_screen.to_plane.project_pos(self.map.vertices()[ch.nearest_node]);
            ch.drag_and_draw(&response, &painter, ui, to_screen.move_rect, node_pos, scale * 10.0);
        }
    }

}


