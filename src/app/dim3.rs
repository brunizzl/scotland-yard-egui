
use std::iter;

use egui::*;

use crate::{ graph::Embedding3D, app::*, geo::{Vec3, Pos3, self} };


#[derive(Clone, Copy, PartialEq)]
enum MapShape { Tetrahedron, Octahedron, Icosahedron, DividedIcosahedron }

const DEFAULT_AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

pub struct State {
    map: Embedding3D,
    map_shape: MapShape,
    map_axes: [Vec3; 3], //rotated by dragging picture
    map_divisions: isize,

    camera_2d: Camera2D,

    info: InfoState,

    /// lives in weird hybrid of 2D and 3D space.
    /// if on node: in 3D, if not on node: in 2D.
    /// he thus turns with the camera if he is on a node, else not.
    tolerance: f32, //how close must a character be to a vertex to count as beeing on that vertex
}

impl State {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut res = Self { 
            map: Embedding3D::new_subdivided_tetrahedron(1.0, 1), 
            map_shape: MapShape::Tetrahedron, 
            map_axes: DEFAULT_AXES, 
            map_divisions: 6,

            camera_2d: Camera2D::new(),

            info: InfoState::new(),
            tolerance: 0.25,
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

    fn camera_projection(&self) -> geo::Project3To2 {
        //something something "project" projects to camera coordinates,
        //so we need to invert the axe's rotation or something
        geo::Project3To2::inverse_of(&self.map_axes)
    }

    pub fn recompute_graph(&mut self) {
        let s = 1.0;
        let r = self.map_divisions as usize;
        self.map = match self.map_shape {
            MapShape::Icosahedron => Embedding3D::new_subdivided_icosahedron(s, r),
            MapShape::Octahedron => Embedding3D::new_subdivided_octahedron(s, r),
            MapShape::Tetrahedron => Embedding3D::new_subdivided_tetrahedron(s, r),
            MapShape::DividedIcosahedron => Embedding3D::new_subdivided_subdivided_icosahedron(s, r, 0),
        };
        self.tolerance = f32::min(0.25, 0.75 / self.map_divisions as f32);
        let to_2d = self.camera_projection();
        self.map.update_vertex_visibility(&to_2d, &mut self.info.visible);
        for char in &mut self.info.characters {
            let char_dir = char.pos3.to_vec3();
            let potential = |_, v_pos: Pos3| -char_dir.dot(v_pos.to_vec3().normalized());
            let (best_new_vertex, _) = self.map.find_local_minimum(potential, 0);
            char.nearest_node = best_new_vertex;
            char.update_distances(self.map.edges(), &mut self.info.queue);
        }
        self.info.forget_move_history();
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
            if add_drag_value(ui, &mut self.map_divisions, "Auflösung: ", 0, 200) {
                self.recompute_graph();
            }
        });
        self.info.draw_menu(ui);
        if let (RobberInfo::SmallRobberDist, Some(r)) = (self.info.robber_info, self.info.robber()) {
            let r_pos = self.map.positions()[r.nearest_node];
            let mut max_dist = f32::MIN;
            let mut min_dist = f32::MAX;
            let bnd = RobberInfo::scale_small_dist_with_radius(self.info.small_robber_dist, self.map_divisions);
            for (&dist, &pos) in r.distances.iter().zip(self.map.positions()) {
                if dist == bnd {
                    let new_dist = (r_pos - pos).length();
                    max_dist = f32::max(max_dist, new_dist);
                    min_dist = f32::min(min_dist, new_dist);
                }
            }
            ui.label(format!("min dist:   {}\nmax dist:   {}\nmin / max: {}", 
                min_dist, max_dist, min_dist / max_dist));
        }
    }

    /// to be reliable, this one must search exhaustive, as min_cop_dist is not monotone,
    /// thus a local minimum may not be the global one
    fn vertex_furthest_from_cops(&self) -> iter::Once<usize> {
        let (furthest_vertex, _) = self.info.min_cop_dist.iter().enumerate()
            .fold((0, 0), |best, (v, &dist)| if dist > best.1 { (v, dist) } else { best });
        iter::once(furthest_vertex)
    }

    fn draw_characters(&mut self, ui: &mut Ui, response: &Response, painter: &Painter, 
        transform: &geo::ToScreen, scale: f32) 
    {
        for (i, ch) in self.info.characters.iter_mut().enumerate() {
            ch.update_3d(self.tolerance, &self.map, &transform.to_plane, 
                &self.info.visible, &mut self.info.queue);
            let node_pos = transform.to_plane.project_pos(self.map.positions()[ch.nearest_node]);
            if ch.on_node && self.info.visible[ch.nearest_node] || !ch.on_node {
                let moved = ch.drag_and_draw(&response, &painter, ui, transform.move_rect, node_pos, scale);
                if moved {
                    self.info.past_moves.push(i);
                    self.info.future_moves.clear();
                }
            }
            else {
                let drawn_node_pos = transform.move_rect.transform_pos(node_pos);
                ch.draw_small_at(drawn_node_pos, &painter, scale);
            }
        }
    }

    pub fn draw_graph(&mut self, ui: &mut Ui) {
        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover()); 

        self.camera_2d.update_screen_centered(ui);
        let transform = {
            let min_size = Vec2::splat(2.05 / self.camera_2d.zoom);
            let contained = Rect::from_center_size(Pos2::ZERO, min_size);
            let shift_rects = build_to_screen_2d(contained, response.rect);
            geo::ToScreen::new(self.camera_projection(), shift_rects)
        };
        let rot = -self.camera_2d.offset / transform.move_rect.scale();
        self.rotate_axes_x(rot.y);
        self.rotate_axes_y(rot.x);
        self.rotate_axes_z(self.camera_2d.rotation);
        geo::gram_schmidt_3d(&mut self.map_axes);
        //use offset tu update rotation -> state is remembered by self.map_axes
        // -> offset can then be set back to 0.
        self.camera_2d.offset = Vec2::ZERO;
        self.camera_2d.rotation = 0.0;

        let to_screen = |p: Pos3| transform.apply(p);

        self.info.maybe_update(self.map.edges(), self.vertex_furthest_from_cops());

        let scale = self.camera_2d.zoom * f32::min(12.0 / self.map_divisions as f32, 4.0);
        let grey_stroke = Stroke::new(scale, GREY);
        self.map.draw_visible_edges(&transform, &painter, grey_stroke, 
            &mut self.info.visible);

        let positions = self.map.positions();
        self.info.draw_convex_cop_hull(positions, &painter, to_screen, scale);
        self.info.draw_green_circles(positions, &painter, to_screen, scale, self.map_divisions);
        self.info.draw_character_tails(positions, &painter, to_screen, scale);
        self.info.draw_robber_strat(self.map.edges(), positions, &painter, to_screen, scale);
        self.info.draw_numbers(positions, ui, &painter, to_screen, scale);

        self.draw_characters(ui, &response, &painter, &transform, scale);
    }

}


