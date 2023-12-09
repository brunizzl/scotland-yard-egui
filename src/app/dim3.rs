
use egui::{*, epaint::TextShape, text::LayoutJob};

use crate::{ graph::Embedding3D, app::*, geo::{Vec3, Pos3, self} };


#[derive(Clone, Copy, PartialEq)]
enum MapShape { Tetrahedron, Octahedron, Icosahedron, DividedIcosahedron }

const DEFAULT_AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];

pub struct State {
    map: Embedding3D,
    visible_vertices: Vec<bool>,
    map_shape: MapShape,
    map_axes: [Vec3; 3], //rotated by dragging picture
    map_radius: usize,

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
            visible_vertices: Vec::new(),
            map_shape: MapShape::Tetrahedron, 
            map_axes: DEFAULT_AXES, 
            map_radius: 6,

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
        self.map.update_vertex_visibility(&to_2d, &mut self.visible_vertices);
        let potential = |_, v_pos: Pos3| -to_2d.signed_dist(v_pos.to_vec3()); 
        let (node_closest_to_cam, _) = self.map.find_local_minimum(potential, 0);
        for char in &mut self.info.characters {
            char.snap_to_node();
            char.nearest_node = node_closest_to_cam;
            char.update_3d(self.tolerance, &self.map, &to_2d, &self.visible_vertices, &mut self.info.queue);
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
        self.info.draw_menu(ui);
    }

    fn draw_convex_cop_hull(&self, painter: &Painter, to_screen: &geo::ToScreen, scale: f32) {
        for (&in_hull, &pos) in self.info.in_convex_cop_hull.iter().zip(self.map.vertices()) {
            if in_hull.yes()  {
                let draw_pos = to_screen.apply(pos);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 9.0, LIGHT_BLUE);
                painter.add(marker_circle);
            }
        }
    }

    fn draw_green_circles(&self, painter: &Painter, to_screen: &geo::ToScreen, scale: f32) {     
        let draw_circle_at = |pos: Pos3|{
            let draw_pos = to_screen.apply(pos);
            let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
            painter.add(marker_circle);
        };
        match (self.info.robber_info, self.info.robber()) {
            (RobberInfo::NearNodes, Some(r)) => 
                for (r_dist, c_dist, &pos) in 
                itertools::izip!(r.distances.iter(), self.info.min_cop_dist.iter(), self.map.vertices()) {
                    if r_dist < c_dist {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::EscapableNodes, _) => 
                for (&adv, &pos) in self.info.cop_advantage.iter().zip(self.map.vertices()) {
                    if adv < -1 {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::SmallRobberDist(bnd), Some(r)) => {
                let bnd = RobberInfo::scale_small_dist_with_radius(bnd, self.map_radius);
                for (&dist, &pos) in r.distances.iter().zip(self.map.vertices()) {
                    if (dist as usize) <= bnd {
                        draw_circle_at(pos);
                    }
                }
            },
            _ => {},
        }
    }

    fn draw_robber_info(&self, painter: &Painter, to_screen: &geo::ToScreen, scale: f32) {
        if let (RobberInfo::NearNodes, Some(r)) = (self.info.robber_info, self.info.robber()) { 
            let dist_vs = r.distances.iter().zip(self.info.min_cop_dist.iter());
            for ((r_dist, c_dist), &pos) in dist_vs.zip(self.map.vertices()) {
                if r_dist < c_dist  {
                    let draw_pos = to_screen.apply(pos);
                    let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
                    painter.add(marker_circle);
                }
            }
        }
        if self.info.robber_info == RobberInfo::EscapableNodes {
            for (&adv, &pos) in self.info.cop_advantage.iter().zip(self.map.vertices()) {
                if adv < -1  {
                    let draw_pos = to_screen.apply(pos);
                    let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
                    painter.add(marker_circle);
                }
            }
        }
    }

    fn draw_numbers(&self, ui: &mut Ui, painter: &Painter, to_screen: &geo::ToScreen, scale: f32) {
        let font = FontId::proportional(scale * 8.0);
        let color = if ui.ctx().style().visuals.dark_mode { WHITE } else { BLACK };
        for (i, &pos) in self.map.vertices().iter().enumerate() {
            let txt = match self.info.vertex_info {
                DrawNumbers::Indices => { i.to_string() }
                DrawNumbers::MinCopDist => { self.info.min_cop_dist[i].to_string() }
                DrawNumbers::None => { panic!() }
                DrawNumbers::RobberAdvantage => { (-1 -self.info.cop_advantage[i]).to_string() }
            };
            let mut layout_job = LayoutJob::simple(txt, font.clone(), color, 100.0 * scale);
            layout_job.halign = Align::Center;
            let galley = ui.fonts(|f| f.layout_job(layout_job));
            let screen_pos = to_screen.apply(pos);
            let text = Shape::Text(TextShape::new(screen_pos, galley));
            painter.add(text);
        }
    }

    fn draw_cop_voronoi(&self, painter: &Painter, to_screen: &geo::ToScreen, scale: f32) {
        for (&multiple, &pos) in self.info.muliple_min_dist_cops.iter().zip(self.map.vertices()) {
            if multiple  {
                let draw_pos = to_screen.apply(pos);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 5.0, RED);
                painter.add(marker_circle);
            }
        }
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
        self.rotate_axes_z(self.camera_2d.rotation);
        geo::gram_schmidt_3d(&mut self.map_axes);
        //use offset tu update rotation -> state is remembered by self.map_axes
        // -> offset can then be set back to 0.
        self.camera_2d.offset = Vec2::ZERO;
        self.camera_2d.rotation = 0.0;

        self.info.maybe_update(self.map.edges(), 0..self.map.surface().nr_vertices());

        let scale = f32::min(10.0 * self.camera_2d.zoom / self.map_radius as f32, 5.0);
        let grey_stroke = Stroke::new(scale, GREY);
        self.map.draw_visible_edges(&to_screen, &painter, grey_stroke, &mut self.visible_vertices);

        if self.info.show_convex_hull {
            self.draw_convex_cop_hull(&painter, &to_screen, scale);
        }
        self.draw_green_circles(&painter, &to_screen, scale);
        self.draw_robber_info(&painter, &to_screen, scale);

        if self.map_radius < 20 && self.info.vertex_info != DrawNumbers::None {
            self.draw_numbers(ui, &painter, &to_screen, scale);
        }
        if self.info.show_cop_voronoi {
            self.draw_cop_voronoi(&painter, &to_screen, scale);
        }

        for ch in &mut self.info.characters {
            ch.update_3d(self.tolerance, &self.map, &to_screen.to_plane, &self.visible_vertices, &mut self.info.queue);
            let node_pos = to_screen.to_plane.project_pos(self.map.vertices()[ch.nearest_node]);
            if ch.on_node && self.visible_vertices[ch.nearest_node] || !ch.on_node {
                ch.drag_and_draw(&response, &painter, ui, to_screen.move_rect, node_pos, scale);
            }
            else {
                let drawn_node_pos = to_screen.move_rect.transform_pos(node_pos);
                ch.draw_small_at(drawn_node_pos, &painter, scale);
            }
        }
    }

}


