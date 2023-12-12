
use egui::*;

use crate::{ graph::{self, Embedding2D}, app::* };

#[derive(Clone, Copy, PartialEq)]
pub enum GraphShape { RegularPolygon(isize), Random, Debug }

pub struct State {
    map: Embedding2D,

    //overall map shape: == 2 is line, == 3 triangle, == 4 square, == 5 pentagon etc.
    map_shape: GraphShape,
    map_radius: isize, //(approx.) shortest path length from origin to rim

    extreme_vertices: [usize; 4], //indices of vertices with extreme coodinates

    info: InfoState,
    
    tolerance: f32, //how close must a character be to a vertex to count as beeing on that vertex

    camera: Camera2D,
}

impl State {

    /// really shitty approximation of convex hull.
    fn compute_extreme_vertices(points: &[Pos2]) -> [usize; 4] {
        let mut res = [0; 4];
        let mut res_vals = [points[0]; 4];
        for (i, p) in points.iter().enumerate() {
            if p.x < res_vals[0].x {
                res[0] = i;
                res_vals[0] = *p;
            }
            if p.y < res_vals[1].y {
                res[1] = i;
                res_vals[1] = *p;
            }
            if p.x > res_vals[2].x {
                res[2] = i;
                res_vals[2] = *p;
            }
            if p.y > res_vals[3].y {
                res[3] = i;
                res_vals[3] = *p;
            }
        }
        res
    }

    fn recompute_graph(&mut self) {
        let r = self.map_radius as usize;
        self.map = match self.map_shape {
            GraphShape::RegularPolygon(n) => graph::triangulated_regular_polygon(n as usize, r),
            GraphShape::Random => graph::random_triangulated(r, 8),
            GraphShape::Debug => graph::debugging_graph(),
        };
        self.tolerance = f32::min(0.25, 0.75 / self.map_radius as f32);
        for char in &mut self.info.characters {
            char.update_2d(self.tolerance, &self.map, &mut self.info.queue);
        }
        self.extreme_vertices = Self::compute_extreme_vertices(self.map.positions());
        let edges = self.map.edges();
        self.info.update_convex_cop_hull(edges, self.extreme_vertices.iter().map(|&v| v));
        self.info.update_min_cop_dist();
        self.info.update_cop_advantage(edges);
    }

    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut res = State { 
            map: Embedding2D::empty(),
            map_shape: GraphShape::RegularPolygon(6),
            map_radius: 6,

            extreme_vertices: [0; 4],

            info: InfoState::new(),

            tolerance: 0.25,

            camera: Camera2D::new(),
         };
         res.recompute_graph();
         res
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) {
        if ui.button("🏠 Position").clicked() {
            self.camera.reset();
        }
        //adjust underlying graph
        ui.collapsing("Form", |ui| {
            let prev_shape = self.map_shape;
            let is_n_gon = matches!(self.map_shape, GraphShape::RegularPolygon(_));
            if ui.add(RadioButton::new(is_n_gon, "ReguläresPolygon")).clicked() {                        
                self.map_shape = GraphShape::RegularPolygon(6);
            }
            if let GraphShape::RegularPolygon(n) = &mut self.map_shape {
                super::add_drag_value(ui, n, "Seiten: ", 3, 10);
            }
            ui.radio_value(&mut self.map_shape, GraphShape::Random, "Zufallsverteilt");
            if self.map_shape == GraphShape::Random {
                if ui.button("neu berechnen").clicked() {
                    self.recompute_graph();
                }
            }
            ui.radio_value(&mut self.map_shape, GraphShape::Debug, "Debugging");
            if prev_shape != self.map_shape {
                self.recompute_graph();
            }
            if add_drag_value(ui, &mut self.map_radius, "Radius: ", 0, 100) {
                self.recompute_graph();
            }
        });
        self.info.draw_menu(ui);
        if let (RobberInfo::SmallRobberDist, Some(r)) = (self.info.robber_info, self.info.robber()) {
            let r_pos = self.map.positions()[r.nearest_node];
            let mut max_dist = f32::MIN;
            let mut min_dist = f32::MAX;
            let bnd = RobberInfo::scale_small_dist_with_radius(self.info.small_robber_dist, self.map_radius);
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

    fn update_visible_vertices(&mut self, to_screen: &emath::RectTransform) {
        let mut visible = std::mem::take(&mut self.info.visible);
        visible.clear();
        for &pos in self.map.positions() {
            visible.push(to_screen.from().contains(pos));
        }
        self.info.visible = visible;
    }

    fn draw_edges(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {

        let grey_stroke = Stroke::new(scale, GREY);
        let edge_stroke = |v1: usize, v2: usize| if self.info.debug_info {
            let seed = v1 * v2 + 100;
            let mut gen = crate::rand::LCG::new(seed as u64);
            gen.waste(5);
            let mut rnd = || (64 + (gen.next() % 128)) as u8;
            Stroke::new(scale, Color32::from_rgb(rnd(), rnd(), rnd()))
        }
        else {
            grey_stroke
        };

        self.map.for_each_edge(|v1, p1, v2, p2| { 
            let edge = [
                to_screen.transform_pos(p1), 
                to_screen.transform_pos(p2)];
            let line = Shape::LineSegment { points: edge, stroke: edge_stroke(v1, v2) };
            painter.add(line);
        });
    }

    fn draw_characters(&mut self, ui: &mut Ui, response: &Response, painter: &Painter, 
        to_screen: emath::RectTransform, scale: f32) 
    {
        for character in self.info.characters.iter_mut() {
            let node_pos = self.map.positions()[character.nearest_node];
            if character.dragging {
                character.update_2d(self.tolerance, &self.map, &mut self.info.queue);
            }
            character.drag_and_draw(response, painter, ui, to_screen, node_pos, scale);
        }
    }

    pub fn draw_graph(&mut self, ui: &mut Ui) {
        self.info.maybe_update(self.map.edges(), self.extreme_vertices.iter().map(|&v| v));

        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover());   
        self.camera.update_cursor_centered(ui, &response); 

        let transform = {
            //our goal is to find out, by what we need to shift the center of our internal coordinates,
            //due to the user moving around the graph on screen.
            //however to compute that shift, we need to already know the transformation, thus
            //the two step process shown below
            let min_size = Vec2::splat(2.05 / self.camera.zoom);
            let contained_centered = Rect::from_center_size(Pos2::ZERO, min_size);
            let screen = response.rect;
            let inverse_centered = build_to_screen_2d(contained_centered, screen).inverse();
            let shift = inverse_centered.transform_pos(screen.center() - self.camera.offset) - Pos2::ZERO;
            let graph_rect = inverse_centered.to().translate(shift);
            emath::RectTransform::from_to(graph_rect, screen)
        };
        let scale = self.camera.zoom * f32::min(12.0 / self.map_radius as f32, 4.0);

        self.update_visible_vertices(&transform);

        let to_screen = |p: Pos2| transform.transform_pos(p);

        self.draw_edges(&painter, transform, scale);
        let positions = self.map.positions();
        self.info.draw_convex_cop_hull(positions, &painter, to_screen, scale);
        self.info.draw_green_circles(positions, &painter, to_screen, scale, self.map_radius);
        self.info.draw_numbers(positions, ui, &painter, to_screen, scale);
        self.info.draw_robber_strat(self.map.edges(), positions, &painter, to_screen, scale);

        self.draw_characters(ui, &response, &painter, transform, scale);
    }
}
