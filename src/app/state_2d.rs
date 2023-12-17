
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
        self.info.forget_move_history();
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
         };
         res.recompute_graph();
         res
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) {
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
        self.info.draw_menu(ui, self.map.edges());
        if let (RobberInfo::SmallRobberDist, Some(r)) = (self.info.robber_info, self.info.robber()) {
            let r_pos = self.map.positions()[r.marker.nearest_node];
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

    fn update_visible_vertices(&mut self) {
        let to_screen = self.info.camera.to_screen.move_rect;
        let mut visible = std::mem::take(&mut self.info.visible);
        visible.clear();
        for &pos in self.map.positions() {
            let screen_pos = self.info.camera.to_screen_2d(pos);
            visible.push(to_screen.to().contains(screen_pos));
        }
        self.info.visible = visible;
    }

    fn draw_edges(&self, painter: &Painter, scale: f32) {
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
                self.info.camera.to_screen_2d(p1), 
                self.info.camera.to_screen_2d(p2)];
            let line = Shape::LineSegment { points: edge, stroke: edge_stroke(v1, v2) };
            painter.add(line);
        });
    }

    fn draw_characters(&mut self, ui: &mut Ui, response: &Response, painter: &Painter, scale: f32) 
    {
        let to_screen = self.info.camera.to_screen.move_rect;
        for (i, ch) in self.info.characters.iter_mut().enumerate() {
            if ch.marker.dragging {
                ch.update_2d(self.tolerance, &self.map, &mut self.info.queue);
            }
            let node_pos = self.map.positions()[ch.marker.nearest_node];
            let moved = ch.drag_and_draw(response, painter, ui, to_screen, node_pos, scale);
            if moved {
                self.info.past_moves.push(i);
                self.info.future_moves.clear();
            }
        }
    }

    fn draw_markers(&mut self, ui: &mut Ui, response: &Response, painter: &Painter, scale: f32)
    {
        let to_screen = self.info.camera.to_screen.move_rect;
        for m in self.info.markers.iter_mut() {
            if m.dragging {
                m.update_2d(self.tolerance, &self.map);
            }
            let node_pos = self.map.positions()[m.nearest_node];
            m.drag_and_draw(response, painter, ui, to_screen, node_pos, scale * 0.7, None);
        }
    }

    pub fn draw_graph(&mut self, ui: &mut Ui) {
        self.info.maybe_update(self.map.edges(), self.extreme_vertices.iter().map(|&v| v));

        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover());   
        self.info.process_input_2d(ui, &response, self.map.edges()); 
        let scale = self.info.camera.zoom * f32::min(12.0 / self.map_radius as f32, 4.0);

        self.update_visible_vertices();

        let to_screen = |p: Pos2| self.info.camera.to_screen_2d(p);

        self.draw_edges(&painter, scale);
        let positions = self.map.positions();
        self.info.draw_convex_cop_hull(positions, &painter, to_screen, scale);
        self.info.draw_green_circles(positions, &painter, to_screen, scale, self.map_radius);
        self.info.draw_character_tails(positions, &painter, to_screen, scale);
        self.info.draw_robber_strat(self.map.edges(), positions, &painter, to_screen, scale);
        self.info.draw_numbers(positions, ui, &painter, to_screen, scale);

        self.draw_markers(ui, &response, &painter, scale);
        self.draw_characters(ui, &response, &painter, scale);
    }
}
