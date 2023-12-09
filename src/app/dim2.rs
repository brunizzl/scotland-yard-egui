
use egui::{*, epaint::TextShape, text::LayoutJob};

use crate::{ graph::{self, Embedding2D}, app::* };

#[derive(Clone, Copy, PartialEq)]
pub enum GraphShape { RegularPolygon(usize), Random, Debug }

pub struct State {
    map: Embedding2D,

    //overall map shape: == 2 is line, == 3 triangle, == 4 square, == 5 pentagon etc.
    map_shape: GraphShape,
    map_radius: usize, //(approx.) shortest path length from origin to rim

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
        self.map = match self.map_shape {
            GraphShape::RegularPolygon(n) => graph::triangulated_regular_polygon(
                n, 
                self.map_radius),
            GraphShape::Random => graph::random_triangulated(self.map_radius, 8),
            GraphShape::Debug => graph::debugging_graph(),
        };
        self.tolerance = f32::min(0.25, 0.75 / self.map_radius as f32);
        for char in &mut self.info.characters {
            char.update_2d(self.tolerance, &self.map, &mut self.info.queue);
            char.snap_to_node();
        }
        self.extreme_vertices = Self::compute_extreme_vertices(self.map.positions());
        let edges = self.map.edges();
        self.info.update_convex_cop_hull(edges, self.extreme_vertices.iter().map(|&v| v));
        self.info.update_min_cop_dist(edges);
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
        if let (RobberInfo::SmallRobberDist(bnd), Some(r)) = (self.info.robber_info, self.info.robber()) {
            let r_pos = self.map.positions()[r.nearest_node];
            let mut max_dist = f32::MIN;
            let mut min_dist = f32::MAX;
            let bnd = RobberInfo::scale_small_dist_with_radius(bnd, self.map_radius);
            for (&dist, &pos) in r.distances.iter().zip(self.map.positions()) {
                if (dist as usize) == bnd {
                    let new_dist = (r_pos - pos).length();
                    max_dist = f32::max(max_dist, new_dist);
                    min_dist = f32::min(min_dist, new_dist);
                }
            }
            ui.label(format!("min dist:   {}\nmax dist:   {}\nmin / max: {}", 
                min_dist, max_dist, min_dist / max_dist));
        }
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

    fn draw_convex_cop_hull(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        for (&in_hull, &pos) in self.info.in_convex_cop_hull.iter().zip(self.map.positions()) {
            if in_hull.yes()  {
                let draw_pos = to_screen.transform_pos(pos);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 9.0, LIGHT_BLUE);
                painter.add(marker_circle);
            }
        }
    }

    fn draw_green_circles(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {     
        let draw_circle_at = |pos: Pos2|{
            let draw_pos = to_screen.transform_pos(pos);
            let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
            painter.add(marker_circle);
        };
        match (self.info.robber_info, self.info.robber()) {
            (RobberInfo::NearNodes, Some(r)) => 
                for (r_dist, c_dist, &pos) in 
                itertools::izip!(r.distances.iter(), self.info.min_cop_dist.iter(), self.map.positions()) {
                    if r_dist < c_dist {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::EscapableNodes, _) => 
                for (&adv, &pos) in self.info.cop_advantage.iter().zip(self.map.positions()) {
                    if adv < -1 {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::SmallRobberDist(bnd), Some(r)) => {
                let bnd = RobberInfo::scale_small_dist_with_radius(bnd, self.map_radius);
                for (&dist, &pos) in r.distances.iter().zip(self.map.positions()) {
                    if (dist as usize) <= bnd {
                        draw_circle_at(pos);
                    }
                }
            },
            _ => {},
        }
    }

    fn draw_robber_info(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        if let (RobberInfo::NearNodes, Some(r)) = (self.info.robber_info, self.info.robber()) { 
            let dist_vs = r.distances.iter().zip(self.info.min_cop_dist.iter());
            for ((r_dist, c_dist), &pos) in dist_vs.zip(self.map.positions()) {
                if r_dist < c_dist  {
                    let draw_pos = to_screen.transform_pos(pos);
                    let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
                    painter.add(marker_circle);
                }
            }
        }
        if self.info.robber_info == RobberInfo::EscapableNodes {
            for (&adv, &pos) in self.info.cop_advantage.iter().zip(self.map.positions()) {
                if adv < -1  {
                    let draw_pos = to_screen.transform_pos(pos);
                    let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
                    painter.add(marker_circle);
                }
            }
        }
    }

    fn draw_numbers(&self, ui: &mut Ui, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        let font = FontId::proportional(scale * 8.0);
        let color = if ui.ctx().style().visuals.dark_mode { WHITE } else { BLACK };
        for (i, &pos) in self.map.positions().iter().enumerate() {
            let txt = match self.info.vertex_info {
                DrawNumbers::Indices => { i.to_string() }
                DrawNumbers::MinCopDist => { self.info.min_cop_dist[i].to_string() }
                DrawNumbers::None => { panic!() }
                DrawNumbers::RobberAdvantage => { (-1 -self.info.cop_advantage[i]).to_string() }
            };
            let mut layout_job = LayoutJob::simple(txt, font.clone(), color, 100.0 * scale);
            layout_job.halign = Align::Center;
            let galley = ui.fonts(|f| f.layout_job(layout_job));
            let screen_pos = to_screen.transform_pos(pos);
            let text = Shape::Text(TextShape::new(screen_pos, galley));
            painter.add(text);
        }
    }

    fn draw_cop_voronoi(&self, painter: &Painter, to_screen: emath::RectTransform, scale: f32) {
        for (&multiple, &pos) in self.info.muliple_min_dist_cops.iter().zip(self.map.positions()) {
            if multiple  {
                let draw_pos = to_screen.transform_pos(pos);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 5.0, RED);
                painter.add(marker_circle);
            }
        }
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

    /// fst maps graph coordinates to screen, snd defines scale to draw edges etc. at
    fn build_to_screen(&self, response: &Response) -> (emath::RectTransform, f32) {
        let camera = self.camera;
        let rect_min = vec2(-1.0, -1.0) / camera.zoom;
        let rect_max = vec2(1.0, 1.0) / camera.zoom;
        let from = Rect::from_min_max(rect_min.to_pos2(), rect_max.to_pos2());

        let rect_len = f32::min(response.rect.height(), response.rect.width());
        let to_middle = (response.rect.width() - rect_len) / 2.0;
        let screen_min = response.rect.min + vec2(to_middle, 0.0) + camera.offset;
        let to = Rect::from_min_size(screen_min, vec2(rect_len, rect_len));

        let to_screen = emath::RectTransform::from_to(from, to);
        let scale = f32::min(rect_len / self.map_radius as f32 * 0.015, 4.0);
        (to_screen, scale * camera.zoom)
    }

    pub fn draw_graph(&mut self, ui: &mut Ui) {
        self.info.maybe_update(self.map.edges(), self.extreme_vertices.iter().map(|&v| v));

        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover());   
        self.camera.update_cursor_centered(ui, &response); 

        let (to_screen, scale) = self.build_to_screen(&response);

        self.draw_edges(&painter, to_screen, scale);
        if self.info.show_convex_hull {
            self.draw_convex_cop_hull(&painter, to_screen, scale);
        }
        self.draw_green_circles(&painter, to_screen, scale);
        self.draw_robber_info(&painter, to_screen, scale);

        if self.map_radius < 20 && self.info.vertex_info != DrawNumbers::None {
            self.draw_numbers(ui, &painter, to_screen, scale);
        }
        if self.info.show_cop_voronoi {
            self.draw_cop_voronoi(&painter, to_screen, scale);
        }
        self.draw_characters(ui, &response, &painter, to_screen, scale);
    }
}
