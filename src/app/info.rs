

use std::collections::VecDeque;

use itertools::{izip, Itertools};

use egui::{*, epaint::TextShape, text::LayoutJob};

use crate::graph::{InSet, EdgeList};

use super::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RobberInfo { None, EscapableNodes, NearNodes, SmallRobberDist, CopDist }

impl RobberInfo {
    pub fn scale_small_dist_with_radius(dist: isize, radius: isize) -> isize {
        (dist * radius) / 100
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum DrawNumbers { None, Indices, RobberAdvantage, MinCopDist }


#[derive(Clone, Copy, PartialEq)]
pub enum RobberStrat { None, EscapeHull }

pub struct InfoState {
    //state kept for each node in map
    pub in_convex_cop_hull: Vec<InSet>, 
    pub min_cop_dist: Vec<isize>, //elementwise minimum of .distance of active cops in self.characters
    pub cop_advantage: Vec<isize>,
    pub visible: Vec<bool>,

    //indices of vertices just inside hull, ordered s.t. two following vertices are neighbors
    pub convex_hull_boundary: Vec<usize>,

    pub queue: VecDeque<usize>, //kept permanentely to reduce allocations when an update of some info per vertex is computed.
    
    pub characters: Vec<Character>,
    pub past_moves: Vec<usize>, //present is at end (same below)
    pub future_moves: Vec<(usize, usize)>, //fst is character index, snd is vertex index

    pub markers: Vec<Marker>,

    pub camera: Camera3D,

    pub robber_info: RobberInfo,
    //both are only used, when the respective RobberInfo is active
    pub marked_cop_dist: isize, //determines cop dist marked in RobberInfo::CopDist
    pub small_robber_dist: isize, //determines max dist marked in RobberInfo::SmallRobberDist

    pub robber_strat: RobberStrat,
    pub vertex_info: DrawNumbers,
    pub show_convex_hull: bool,
    pub show_steps: bool,
    pub debug_info: bool,
}

impl InfoState {

    fn last_moved(&self) -> Option<&Character> {
        self.past_moves.last().map(|&c| &self.characters[c])
    }

    fn next_moved(&self) -> Option<&Character> {
        self.future_moves.last().map(|(c, _)| &self.characters[*c])
    }

    fn reverse_move(&mut self, edges: &EdgeList) {
        if let Some(i) = self.past_moves.pop() {
            let ch = &mut self.characters[i];
            if let Some(v_curr) = ch.last_positions.pop() {
                self.future_moves.push((i, v_curr));
                if let Some(&v_last) = ch.last_positions.last() {
                    ch.marker.nearest_node = v_last;
                    ch.update_distances(edges, &mut self.queue);
                }
            }
        }
    }

    fn redo_move(&mut self, edges: &EdgeList) {
        if let Some((i, v)) = self.future_moves.pop() {
            self.past_moves.push(i);
            let ch = &mut self.characters[i];
            ch.last_positions.push(v);
            ch.marker.nearest_node = v;
            ch.update_distances(edges, &mut self.queue);
        }
    }

    pub fn new() -> Self {
        Self { 
            in_convex_cop_hull: Vec::new(),
            min_cop_dist: Vec::new(),
            cop_advantage: Vec::new(),
            visible: Vec::new(),

            convex_hull_boundary: Vec::new(),

            queue: VecDeque::new(),
            
            characters: vec![
                Character::new(&ROBBER, Pos2::ZERO),
                Character::new(&COPS[0], pos2(0.25, 0.0)),
                ],
            past_moves: Vec::new(),
            future_moves: Vec::new(),

            markers: Vec::new(),

            camera: Camera3D::new(),

            robber_info: RobberInfo::None,
            small_robber_dist: 10,
            marked_cop_dist: 0,

            robber_strat: RobberStrat::None,
            vertex_info: DrawNumbers::None,
            show_convex_hull: false,
            show_steps: false,
            debug_info: false,
        }
    }

    pub fn robber(&self) -> Option<&Character> {
        self.characters.first()
    }

    fn active_cops(&self) -> impl Iterator<Item = &Character> {
        let start = usize::min(1, self.characters.len());
        self.characters[start..].iter().filter(|&c| c.marker.on_node)
    }

    pub fn forget_move_history(&mut self) {
        self.past_moves.clear();
        self.future_moves.clear();
        for ch in &mut self.characters {
            ch.last_positions.clear();
        }
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, edges: &EdgeList) {
        if ui.button("🏠 Position").clicked() {
            self.camera.reset();
        }
        ui.collapsing("Knoteninfo", |ui|{                    
            //settings to draw extra information
            ui.radio_value(&mut self.robber_info, RobberInfo::None, 
                "keine Marker");

            ui.radio_value(&mut self.robber_info, RobberInfo::NearNodes, 
                "markiere für Räuber\nnähere Knoten");

            ui.radio_value(&mut self.robber_info, RobberInfo::EscapableNodes, 
                "markiere Punkte mit\ndirekter Fluchtoption");

            ui.radio_value(&mut self.robber_info, RobberInfo::SmallRobberDist, 
                "markiere Punkte nah\nan Räuber");
            if self.robber_info == RobberInfo::SmallRobberDist {
                add_drag_value(ui, &mut self.small_robber_dist, "% Radius: ", 1, 100);
            }

            ui.radio_value(&mut self.robber_info, RobberInfo::CopDist, 
                "markiere Punkte mit\nAbstand zu Cops");
            if self.robber_info == RobberInfo::CopDist {
                add_drag_value(ui, &mut self.marked_cop_dist, "Abstand: ", 0, 1000);
            }

            ui.add(Checkbox::new(&mut self.show_convex_hull, "zeige \"Konvexe Hülle\"\n um Cops"));   
            ui.add(Checkbox::new(&mut self.debug_info, "bunte Kanten"));
        });
        ui.collapsing("Zahlen", |ui|{
            ui.radio_value(&mut self.vertex_info, DrawNumbers::None, 
                "Keine");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::Indices, 
                "Knotenindizes");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::RobberAdvantage, 
                "Räubervorteil");
            ui.radio_value(&mut self.vertex_info, DrawNumbers::MinCopDist, 
                "minimaler Cop Abstand");
        });
        ui.collapsing("Strategie Räuber", |ui|{
            ui.radio_value(&mut self.robber_strat, RobberStrat::None, 
                "Keine");
            ui.radio_value(&mut self.robber_strat, RobberStrat::EscapeHull, 
                "Entkomme Hülle");
        });
        ui.horizontal(|ui| {
            let nr_characters = self.characters.len();
            let minus_emoji = self.characters.last().map_or("🚫", |c| c.marker.data.emoji);
            let next_data = if nr_characters == 0 { 
                &ROBBER 
            } else {
                let next_cop = (nr_characters - 1) % 7;
                &COPS[next_cop]
            };
            let minus_text = format!("- Figur ({})", minus_emoji);
            let plus_text = format!("+ Figur ({})", next_data.emoji);
            if ui.button(minus_text).clicked() {
                self.characters.pop();
                self.forget_move_history();
            }
            if ui.button(plus_text).clicked() {
                self.characters.push(Character::new(next_data, Pos2::ZERO));
            }
        });
        ui.horizontal(|ui| {
            ui.add(Checkbox::new(&mut self.show_steps, "zeige Schritte"));
            if ui.button("Reset").clicked() {
                self.forget_move_history();
            }
        });        
        ui.horizontal(|ui| {
            if ui.button(" ⟲ ").on_hover_text("strg + z").clicked() {
                self.reverse_move(edges);
            }
            if ui.button(" ⟳ ").on_hover_text("strg + y").clicked() {
                self.redo_move(edges);
            }
        });
        if let Some(ch) = self.last_moved() {
            ui.label(format!("letzter Schritt: {} ({})", ch.marker.data.job, ch.marker.data.emoji));
        }
        if let Some(ch) = self.next_moved() {
            ui.label(format!("nächster Schritt: {} ({})", ch.marker.data.job, ch.marker.data.emoji));
        }
        ui.horizontal(|ui| {
            if ui.button("- Marker").on_hover_text("n an 🖱").clicked() {
                self.markers.pop();
            }
            if ui.button("+ Marker").on_hover_text("m an 🖱").clicked() {
                self.markers.push(Marker::new(&MARKER, Pos2::ZERO));
            }
        });

    }

    pub fn update_min_cop_dist(&mut self) {
        let mut min_cop_dist = std::mem::take(&mut self.min_cop_dist);
        min_cop_dist.clear();
        {
            let mut active_cops = self.active_cops();
            if let Some(cop) = active_cops.next() {
                min_cop_dist.clone_from(&cop.distances);
            }
            for cop in active_cops {
                for (this, curr_min) in izip!(&cop.distances, &mut min_cop_dist) {
                    if this < curr_min {
                        *curr_min = *this;
                    }
                }
            }
        }
        self.min_cop_dist = min_cop_dist;
    }

    pub fn update_cop_advantage(&mut self, edges: &EdgeList) {
        let in_cop_hull = &self.in_convex_cop_hull;
        let mut advantage = std::mem::take(&mut self.cop_advantage);
        advantage.resize(edges.nr_vertices(), isize::MAX);        
        let mut queue = std::mem::take(&mut self.queue);
        queue.clear();

        let zipped = itertools::izip!(0.., 
            in_cop_hull.iter(), 
            advantage.iter_mut(), 
            self.min_cop_dist.iter(), 
            edges.neighbors());
        for (node, &in_hull, adv, &cop_dist, mut neighs) in zipped {
            if !in_hull.yes() && neighs.any(|n| in_cop_hull[n].yes()) {
                queue.push_back(node);
            }
            *adv = if in_hull.yes() { isize::MAX } else { -cop_dist };
        }
        edges.calc_distances_to(&mut queue, &mut advantage);
        self.cop_advantage = advantage;
        self.queue = queue;
    }

    pub fn update_convex_hull_boundary(&mut self, edges: &EdgeList) {
        //idea: walk outside hull and look inside, but actually store the inside nodes
        //this approach will fail, if one cop stands at the border of the map, but this is something
        //2d no happening in the intresting real 3d case anyway.
        let mut boundary = std::mem::take(&mut self.convex_hull_boundary);
        boundary.clear();
        let fst_outside = 'search_start_vertex: loop {
            for cop in self.active_cops() {
                for n in edges.neighbors_of(cop.marker.nearest_node) {
                    if !self.in_convex_cop_hull[n].yes() {
                        boundary.push(cop.marker.nearest_node);
                        break 'search_start_vertex n;
                    }
                }
            }
            return;
        };
        let mut last_outside = usize::MAX;
        let mut curr_outside = fst_outside;
        loop {
            let mut change = false;
            for n in edges.neighbors_of(curr_outside) {
                //find vertices on hull boundary
                if self.in_convex_cop_hull[n].yes() {
                    let len = boundary.len();
                    let range_min = if len < 5 { 0 } else { len - 5 };
                    if !boundary[range_min..].contains(&n) {
                        boundary.push(n);
                    }
                }
                else { //find next vertex just outside hull
                    let still_searching = !change;
                    let is_new = n != last_outside; //can only be trusted if still_searching
                    let borders_hull = edges.neighbors_of(n).any(|nn| self.in_convex_cop_hull[nn].yes());
                    if still_searching && is_new && borders_hull {
                        last_outside = curr_outside;
                        curr_outside = n;
                        self.convex_hull_boundary.push(curr_outside);
                        change = true;
                    }
                }
            }
            if curr_outside == fst_outside || !change {
                break;
            }
        }
        self.convex_hull_boundary = boundary;
    }

    pub fn update_convex_cop_hull(&mut self, edges: &EdgeList, extreme_vertices: impl Iterator<Item = usize>) {
        self.queue.clear();

        let in_hull = &mut self.in_convex_cop_hull;
        in_hull.clear();
        in_hull.resize(edges.nr_vertices(), InSet::Perhaps);
        for i in 1..self.characters.len() {
            let cop_i = &self.characters[i];
            if !cop_i.marker.on_node {
                continue;
            }
            for cop_j in self.characters[(i + 1)..].iter().filter(|c| c.marker.on_node) {                
                //walk from cop i to cop j on all shortest paths
                in_hull[cop_i.marker.nearest_node] = InSet::NewlyAdded;
                self.queue.push_back(cop_i.marker.nearest_node);
                while let Some(node) = self.queue.pop_front() {
                    let curr_dist_to_j = cop_j.distances[node];
                    for neigh in edges.neighbors_of(node) {
                        if cop_j.distances[neigh] < curr_dist_to_j && in_hull[neigh] != InSet::NewlyAdded {
                            in_hull[neigh] = InSet::NewlyAdded;
                            self.queue.push_back(neigh);
                        }
                    }
                }
                //change these paths from InSet::NewlyAdded to InSet::Yes
                //(to allow new paths to go through the current one)
                self.queue.push_back(cop_i.marker.nearest_node);
                in_hull[cop_i.marker.nearest_node] = InSet::Yes;
                edges.recolor_region((InSet::NewlyAdded, InSet::Yes), in_hull, &mut self.queue);
            }
        }
        //color outside as InSet::No (note: this might miss some edge cases; best to not place cops at rim)
        for p in extreme_vertices {
            if in_hull[p] == InSet::Perhaps {
                in_hull[p] = InSet::No;
                self.queue.push_back(p);
            }
        }
        edges.recolor_region((InSet::Perhaps, InSet::No), in_hull, &mut self.queue);

        //color remaining InSet::Perhaps as InSet::Yes
        for x in in_hull {
            if *x == InSet::Perhaps {
                *x = InSet::Yes;
            }
        }
        self.update_convex_hull_boundary(edges);
    }

    pub fn maybe_update(&mut self, edges: &EdgeList, extreme_vertices: impl Iterator<Item = usize>) {
        let require_update = self.show_convex_hull 
            || self.robber_strat != RobberStrat::None
            || self.robber_info != RobberInfo::None 
            || self.vertex_info != DrawNumbers::None;
        if require_update {
            self.update_convex_cop_hull(edges, extreme_vertices);
            self.update_min_cop_dist();
            self.update_cop_advantage(edges);
        }
    }

    pub fn draw_convex_cop_hull<T, F>(&self, positions: &[T], painter: &Painter, mut to_screen: F, scale: f32) 
    where T: Copy, F: FnMut(T) -> Pos2
    {
        if !self.show_convex_hull {
            return;
        }
        let iter = izip!(&self.in_convex_cop_hull, positions, &self.visible);
        for (&in_hull, &pos, &vis) in iter {
            if vis && in_hull.yes()  {
                let draw_pos = to_screen(pos);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 9.0, LIGHT_BLUE);
                painter.add(marker_circle);
            }
        }
        for &v in &self.convex_hull_boundary {
            if self.visible[v] {
                let draw_pos = to_screen(positions[v]);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 3.0, WHITE);
                painter.add(marker_circle);
            }
        }
    }
    

    pub fn draw_green_circles<T, F>(&self, positions: &[T], painter: &Painter, mut to_screen: F, scale: f32, radius: isize) 
    where T: Copy, F: FnMut(T) -> Pos2
    {
        let mut draw_circle_at = |pos: T|{
            let draw_pos = to_screen(pos);
            let marker_circle = Shape::circle_filled(draw_pos, scale * 6.0, GREEN);
            painter.add(marker_circle);
        };
        match (self.robber_info, self.robber()) {
            (RobberInfo::NearNodes, Some(r)) => 
                for (r_dist, c_dist, &pos, &vis) in 
                izip!(&r.distances, &self.min_cop_dist, positions, &self.visible) {
                    if vis && r_dist < c_dist {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::EscapableNodes, _) => 
                for (&adv, &pos, &vis) in izip!(&self.cop_advantage, positions, &self.visible) {
                    if vis && adv < -1 {
                        draw_circle_at(pos);
                    }
                },
            (RobberInfo::SmallRobberDist, Some(r)) => {
                let bnd = RobberInfo::scale_small_dist_with_radius(self.small_robber_dist, radius);
                for (&dist, &pos, &vis) in izip!(&r.distances, positions, &self.visible) {
                    if vis && dist <= bnd {
                        draw_circle_at(pos);
                    }
                }
            },
            (RobberInfo::CopDist, _) => 
            for (&dist, &pos, &vis) in izip!(&self.min_cop_dist, positions, &self.visible) {
                if vis && dist == self.marked_cop_dist {
                    draw_circle_at(pos);
                }
            }
            _ => {},
        }
    }    

    pub fn draw_numbers<T, F>(&self, positions: &[T], ui: &Ui, painter: &Painter, mut to_screen: F, scale: f32) 
    where T: Copy, F: FnMut(T) -> Pos2
    {
        if self.vertex_info == DrawNumbers::None {
            return;
        }
        let font = FontId::proportional(scale * 8.0);
        let color = if ui.ctx().style().visuals.dark_mode { WHITE } else { BLACK };
        for (i, &pos, &vis) in izip!(0.., positions, &self.visible) {
            if vis {
                let txt = match self.vertex_info {
                    DrawNumbers::Indices => { i.to_string() }
                    DrawNumbers::MinCopDist => { self.min_cop_dist[i].to_string() }
                    DrawNumbers::None => { panic!() }
                    DrawNumbers::RobberAdvantage => { (-1 -self.cop_advantage[i]).to_string() }
                };
                let mut layout_job = LayoutJob::simple(txt, font.clone(), color, 100.0 * scale);
                layout_job.halign = Align::Center;
                let galley = ui.fonts(|f| f.layout_job(layout_job));
                let screen_pos = to_screen(pos);
                let text = Shape::Text(TextShape::new(screen_pos, galley));
                painter.add(text);
            }
        }
    }

    pub fn draw_robber_strat<T, F>(&self, edges: &EdgeList, positions: &[T], painter: &Painter, 
        mut to_screen: F, scale: f32) 
    where T: Copy, F: FnMut(T) -> Pos2
    {
        if self.robber_strat == RobberStrat::None {
            return;
        }
        let potential = |v| match self.robber_strat {
            RobberStrat::None => panic!(),
            RobberStrat::EscapeHull => {
                //the robber will always try to increase the robber advantage, except for when that would lead to beeing
                //catched in the next move
                let min_cop_dist = self.min_cop_dist[v];
                if min_cop_dist < 2 {
                    10000.0
                }
                else {
                    //if multiple vertices have the same robber advantage, we use the min_cop_dist as tiebreaker.
                    //(because all vertices v tested are neighbors of the current robber position, their cop advantage
                    //  and min_cop_dist can vary between all of them by at most 2. thus 0.1 is sufficiently small
                    //  as factor to only let the min_cop_dist work as tiebreaker)
                    self.cop_advantage[v] as f32 - 0.1 * min_cop_dist as f32
                }
            }
        };
        if let Some(r) = self.robber() {
            let mut best = Vec::with_capacity(10);   
            best.push(r.marker.nearest_node);
            let mut smallest_pot = potential(r.marker.nearest_node);
            for neigh in edges.neighbors_of(r.marker.nearest_node) {
                let neigh_pot = potential(neigh);
                if neigh_pot < smallest_pot {
                    best.clear();
                    best.push(neigh);
                    smallest_pot = neigh_pot;
                }
                else if neigh_pot == smallest_pot {
                    best.push(neigh);
                }
            }
            for v in best {
                let draw_pos = to_screen(positions[v]);
                let marker_circle = Shape::circle_filled(draw_pos, scale * 4.0, RED);
                painter.add(marker_circle);
            }
        }
    }

    pub fn draw_character_tails<T, F>(&self, positions: &[T], painter: &Painter, mut to_screen: F, scale: f32)
    where T: Copy, F: FnMut(T) -> Pos2 
    {
        if !self.show_steps {
            return;
        }
        for ch in &self.characters {
            let f_len = ch.last_positions.len() as f32;
            for (i, (&v1, &v2)) in ch.last_positions.iter().tuple_windows().enumerate() { 
                if !self.visible[v1] {
                    continue;
                }      
                let draw_pos_1 = to_screen(positions[v1]);
                let size = scale * 2.5 * (i as f32 + 0.8 * f_len) / f_len;
                let marker_circle = Shape::circle_filled(draw_pos_1, size, ch.marker.data.glow);
                painter.add(marker_circle);
                if !self.visible[v2] {
                    continue;
                } 
                let points = [draw_pos_1, to_screen(positions[v2])];
                let stroke = Stroke::new(size * 0.5, ch.marker.data.glow);
                let line = Shape::LineSegment { points, stroke };
                painter.add(line);
            }
        }
    }

    fn add_marker_at(&mut self, screen_pos: Pos2) {
        let pos = self.camera.to_screen.move_rect.inverse().transform_pos(screen_pos);
        self.markers.push(Marker::new(&MARKER, pos));
    }

    fn remove_marker_at(&mut self, screen_pos: Pos2) {
        let pos = self.camera.to_screen.move_rect.inverse().transform_pos(screen_pos);
        let mut best_i = usize::MAX;
        let mut best_dist = f32::MAX;
        for (i, m) in self.markers.iter().enumerate() {
            if m.on_node && !self.visible[m.nearest_node] {
                continue;
            }
            let m_dist = (pos - m.pos2).length();
            if m_dist < best_dist {
                best_i = i;
                best_dist = m_dist;
            }
        }
        if best_i != usize::MAX {
            self.markers.remove(best_i);
        }
    }

    pub fn process_general_input(&mut self, ui: &mut Ui, edges: &EdgeList) {
        ui.input(|info| {
            if info.modifiers.ctrl && info.key_pressed(Key::Z) {
                self.reverse_move(edges);
            }
            if info.modifiers.ctrl && info.key_pressed(Key::Y) {
                self.redo_move(edges);
            }  
            if info.key_pressed(Key::M) {
                if let Some(pointer_pos) = info.pointer.latest_pos() {
                    self.add_marker_at(pointer_pos);
                }
            }  
            if info.key_pressed(Key::N) {
                if let Some(pointer_pos) = info.pointer.latest_pos() {
                    self.remove_marker_at(pointer_pos);
                }
            }        
        });
    }

    /// zoom changes happen with the cursor position as fixed point, thus 
    /// with zooming we also change the offset
    pub fn process_input_2d(&mut self, ui: &mut Ui, response: &Response, edges: &EdgeList) {
        self.process_general_input(ui, edges);
        self.camera.update_2d(ui, response.rect);
    }

    /// zoom changes don't change the offset at all
    pub fn process_input_3d(&mut self, ui: &mut Ui, response: &Response, edges: &EdgeList) {
        self.process_general_input(ui, edges);
        self.camera.update_3d(ui, response.rect);
    }
}  
