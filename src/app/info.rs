

use std::collections::VecDeque;

use itertools::{ izip, Itertools };

use egui::*;

use crate::graph::{EdgeList, ConvexHull, EscapeableNodes, self};
use crate::app::character::CharacterState;

use super::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Deserialize, serde::Serialize)]
pub enum RobberInfo { None, RobberAdvantage, EscapeableNodes, NearNodes, SmallRobberDist, CopDist }

impl RobberInfo {
    pub fn scale_small_dist_with_resolution(dist: isize, radius: isize) -> isize {
        (dist * radius) / 100
    }
}

#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum DrawNumbers { None, Indices, RobberAdvantage, EscapeableNodes, MinCopDist }


#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum RobberStrat { None, EscapeHullNonLazy }

pub struct Info {
    //state kept for each node in map
    cop_hull: ConvexHull,
    escapable: EscapeableNodes,
    min_cop_dist: Vec<isize>, //elementwise minimum of .distance of active cops in self.characters
    cop_advantage: Vec<isize>,
    pub marked_manually: Vec<bool>,

    /// this is only used as intermediary variable during computations. 
    /// to not reallocate between frames/ algorithms, the storage is kept and passed to where needed.
    pub queue: VecDeque<usize>,
    
    pub characters: CharacterState,

    robber_info: RobberInfo,
    //both are only used, when the respective RobberInfo is active
    marked_cop_dist: isize, //determines cop dist marked in RobberInfo::CopDist
    small_robber_dist: isize, //determines max dist marked in RobberInfo::SmallRobberDist

    robber_strat: RobberStrat,
    vertex_info: DrawNumbers,
    show_convex_hull: bool,
    menu_change: bool,
}

mod storage_keys {
    pub const CHARACTERS: &'static str = "app::info::characters";
    pub const MARKED_MANUALLY: &'static str = "app::info::manually_marked";
    pub const ROBBER_INFO: &'static str = "app::info::robber_info";
    pub const ROBBER_STRAT: &'static str = "app::info::robber_strat";
    pub const VERTEX_INFO: &'static str = "app::info::vertex_info";
    pub const SHOW_HULL: &'static str = "app::info::show_convex_hull";
}

impl Info {
    pub fn save(&mut self, storage: &mut dyn eframe::Storage) {
        //idea: only store settings, not state that is recomputable.
        let all_dists = self.characters.all_mut().iter_mut()
            .map(|c| std::mem::take(&mut c.distances)).collect_vec();
        use storage_keys::*;
        eframe::set_value(storage, CHARACTERS, &self.characters);
        for (ch, dists) in izip!(self.characters.all_mut(), all_dists.into_iter()) {
            ch.distances = dists;
        }

        if self.marked_manually.iter().any(|&x| x) {
            eframe::set_value(storage, MARKED_MANUALLY, &self.marked_manually);
        }
        eframe::set_value(storage, ROBBER_INFO, &self.robber_info);
        eframe::set_value(storage, ROBBER_STRAT, &self.robber_strat);
        eframe::set_value(storage, VERTEX_INFO, &self.vertex_info);
        eframe::set_value(storage, SHOW_HULL, &self.show_convex_hull);
    }

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        use storage_keys::*;
        let characters = load_or(cc.storage, CHARACTERS, CharacterState::new);
        let marked_manually = load_or(cc.storage, MARKED_MANUALLY, Vec::new);
        let robber_info = load_or(cc.storage, ROBBER_INFO, || RobberInfo::None);
        let robber_strat = load_or(cc.storage, ROBBER_STRAT, || RobberStrat::None);
        let vertex_info = load_or(cc.storage, VERTEX_INFO, || DrawNumbers::None);
        let show_convex_hull = load_or(cc.storage, SHOW_HULL, || false);

        Self { 
            cop_hull: ConvexHull::new(),
            escapable: EscapeableNodes::new(),
            min_cop_dist: Vec::new(),
            cop_advantage: Vec::new(),
            marked_manually,

            queue: VecDeque::new(),
            
            characters,

            robber_info,
            small_robber_dist: 10,
            marked_cop_dist: 0,

            robber_strat,
            vertex_info,
            show_convex_hull,
            menu_change: false,
        }
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, edges: &EdgeList) {
        self.menu_change = false;
        ui.collapsing("Knoteninfo", |ui|{
            self.menu_change |= 
                ui.add(Checkbox::new(&mut self.show_convex_hull, "zeige Konvexe Hülle um Cops"))
                .on_hover_text("Berechnung des Randes kann im 2D Fall und insbesondere im \
                zufällig triangulierten Fall versagen. Manche Marker brauchen diesen Rand und \
                werden in diesen Fällen dann nicht angezeigt.")
                .changed();

            ui.add_space(5.0);
            ui.label("Marker:");
            let old = self.robber_info;
            //settings to draw extra information
            ui.radio_value(&mut self.robber_info, RobberInfo::None, 
                "Keine");

            ui.radio_value(&mut self.robber_info, RobberInfo::NearNodes, 
                "für Räuber nähere Knoten")
                .on_hover_text("alle Knoten näher am Räuber als am nächsten Cop");

            ui.radio_value(&mut self.robber_info, RobberInfo::RobberAdvantage, 
                "Punkte mit direkter Fluchtoption 1")
                .on_hover_text("alle Punkte in der Konvexen Hülle, \
                die näher an einem Punkt ausserhalb der Hülle sind, als der nächste Cop an diesem Punkt ist");

            ui.radio_value(&mut self.robber_info, RobberInfo::EscapeableNodes, 
                "Punkte mit direkter Fluchtoption 2")
                .on_hover_text("Jedes Paar von benauchbarten Cops am Hüllenrand kontrolliert einen Randbereich. \
                Will der Räuber durch diesen Bereich fliehen, dürfen die Cops in der Zeit, \
                die der Räuber zum Rand braucht, diesen nicht auf Länge 0 kürzen können. \n\
                Markiert werden alle Punkte, die schneller an jedem Punkt des Randabschnittes sind, \
                als die Cops diesen Abschnitt dicht machen können. \n\
                Nicht berücksichtigt für die Berechnung aller Fluchtoptionen zu einem Segment zwischen zwei Cops sind alle anderen Cops. \
                Insbesondere werden Cops im inneren der Hülle zu keinem Zeitpunkt berücksichtigt. \
                (TODO: ändere das)");

            ui.radio_value(&mut self.robber_info, RobberInfo::SmallRobberDist, 
                "Punkte nah an Räuber")
                .on_hover_text("Alle Punkte die Abstand <= Auflösung * (% Auflösung) / 100 zu Räuber haben");
            if self.robber_info == RobberInfo::SmallRobberDist {
                add_drag_value(ui, &mut self.small_robber_dist, "% Auflösung: ", 1, 100);
            }

            ui.radio_value(&mut self.robber_info, RobberInfo::CopDist, 
                "Punkte mit Abstand zu Cops")
                .on_hover_text("Abstand einstellbar bei ausgewählter Option");
            if self.robber_info == RobberInfo::CopDist {
                add_drag_value(ui, &mut self.marked_cop_dist, "Abstand: ", 0, 1000);
            }
            self.menu_change |= old != self.robber_info;
        });
        ui.collapsing("Zahlen", |ui|{
            let old = self.vertex_info;
            ui.radio_value(&mut self.vertex_info, DrawNumbers::None, 
                "Keine");

            ui.radio_value(&mut self.vertex_info, DrawNumbers::Indices, 
                "Knotenindizes")
                .on_hover_text("nur relevant für Debugging");

            ui.radio_value(&mut self.vertex_info, DrawNumbers::RobberAdvantage, 
                "Marker Fluchtoption 1")
                .on_hover_text("Helfer zur Berechnung von Fluchtoption 1");

            ui.radio_value(&mut self.vertex_info, DrawNumbers::EscapeableNodes, 
                "Marker Fluchtoption 2")
                .on_hover_text("jedes benachbarte Cop-Paar auf dem Hüllenrand hat einen Namen in { 0 .. C }. \
                Der Marker listet alle Paare auf, zwischen denen der Räuber durchschlüpfen kann.");

            ui.radio_value(&mut self.vertex_info, DrawNumbers::MinCopDist, 
                "minimaler Cop Abstand")
                .on_hover_text("punktweises Minimum aus den Abständen aller Cops");
            self.menu_change |= old != self.vertex_info;
        });
        ui.collapsing("Strategie Räuber", |ui|{
            let old = self.robber_strat;
            ui.radio_value(&mut self.robber_strat, RobberStrat::None, 
                "Keine");

            ui.radio_value(&mut self.robber_strat, RobberStrat::EscapeHullNonLazy, 
                "Entkomme Hülle")
                .on_hover_text("Heuristik für Fluchtoption (1)");
            self.menu_change |= old != self.robber_strat;
        });
        self.characters.draw_menu(ui, edges, &mut self.queue);
    }

    fn update_min_cop_dist(&mut self, edges: &EdgeList) {
        let mut min_cop_dist = std::mem::take(&mut self.min_cop_dist);
        min_cop_dist.clear();
        let mut active_cops = self.characters.active_cops();
        if let Some(cop) = active_cops.next() {
            min_cop_dist.clone_from(&cop.distances);
        }
        else {
            min_cop_dist.resize(edges.nr_vertices(), isize::MAX);
        }
        for cop in active_cops {
            for (this, curr_min) in izip!(&cop.distances, &mut min_cop_dist) {
                if this < curr_min {
                    *curr_min = *this;
                }
            }
        }
        self.min_cop_dist = min_cop_dist;
    }

    fn update_cop_advantage(&mut self, edges: &EdgeList) {
        let in_cop_hull = &self.cop_hull.inside();
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

    fn update_convex_cop_hull(&mut self, con: &DrawContext<'_>) {
        let mut temp = [usize::MAX];
        let vertices_outside_hull = if con.extreme_vertices.len() > 0 {
            con.extreme_vertices
        }
        else {
            debug_assert_eq!(con.positions.len(), self.min_cop_dist.len());
            let (furthest_vertex, _) = self.min_cop_dist.iter().enumerate()
                .fold((0, 0), |best, (v, &dist)| if dist > best.1 { (v, dist) } else { best });
            temp[0] = furthest_vertex;
            &temp
        };

        self.cop_hull.update(
            &self.characters.cops(), 
            con.edges, 
            &self.min_cop_dist,
            &mut self.queue, 
            vertices_outside_hull,
        );
    }

    fn update_escapable(&mut self, con: &DrawContext<'_>) {
        self.escapable.update(&self.cop_hull, con.edges, &mut self.queue)
    }

    /// recomputes everything
    fn definitely_update(&mut self, con: &DrawContext<'_>) {
        self.update_min_cop_dist(con.edges);
        self.update_convex_cop_hull(con);
        self.update_escapable(con);
        self.update_cop_advantage(con.edges);
    }

    /// recomputes only things currently shown or required by things currently shown
    /// and only if something relevant (e.g. a cop's position) changed
    fn maybe_update(&mut self, con: &DrawContext<'_>) {
        self.characters.update(con, &mut self.queue);
        let robber_moved = self.characters.robber_updated();
        let cop_moved = self.characters.active_cop_updated();

        let update_cop_advantage = self.robber_info == RobberInfo::RobberAdvantage
            || self.vertex_info == DrawNumbers::RobberAdvantage;
            
        let update_escapable = self.robber_info == RobberInfo::EscapeableNodes
            || self.vertex_info == DrawNumbers::EscapeableNodes;
        
        let update_hull = update_cop_advantage 
            || update_escapable 
            || self.show_convex_hull;

        let update_min_cop_dist = update_hull
            || self.robber_info == RobberInfo::CopDist
            || self.robber_info == RobberInfo::NearNodes
            || self.vertex_info == DrawNumbers::MinCopDist;

        let nr_vertices = con.edges.nr_vertices();
        if (cop_moved || self.min_cop_dist.len() != nr_vertices) && update_min_cop_dist {
            self.update_min_cop_dist(con.edges);
        }
        if (cop_moved || self.cop_hull.inside().len() != nr_vertices) && update_hull {
            self.update_convex_cop_hull(con);
        }
        if (cop_moved || self.escapable.escapable().len() != nr_vertices) && update_escapable {
            self.update_escapable(con);
        }
        if (cop_moved || robber_moved || self.cop_advantage.len() != nr_vertices) && update_cop_advantage {
            self.update_cop_advantage(con.edges);
        }
    }
    
    fn change_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>, new_val: bool) {
        if con.positions.len() > 0 {
            let to_screen_pos = |v| con.vertex_draw_pos(v);
            let (best_vertex, _) = graph::find_nearest_node(con.visible, con.edges, screen_pos, to_screen_pos, 0);
            self.marked_manually[best_vertex] = new_val;
        }
    } 
    
    fn add_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>) {
        self.change_marker_at(screen_pos, con, true)
    }
    
    fn remove_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>) {
        self.change_marker_at(screen_pos, con, false)
    }

    pub fn process_general_input(&mut self, ui: &mut Ui, con: &DrawContext<'_>) {
        
        ui.input(|info| {
            if info.modifiers.ctrl && info.key_pressed(Key::Z) {
                self.characters.reverse_move(con.edges, &mut self.queue);
            }
            if info.modifiers.ctrl && info.key_pressed(Key::Y) {
                self.characters.redo_move(con.edges, &mut self.queue);
            }  
            if info.key_pressed(Key::M) {
                if let Some(pointer_pos) = info.pointer.latest_pos() {
                    self.add_marker_at(pointer_pos, con);
                }
            }  
            if info.key_pressed(Key::N) {
                if let Some(pointer_pos) = info.pointer.latest_pos() {
                    self.remove_marker_at(pointer_pos, con);
                }
            }        
        });
    }

    fn draw_convex_cop_hull(&self, con: &DrawContext<'_>) {
        if !self.show_convex_hull {
            return;
        }
        for (&in_hull, &pos, &vis) in izip!(self.cop_hull.inside(), con.positions, con.visible) {
            if vis && in_hull.yes()  {
                let draw_pos = con.cam.transform(pos);
                let marker_circle = Shape::circle_filled(draw_pos, con.scale * 9.0, LIGHT_BLUE);
                con.painter.add(marker_circle);
            }
        }
        for seg in self.cop_hull.boundary_segments() {
            for &v in seg {
                if con.visible[v] {
                    let draw_pos = con.vertex_draw_pos(v);
                    let marker_circle = Shape::circle_filled(draw_pos, con.scale * 2.0, WHITE);
                    con.painter.add(marker_circle);
                }
            }
        }
    } 

    fn draw_green_circles(&self, con: &DrawContext<'_>) {
        let random_color = |seed| {
            let mut gen = crate::rand::LCG::new(seed as u64);
            gen.waste(3);
            let mut rnd = || (gen.next() % 128) as u8;

            //idea: hit at least one 100 in every view of length 3
            const BRIGHT: [u8; 8] = [0, 100, 0, 0, 100, 100, 0, 100];
            let off = rnd() as usize % (BRIGHT.len() - 2);
            let [b0, b1, b2, ..] = BRIGHT[off..] else { panic!() };

            Color32::from_rgb(rnd() + b0, rnd() + b1, rnd() + b2)
        };
        let draw_circle_at = |pos, color|{
            let draw_pos = con.cam.transform(pos);
            let marker_circle = Shape::circle_filled(draw_pos, con.scale * 6.0, color);
            con.painter.add(marker_circle);
        };
        match (self.robber_info, self.characters.robber()) {
            (RobberInfo::NearNodes, Some(r)) => 
                for (r_dist, c_dist, &pos, &vis) in 
                izip!(&r.distances, &self.min_cop_dist, con.positions, con.visible) {
                    if vis && r_dist < c_dist {
                        draw_circle_at(pos, GREEN);
                    }
                },
            (RobberInfo::RobberAdvantage, _) => 
                for (&adv, &pos, &vis, &hull) in 
                izip!(&self.cop_advantage, con.positions, con.visible, self.cop_hull.inside()) {
                    if vis && hull.yes() && adv < -1 {
                        draw_circle_at(pos, GREEN);
                    }
                },
            (RobberInfo::SmallRobberDist, Some(r)) => {
                let bnd = RobberInfo::scale_small_dist_with_resolution(
                    self.small_robber_dist, con.resolution);
                for (&dist, &pos, &vis) in izip!(&r.distances, con.positions, con.visible) {
                    if vis && dist <= bnd {
                        draw_circle_at(pos, GREEN);
                    }
                }
            },
            (RobberInfo::CopDist, _) => 
            for (&dist, &pos, &vis) in izip!(&self.min_cop_dist, con.positions, con.visible) {
                if vis && dist == self.marked_cop_dist {
                    draw_circle_at(pos, GREEN);
                }
            }
            (RobberInfo::EscapeableNodes, _) => 
            for (&esc, &pos, &vis) in izip!(self.escapable.escapable(), con.positions, con.visible) {
                if vis && esc != 0 {
                    draw_circle_at(pos, random_color(esc));
                }
            }
            _ => {},
        }
    }    

    fn draw_numbers(&self, ui: &Ui, con: &DrawContext<'_>) {
        if self.vertex_info == DrawNumbers::None {
            return;
        }
        let true_bits = |x:u16| -> String {
            const HEX_CHARS: [char; 16] = 
                ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'];
            let pow2 = (0..16).map(|i| 2u16.pow(i));
            izip!(HEX_CHARS, pow2).filter_map(|(c, i)| (i & x != 0).then_some(c)).collect()
        };
        let font = FontId::proportional(con.scale * 8.0);
        let color = if ui.ctx().style().visuals.dark_mode { WHITE } else { BLACK };
        for (i, &pos, &vis) in izip!(0.., con.positions, con.visible) {
            if vis {
                let txt = match self.vertex_info {
                    DrawNumbers::Indices => { i.to_string() }
                    DrawNumbers::MinCopDist => { self.min_cop_dist[i].to_string() }
                    DrawNumbers::None => { panic!() }
                    DrawNumbers::RobberAdvantage => { (-1 -self.cop_advantage[i]).to_string() }
                    DrawNumbers::EscapeableNodes => { true_bits(self.escapable.escapable()[i]) }
                };
                let mut layout_job = LayoutJob::simple(txt, font.clone(), color, 100.0 * con.scale);
                layout_job.halign = Align::Center;
                let galley = ui.fonts(|f| f.layout_job(layout_job));
                let screen_pos = con.cam.transform(pos);
                let text = Shape::Text(TextShape::new(screen_pos, galley));
                con.painter.add(text);
            }
        }
    }

    fn draw_robber_strat(&self, con: &DrawContext<'_>) {
        if self.robber_strat == RobberStrat::None {
            return;
        }
        let potential = |v| match self.robber_strat {
            RobberStrat::None => panic!(),
            RobberStrat::EscapeHullNonLazy => {
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
        if let Some(r) = self.characters.robber() {
            let mut best = Vec::with_capacity(10);   
            best.push(r.nearest_node);
            let mut smallest_pot = potential(r.nearest_node);
            for neigh in con.edges.neighbors_of(r.nearest_node) {
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
                let draw_pos = con.cam.transform(con.positions[v]);
                let marker_circle = Shape::circle_filled(draw_pos, con.scale * 4.0, RED);
                con.painter.add(marker_circle);
            }
        }
    }

    fn draw_character_tails(&self, con: &DrawContext<'_>) {
        if !self.characters.show_steps() {
            return;
        }
        for ch in self.characters.all() {
            let f_len = ch.last_positions.len() as f32;
            for (i, (&v1, &v2)) in ch.last_positions.iter().tuple_windows().enumerate() { 
                if !con.visible[v1] {
                    continue;
                }      
                let draw_pos_1 = con.vertex_draw_pos(v1);
                let size = con.scale * 2.5 * (i as f32 + 0.8 * f_len) / f_len;
                let marker_circle = Shape::circle_filled(draw_pos_1, size, ch.style().glow);
                con.painter.add(marker_circle);
                if !con.visible[v2] {
                    continue;
                } 
                let points = [draw_pos_1, con.vertex_draw_pos(v2)];
                let stroke = Stroke::new(size * 0.5, ch.style().glow);
                let line = Shape::LineSegment { points, stroke };
                con.painter.add(line);
            }
        }
    }

    fn draw_manual_markers(&self, con: &DrawContext<'_>) {
        for (&vis, &marked, &pos) in izip!(con.visible, &self.marked_manually, con.positions) {
            if vis && marked {
                let draw_pos = con.cam.transform(pos);
                let marker_circle = Shape::circle_filled(draw_pos, con.scale * 4.5, YELLOW);
                con.painter.add(marker_circle);
            }
        }
    }

    pub fn update_and_draw(&mut self, ui: &mut Ui, con: &DrawContext<'_>) {
        self.process_general_input(ui, con);
        if self.menu_change { self.definitely_update(con); } else { self.maybe_update(con); }
        self.draw_convex_cop_hull(con);
        self.draw_green_circles(con);
        self.draw_manual_markers(con);
        self.draw_character_tails(con);
        self.draw_robber_strat(con);
        self.draw_numbers(ui, con);
        self.characters.draw(ui, con);
        self.characters.frame_is_finished();
    }
}  
