use std::collections::VecDeque;

use egui::{Color32, Pos2, Ui};
use itertools::izip;

use crate::graph::InSet;

use super::{color, style, DrawContext};

/// maximum number of entries in history
const HISTORY_LEN: usize = 32;

#[derive(Clone, Copy, PartialEq, Eq)]
enum DrawTool {
    None,
    Draw,
    Erase,
    Paintbucket,
    SetOp,
}

impl DrawTool {
    fn symbol(&self) -> &'static str {
        match self {
            Self::None => " ",
            Self::Draw => "✏",
            Self::Erase => "📗",
            Self::Paintbucket => "💦",
            Self::SetOp => "✨",
        }
    }
}

/// copy of canvas at some point in time + last tool / active bit that was used at that point
struct HistoryEntry {
    tool: DrawTool,
    bit: usize,
    /// each bit represents one marker color -> there are 8 distinct manual markers
    state: Vec<u8>,
}

impl HistoryEntry {
    fn new(state: Vec<u8>) -> Self {
        HistoryEntry {
            tool: DrawTool::None,
            bit: 0,
            state,
        }
    }
}

#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct ManualMarkerOptions {
    pub sizes: [f32; 8],
    pub colors: [Color32; 8],
    /// bitmask which markers are actually painted on screen
    pub shown: u8,
    /// mush all colors of manual markers at given vertex together
    pub combine_colors: bool,
}

pub struct ManualMarkers {
    history: Vec<HistoryEntry>,
    /// indexes into [`Self::history`]
    index: usize,
    /// expected in 0..8
    pub active_bit: usize,
}

impl ManualMarkers {
    pub fn new_init(state: Vec<u8>) -> Self {
        Self {
            history: vec![HistoryEntry::new(state)],
            index: 0,
            active_bit: 0,
        }
    }

    pub fn new() -> Self {
        Self::new_init(Vec::new())
    }

    fn curr_mut(&mut self) -> &mut [u8] {
        &mut self.history[self.index].state
    }

    pub fn curr(&self) -> &[u8] {
        &self.history[self.index].state
    }

    /// the current HistoryEntry has a tool and a bit. if one of those two differ from
    /// `tool` and [`Self::active_bit`], we construct a new current history entry.
    /// Note: this function should only be called if something is about to happen.
    /// E.g. if some vertex is erased, drawn, paintbucket is called ...
    fn log_action(&mut self, tool: DrawTool) {
        let nr_vertices = {
            let curr_entry = &mut self.history[self.index];
            // if action is performed with same tool as before: no need to make new history entry
            if curr_entry.tool == tool && curr_entry.bit == self.active_bit {
                return;
            }
            curr_entry.state.len()
        };

        // we do not delete the future history if it exists.
        // it is easier to just overwrite it bit by bit.
        self.index += 1;
        if self.index >= self.history.len() {
            if self.history.len() >= HISTORY_LEN {
                let new = self.history.remove(0);
                self.history.push(new);
                self.index -= 1;
            } else {
                self.history.push(HistoryEntry::new(vec![0; nr_vertices]));
            };
        };
        debug_assert!(self.index > 0);
        let (curr_entry, new_entry) = {
            let (past, future) = self.history.split_at_mut(self.index);
            (past.last().unwrap(), future.first_mut().unwrap())
        };
        new_entry.state.copy_from_slice(&curr_entry.state);
        new_entry.tool = tool;
        new_entry.bit = self.active_bit;
    }

    pub fn undo(&mut self) {
        if self.index > 0 {
            self.index -= 1;
        }
    }

    pub fn redo(&mut self) {
        if self.index + 1 < self.history.len() {
            self.index += 1;
        }
    }

    /// if the number of vertices changes, all history is deleted.
    pub fn update_len(&mut self, nr_vertices: usize) {
        let fst = &mut self.history[0];
        if fst.state.len() != nr_vertices {
            self.index = 0;
            fst.state.clear();
            fst.state.resize(nr_vertices, 0);
            self.history.truncate(1);
        }
    }

    /// we assume to already be inside the selection window and now fill it with content.
    pub fn draw_selection_window_contents(
        &mut self,
        ui: &mut Ui,
        opts: &mut ManualMarkerOptions,
        automatic_markers_shown: bool,
        hull_shown: bool,
        automatic_markers: &[bool],
        hull: &[InSet],
    ) {
        ui.horizontal(|ui| {
            if ui.button(" ⟲ ").on_hover_text("zurück (strg + z)").clicked() {
                self.undo();
            }
            // show part of history, including current entry
            let (start, end) = {
                let len = self.history.len() as isize;
                let mut start = self.index as isize - 3;
                let mut end = start + 7.min(len);
                if start < 0 {
                    end -= start;
                    start = 0;
                }
                if end > len {
                    start -= end - len;
                    end = len;
                }
                debug_assert!((0..len).contains(&start));
                debug_assert!((0..=len).contains(&end));
                (start as usize, end as usize)
            };
            for (i, entry) in izip!(start.., &self.history[start..end]) {
                let text = if i == self.index {
                    format!("({})", entry.tool.symbol())
                } else {
                    entry.tool.symbol().to_string()
                };
                let rich = egui::RichText::new(text).color(opts.colors[entry.bit]);
                ui.add(egui::Label::new(rich));
            }
            if ui.button(" ⟳ ").on_hover_text("vorwärts (strg + y)").clicked() {
                self.redo();
            }
        });

        ui.horizontal(|ui| {
            ui.add(egui::Checkbox::new(
                &mut opts.combine_colors,
                "kombiniere Marker",
            ))
            .on_hover_text(
                "Die Farben aller Marker eines Knotens werden gemischt angezeigt\n\
                und alle Marker haben die Größe des ersten Markers.",
            );
            if opts.combine_colors {
                style::draw_options(ui, &mut opts.sizes[0], &mut [], &[]);
            }
        });

        for (i, size) in izip!(0.., &mut opts.sizes) {
            ui.horizontal(|ui| {
                let bit_i = 1u8 << i;
                let hover_choose = format!("wähle Farbe (f + {})", i + 1);
                if ui.radio(self.active_bit == i, "").on_hover_text(hover_choose).clicked() {
                    self.active_bit = i;
                    opts.shown |= bit_i;
                }
                let mut show = opts.shown & bit_i != 0;
                if ui.checkbox(&mut show, "").on_hover_text("Anzeigen").clicked() {
                    opts.shown ^= bit_i;
                    debug_assert_eq!(show, (opts.shown & bit_i) != 0);
                };

                {
                    let color = &mut opts.colors[i];
                    ui.color_edit_button_srgba(color);
                    if !opts.combine_colors {
                        let active = &mut opts.colors[i..(i + 1)];
                        let default = &color::HAND_PICKED_MARKER_COLORS[i..(i + 1)];
                        style::draw_options(ui, size, active, default);
                    } else if ui.button("Reset").clicked() {
                        *color = color::HAND_PICKED_MARKER_COLORS[i];
                    }
                }
                if ui.button(" 🗑 ").on_hover_text("diese Marker löschen").clicked() {
                    let active = std::mem::replace(&mut self.active_bit, i);
                    self.log_action(DrawTool::SetOp);
                    self.active_bit = active;
                    let mask = u8::MAX - bit_i;
                    for marker in self.curr_mut() {
                        *marker &= mask;
                    }
                }

                let what_set = match () {
                    () if automatic_markers_shown => "automatische Marker",
                    () if hull_shown => "Konvexe Hülle",
                    () => "aktive manuelle Marker",
                };
                enum Op {
                    Add,
                    Prod,
                    Sub,
                }
                for operation in [Op::Add, Op::Prod, Op::Sub] {
                    let hint = match operation {
                        Op::Add => format!("füge {what_set} hinzu (Vereinigung)"),
                        Op::Prod => format!("behalte nur {what_set} (Schnitt)"),
                        Op::Sub => format!("entferne {what_set} (Differenz)"),
                    };
                    let name = match operation {
                        Op::Add => " + ",
                        Op::Prod => " · ",
                        Op::Sub => " - ",
                    };
                    if ui.button(name).on_hover_text(hint).clicked() {
                        let active = std::mem::replace(&mut self.active_bit, i);
                        self.log_action(DrawTool::SetOp);
                        self.active_bit = active;
                        let apply = |marker: &mut u8, other: bool| match operation {
                            Op::Add => {
                                if other {
                                    *marker |= bit_i;
                                }
                            },
                            Op::Prod => {
                                if !other && (*marker & bit_i != 0) {
                                    *marker -= bit_i;
                                }
                            },
                            Op::Sub => {
                                if other && (*marker & bit_i != 0) {
                                    *marker -= bit_i;
                                }
                            },
                        };
                        let active_marker = 1u8 << self.active_bit;
                        if automatic_markers_shown {
                            let iter = izip!(automatic_markers, self.curr_mut());
                            for (&set, marker) in iter {
                                apply(marker, set);
                            }
                        } else if hull_shown {
                            let iter = izip!(hull, self.curr_mut());
                            for (&hull, marker) in iter {
                                apply(marker, hull.contained());
                            }
                        } else {
                            for marker in self.curr_mut() {
                                apply(marker, *marker & active_marker != 0);
                            }
                        }
                    }
                }
            });
        }
    }

    fn draw_combined(&self, con: &DrawContext<'_>, opts: &ManualMarkerOptions) {
        let size = 4.5 * opts.sizes[0] * con.scale;
        let mut f32_colors = [color::F32Color::default(); 8];
        let mask = opts.shown;
        color::zip_to_f32(f32_colors.iter_mut(), opts.colors.iter());
        for (&vis, &marked, &pos) in izip!(con.visible, self.curr(), con.positions) {
            let masked = marked & mask;
            if vis && masked != 0 {
                let draw_pos = con.cam().transform(pos);
                let color = color::u8_marker_color(masked, &f32_colors);
                let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
                con.painter.add(marker_circle);
            }
        }
    }

    fn draw_seperated(&self, con: &DrawContext<'_>, opts: &ManualMarkerOptions) {
        let draw_sizes = {
            let mut res = opts.sizes;
            for s in &mut res {
                *s *= 4.5 * con.scale;
            }
            res
        };
        let mask = opts.shown;
        for (&vis, &marked, &pos) in izip!(con.visible, self.curr(), con.positions) {
            let masked = marked & mask;
            if vis && masked != 0 {
                let draw_pos = con.cam().transform(pos);
                let mut single_mask: u8 = 1;
                for (&color, &size) in izip!(&opts.colors, &draw_sizes) {
                    if masked & single_mask != 0 {
                        let marker_circle = egui::Shape::circle_filled(draw_pos, size, color);
                        con.painter.add(marker_circle);
                    }
                    single_mask <<= 1;
                }
            }
        }
    }

    pub fn draw(&self, con: &DrawContext<'_>, opts: &ManualMarkerOptions) {
        if opts.combine_colors {
            self.draw_combined(con, opts);
        } else {
            self.draw_seperated(con, opts);
        }
    }

    fn change_marker_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>, set: bool) {
        let bit = 1u8 << self.active_bit;
        if !con.positions.is_empty() {
            let (vertex, dist) = con.find_closest_vertex(screen_pos);
            if dist <= 35.0 * con.scale {
                let is_set = self.curr_mut()[vertex] & bit != 0;
                if set && !is_set {
                    self.log_action(DrawTool::Draw);
                    self.curr_mut()[vertex] |= bit;
                }
                if !set && is_set {
                    self.log_action(DrawTool::Erase);
                    self.curr_mut()[vertex] -= bit;
                }
            }
        }
    }

    pub fn add_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>) {
        self.change_marker_at(screen_pos, con, true);
    }

    pub fn remove_at(&mut self, screen_pos: Pos2, con: &DrawContext<'_>) {
        self.change_marker_at(screen_pos, con, false);
    }

    pub fn paint_bucket_at(
        &mut self,
        screen_pos: Pos2,
        con: &DrawContext<'_>,
        queue: &mut VecDeque<usize>,
    ) {
        if con.positions.is_empty() {
            return;
        }
        let (v0, dist) = con.find_closest_vertex(screen_pos);
        if dist > 35.0 * con.scale {
            return;
        }
        self.log_action(DrawTool::Paintbucket);

        let bit = 1u8 << self.active_bit;
        let canvas = self.curr_mut();
        // the action will eighter remove a connected component or fill it in,
        // depending on the current state on the vertex which is clicked.
        let erase = canvas[v0] & bit != 0;
        queue.push_back(v0);
        while let Some(v) = queue.pop_front() {
            let is_colored = canvas[v] & bit != 0;
            if erase && is_colored {
                // we act as if we have eight independent canvasses wich just two colors each.
                canvas[v] -= bit;
                queue.extend(con.edges.neighbors_of(v));
            }
            if !erase && !is_colored {
                // we act as if we have eight independent canvasses wich just two colors each.
                canvas[v] |= bit;
                queue.extend(con.edges.neighbors_of(v));
            }
        }
    }
}
