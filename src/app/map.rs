use itertools::izip;

use crate::app::cam::Camera3D;
use crate::geo::Pos3;
use crate::graph::{EdgeList, Embedding3D, Shape, Z_OFFSET_2D, shape};

use super::*;

pub struct Map {
    data: Embedding3D,
    visible: Vec<bool>, //one entry per vertex, stores if that vertex can currently be seen on screen
    extreme_vertices: Vec<usize>,
}

mod storage_keys {
    pub const SHAPE: &str = "app::map::shape";
    pub const RESOLUTION: &str = "app::map::resolution";
}

impl Map {
    pub fn shape(&self) -> &Shape {
        self.data.shape()
    }

    pub fn data(&self) -> &Embedding3D {
        &self.data
    }

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        use storage_keys::*;
        let shape = load_or(cc.storage, SHAPE, || {
            type OldShape = shape::v1::Shape;
            let old: OldShape = load_or(cc.storage, SHAPE, || OldShape::Icosahedron);
            let res = {
                let last_res = load_or(cc.storage, RESOLUTION, || 12);
                //to not accidentally lag on restart, we limit maximal initial resolution for
                //graphs that are slow to build. currently this is only Random2D.
                if last_res > 50 && matches!(old, OldShape::Random2D(_)) {
                    50
                } else {
                    last_res
                }
            };
            (old, shape::Resolution(res)).into()
        });

        let mut result = Self {
            data: Embedding3D::default(),
            visible: Vec::new(),
            extreme_vertices: Vec::new(),
        };
        result.change_to(shape);

        result
    }

    pub fn save(&self, storage: &mut dyn eframe::Storage) {
        use storage_keys::*;
        eframe::set_value(storage, SHAPE, &self.shape());
    }

    /// really shitty approximation of convex hull for 2D graphs
    fn update_extreme_vertices_2d(&mut self) {
        let mut extreme_nodes = std::mem::take(&mut self.extreme_vertices);
        extreme_nodes.clear();
        extreme_nodes.resize(4, usize::MAX);
        let mut extreme_vals = [1e10, 1e10, -1e10, -1e10];
        for (i, p) in self.positions().iter().enumerate() {
            if p.x < extreme_vals[0] {
                extreme_nodes[0] = i;
                extreme_vals[0] = p.x;
            }
            if p.y < extreme_vals[1] {
                extreme_nodes[1] = i;
                extreme_vals[1] = p.y;
            }
            if p.x > extreme_vals[2] {
                extreme_nodes[2] = i;
                extreme_vals[2] = p.x;
            }
            if p.y > extreme_vals[3] {
                extreme_nodes[3] = i;
                extreme_vals[3] = p.y;
            }
        }
        self.extreme_vertices = extreme_nodes;
    }

    pub fn edges(&self) -> &EdgeList {
        self.data.edges()
    }

    pub fn positions(&self) -> &[Pos3] {
        self.data.positions()
    }

    pub fn visible(&self) -> &[bool] {
        &self.visible
    }

    pub fn is_3d(&self) -> bool {
        self.data.shape().is_3d()
    }

    /// this function takes `&mut self` instead of returning a new `Self`,
    /// because sometimes info is kept, e.g. `self.camera` if current and new shape are both 3D / 2D
    pub fn change_to(&mut self, new_shape: Shape) {
        self.data = Embedding3D::new_map_from(new_shape);
        if self.is_3d() {
            self.extreme_vertices.clear();
        } else {
            self.update_extreme_vertices_2d();
        }
        self.visible.clear();
        self.visible.resize(self.data.nr_vertices(), true);
    }

    /// this is really needed if one wants to find many vertices per frame,
    /// as otherwise the _slow_ version is only one of many operations taking `O(vertex count)`
    /// every frame anyway.
    #[allow(dead_code)]
    pub fn find_closest_vertex_fast(&self, cam: &Camera3D, screen_pos: Pos2) -> (usize, f32) {
        debug_assert!(!self.positions().is_empty());
        let find_screen_facing =
            |v: usize| -self.positions()[v].to_vec3().normalized().dot(cam.screen_normal());
        let (screen_facing, _) = self.edges().find_local_minimum(find_screen_facing, 0);
        let screen_pos_diff = |v: usize| {
            let dist_2d = (cam.transform(self.positions()[v]) - screen_pos).length();
            let backface_penalty = 10.0 * (!self.visible[v]) as isize as f32;
            dist_2d + backface_penalty
        };
        self.edges().find_local_minimum(screen_pos_diff, screen_facing)
    }

    pub fn find_closest_vertex_slow(&self, cam: &Camera3D, screen_pos: Pos2) -> (usize, f32) {
        debug_assert!(!self.positions().is_empty());
        let mut best_v = 0;
        let mut best_dist = f32::MAX;
        for (v, &pos, &vis) in izip!(0.., self.positions(), self.visible()) {
            let dist_2d = (cam.transform(pos) - screen_pos).length();
            let backface_penalty = 10.0 * (!vis) as isize as f32;
            let new_dist = dist_2d + backface_penalty;
            if new_dist < best_dist {
                best_v = v;
                best_dist = new_dist;
            }
        }
        (best_v, best_dist)
    }

    /// if self currently has shape [`Shape::Custom`], this function handles input to add vertices or edges
    /// via mouse clicking (and other things). returns whether a change to self was made.
    fn modify_custom_graph(
        &mut self,
        ui: &mut Ui,
        cam: &Camera3D,
        tool: &mut info::MouseTool,
    ) -> bool {
        let Shape::Custom(old_data) = self.shape() else {
            return false;
        };
        if !tool.used_for_building() {
            return false;
        }
        let (modifiers, Some(pointer_pos), clicking, clicked) = ui.input(|info| {
            let modifiers = info.modifiers;
            let pos = info.pointer.latest_pos();
            let clicking = info.pointer.button_down(egui::PointerButton::Primary);
            let clicked = info.pointer.button_released(egui::PointerButton::Primary);
            (modifiers, pos, clicking, clicked)
        }) else {
            return false;
        };

        if modifiers.command {
            enum Operation {
                Redo,
                Undo,
            }
            let action = ui.input(|info| {
                if info.key_pressed(egui::Key::Z) {
                    Some(Operation::Undo)
                } else if info.key_pressed(egui::Key::Y) {
                    Some(Operation::Redo)
                } else {
                    None
                }
            });
            if let Some(operation) = action {
                let mut new_data = old_data.clone();
                match operation {
                    Operation::Undo => {
                        let last_step = new_data.build_steps.pop();
                        new_data.future_build_steps.extend(last_step);
                    },
                    Operation::Redo => {
                        let next_step = new_data.future_build_steps.pop();
                        new_data.build_steps.extend(next_step);
                    },
                }
                new_data.build_steps_string = new_data.print_build_steps(false);
                let new_shape = Shape::Custom(new_data);
                self.change_to(new_shape);
                return true;
            }
        }

        if modifiers.command && matches!(tool, info::MouseTool::AddVertex) {
            *tool = info::MouseTool::DragVertex(None);
        }

        if let info::MouseTool::AddEdge(walk, _) = tool {
            if !*walk && modifiers.command {
                *walk = true;
            }
            if *walk && !modifiers.command {
                *tool = info::MouseTool::AddEdge(false, None);
            }
        }

        if !cam.to_screen().draw_rect().contains(pointer_pos) {
            return false;
        }
        let in_delete_mode = modifiers.shift;
        let new_step = match tool {
            info::MouseTool::AddEdge(walk, fst) => {
                if !clicked {
                    return false;
                }
                let (v, _) = self.find_closest_vertex_slow(cam, pointer_pos);
                let Some(u) = *fst else {
                    *fst = Some(v);
                    return false;
                };
                *fst = (*walk).then_some(v);
                if u == v {
                    return false;
                }
                if in_delete_mode {
                    shape::BuildStep::DeleteEdge(u, v)
                } else {
                    shape::BuildStep::Edge(u, v)
                }
            },
            info::MouseTool::DragVertex(dragged_vertex) => {
                if !modifiers.command && !clicking {
                    *tool = info::MouseTool::AddVertex;
                    return false;
                }
                let (drag_start, drag_delta) = ui.input(|info| {
                    let start = info.pointer.button_pressed(egui::PointerButton::Primary);
                    let delta = info.pointer.delta();
                    (start, delta)
                });
                if drag_start {
                    let (v, _) = self.find_closest_vertex_slow(cam, pointer_pos);
                    *dragged_vertex = Some(v);
                }
                if !clicking {
                    *dragged_vertex = None;
                }
                if let Some(v) = *dragged_vertex {
                    let dv = {
                        let prev = cam.to_screen().apply_inverse(pointer_pos - drag_delta, 0.0);
                        let curr = cam.to_screen().apply_inverse(pointer_pos, 0.0);
                        curr - prev
                    };
                    let mov = [
                        (dv.x * 1000.0) as i32,
                        (dv.y * 1000.0) as i32,
                        (dv.z * 1000.0) as i32,
                    ];
                    shape::BuildStep::MoveVertex(v, mov)
                } else {
                    return false;
                }
            },
            info::MouseTool::AddVertex if !in_delete_mode => {
                if !clicked {
                    return false;
                }
                let v = self.data.nr_vertices();
                let pos = cam.to_screen().apply_inverse(pointer_pos, Z_OFFSET_2D);
                let x = (pos.x * 1000.0) as i32;
                let y = (pos.y * 1000.0) as i32;
                let z = (pos.z * 1000.0) as i32;
                shape::BuildStep::Vertex(v, [x, y, z])
            },
            info::MouseTool::AddVertex if in_delete_mode => {
                if !clicked {
                    return false;
                }
                let (v, _) = self.find_closest_vertex_slow(cam, pointer_pos);
                shape::BuildStep::DeleteVertex(v)
            },
            _ => return false,
        };
        let mut new_data = old_data.clone();
        new_data.build_steps.push(new_step);
        shape::combine_last_move_operations(&mut new_data.build_steps);
        new_data.future_build_steps.clear();
        new_data.build_steps_string = new_data.print_build_steps(false);
        let new_shape = Shape::Custom(new_data);
        self.change_to(new_shape);
        true
    }

    #[must_use]
    pub fn draw_menu(&mut self, ui: &mut Ui) -> bool {
        ui.collapsing("Map", |ui| {
            let curr_shape = self.data.shape_mut();
            let old_shape = curr_shape.clone();
            let shape_changed = curr_shape.draw_menu(ui);
            debug_assert!(curr_shape == &old_shape || shape_changed);
            if shape_changed {
                let new_shape = std::mem::replace(curr_shape, old_shape);
                self.change_to(new_shape);
            }
            let nr_vertices = self.data.nr_vertices();
            ui.label(if nr_vertices == 1 {
                "    ➡ one vertex".to_string()
            } else {
                format!("    ➡ {nr_vertices} vertices")
            });
            shape_changed
        })
        .body_returned
        .is_some_and(|change| change)
    }

    pub fn scale(&self, cam: &Camera3D) -> f32 {
        let zoom = cam.zoom();

        let detail = {
            let nr_sampled_vertices = 10;
            let max_samples_per_vertex = 6;
            let mut samples = Vec::with_capacity(nr_sampled_vertices * max_samples_per_vertex);

            let iter = izip!(self.data.edges().neighbors(), self.data.positions());
            for (neighs, &p1) in iter.take(nr_sampled_vertices) {
                for n in neighs.take(max_samples_per_vertex) {
                    let p2 = self.data.positions()[n];
                    let len = (p1 - p2).length() as f64;
                    if !len.is_nan() {
                        samples.push(len);
                    }
                }
            }
            if samples.is_empty() {
                12.0 / (self.shape().resolution() as f32)
            } else {
                samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                // we only consider edges that are not too much longer than the average we saw so far.
                // how much is too much is determined by cutoff.
                let cutoff = 1.5;
                let mut running_avg = 1e10;
                for (i, val) in izip!(0.., samples) {
                    if val > running_avg * cutoff {
                        break;
                    }
                    let fi = i as f64;
                    running_avg = (running_avg * fi + val) / (fi + 1.0);
                }
                running_avg as f32 * 10.0
            }
        };
        let detail_factor = f32::min(detail, 4.0);

        let screen_size = cam.to_screen().move_rect.to().size();
        let screen_res_factor = screen_size.x.min(screen_size.y) * 0.001;

        screen_res_factor * zoom * detail_factor
    }

    pub fn tolerance(&self) -> f32 {
        f32::min(0.25, 1.25 / self.shape().resolution() as f32)
    }

    fn identity(&self) -> &Self {
        self
    }

    /// fst return value is wether the map has changed.
    pub fn update_and_draw<'a>(
        &'a mut self,
        ui: &mut Ui,
        cam: &mut Camera3D,
        tool: &mut info::MouseTool,
    ) -> (bool, DrawContext<'a>) {
        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, egui::Sense::hover());
        let screen = response.rect;
        if self.is_3d() {
            cam.update_3d(ui, screen);
        } else {
            cam.update_2d(ui, screen);
        }
        // only try to extend custom graph if mouse is not over menus.
        let change = response.contains_pointer() && self.modify_custom_graph(ui, cam, tool);

        let scale = self.scale(cam);
        let color = if ui.global_style().visuals.dark_mode {
            color::DARK_GREY
        } else {
            color::LIGHT_GREY
        };
        let grey_stroke = egui::Stroke::new(scale, color);

        self.data.draw_edges_and_update_visibility(
            cam.to_screen(),
            &painter,
            grey_stroke,
            &mut self.visible,
        );

        let con = DrawContext {
            map: self.identity(),
            cam: cam.clone(),
            extreme_vertices: &self.extreme_vertices,
            edges: self.edges(),
            visible: self.visible(),
            positions: self.positions(),
            tolerance: self.tolerance(),
            scale,
            painter,
            response,
        };
        (change, con)
    }
}
