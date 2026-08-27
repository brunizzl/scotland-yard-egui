use itertools::izip;

use crate::app::cam::Camera3D;
use crate::geo::Pos3;
use crate::graph::{EdgeList, Embedding3D, Shape, Z_OFFSET_2D, shape};

use super::*;

pub struct Map {
    data: Embedding3D,
    /// one entry per vertex, stores if that vertex can currently be seen on screen
    visible: Vec<bool>,
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
        let shape = load_or(cc.storage, storage_keys::SHAPE, || {
            type Old = shape::v1::Shape;
            let old: Old = load_or(cc.storage, storage_keys::SHAPE, || Old::Icosahedron);
            let res = load_or(cc.storage, storage_keys::RESOLUTION, || 12);
            (old, shape::Resolution(res)).into()
        });

        let data = Embedding3D::new_map_from(shape);
        let visible = vec![true; data.nr_vertices()];
        Self { data, visible }
    }

    pub fn save(&self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, storage_keys::SHAPE, self.shape());
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
        self.visible.clear();
        self.visible.resize(self.data.nr_vertices(), true);
    }

    pub fn find_closest_vertex(&self, cam: &Camera3D, screen_pos: Pos2) -> (usize, f32) {
        debug_assert!(!self.positions().is_empty());
        let mut best_v = 0;
        let mut best_dist = f32::MAX;
        for (v, &pos, &vis) in izip!(0.., self.positions(), self.visible()) {
            let dist_2d = (cam.transform(pos) - screen_pos).length();
            let invisible_penalty = 10.0 * (!vis) as isize as f32;
            let new_dist = dist_2d + invisible_penalty;
            if new_dist < best_dist {
                best_v = v;
                best_dist = new_dist;
            }
        }
        (best_v, best_dist)
    }

    /// if self currently has shape [`Shape::Custom`], this function handles input to add vertices or edges
    /// via mouse clicking (and other things). returns whether a change to self was made.
    #[must_use]
    fn modify_custom_graph(
        &mut self,
        ui: &mut Ui,
        cam: &Camera3D,
        tool: &mut info::MouseTool,
    ) -> bool {
        if !matches!(self.data.shape(), Shape::Custom(_)) || !tool.used_for_building() {
            return false;
        }
        fn yoink_shape(data: &mut Embedding3D) -> Box<shape::CustomBuild> {
            match std::mem::replace(data.shape_mut(), Shape::SingleVertex) {
                Shape::Custom(c) => c,
                _ => panic!(),
            }
        }

        {
            enum Action {
                Redo,
                Undo,
            }
            let maybe_action = ui.input_mut(|info| {
                if info.consume_shortcut(&crate::app::shortcuts::UNDO) {
                    Some(Action::Undo)
                } else if info.consume_shortcut(&crate::app::shortcuts::REDO) {
                    Some(Action::Redo)
                } else {
                    None
                }
            });
            if let Some(action) = maybe_action {
                let mut new_data = yoink_shape(&mut self.data);
                match action {
                    Action::Undo => {
                        let last_step = new_data.build_steps.pop();
                        new_data.future_build_steps.extend(last_step);
                    },
                    Action::Redo => {
                        let next_step = new_data.future_build_steps.pop();
                        new_data.build_steps.extend(next_step);
                    },
                }
                new_data.build_steps_string = new_data.print_build_steps();
                let new_shape = Shape::Custom(new_data);
                self.change_to(new_shape);
                return true;
            }
        }

        use egui::PointerButton as Button;
        let (modifiers, Some(pointer_pos), mouse_released) = ui.input(|info| {
            let modifiers = info.modifiers;
            let pos = info.pointer.latest_pos();
            let mouse_released = info.pointer.button_released(Button::Primary);
            (modifiers, pos, mouse_released)
        }) else {
            return false;
        };

        use info::MouseTool as Tool;
        *tool = match (modifiers.command, *tool) {
            // the menu only shows one button for both. drag is active while [ctrl] is held.
            (true, Tool::AddVertex) => Tool::DragVertex(None),
            (false, Tool::DragVertex(_)) => Tool::AddVertex,

            // while [ctrl] is held, we create a path of clicked-at vertices, otherwise a matching.
            // (assuming no vertex is clicked at twice)
            (true, Tool::AddEdge(false, prev)) => Tool::AddEdge(true, prev),
            (false, Tool::AddEdge(true, _)) => Tool::AddEdge(false, None),
            (_, tool) => tool,
        };

        if !cam.to_screen().draw_rect().contains(pointer_pos) {
            return false;
        }
        let in_delete_mode = modifiers.shift;
        let new_step = match (mouse_released, tool) {
            (true, Tool::AddEdge(walk, prev)) => {
                let (v, _) = self.find_closest_vertex(cam, pointer_pos);
                let Some(u) = *prev else {
                    *prev = Some(v);
                    return false;
                };
                if u == v {
                    return false;
                }
                *prev = (*walk).then_some(v);
                if in_delete_mode {
                    shape::BuildStep::DeleteEdge(u, v)
                } else {
                    shape::BuildStep::Edge(u, v)
                }
            },
            (true, Tool::DragVertex(dragging)) => {
                *dragging = None;
                return false;
            },
            (_, Tool::DragVertex(dragging @ None)) => {
                let start_drag = ui.input(|info| info.pointer.button_pressed(Button::Primary));
                *dragging = start_drag.then(|| self.find_closest_vertex(cam, pointer_pos).0);
                return false;
            },
            (_, Tool::DragVertex(Some(v))) => {
                debug_assert!(ui.input(|info| info.pointer.button_down(Button::Primary)));
                let drag_delta = ui.input(|info| info.pointer.delta());
                let v_delta = {
                    let prev = cam.to_screen().apply_inverse(pointer_pos - drag_delta, 0.0);
                    let curr = cam.to_screen().apply_inverse(pointer_pos, 0.0);
                    curr - prev
                };
                shape::BuildStep::MoveVertex(*v, shape::float_to_custom(v_delta.into()))
            },
            (true, Tool::AddVertex) if !in_delete_mode => {
                let v = self.data.nr_vertices();
                let pos = cam.to_screen().apply_inverse(pointer_pos, Z_OFFSET_2D);
                shape::BuildStep::Vertex(v, shape::float_to_custom(pos.into()))
            },
            (true, Tool::AddVertex) if in_delete_mode => {
                let (v, _) = self.find_closest_vertex(cam, pointer_pos);
                shape::BuildStep::DeleteVertex(v)
            },
            _ => return false,
        };
        let mut new_data = yoink_shape(&mut self.data);
        new_data.build_steps.push(new_step);
        shape::combine_last_move_operations(&mut new_data.build_steps);
        new_data.future_build_steps.clear();
        new_data.build_steps_string = new_data.print_build_steps();
        let new_shape = Shape::Custom(new_data);
        self.change_to(new_shape);
        true
    }

    #[must_use]
    pub fn draw_menu(&mut self, ui: &mut Ui) -> bool {
        ui.collapsing("Map", |ui| {
            let curr_shape = self.data.shape_mut();
            let old_shape = if cfg!(debug_assertions) {
                curr_shape.clone()
            } else {
                Shape::SingleVertex
            };

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
