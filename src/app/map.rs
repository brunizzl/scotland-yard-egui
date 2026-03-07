use crate::graph::{Shape, shape};
use itertools::izip;

use crate::app::cam::Camera3D;
use crate::geo::Pos3;
use crate::graph::{EdgeList, Embedding3D};

use super::*;

pub struct Map {
    data: Embedding3D,
    visible: Vec<bool>, //one entry per vertex, stores if that vertex can currently be seen on screen
    extreme_vertices: Vec<usize>,

    resolution: isize,
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

    pub fn resolution(&self) -> usize {
        self.resolution as usize
    }

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        use storage_keys::*;
        let shape = load_or(cc.storage, SHAPE, || Shape::Icosahedron);
        let resolution = {
            let last_res = load_or(cc.storage, RESOLUTION, || 12);
            //to not accidentally lag on restart, we limit maximal initial resolution for
            //graphs that are slow to build. currently this is only Random2D.
            if last_res > 50 && matches!(shape, Shape::Random2D(_)) {
                50
            } else {
                last_res
            }
        };

        let mut result = Self {
            data: Embedding3D::default(),
            visible: Vec::new(),
            extreme_vertices: Vec::new(),

            resolution,
        };
        result.recompute(shape);

        result
    }

    pub fn save(&self, storage: &mut dyn eframe::Storage) {
        use storage_keys::*;
        eframe::set_value(storage, SHAPE, &self.shape());
        eframe::set_value(storage, RESOLUTION, &self.resolution);
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
    fn recompute(&mut self, new_shape: Shape) {
        self.data = Embedding3D::new_map_from(new_shape, self.resolution as usize);
        if self.is_3d() {
            self.extreme_vertices.clear();
        } else {
            self.update_extreme_vertices_2d();
        }
        self.visible.clear();
        self.visible.resize(self.data.nr_vertices(), true);
    }

    pub fn change_to(&mut self, shape: Shape, resolution: isize) {
        self.resolution = resolution;
        self.recompute(shape);
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
    /// via mouse clicking.
    fn extend_custom_graph(&mut self, ui: &mut Ui, cam: &Camera3D, tool: &mut info::MouseTool) {
        let Shape::Custom(old_data) = self.shape() else {
            return;
        };
        let (Some(pointer_pos), shift, true) = ui.input(|info| {
            let pos = info.pointer.latest_pos();
            let shift = info.modifiers.shift;
            let clicked = info.pointer.button_released(egui::PointerButton::Primary);
            (pos, shift, clicked)
        }) else {
            return;
        };
        if !cam.to_screen().draw_rect().contains(pointer_pos) {
            return;
        }
        let new_step = match *tool {
            info::MouseTool::AddEdge(fst) => {
                let (v, _) = self.find_closest_vertex_slow(cam, pointer_pos);
                let Some(u) = fst else {
                    *tool = info::MouseTool::AddEdge(Some(v));
                    return;
                };
                *tool = info::MouseTool::AddEdge(shift.then_some(v));
                if u == v {
                    return;
                }
                shape::BuildStep::Edge(u, v)
            },
            info::MouseTool::AddVertex => {
                let pos = cam.to_screen().apply_inverse(pointer_pos);
                let x = (pos.x * 1000.0) as i32;
                let y = (pos.y * 1000.0) as i32;
                let z = (pos.z * 1000.0) as i32;
                shape::BuildStep::Vertex(x, y, z)
            },
            _ => return,
        };
        let mut new_data = old_data.clone();
        new_data.build_steps.push(new_step);
        new_data.build_steps_string = new_data.print_build_steps(false);
        let new_shape = Shape::Custom(new_data);
        self.recompute(new_shape);
    }

    pub fn draw_menu(&mut self, ui: &mut Ui) -> bool {
        let mut change = false;
        ui.collapsing("Spielfeld", |ui| {
            let mut new_shape = self.shape().clone();
            ui.label("Form:");
            egui::ComboBox::from_id_salt(&self.data as *const _)
                .selected_text(self.shape().name_str())
                .show_ui(ui, |ui| {
                    macro_rules! radio {
                        ($repr:expr, $case:pat) => {
                            let selected = matches!(new_shape, $case);
                            let name = $repr.name_str();
                            let button = egui::RadioButton::new(selected, name);
                            if ui.add(button).clicked() {
                                new_shape = $repr;
                            }
                        };
                    }
                    use Shape::*;
                    radio!(Tetrahedron, Tetrahedron);
                    radio!(Octahedron, Octahedron);
                    radio!(Icosahedron, Icosahedron);
                    radio!(DividedIcosahedron(0), DividedIcosahedron(_));
                    radio!(Dodecahedron, Dodecahedron);
                    radio!(Cube, Cube);
                    radio!(Football, Football);
                    radio!(FabianHamann, FabianHamann);
                    radio!(TriangTorus, TriangTorus | TriangGrid);
                    radio!(TriangTorusSkewed(0), TriangTorusSkewed(_));
                    radio!(SquareTorus, SquareTorus | SquareGrid);
                    //radio!(RegularPolygon2D(6), RegularPolygon2D(_));
                    radio!(Random2D(1337), Random2D(_));

                    if NATIVE {
                        let selected = matches!(new_shape, Custom(_));
                        let custom_msg = "Custom";
                        let custom_radio = egui::RadioButton::new(selected, custom_msg);
                        if ui.add(custom_radio).clicked() {
                            // use this as basis, as it only is a single vertex.
                            let basis = Shape::Random2D(1312);
                            self.resolution = basis.min_res();
                            let custom_data = shape::CustomBuild::new(basis);
                            let custom_box = Box::new(custom_data);
                            new_shape = Custom(custom_box);
                            change = true;
                        }
                        if !selected {
                            let extend_msg = "erweitere aktuellen Graph";
                            let extend_radio = egui::RadioButton::new(false, extend_msg);
                            if ui.add(extend_radio).clicked() {
                                let custom_data = shape::CustomBuild::new(new_shape.clone());
                                let custom_box = Box::new(custom_data);
                                new_shape = Custom(custom_box);
                            }
                        }
                    }
                });
            match &mut new_shape {
                Shape::DividedIcosahedron(pressure) => {
                    add_drag_value(ui, pressure, "Druck", 0..=self.resolution, 1);
                },
                Shape::RegularPolygon2D(nr_sides) => {
                    add_drag_value(ui, nr_sides, "Seiten", 3..=10, 1);
                },
                Shape::Random2D(seed) => {
                    add_drag_value(ui, seed, "Seed", 0..=u32::MAX, 1);
                },
                Shape::TriangTorusSkewed(dy) => {
                    add_drag_value(ui, dy, "Schräglage", 0..=800, 1);
                },
                Shape::SquareGrid => {
                    if ui.button(" ↪↩ ").on_hover_text("klebe zu Torus").clicked() {
                        new_shape = Shape::SquareTorus;
                    }
                },
                Shape::SquareTorus => {
                    if ui.button("   ✂   ").on_hover_text("zerschneiden").clicked() {
                        new_shape = Shape::SquareGrid;
                    }
                },
                Shape::TriangGrid => {
                    if ui.button(" ↪↩ ").on_hover_text("klebe zu Torus").clicked() {
                        new_shape = Shape::TriangTorus;
                    }
                },
                Shape::TriangTorus => {
                    if ui.button("   ✂   ").on_hover_text("zerschneiden").clicked() {
                        new_shape = Shape::TriangGrid;
                    }
                },
                Shape::Custom(c) => {
                    crate::app::menu_button_closing_outside(ui, "Bauschritte", |ui| {
                        ui.label(shape::BuildStep::EXPLAINER);
                        ui.add_space(5.0);
                        let recompute_button = ui.button("Übernehmen");
                        ui.add_space(5.0);
                        let text_edit = egui::ScrollArea::vertical()
                            .show(ui, |ui| ui.text_edit_multiline(&mut c.build_steps_string));
                        if text_edit.inner.lost_focus() || recompute_button.clicked() {
                            c.parse_build_steps();
                        }
                    });
                },
                _ => {
                    add_disabled_drag_value(ui);
                },
            }
            change |= &new_shape != self.shape();
            ui.add_space(8.0);
            let min = new_shape.min_res();
            let max = new_shape.max_res();
            change |= add_drag_value(ui, &mut self.resolution, "Auflösung", min..=max, 1);
            if change {
                self.recompute(new_shape);
            }
            ui.label(format!("    ➡ {} Knoten", self.data.nr_vertices()));
        });
        change
    }

    pub fn scale(&self, cam: &Camera3D) -> f32 {
        let zoom = cam.zoom();

        let detail = {
            let max_shown_len = self.data.max_scaling_edge_length();
            let mut cum_len = 0.0;
            let mut nr_counted = 0;
            let samples = izip!(self.data.edges().neighbors(), self.data.positions());
            for (neighs, &p1) in samples.take(10) {
                for p2 in neighs.map(|n| self.data.positions()[n]) {
                    let len = (p1 - p2).length();
                    if len <= max_shown_len {
                        cum_len += len;
                        nr_counted += 1;
                    }
                }
            }
            if nr_counted > 0 {
                //estimate average shown edge length
                cum_len / (nr_counted as f32) * 10.0
            } else {
                12.0 / self.resolution as f32
            }
        };
        let detail_factor = f32::min(detail, 4.0);

        let screen_size = cam.to_screen().move_rect.to().size();
        let screen_res_factor = screen_size.x.min(screen_size.y) * 0.001;

        screen_res_factor * zoom * detail_factor
    }

    pub fn tolerance(&self) -> f32 {
        f32::min(0.25, 1.25 / self.resolution as f32)
    }

    fn identity(&self) -> &Self {
        self
    }

    pub fn update_and_draw<'a>(
        &'a mut self,
        ui: &mut Ui,
        cam: &mut Camera3D,
        tool: &mut info::MouseTool,
    ) -> DrawContext<'a> {
        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, egui::Sense::hover());
        let screen = response.rect;
        if self.is_3d() {
            cam.update_3d(ui, screen);
        } else {
            cam.update_2d(ui, screen);
        }
        self.extend_custom_graph(ui, cam, tool);

        let scale = self.scale(cam);
        let color = if ui.ctx().style().visuals.dark_mode {
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

        DrawContext {
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
        }
    }
}
