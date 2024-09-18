use crate::graph::Shape;
use itertools::izip;

use crate::app::{cam::Camera3D, info::Info};
use crate::geo::Pos3;
use crate::graph::{EdgeList, Embedding3D};

use super::*;

pub struct Map {
    data: Embedding3D,
    visible: Vec<bool>, //one entry per vertex, stores if that vertex can currently be seen on screen
    extreme_vertices: Vec<usize>,

    resolution: isize,
    camera: Camera3D,
}

mod storage_keys {
    pub const SHAPE: &str = "app::map::shape";
    pub const RESOLUTION: &str = "app::map::resolution";
    pub const CAMERA: &str = "app::map::camera";
}

impl Map {
    pub fn shape(&self) -> Shape {
        self.data.shape()
    }

    pub fn data(&self) -> &Embedding3D {
        &self.data
    }

    pub fn resolution(&self) -> usize {
        self.resolution as usize
    }

    pub fn new(info: &mut Info, cc: &eframe::CreationContext<'_>) -> Self {
        use storage_keys::*;
        let shape = load_or(cc.storage, SHAPE, || Shape::Icosahedron);
        let (resolution, shrunk) = {
            let last_res = load_or(cc.storage, RESOLUTION, || 12);
            //to not accidentally lag on restart, we limit maximal initial resolution for
            //graphs that are slow to build. currently this is only Random2D.
            if last_res > 50 && matches!(shape, Shape::Random2D(_)) {
                (50, true)
            } else {
                (last_res, false)
            }
        };
        let camera = load_or(cc.storage, CAMERA, Camera3D::default);

        let mut result = Self {
            data: Embedding3D::default(),
            visible: Vec::new(),
            extreme_vertices: Vec::new(),

            resolution,
            camera,
        };
        result.recompute(shape);
        result.adjust_info(info);

        //graph is not exactly the same -> vertex indices are now worthless
        //(and in worst case larger than curr number of vertices)
        if shrunk {
            info.characters.forget_move_history();
        }

        result
    }

    pub fn save(&self, storage: &mut dyn eframe::Storage) {
        use storage_keys::*;
        eframe::set_value(storage, SHAPE, &self.shape());
        eframe::set_value(storage, RESOLUTION, &self.resolution);
        eframe::set_value(storage, CAMERA, &self.camera);
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

    pub fn camera(&self) -> &Camera3D {
        &self.camera
    }

    fn update_vertex_visibility(&mut self) {
        debug_assert_eq!(self.visible.len(), self.data.nr_vertices());
        let to_screen = *self.camera.to_screen();
        for (vis, &pos) in izip!(&mut self.visible, self.data.positions()) {
            *vis = to_screen.pos_visible(pos);
        }
    }

    /// this function takes `&mut self` instead of returning a new `Self`,
    /// because sometimes info is kept, e.g. `self.camera` if current and new shape are both 3D / 2D
    fn recompute(&mut self, new_shape: Shape) {
        self.data = Embedding3D::new_map_from(new_shape, self.resolution as usize);
        if self.is_3d() {
            self.extreme_vertices.clear();
            self.camera.reset_position();
        } else {
            self.update_extreme_vertices_2d();
            self.camera.reset_direction();
        }
        self.visible.resize(self.data.nr_vertices(), false);
        self.update_vertex_visibility();
    }

    pub fn change_to(&mut self, shape: Shape, resolution: isize) {
        self.resolution = resolution;
        self.recompute(shape);
    }

    pub fn adjust_info(&self, info: &mut Info) {
        for ch in info.characters.all_mut() {
            ch.adjust_to_new_map(&self.data, &mut info.queue);
        }

        let nr_vertices = self.data.nr_vertices();
        if info.marked_manually.len() != nr_vertices {
            info.marked_manually.clear();
            info.marked_manually.resize(nr_vertices, 0);
        }
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, info: &mut Info) {
        let mut new_shape = self.shape();
        self.camera.draw_menu(ui);
        ui.collapsing("Spielfeld", |ui| {
            let mut change = false;
            ui.label("Form:");
            ComboBox::from_id_source(&self.data as *const _)
                .selected_text(self.shape().name_str())
                .show_ui(ui, |ui| {
                    macro_rules! radio {
                        ($repr:expr, $case:pat) => {
                            let selected = matches!(new_shape, $case);
                            let name = $repr.name_str();
                            let button = RadioButton::new(selected, name);
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
                    radio!(SquareTorus, SquareTorus | SquareGrid);
                    //radio!(RegularPolygon2D(6), RegularPolygon2D(_));
                    radio!(Random2D(1337), Random2D(_));
                });
            match &mut new_shape {
                Shape::DividedIcosahedron(pressure) => {
                    add_drag_value(ui, pressure, "Druck", (0, self.resolution), 1);
                },
                Shape::RegularPolygon2D(nr_sides) => {
                    add_drag_value(ui, nr_sides, "Seiten", (3, 10), 1);
                },
                Shape::Random2D(seed) => {
                    add_drag_value(ui, seed, "Seed", (0, u32::MAX), 1);
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
                _ => {
                    add_disabled_drag_value(ui);
                },
            }
            change |= new_shape != self.shape();
            ui.add_space(8.0);
            let min = new_shape.min_res();
            let max = new_shape.max_res();
            change |= add_drag_value(ui, &mut self.resolution, "Auflösung", (min, max), 1);
            if change {
                self.recompute(new_shape);
                self.adjust_info(info);
                info.characters.forget_move_history();
            }
            ui.label(format!("    ➡ {} Knoten", self.data.nr_vertices()));
        });
    }

    pub fn scale(&self) -> f32 {
        let zoom = self.camera.zoom();

        let detail = {
            let max_shown_len = self.data.max_shown_edge_length();
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

        let screen_size = self.camera.to_screen().move_rect.to().size();
        let screen_res_factor = screen_size.x.min(screen_size.y) * 0.001;

        screen_res_factor * zoom * detail_factor
    }

    pub fn tolerance(&self) -> f32 {
        f32::min(0.25, 1.25 / self.resolution as f32)
    }

    fn identity(&self) -> &Self {
        self
    }

    pub fn update_and_draw<'a>(&'a mut self, ui: &mut Ui) -> DrawContext<'a> {
        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover());
        let screen = response.rect;
        if self.is_3d() {
            self.camera.update_3d(ui, screen);
        } else {
            self.camera.update_2d(ui, screen);
        }

        let scale = self.scale();
        let color = if ui.ctx().style().visuals.dark_mode {
            color::DARK_GREY
        } else {
            color::LIGHT_GREY
        };
        let grey_stroke = Stroke::new(scale, color);

        self.data.draw_edges_and_update_visibility(
            self.camera.to_screen(),
            &painter,
            grey_stroke,
            &mut self.visible,
        );

        DrawContext {
            map: self.identity(),
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
