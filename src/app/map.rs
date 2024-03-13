use egui::*;
use itertools::izip;

use crate::app::{cam::Camera3D, info::Info};
use crate::geo::Pos3;
use crate::graph::{self, EdgeList, Embedding3D};

use super::*;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize, serde::Serialize)]
pub enum Shape {
    Tetrahedron,
    Octahedron,
    Icosahedron,
    DividedIcosahedron(isize),
    Cube,
    Football,
    FabianHamann,
    Dodecahedron,
    TriangTorus,
    RegularPolygon2D(isize),
    Random2D,
}

impl Shape {
    pub fn to_sting(self) -> String {
        match self {
            Self::Cube => "Wuerfel".to_string(),
            Self::DividedIcosahedron(pressure) => format!("Ikosaeder-{pressure}x-aufgepustet"),
            Self::Dodecahedron => "Dodekaeder".to_string(),
            Self::FabianHamann => "Fabian-Hamann".to_string(),
            Self::Football => "Fussball".to_string(),
            Self::Octahedron => "Oktaeder".to_string(),
            Self::Random2D => "Zufaellig".to_string(),
            Self::TriangTorus => "Torus-Dreiecke".to_string(),
            Self::RegularPolygon2D(nr_sides) => format!("2d-Polygon-{nr_sides}-seitig"),
            Self::Tetrahedron => "Tetraeder".to_string(),
            Self::Icosahedron => "Ikosaeder".to_string(),
        }
    }
}

pub fn new_map_from(shape: Shape, res: usize) -> Embedding3D {
    match shape {
        Shape::Icosahedron => Embedding3D::new_subdivided_icosahedron(res),
        Shape::Octahedron => Embedding3D::new_subdivided_octahedron(res),
        Shape::Tetrahedron => Embedding3D::new_subdivided_tetrahedron(res),
        Shape::DividedIcosahedron(pressure) => {
            let res1 = usize::min(res, pressure as usize);
            let res2 = if res1 == 0 {
                res
            } else {
                (usize::max(res, 1) - 1) / (res1 + 1)
            };
            Embedding3D::new_subdivided_subdivided_icosahedron(res1, res2)
        },
        Shape::RegularPolygon2D(nr_sides) => {
            let sides = nr_sides as usize;
            Embedding3D::new_2d_triangulated_regular_polygon(sides, res)
        },
        Shape::Cube => Embedding3D::new_subdivided_cube(res),
        Shape::Dodecahedron => Embedding3D::new_subdivided_dodecahedron(res, false, false),
        Shape::Football => Embedding3D::new_subdivided_football(res, false),
        Shape::FabianHamann => Embedding3D::new_subdivided_football(res, true),
        Shape::Random2D => Embedding3D::from_2d(graph::random_triangulated(res, 8)),
        Shape::TriangTorus => Embedding3D::new_subdivided_triangle_torus(res),
    }
}

pub struct Map {
    data: Embedding3D,
    visible: Vec<bool>, //one entry per vertex, stores if that vertex can currently be seen on screen
    extreme_vertices: Vec<usize>,

    shape: Shape,
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
        self.shape
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
            if last_res > 25 && matches!(shape, Shape::Random2D) {
                (25, true)
            } else {
                (last_res, false)
            }
        };
        let camera = load_or(cc.storage, CAMERA, Camera3D::new);

        let mut result = Self {
            data: Embedding3D::empty(),
            visible: Vec::new(),
            extreme_vertices: Vec::new(),

            shape,
            resolution,
            camera,
        };
        result.recompute();
        result.adjust_info(info);

        //graph is not exactly the same -> vertex indices are now worthless
        //(and in worst case larger than curr number of vertices)
        if shrunk || matches!(shape, Shape::Random2D) {
            info.characters.forget_move_history();
        }

        result
    }

    pub fn save(&self, storage: &mut dyn eframe::Storage) {
        use storage_keys::*;
        eframe::set_value(storage, SHAPE, &self.shape);
        eframe::set_value(storage, RESOLUTION, &self.resolution);
        eframe::set_value(storage, CAMERA, &self.camera);
    }

    /// really shitty approximation of convex hull for 2D graphs
    fn update_extreme_vertices_2d(&mut self) {
        let mut extreme_nodes = std::mem::take(&mut self.extreme_vertices);
        extreme_nodes.clear();
        extreme_nodes.resize(4, usize::MAX);
        let mut extreme_vals = [0.0; 4];
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
        !self.data.is_flat()
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

    fn recompute(&mut self) {
        self.data = new_map_from(self.shape, self.resolution as usize);
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

    fn adjust_info(&self, info: &mut Info) {
        for ch in info.characters.all_mut() {
            let char_dir = ch.pos3.to_vec3();
            let potential = |_, v_pos: Pos3| -char_dir.dot(v_pos.to_vec3().normalized());
            let (best_new_vertex, _) = self.data.find_local_minimum(potential, 0);
            ch.nearest_node = best_new_vertex;
            ch.update_distances(self.data.edges(), &mut info.queue);
        }

        let nr_vertices = self.data.nr_vertices();
        if info.marked_manually.len() != nr_vertices {
            info.marked_manually.clear();
            info.marked_manually.resize(nr_vertices, 0);
        }
    }

    fn recompute_and_adjust(&mut self, info: &mut Info) {
        self.recompute();
        self.adjust_info(info);
        info.characters.forget_move_history();
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, info: &mut Info) {
        if ui.button("🏠 Position").clicked() {
            self.camera.reset();
        }
        ui.collapsing("Form", |ui| {
            let old_shape = self.shape;
            let mut change = false;
            ui.radio_value(&mut self.shape, Shape::Tetrahedron, "Tetraeder");
            ui.radio_value(&mut self.shape, Shape::Octahedron, "Oktaeder");
            ui.radio_value(&mut self.shape, Shape::Icosahedron, "Ikosaeder");
            if ui
                .add(RadioButton::new(
                    matches!(self.shape, Shape::DividedIcosahedron(_)),
                    "aufgepusteter Ikosaeder",
                ))
                .clicked()
            {
                self.shape = Shape::DividedIcosahedron(0);
            }
            if let Shape::DividedIcosahedron(pressure) = &mut self.shape {
                change |= add_drag_value(ui, pressure, "Druck: ", 0, self.resolution);
            }
            ui.radio_value(&mut self.shape, Shape::Dodecahedron, "Dodekaeder");
            ui.radio_value(&mut self.shape, Shape::Cube, "Würfel");
            ui.radio_value(&mut self.shape, Shape::Football, "Fußball");
            ui.radio_value(&mut self.shape, Shape::FabianHamann, "Fabian Hamanns Graph");
            ui.radio_value(&mut self.shape, Shape::TriangTorus, "Torus (Dreiecke)");
            if ui
                .add(RadioButton::new(
                    matches!(self.shape, Shape::RegularPolygon2D(_)),
                    "2D Polygon trianguliert",
                ))
                .clicked()
            {
                self.shape = Shape::RegularPolygon2D(6);
            }
            if let Shape::RegularPolygon2D(nr_sides) = &mut self.shape {
                change |= add_drag_value(ui, nr_sides, "Seiten: ", 3, 10);
            }
            ui.radio_value(
                &mut self.shape,
                Shape::Random2D,
                "2D Kreisscheibe zufällig trianguliert",
            );
            if self.shape == Shape::Random2D {
                change |= ui.button("neu berechnen").clicked();
            }
            change |= self.shape != old_shape;
            change |= add_drag_value(ui, &mut self.resolution, "Auflösung: ", 0, 200);

            if change {
                self.recompute_and_adjust(info);
            }
            ui.label(format!("    ➡ {} Knoten", self.data.nr_vertices()));
        });
    }

    pub fn scale(&self) -> f32 {
        let zoom = self.camera.zoom();

        let detail = (self.data.nr_vertices() > 1)
            .then(|| self.data.positions()[0])
            .and_then(|v0_pos| {
                self.data
                    .edges()
                    .neighbors_of(0)
                    .next()
                    .map(|n| (v0_pos - self.data.positions()[n]).length() * 10.0)
            })
            .unwrap_or(12.0 / self.resolution as f32);
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
