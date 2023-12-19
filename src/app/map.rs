
use egui::*;
use itertools::izip;

use crate::graph::{ self, Embedding3D, EdgeList };
use crate::app::{ cam::Camera3D, info::Info };
use crate::geo::Pos3;

use super::*;

#[derive(Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum Shape { 
    Tetrahedron, 
    Octahedron, 
    Icosahedron, 
    DividedIcosahedron, 
    RegularPolygon2D, 
    Random2D, 
    Debug2D 
}

pub struct Map {
    data: Embedding3D,
    visible: Vec<bool>, //one entry per vertex, stores if that vertex can currently be seen on screen
    extreme_vertices: Vec<usize>,

    shape: Shape,
    resolution: isize,
    second_shape_parameter: isize,
    camera: Camera3D,
}

mod storage_keys {
    pub const SHAPE: &'static str = "map::shape";
    pub const RESOLUTION: &'static str = "map::resolution";
    pub const SND_PARAM: &'static str = "map::second_shape_param";
    pub const CAMERA: &'static str = "map::camera";
}

impl Map {

    pub fn new(info: &mut Info, cc: &eframe::CreationContext<'_>) -> Self {
        let (shape, resolution, second_shape_parameter, camera) = if let Some(storage) = cc.storage {
            use storage_keys::*;
            (
                eframe::get_value(storage, SHAPE).unwrap_or(Shape::Icosahedron),
                eframe::get_value(storage, RESOLUTION).unwrap_or(12),
                eframe::get_value(storage, SND_PARAM).unwrap_or(3),
                eframe::get_value(storage, CAMERA).unwrap_or(Camera3D::new())
            )
        }
        else {
            (Shape::Icosahedron, 12, 3, Camera3D::new())
        };

        let mut result = Self { 
            data: Embedding3D::empty(), 
            visible: Vec::new(),
            extreme_vertices: Vec::new(),

            shape, 
            resolution, 
            second_shape_parameter,
            camera,
        };
        result.recompute();

        let nr_vertices = result.data.nr_vertices();
        if info.marked_manually.len() != nr_vertices {
            info.marked_manually.clear();
            info.marked_manually.resize(nr_vertices, false);
        }

        result
    }

    pub fn save(&self, storage: &mut dyn eframe::Storage) {
        use storage_keys::*;
        eframe::set_value(storage, SHAPE, &self.shape);
        eframe::set_value(storage, RESOLUTION, &self.resolution);
        eframe::set_value(storage, SND_PARAM, &self.second_shape_parameter);
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
        self.data.is_3d()
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
        let res = self.resolution as usize;
        let snd = self.second_shape_parameter as usize;
        let s = 1.0;
        self.data = match self.shape {
            Shape::Icosahedron => 
                Embedding3D::new_subdivided_icosahedron(s, res),
            Shape::Octahedron => 
                Embedding3D::new_subdivided_octahedron(s, res),
            Shape::Tetrahedron => 
                Embedding3D::new_subdivided_tetrahedron(s, res),
            Shape::DividedIcosahedron => { 
                let res = (res + snd - 1) / snd;
                Embedding3D::new_subdivided_subdivided_icosahedron(s, snd, res) 
            },
            Shape::RegularPolygon2D => {
                let snd = snd.clamp(3, 10);
                Embedding3D::from_2d(graph::triangulated_regular_polygon(snd, res))
            },
            Shape::Random2D => 
                Embedding3D::from_2d(graph::random_triangulated(res, 8)),
            Shape::Debug2D => 
                Embedding3D::from_2d(graph::debugging_graph()),
        };
        if self.is_3d() {
            self.extreme_vertices.clear();
            self.camera.reset_position();
        }
        else {
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
        info.characters.forget_move_history();
        info.marked_manually.clear();
        info.marked_manually.resize(self.data.nr_vertices(), false);
    }

    fn recompute_and_adjust(&mut self, info: &mut Info) {
        self.recompute();
        self.adjust_info(info);
    }

    pub fn draw_menu(&mut self, ui: &mut Ui, info: &mut Info) { 
        if ui.button("🏠 Position").clicked() {
            self.camera.reset();
        }
        ui.collapsing("Form", |ui| {
            let old_shape = self.shape;
            ui.radio_value(&mut self.shape, Shape::Tetrahedron, "Tetraeder");
            ui.radio_value(&mut self.shape, Shape::Octahedron, "Oktaeder");
            ui.radio_value(&mut self.shape, Shape::Icosahedron, "Ikosaeder");
            ui.radio_value(&mut self.shape, Shape::DividedIcosahedron, "aufgepusteter\nIkosaeder");
            if self.shape == Shape::DividedIcosahedron {
                if add_drag_value(ui, &mut self.second_shape_parameter, "Druck: ", 1, self.resolution) {
                    self.recompute_and_adjust(info);
                }
            }
            ui.radio_value(&mut self.shape, Shape::RegularPolygon2D, "2D Polygon\ntrianguliert");
            if self.shape == Shape::RegularPolygon2D {
                if add_drag_value(ui, &mut self.second_shape_parameter, "Seiten: ", 3, 10) {
                    self.recompute_and_adjust(info);
                }
            }
            ui.radio_value(&mut self.shape, Shape::Random2D, "2D Kreisscheibe\ntrianguliert");
            if self.shape == Shape::Random2D {
                if ui.button("neu berechnen").clicked() {
                    self.recompute_and_adjust(info);
                }
            }
            ui.radio_value(&mut self.shape, Shape::Debug2D, "2D Debugging");
            if self.shape != old_shape {
                self.recompute_and_adjust(info);
            }
            if add_drag_value(ui, &mut self.resolution, "Auflösung: ", 0, 200) {
                self.recompute_and_adjust(info);
            }
        });
    }

    pub fn scale(&self) -> f32 {
        let zoom = self.camera.zoom();

        let detail_factor = f32::min(12.0 / self.resolution as f32, 4.0);

        let screen_size = self.camera.to_screen().move_rect.to().size();
        let screen_res_factor = screen_size.x.min(screen_size.y) * 0.001;
        
        screen_res_factor * zoom * detail_factor
    }

    pub fn tolerance(&self) -> f32 {
        f32::min(0.25, 0.75 / self.resolution as f32)
    }

    pub fn update_and_draw<'a>(&'a mut self, ui: &mut Ui) -> DrawContext<'a> {
        let draw_space = Vec2::new(ui.available_width(), ui.available_height());
        let (response, painter) = ui.allocate_painter(draw_space, Sense::hover()); 
        let screen = response.rect;
        if self.is_3d() {
            self.camera.update_3d(ui, screen);
        }
        else {
            self.camera.update_2d(ui, screen);
        }

        let scale = self.scale();
        let grey_stroke = Stroke::new(scale, GREY);
        if self.is_3d() {
            self.data.draw_visible_edges_3d(self.camera.to_screen(), 
                &painter, grey_stroke, &mut self.visible);
        }
        else {
            self.update_vertex_visibility();
            self.data.draw_all_edges(self.camera.to_screen(), &painter, grey_stroke);
        }

        DrawContext { 
            extreme_vertices: &self.extreme_vertices,
            edges: self.edges(),
            visible: self.visible(), 
            positions: self.positions(), 
            cam: self.camera(), 
            tolerance: self.tolerance(),
            scale, 
            resolution: self.resolution, 
            painter,
            response
        }
    }
}

