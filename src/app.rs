use egui::{epaint::TextShape, text::LayoutJob, *};

use crate::geo::Pos3;
use crate::graph::{EdgeList, SymGroup};

use self::cam::Camera3D;

mod cam;
pub mod character;
mod color;
mod info;
pub mod map;
mod tikz;

mod bruteforce_state;

const NATIVE: bool = cfg!(not(target_arch = "wasm32"));

pub struct DrawContext<'a> {
    pub map: &'a map::Map,
    pub extreme_vertices: &'a [usize],
    pub edges: &'a EdgeList,
    pub visible: &'a [bool],
    pub positions: &'a [Pos3],
    pub tolerance: f32,
    pub scale: f32,
    pub painter: Painter,
    pub response: Response,
}

impl<'a> DrawContext<'a> {
    pub fn cam(&self) -> &Camera3D {
        self.map.camera()
    }

    pub fn screen(&self) -> &Rect {
        self.cam().to_screen().screen()
    }

    pub fn sym_group(&self) -> &SymGroup {
        self.map.data().sym_group()
    }

    pub fn vertex_draw_pos(&self, v: usize) -> Pos2 {
        self.cam().transform(self.positions[v])
    }

    pub fn find_closest_vertex(&self, screen_pos: Pos2) -> (usize, f32) {
        debug_assert!(!self.positions.is_empty());
        let find_screen_facing =
            |v: usize| -self.positions[v].to_vec3().normalized().dot(self.cam().screen_normal());
        let (screen_facing, _) = self.edges.find_local_minimum(find_screen_facing, 0);
        let screen_pos_diff = |v| {
            let dist_2d = (self.vertex_draw_pos(v) - screen_pos).length();
            let backface_penalty = 10.0 * (!self.visible[v]) as isize as f32;
            dist_2d + backface_penalty
        };
        self.edges.find_local_minimum(screen_pos_diff, screen_facing)
    }
}

/// returns if val was changed
fn add_drag_value(ui: &mut Ui, val: &mut isize, name: &str, min: isize, max: isize) -> bool {
    ui.horizontal(|ui| {
        let prev = *val;
        ui.label(name);
        if ui.button(" - ").clicked() && prev > min {
            *val -= 1;
        }
        ui.add(DragValue::new(val).clamp_range(min..=max));
        if ui.button(" + ").clicked() && prev < max {
            *val += 1;
        }
        prev != *val
    })
    .inner
}

pub struct State {
    map: map::Map,
    info: info::Info,
}

impl State {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut info = info::Info::new(cc);
        let map = map::Map::new(&mut info, cc);
        Self { map, info }
    }
}

fn draw_usage_info(ui: &mut Ui) {
    ui.collapsing("Bedienung", |ui| {
        ui.label(
            "\
Spielfeld rotieren / verschieben: 
ziehen mit rechter Maustaste oder scrollen, 
horizontal: shift + scrollen

zoom: 
strg + scrollen

verschieben einer Figur:
ziehen mit linker Maustaste",
        );
    });
}

fn load_or<T, F>(storage: Option<&dyn eframe::Storage>, key: &str, f: F) -> T
where
    T: serde::de::DeserializeOwned,
    F: FnOnce() -> T,
{
    storage.and_then(|s| eframe::get_value(s, key)).unwrap_or_else(f)
}

impl eframe::App for State {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        self.map.save(storage);
        self.info.save(storage);
    }

    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        SidePanel::left("left_panel").show(ctx, |ui| {
            ScrollArea::vertical().show(ui, |ui| {
                let compile_datetime = compile_time::datetime_str!();
                ui.label(format!("kompiliert am {compile_datetime}"));
                widgets::global_dark_light_mode_buttons(ui);
                draw_usage_info(ui);
                self.map.draw_menu(ui, &mut self.info);
                self.info.draw_menu(ui, &self.map);
                ui.add_space(50.0);
            });
        });

        CentralPanel::default().show(ctx, |ui| {
            let con = self.map.update_and_draw(ui);
            self.info.update_and_draw(ui, &con);
        });
    }
}
