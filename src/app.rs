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

/// returns if val was changed.
/// step is a multiplier for floats and is added / subtracted for ints.
fn add_drag_value<T, U>(ui: &mut Ui, val: &mut T, name: &str, (min, max): (T, T), step: U) -> bool
where
    T: egui::emath::Numeric,
    U: Into<f64>,
{
    ui.horizontal(|ui| {
        let prev = *val;
        let fval = val.to_f64();
        let step = step.into();
        if ui.button(" - ").clicked() && prev > min {
            *val = T::from_f64(if T::INTEGRAL {
                fval - step
            } else {
                fval / step
            });
        }
        ui.add(DragValue::new(val).clamp_range(min..=max));
        if ui.button(" + ").clicked() && prev < max {
            *val = T::from_f64(if T::INTEGRAL {
                fval + step
            } else {
                fval * step
            });
        }
        ui.label(name);
        prev != *val
    })
    .inner
}

fn add_disabled_drag_value(ui: &mut Ui) -> bool {
    ui.horizontal(|ui| {
        ui.add_enabled(false, Button::new(" - "));
        let mut val: isize = 0;
        ui.add_enabled(false, DragValue::new(&mut val));
        ui.add_enabled(false, Button::new(" + "));
        false
    })
    .inner
}

pub struct State {
    map: map::Map,
    info: info::Info,

    menu_visible: bool,
}

impl State {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut info = info::Info::new(cc);
        let map = map::Map::new(&mut info, cc);
        Self { map, info, menu_visible: true }
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
        if self.menu_visible {
            SidePanel::left("left_panel").min_width(0.0).show(ctx, |ui| {
                let compile_datetime = compile_time::datetime_str!();
                ui.horizontal(|ui| {
                    //add spaces to force minimum width of sidebar
                    let compile_info = format!("kompiliert am {compile_datetime}  ");
                    ui.add(Label::new(compile_info).wrap(false));
                    if ui.button("⏴").on_hover_text("Menü einklappen").clicked() {
                        self.menu_visible = false;
                    }
                });
                widgets::global_dark_light_mode_buttons(ui);
                ui.separator();

                ScrollArea::vertical().show(ui, |ui| {
                    draw_usage_info(ui);
                    self.map.draw_menu(ui, &mut self.info);
                    self.info.draw_menu(ui, &self.map);
                    ui.add_space(50.0);
                });
                
                ui.add_space(5.0);
            });
        }

        CentralPanel::default().show(ctx, |ui| {
            let con = self.map.update_and_draw(ui);
            self.info.update_and_draw(ui, &con);

            if !self.menu_visible {
                let pos = Rect::from_center_size(pos2(12.0, 2.0), Vec2::ZERO);
                let open = Button::new("⏵");
                if ui.put(pos, open).on_hover_text("Menü ausklappen").clicked() {
                    self.menu_visible = true;
                }
            }
        });

        self.info.draw_windows(ctx);
    }
}
