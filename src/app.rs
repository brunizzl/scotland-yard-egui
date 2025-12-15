use egui::{Button, Context, Label, Painter, Pos2, Rect, Stroke, Ui, Vec2, pos2};

use crate::geo::Pos3;
use crate::graph::{EdgeList, SymGroup};

use self::cam::Camera3D;

mod cam;
mod cetz;
pub mod character;
mod color;
mod info;
mod manual_markers;
pub mod map;
mod saves;
mod style;
mod tikz;

mod bruteforce_state;

const NATIVE: bool = cfg!(not(target_arch = "wasm32"));

pub struct DrawContext<'a> {
    pub map: &'a map::Map,
    pub cam: Camera3D,
    pub extreme_vertices: &'a [usize],
    pub edges: &'a EdgeList,
    pub visible: &'a [bool],
    pub positions: &'a [Pos3],
    pub tolerance: f32,
    pub scale: f32,
    pub painter: egui::Painter,
    pub response: egui::Response,
}

impl DrawContext<'_> {
    pub fn cam(&self) -> &Camera3D {
        &self.cam
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
/// thus for floats, step is expected to be of form `1.0 + delta` for small growth rates
fn add_drag_value<T, U>(
    ui: &mut Ui,
    val: &mut T,
    name: &str,
    range: std::ops::RangeInclusive<T>,
    step: U,
) -> bool
where
    T: egui::emath::Numeric,
    U: Into<f64>,
{
    let (min, max) = range.into_inner();
    ui.horizontal(|ui| {
        let prev = *val;
        let f_val = val.to_f64();
        let step = step.into();
        if ui.button(" - ").clicked() && prev > min {
            *val = T::from_f64(if T::INTEGRAL {
                f_val - step
            } else {
                f_val / step
            });
        }
        ui.add(egui::DragValue::new(val).range(min..=max).update_while_editing(false));
        if ui.button(" + ").clicked() && prev < max {
            *val = T::from_f64(if T::INTEGRAL {
                f_val + step
            } else {
                f_val * step
            });
        }
        ui.label(name);
        prev != *val
    })
    .inner
}

fn add_disabled_drag_value(ui: &mut Ui) -> bool {
    ui.horizontal(|ui| {
        ui.add_enabled(false, egui::Button::new(" - "));
        let mut val: isize = 0;
        ui.add_enabled(false, egui::DragValue::new(&mut val));
        ui.add_enabled(false, egui::Button::new(" + "));
        false
    })
    .inner
}

fn add_arrow(painter: &Painter, origin: Pos2, vec: Vec2, stroke: Stroke, tip_scale: f32) {
    let vec_normal = vec.normalized();
    let orthogonal = vec_normal.rot90();
    const SQRT_3: f32 = 1.732_050_8;
    let tip_len = tip_scale * stroke.width;
    let tip_tip = origin + vec;
    let tip = tip_tip - (SQRT_3 * tip_len) * vec_normal;
    {
        let tip_side_1 = tip + tip_len * orthogonal;
        let tip_side_2 = tip - tip_len * orthogonal;
        let points = [tip_tip, tip_side_1, tip_side_2].into();
        painter.add(egui::Shape::convex_polygon(
            points,
            stroke.color,
            Stroke::NONE,
        ));
    }
    painter.line_segment([origin, tip], stroke);
}

fn menu_button_closing_outside<'a, R>(
    ui: &mut Ui,
    atoms: impl egui::IntoAtoms<'a>,
    add_contents: impl FnOnce(&mut Ui) -> R,
) -> egui::InnerResponse<Option<R>> {
    use egui::containers::menu;
    let close =
        menu::MenuConfig::new().close_behavior(egui::PopupCloseBehavior::CloseOnClickOutside);
    let (response, inner) = if menu::is_in_menu(ui) {
        menu::SubMenuButton::new(atoms).config(close).ui(ui, add_contents)
    } else {
        menu::MenuButton::new(atoms).config(close).ui(ui, add_contents)
    };
    egui::InnerResponse::new(inner.map(|i| i.inner), response)
}

pub struct State {
    map: map::Map,
    info: info::Info,
    camera: cam::Camera3D,

    menu_visible: bool,
    fullscreen: bool,
    dark_mode: bool,

    saves: saves::SavedStates,
}

impl State {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let dark_mode = load_or(cc.storage, Self::DARK_MODE_KEY, || false);
        if dark_mode {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
        } else {
            cc.egui_ctx.set_visuals(egui::Visuals::light());
        }

        let map = map::Map::new(cc);

        let mut info = info::Info::new(cc);
        info.adjust_to_new_map(map.data());

        let mut camera = load_or(cc.storage, Self::CAMERA, Camera3D::default);
        camera.adjust_to_new_map(map.shape());

        let saves = saves::SavedStates::new(cc);
        Self {
            map,
            info,
            camera,
            menu_visible: true,
            fullscreen: false,
            dark_mode,
            saves,
        }
    }

    const DARK_MODE_KEY: &'static str = "app::in_dark_mode";
    pub const CAMERA: &str = "app::map::camera";
}

fn draw_usage_info(ui: &mut Ui) {
    menu_button_closing_outside(ui, " ？ Hilfe", |ui| {
        ui.add(
            Label::new(
                "\
Spielfeld rotieren / verschieben: 
ziehen mit rechter Maustaste oder scrollen, 
horizontal: shift + scrollen

zoom: 
strg + scrollen

verschieben einer Figur:
ziehen mit linker Maustaste

Viele Menüpunkte zeigen Extrainformation, 
wenn die Maus über ihnen schwebt.",
            )
            .wrap_mode(egui::TextWrapMode::Extend),
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
        eframe::set_value(storage, Self::DARK_MODE_KEY, &self.dark_mode);

        self.map.save(storage);
        self.info.save(storage);
        eframe::set_value(storage, Self::CAMERA, &self.camera);
        self.saves.save(storage);
    }

    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        if NATIVE && ctx.input(|info| info.key_pressed(egui::Key::F11)) {
            self.fullscreen ^= true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(self.fullscreen));
        }
        if ctx.input(|info| info.modifiers.ctrl && info.key_pressed(egui::Key::B)) {
            self.menu_visible ^= true;
        }

        if self.menu_visible {
            egui::SidePanel::left("left_panel").show(ctx, |ui| {
                ui.horizontal(|ui| {
                    const COMPILE_DATETIME: &str = compile_time::datetime_str!();
                    //add spaces to force minimum width of sidebar
                    let compile_info = format!("kompiliert: {COMPILE_DATETIME} ");
                    ui.add(Label::new(compile_info).wrap_mode(egui::TextWrapMode::Extend));
                    if ui.button("⏴").on_hover_text("Menü einklappen (strg + b)").clicked() {
                        self.menu_visible = false;
                    }
                });
                ui.horizontal(|ui| {
                    egui::widgets::global_theme_preference_switch(ui);
                    self.dark_mode = ui.visuals().dark_mode;

                    draw_usage_info(ui);

                    self.saves.update(ui, &mut self.map, &mut self.info, &mut self.camera);
                });

                ui.separator();
                self.info.draw_mouse_tool_controls(ui);

                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    //this forces the scroll bar to the right edge of the left panel
                    let hover = egui::Sense::hover();
                    ui.allocate_at_least(Vec2::new(ui.available_width(), 0.0), hover);

                    self.camera.draw_menu(ui);
                    let map_change = self.map.draw_menu(ui);
                    if map_change {
                        self.info.adjust_to_new_map(self.map.data());
                        self.camera.adjust_to_new_map(self.map.shape());
                    }
                    self.info.draw_menu(ui, &self.map, &self.camera);
                    ui.add_space(50.0);
                });
            });
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            let mut con = self.map.update_and_draw(ui, &mut self.camera);
            self.info.update_and_draw(ui, &mut con);

            if !self.menu_visible {
                let pos = Rect::from_center_size(pos2(12.0, 2.0), Vec2::ZERO);
                let open = Button::new("⏵");
                if ui.put(pos, open).on_hover_text("Menü ausklappen (strg + b)").clicked() {
                    self.menu_visible = true;
                }
            }
        });

        self.info.draw_windows(ctx);
    }
}
