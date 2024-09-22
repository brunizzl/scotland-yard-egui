use egui::{epaint::TextShape, text::LayoutJob, *};

use crate::geo::Pos3;
use crate::graph::{EdgeList, SymGroup};

use self::cam::Camera3D;

mod cam;
pub mod character;
mod color;
mod info;
pub mod map;
mod saves;
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
/// thus for floats, step is expected to be of form `1.0 + delta` for small growth rates
fn add_drag_value<T, U>(ui: &mut Ui, val: &mut T, name: &str, (min, max): (T, T), step: U) -> bool
where
    T: egui::emath::Numeric,
    U: Into<f64>,
{
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
        ui.add(
            egui::DragValue::new(val)
                .clamp_range(min..=max)
                .update_while_editing(false),
        );
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

fn add_scale_drag_value(ui: &mut Ui, val: &mut f32, name: &str) -> bool {
    const FIFTH_ROOT_OF_TWO: f64 = 1.148_698_354_997_035;
    add_drag_value(ui, val, name, (0.125, 8.0), FIFTH_ROOT_OF_TWO)
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
        painter.add(Shape::convex_polygon(points, stroke.color, Stroke::NONE));
    }
    painter.line_segment([origin, tip], stroke);
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum MouseTool {
    Drag,
    Draw,
    Erase,
}

impl MouseTool {
    fn symbol(self) -> &'static str {
        match self {
            MouseTool::Drag => " ♟ ",
            MouseTool::Draw => " ✏ ",
            MouseTool::Erase => " 📗 ",
        }
    }

    fn what(self) -> &'static str {
        match self {
            MouseTool::Drag => "bewege Figuren",
            MouseTool::Draw => "zeichne",
            MouseTool::Erase => "radiere",
        }
    }
}

pub struct State {
    map: map::Map,
    info: info::Info,

    tool: MouseTool,
    menu_visible: bool,
    fullscreen: bool,

    saves: saves::SavedStates,
}

impl State {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut info = info::Info::new(cc);
        let map = map::Map::new(cc);
        info.adjust_to_new_map(&cc.egui_ctx, map.data());
        let saves = saves::SavedStates::new(cc);
        Self {
            map,
            info,
            tool: MouseTool::Drag,
            menu_visible: true,
            fullscreen: false,
            saves,
        }
    }

    fn show_tool_tooltip(&mut self, ctx: &Context) {
        let next = match self.tool {
            MouseTool::Drag => MouseTool::Draw,
            MouseTool::Draw => MouseTool::Erase,
            MouseTool::Erase => MouseTool::Drag,
        };
        let f1_down = ctx.input(|info| {
            if info.key_released(Key::F1) {
                self.tool = next;
            }
            info.key_down(Key::F1)
        });
        if f1_down {
            let symbol = RichText::new(next.symbol()).size(30.0);
            show_tooltip_text(ctx, Id::new(&self.tool as *const _), symbol);
        }
    }
}

fn draw_usage_info(ui: &mut Ui) {
    ui.menu_button(" ？ Hilfe", |ui| {
        ui.add(
            Label::new(
                "\
Spielfeld rotieren / verschieben: 
ziehen mit rechter Maustaste oder scrollen, 
horizontal: shift + scrollen

zoom: 
strg + scrollen

verschieben einer Figur:
ziehen mit linker Maustaste",
            )
            .wrap(false),
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
        self.saves.save(storage);
    }

    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        if NATIVE && ctx.input(|info| info.key_pressed(Key::F11)) {
            self.fullscreen ^= true;
            ctx.send_viewport_cmd(ViewportCommand::Fullscreen(self.fullscreen));
        }
        if ctx.input(|info| info.modifiers.ctrl && info.key_pressed(Key::B)) {
            self.menu_visible ^= true;
        }
        self.show_tool_tooltip(ctx);

        if self.menu_visible {
            SidePanel::left("left_panel").show(ctx, |ui| {
                let compile_datetime = compile_time::datetime_str!();
                ui.horizontal(|ui| {
                    //add spaces to force minimum width of sidebar
                    let compile_info = format!("kompiliert: {compile_datetime} ");
                    ui.add(Label::new(compile_info).wrap(false));
                    if ui.button("⏴").on_hover_text("Menü einklappen (strg + b)").clicked() {
                        self.menu_visible = false;
                    }
                });
                ui.horizontal(|ui| {
                    widgets::global_dark_light_mode_switch(ui);

                    draw_usage_info(ui);

                    self.saves.update(ui, &mut self.map, &mut self.info);
                });

                ui.separator();
                ui.horizontal(|ui| {
                    ui.label("Werkzeug: ").on_hover_text("rotiere mit F1");
                    for mode in [MouseTool::Drag, MouseTool::Draw, MouseTool::Erase] {
                        let button = Button::new(mode.symbol()).selected(self.tool == mode);
                        if ui.add(button).on_hover_text(mode.what()).clicked() {
                            self.tool = mode;
                        }
                    }
                });

                ui.separator();
                ScrollArea::vertical().show(ui, |ui| {
                    //this forces the scroll bar to the right edge of the left panel
                    ui.allocate_at_least(Vec2::new(ui.available_width(), 0.0), Sense::hover());

                    let map_change = self.map.draw_menu(ui);
                    if map_change {
                        self.info.adjust_to_new_map(ui.ctx(), self.map.data());
                    }
                    self.info.draw_menu(ui, &self.map);
                    ui.add_space(50.0);
                });
            });
        }

        CentralPanel::default().show(ctx, |ui| {
            let con = self.map.update_and_draw(ui);
            self.info.update_and_draw(ui, &con, self.tool);

            if !self.menu_visible {
                let pos = Rect::from_center_size(pos2(12.0, 2.0), Vec2::ZERO);
                let open = Button::new("⏵");
                if ui.put(pos, open).on_hover_text("Menü ausklappen (strg + b)").clicked() {
                    self.menu_visible = true;
                }
            }
            if matches!(self.tool, MouseTool::Draw | MouseTool::Erase)
                && con.response.contains_pointer()
            {
                ctx.set_cursor_icon(CursorIcon::Crosshair);
            }
        });

        self.info.draw_windows(ctx);
    }
}
