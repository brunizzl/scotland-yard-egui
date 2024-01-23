
use egui::{*, epaint::TextShape, text::LayoutJob};
use serde::{Deserialize, Serialize};

use crate::graph::{EdgeList, ExplicitClasses};
use crate::geo::Pos3;

use self::cam::Camera3D;

mod cam;
pub mod character;
mod info;
pub mod map;
mod color;

mod bruteforce_state;

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
        &self.map.camera()
    }

    pub fn resolution(&self) -> isize {
        self.map.resolution()
    }

    pub fn shape(&self) -> map::Shape {
        self.map.shape()
    }

    pub fn equivalence(&self) -> Option<&ExplicitClasses> {
        self.map.data().equivalence()
    }

    pub fn vertex_draw_pos(&self, v: usize) -> Pos2 {
        self.cam().transform(self.positions[v]) 
    }

    pub fn find_closest_vertex(&self, screen_pos: Pos2) -> (usize, f32) {
        debug_assert!(self.positions.len() > 0);
        let find_screen_facing = |v: usize| -self.positions[v].to_vec3().normalized().dot(self.cam().screen_normal());
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
    }).inner
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
pub enum Lang { DE, EN }

pub struct State {
    map: map::Map,
    info: info::Info,
    language: Lang
}

mod storage_keys {
    pub const LANGUAGE: &'static str = "app::Language";
}

impl State {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut info = info::Info::new(cc);
        let map = map::Map::new(&mut info, cc);
        let language = load_or(cc.storage, storage_keys::LANGUAGE, || Lang::EN);
        Self { 
            map,
            info,
            language,
        }
    }

}

fn draw_usage_info(ui: &mut Ui, lang: Lang) {
    let title = match lang { Lang::DE => "Bedienung", Lang::EN => "Usage" };
    ui.collapsing(title, |ui| {
        ui.label(
            match lang {
                Lang::DE => "Spielfeld rotieren / verschieben: 
ziehen mit rechter Maustaste
oder skrollen, horizontal shift + skrollen

zoom: 
strg + scrollen

Figuren:
verschieben: ziehen mit linker Maustaste
Zug rückgängig: strg + z
Zug wiederholen: strg + y

manuelle Marker:
setzen an cursor: m-Taste
entfernen an cursor: n-Taste",

                Lang::EN => "rotate / translate map:
drag with right mouse button
or scrolling / shift + scrolling

zoom:
ctrl + scrolling

characters:
drag with left mouse button
undo move: ctrl + z
redo move: ctrl + y

manual markers:
add at cursor: m-key
remove at cursor: n-key",
            }
        );
    });
}

fn load_or<T, F>(storage: Option<&dyn eframe::Storage>, key: &str, f: F) -> T 
where T: serde::de::DeserializeOwned, 
      F: FnOnce() -> T
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
                ui.label(format!("last compilation: {compile_datetime}"));
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.language, Lang::EN, "󾓦  ENG");
                    ui.selectable_value(&mut self.language, Lang::DE, "󾓬  GER");
                });
                widgets::global_dark_light_mode_buttons(ui);
                draw_usage_info(ui, self.language);
                self.map.draw_menu(ui, &mut self.info, self.language);
                self.info.draw_menu(ui, &self.map, self.language);
                ui.add_space(50.0);
            });
        });

        CentralPanel::default().show(ctx, |ui| {
            let con = self.map.update_and_draw(ui);
            self.info.update_and_draw(ui, &con);
        });
    }
}