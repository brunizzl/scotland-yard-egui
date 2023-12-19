
use egui::{*, epaint::TextShape, text::LayoutJob};

use crate::graph::EdgeList;
use crate::geo::Pos3;

mod cam;
pub mod character;
mod info;
mod map;


const GREY: Color32 = Color32::from_rgb(130, 130, 150);
const LIGHT_BLUE: Color32 = Color32::from_rgb(100, 100, 230);
const GREEN: Color32 = Color32::from_rgb(120, 210, 80);
const WHITE: Color32 = Color32::from_rgb(255, 255, 255);
const BLACK: Color32 = Color32::from_rgb(0, 0, 0);
const RED: Color32 = Color32::from_rgb(230, 50, 50);
const YELLOW: Color32 = Color32::from_rgb(240, 240, 50);

pub struct DrawContext<'a> {
    pub extreme_vertices: &'a [usize],
    pub edges: &'a EdgeList,
    pub visible: &'a [bool],
    pub positions: &'a [Pos3],
    pub cam: &'a cam::Camera3D,
    pub tolerance: f32,
    pub scale: f32,
    pub resolution: isize,
    pub painter: Painter,
    pub response: Response,
}

impl<'a> DrawContext<'a> {
    pub fn vertex_draw_pos(&self, v: usize) -> Pos2 {
        self.cam.transform(self.positions[v]) 
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

pub struct State {
    map: map::Map,
    info: info::Info,
}

impl State {
    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut info = info::Info::new();
        let map = map::Map::new(&mut info);
        Self { 
            map,
            info,
        }
    }

}

fn draw_usage_info(ui: &mut Ui) {
    ui.collapsing("Bedienung", |ui| {
        ui.label("Spielfeld rotieren / verschieben: 
ziehen mit rechter Maustaste

verschieben einer Figur:
ziehen mit linker Maustaste

manuell Marker an Mausposition:
setzen: m-Taste
entfernen: n-Taste

Zug rückgängig: strg + z
Zug wiederholen: strg + y");
    });
}

impl eframe::App for State {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
    }

    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        SidePanel::left("left_panel").show(ctx, |ui| {
            ui.vertical(|ui| {
                widgets::global_dark_light_mode_buttons(ui);
                draw_usage_info(ui);
                self.map.draw_menu(ui, &mut self.info);
                self.info.draw_menu(ui, self.map.edges());
            });
        });

        CentralPanel::default().show(ctx, |ui| {
            let con = self.map.update_and_draw(ui);
            self.info.update_and_draw(ui, &con);
        });
    }
}