

use egui::*;


mod state_2d;
mod state_3d;
mod cam;
pub mod character;
mod info;

use cam::*;
use character::*;
use info::*;


const GREY: Color32 = Color32::from_rgb(130, 130, 150);
const LIGHT_BLUE: Color32 = Color32::from_rgb(100, 100, 230);
const GREEN: Color32 = Color32::from_rgb(120, 210, 80);
const WHITE: Color32 = Color32::from_rgb(255, 255, 255);
const BLACK: Color32 = Color32::from_rgb(0, 0, 0);
const RED: Color32 = Color32::from_rgb(230, 50, 50);

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
    state_2d: state_2d::State,
    state_3d: state_3d::State,
    show_2d: bool, //else show 3d
}

impl State {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self { 
            state_2d: state_2d::State::new(cc), 
            state_3d: state_3d::State::new(cc), 
            show_2d: true,
        }
    }
}

impl eframe::App for State {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
    }

    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {


        SidePanel::left("left_panel").show(ctx, |ui| {
            ui.vertical(|ui| {
                widgets::global_dark_light_mode_buttons(ui);
                let button_switch_text = if self.show_2d { "Zeige 3D" } else { "Zeige 2D" };
                if ui.button(button_switch_text).clicked() {
                    self.show_2d = !self.show_2d;
                }

                if self.show_2d {
                    self.state_2d.draw_menu(ui);
                }
                else {
                    self.state_3d.draw_menu(ui);
                }
            });
        });

        CentralPanel::default().show(ctx, |ui| {
            if self.show_2d {
                self.state_2d.draw_graph(ui);
            }
            else {
                self.state_3d.draw_graph(ui);
            }
        });
    }
}