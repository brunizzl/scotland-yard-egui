
use itertools::izip;

use egui::*;

pub const GREY: Color32 = Color32::from_rgb(130, 130, 150);
pub const LIGHT_BLUE: Color32 = Color32::from_rgb(100, 100, 230);
pub const GREEN: Color32 = Color32::from_rgb(120, 210, 80);
pub const WHITE: Color32 = Color32::from_rgb(255, 255, 255);
pub const BLACK: Color32 = Color32::from_rgb(0, 0, 0);
pub const RED: Color32 = Color32::from_rgb(230, 50, 50);
pub const YELLOW: Color32 = Color32::from_rgb(240, 240, 50);

const MARKER_COLORS: [[f32; 3]; 16] = [
    [171.0, 130.0, 255.0], //MediumPurple1
    [152.0, 245.0, 255.0], //CadetBlue1
    [124.0, 252.0,   0.0], //LawnGreen
    [255.0,  20.0, 147.0], //DeepPink
    [188.0, 238.0, 104.0], //DarkOliveGreen2
    [255.0, 165.0,   0.0], //orange1
    [255.0,  62.0, 150.0], //VioletRed1
    [255.0, 215.0,   0.0], //gold1
    [127.0, 255.0, 212.0], //aquamarine
    [255.0,  64.0,  64.0], //brown1
    [240.0, 128.0, 128.0], //LightCoral
    [  0.0, 191.0, 255.0], //DeepSkyBlue
    [255.0, 211.0, 155.0], //burlywood1
    [255.0, 185.0,  15.0], //DarkGoldenrod1
    [131.0, 111.0, 255.0], //SlateBlue1
    [255.0, 193.0, 193.0], //RosyBrown1
];

fn plus_eq(color1: &mut [f32; 3], color2: &[f32; 3]) {
    for (c1, c2) in color1.iter_mut().zip(color2.iter()) {
        *c1 += c2;
    }
}

fn floats_to_color(cs: [f32; 3], factor: f32) -> Color32 {
    let to_u8 = |c| (c * factor) as u8;
    Color32::from_rgb(to_u8(cs[0]), to_u8(cs[1]), to_u8(cs[2]))
}

fn blend<'a>(colors: impl Iterator<Item = &'a [f32; 3]>) -> Color32 {
    let mut res = [0.0f32; 3];
    let mut count = 0.0f32;
    for col in colors {
        count += 1.0;
        plus_eq(&mut res, col);
    }
    floats_to_color(res, 1.0 / count)
}

pub fn u16_marker_color(marker: u16) -> Color32 {
    let iter = izip!(&MARKER_COLORS, 0..).filter_map(|(col, i)|
        (2u16.pow(i) & marker != 0).then_some(col)
    );
    blend(iter)
}

