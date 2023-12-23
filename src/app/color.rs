
use itertools::izip;

use egui::*;

pub const GREY: Color32 = Color32::from_rgb(130, 130, 150);
pub const LIGHT_BLUE: Color32 = Color32::from_rgb(100, 100, 230);
pub const GREEN: Color32 = Color32::from_rgb(120, 210, 80);
pub const WHITE: Color32 = Color32::from_rgb(255, 255, 255);
pub const BLACK: Color32 = Color32::from_rgb(0, 0, 0);
pub const RED: Color32 = Color32::from_rgb(230, 50, 50);
pub const YELLOW: Color32 = Color32::from_rgb(240, 240, 50);

#[allow(dead_code)]
const HAND_PICKED_MARKER_COLORS: [[f32; 3]; 16] = [
    [171.0, 130.0, 255.0], //MediumPurple1
    [255.0,  64.0,  64.0], //brown1
    [255.0, 215.0,   0.0], //gold1
    [188.0, 238.0, 104.0], //DarkOliveGreen2
    [240.0, 128.0, 128.0], //LightCoral
    [127.0, 255.0, 212.0], //aquamarine
    [255.0, 165.0,   0.0], //orange1
    [255.0,  62.0, 150.0], //VioletRed1
    [  0.0, 191.0, 255.0], //DeepSkyBlue
    [255.0, 211.0, 155.0], //burlywood1
    [255.0, 185.0,  15.0], //DarkGoldenrod1
    [152.0, 245.0, 255.0], //CadetBlue1
    [255.0, 193.0, 193.0], //RosyBrown1
    [255.0,  20.0, 147.0], //DeepPink
    [124.0, 252.0,   0.0], //LawnGreen
    [131.0, 111.0, 255.0], //SlateBlue1
];

const AUTOMATIC_MARKER_COLORS: [[f32; 3]; 32] = create_distinct_colors();

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

pub fn u16_marker_color(marker: u32) -> Color32 {
    let iter = izip!(&AUTOMATIC_MARKER_COLORS, 0..).filter_map(|(col, i)|
        (2u32.pow(i) & marker != 0).then_some(col)
    );
    blend(iter)
}

//assume s, v, in interval 0..1000, h in interval 0..6000000
//we use integers, as floating point is disallowed in compile time...
const fn hsv_to_rgb(h: usize, s: usize, v: usize) -> [f32; 3] {
    assert!(h <= 6000000);
    assert!(s <= 1000);
    assert!(v <= 1000);

    let h_interval = h / 1000000; //in interval 0..6
    let f = (h / 1000) - h_interval * 1000;
    let p = (v * (1000 - s)) / 1000;
    let q = (v * (1000 - (s * f)  / 1000)) / 1000;
    let t = (v * (1000 - (s * (1000 - f))  / 1000))  / 1000;
    let (r, g, b) = match h_interval {
        0 | 6 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        5 => (v, p, q),
        _ => panic!(),
    };
    let r_f32 = ((r * 255) / 1000) as f32;
    let g_f32 = ((g * 255) / 1000) as f32;
    let b_f32 = ((b * 255) / 1000) as f32;
    [r_f32, g_f32, b_f32]
}

const fn create_distinct_colors() -> [[f32; 3]; 32] {
    let mut res = [[0.0; 3]; 32];
    let tau = 6000000; //circular constant, e.g. 2 * pi, only blown up and int, because compiletime
    let ang_offsets = [
        tau/32, 
        tau/32 + tau/8, 
        tau/32 + tau/16, 
        tau/32 + tau/16 + tau/8, 
        0, 
        tau/8, 
        tau/16, 
        tau/16 + tau/8, 
    ];
    let mut i = 0;
    let mut nr_ang_offset = 0;
    while nr_ang_offset < ang_offsets.len() {
        let ang_offset = ang_offsets[nr_ang_offset];
        let mut k = 0;
        while k < 32 {
            let ang = (tau * k) / 32 + ang_offset;
            let sat = 900;
            let val = 950;
            res[i] = hsv_to_rgb(ang, sat, val);
            i += 1;
            k += 32 / 4;
        }
        nr_ang_offset += 1;
    }
    assert!(i == 32);

    res
}

