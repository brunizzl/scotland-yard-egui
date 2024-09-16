use itertools::izip;

use egui::Color32;

#[allow(dead_code)]
mod names {
    use egui::Color32;
    pub const LIGHT_GREY: Color32 = Color32::from_rgb(190, 190, 205);
    pub const DARK_GREY: Color32 = Color32::from_rgb(130, 130, 150);
    pub const LIGHT_BLUE: Color32 = Color32::from_rgb(100, 100, 230);
    pub const GREEN: Color32 = Color32::from_rgb(120, 210, 80);
    pub const WHITE: Color32 = Color32::from_rgb(255, 255, 255);
    pub const BLACK: Color32 = Color32::from_rgb(0, 0, 0);
    pub const RED: Color32 = Color32::from_rgb(230, 50, 50);
    pub const YELLOW: Color32 = Color32::from_rgb(240, 240, 50);
}
pub use names::*;

#[derive(Copy, Clone, Debug, Default)]
pub struct F32Color([f32; 3], f32);

impl From<F32Color> for Color32 {
    fn from(val: F32Color) -> Self {
        let [r, g, b] = val.0;
        Color32::from_rgb(r as u8, g as u8, b as u8)
    }
}

pub const HAND_PICKED_MARKER_COLORS: [Color32; 8] = [
    Color32::from_rgb(171, 130, 255), //MediumPurple1
    Color32::from_rgb(255, 64, 64),   //brown1
    Color32::from_rgb(255, 215, 0),   //gold1
    Color32::from_rgb(188, 238, 104), //DarkOliveGreen2
    Color32::from_rgb(240, 128, 128), //LightCoral
    Color32::from_rgb(127, 255, 212), //aquamarine
    Color32::from_rgb(255, 165, 0),   //orange1
    Color32::from_rgb(255, 62, 150),  //VioletRed1
                                      //Color32::from_rgb(  0, 191, 255), //DeepSkyBlue
                                      //Color32::from_rgb(255, 211, 155), //burlywood1
                                      //Color32::from_rgb(255, 185,  15), //DarkGoldenrod1
                                      //Color32::from_rgb(152, 245, 255), //CadetBlue1
                                      //Color32::from_rgb(255, 193, 193), //RosyBrown1
                                      //Color32::from_rgb(255,  20, 147), //DeepPink
                                      //Color32::from_rgb(124, 252,   0), //LawnGreen
                                      //Color32::from_rgb(131, 111, 255), //SlateBlue1
];

pub const DARK_MARKER_COLORS_F32: [F32Color; 32] = create_distinct_colors(750);
pub const BRIGHT_MARKER_COLORS_F32: [F32Color; 32] = create_distinct_colors(350);

fn floats_to_color(cs: [f32; 3], scale: f32, alpha_f: f32) -> Color32 {
    debug_assert!((0.0..=1.0).contains(&alpha_f));

    let to_u8 = |c| (c * scale) as u8;
    let alpha = (255.0 * alpha_f) as u8;
    Color32::from_rgba_premultiplied(to_u8(cs[0]), to_u8(cs[1]), to_u8(cs[2]), alpha)
}

fn blend<'a>(colors: impl Iterator<Item = &'a F32Color>) -> Color32 {
    let mut res_rgb = [0.0f32; 3];
    let mut res_alpha = 0.0f32;
    let mut alpha_sum = 0.0f32;
    for F32Color(rgb, alpha) in colors {
        let alpha = *alpha;
        debug_assert!(alpha <= 1.0);
        alpha_sum += alpha;
        for (res_channel, new_channel) in izip!(&mut res_rgb, rgb) {
            *res_channel += alpha * *new_channel;
        }
        res_alpha += (1.0 - res_alpha) * alpha;
    }
    floats_to_color(res_rgb, 1.0 / alpha_sum, res_alpha)
}

pub fn blend_picked(colors: &[F32Color], choices: impl Iterator<Item = bool>) -> Color32 {
    let iter = izip!(colors.iter().cycle(), choices).filter_map(|(c, b)| b.then_some(c));
    blend(iter)
}

pub fn u32_marker_color(marker: u32, colors: &[F32Color]) -> Color32 {
    blend_picked(colors, (0..u32::BITS).map(|i| (1u32 << i) & marker != 0))
}

pub fn u8_marker_color(marker: u8, colors: &[F32Color]) -> Color32 {
    blend_picked(colors, (0..u8::BITS).map(|i| (1u8 << i) & marker != 0))
}

//assume s, v, in interval 0..=1000, h in interval 0..5_000_000
//we use integers, as floating point is disallowed at compile time...
const fn hsv_to_rgb(h: usize, s: usize, v: usize) -> [usize; 3] {
    assert!(h < 6_000_000);
    assert!(s <= 1000);
    assert!(v <= 1000);

    let h_interval = h / 1_000_000; //in interval 0..6
    let f = (h / 1000) - h_interval * 1000;
    let p = (v * (1000 - s)) / 1000;
    let q = (v * (1000 - (s * f) / 1000)) / 1000;
    let t = (v * (1000 - (s * (1000 - f)) / 1000)) / 1000;
    match h_interval {
        0 => [v, t, p],
        1 => [q, v, p],
        2 => [p, v, t],
        3 => [p, q, v],
        4 => [t, p, v],
        5 => [v, p, q],
        _ => unreachable!(),
    }
}

/// all values of same saturation and value have same apparent brightness.
const fn hsv_to_rgb_normalized(h: usize, s: usize, v: usize) -> [usize; 3] {
    let [r, g, b] = hsv_to_rgb(h, s, v);
    // we need to guarantee that the result never exceeds 1000 in any channel.
    // thus: the more pure blue something can be, the larger a number we need here.
    let max_val = (200 * s + 1000 * (1000 - s)) / 1000;
    // the brightness is roughly calculated as Y = 0.33 R + 0.5 G + 0.16 B.
    // the problem is: the larger the difference, the smaller the brightest possible color.
    // we thus use the even more rough estimate Y = 0.32 R + 0.48 G + 0.2 B
    let curr_val = (320 * r + 480 * g + 200 * b) / max_val;
    [(r * v) / curr_val, (g * v) / curr_val, (b * v) / curr_val]
}

const fn hsv_to_rgbf_normalized(h: usize, s: usize, v: usize) -> [f32; 3] {
    let [r, g, b] = hsv_to_rgb_normalized(h, s, v);
    assert!(r <= 1000);
    assert!(g <= 1000);
    assert!(b <= 1000);
    [
        ((r * 255) / 1000) as f32,
        ((g * 255) / 1000) as f32,
        ((b * 255) / 1000) as f32,
    ]
}

const fn create_distinct_colors(sat: usize) -> [F32Color; 32] {
    let mut res = [F32Color([0.0; 3], 1.0); 32];
    let tau = 5_999_999; //circular constant, e.g. 2 * pi, only blown up and int, because compiletime
    let ang_offsets = [
        tau / 32,
        tau / 32 + tau / 8,
        tau / 32 + tau / 16,
        tau / 32 + tau / 16 + tau / 8,
        0,
        tau / 8,
        tau / 16,
        tau / 16 + tau / 8,
    ];
    let mut i = 0;
    let mut nr_ang_offset = 0;
    while nr_ang_offset < ang_offsets.len() {
        let ang_offset = ang_offsets[nr_ang_offset];
        let mut k = 0;
        while k < 32 {
            let ang = (tau * k) / 32 + ang_offset;
            let val = 1000;
            res[i].0 = hsv_to_rgbf_normalized(ang, sat, val);
            i += 1;
            k += 32 / 4;
        }
        nr_ang_offset += 1;
    }
    assert!(i == 32);

    res
}

pub const fn sample_color_wheel<const N: usize>(sat: usize, off: usize) -> [F32Color; N] {
    let mut res = [F32Color([0.0; 3], 1.0); N];
    let tau = 5_999_999;
    let mut i = 0;
    while i < N {
        let ang = (((i + off) % N) * tau) / N;
        let val = 1000;
        res[i].0 = hsv_to_rgbf_normalized(ang, sat, val);
        i += 1;
    }
    res
}

pub fn zip_to_f32<'a>(
    drain: impl Iterator<Item = &'a mut F32Color>,
    source: impl Iterator<Item = &'a Color32>,
) {
    for (d, s) in izip!(drain, source) {
        d.0 = [s.r() as f32, s.g() as f32, s.b() as f32];
        d.1 = s.a() as f32 / 255.0;
    }
}
