
use egui::{ Pos2, Vec2 };


#[derive(Clone, Copy)]
pub struct HexPos {
    //how far to walk from origin in horizontal direction
    pub horizontal: isize,
    //how far to walk upwards to the right
    pub ascending: isize,
    //how far to walk downwards to the right
    pub descending: isize,
}

impl HexPos {
    pub const fn new(horizontal: isize, ascending: isize, descending: isize) -> HexPos {
        HexPos{ horizontal, ascending, descending }
    }

    pub fn new_from(level: usize, sector: usize, steps_from_sector_start: usize) -> Self {
        let (level, steps) = (level as isize, steps_from_sector_start as isize);
        match sector {
            0 => HexPos::new(level, 0, -steps),
            1 => HexPos::new(-steps, level, 0),
            2 => HexPos::new(0, -steps, -level),
            3 => HexPos::new(-level, 0, steps),
            4 => HexPos::new(steps, -level, 0),
            5 => HexPos::new(0, steps, level),
            _ => panic!("expected sector in 0..6"),
        }.normalize()
    }

    pub const UNIT_H: HexPos = HexPos::new(1, 0, 0);
    pub const UNIT_A: HexPos = HexPos::new(0, 1, 0);
    pub const UNIT_D: HexPos = HexPos::new(0, 0, 1);

    pub fn is_normalized(&self) -> bool {
        match (self.horizontal, self.ascending, self.descending) {
            (0, a, d) => a * d <= 0,
            (h, 0, d) => h * d >= 0,
            (h, a, 0) => h * a >= 0,
            _ => false,
        }
    }

    pub fn normalize(&self) -> Self {
        //first step: describe pos using only horizontal and ascending
        let h = self.horizontal + self.descending;
        let a = self.ascending - self.descending;

        let res = if h * a >= 0 { 
            //we are already in one of the two sectors spanned by h and a, thus done
            //(these are the upper right sector and the lower left sector)
            HexPos::new(h, a, 0)
        }
        else if a.abs() > h.abs() {
            //sector straight down or sector straight up
            HexPos::new(0, a + h, h)
        }
        else {
            //upper left sector or lower right sector
            HexPos::new(h + a, 0, -a)
        };
        debug_assert!(res.is_normalized());
        res
    }

    //distance from origin, thus same as "level"
    pub fn norm(&self) -> usize {
        debug_assert!(self.is_normalized());
        (self.horizontal.abs() + self.ascending.abs() + self.descending.abs()) as usize
    }

    pub fn index(&self) -> usize {
        debug_assert!(self.is_normalized());
        let level = self.norm();
        if level == 0 {
            return 0;
        }

        //number of vertices per hexagon side (plus one corner vertex)
        // -> level_side_len * 6 is number of vertices of given level
        let level_side_len = level as isize; 
        //gaussian sum formula n * (n - 1) / 2, then times 6 because hexagon 
        //  and plus 1 because level 0 also has one vertex
        let level_start = level * (level - 1) * 3 + 1;

        //idea: compute for each of six sectors individually  
        //in a given sector, one axis doesnt contribute to turn further, one turns once with each step
        //thus we only need to add / subtract one axis per case.
        let index_in_level = match (self.horizontal, self.ascending, self.descending) {
            (h, a, 0) if h >= 0 && a >= 0           => 0 * level_side_len + a,
            (0, a, d) if           a >= 0 && d <= 0 => 1 * level_side_len - d,
            (h, 0, d) if h <= 0 &&           d <= 0 => 2 * level_side_len - h,
            (h, a, 0) if h <= 0 && a <= 0           => 3 * level_side_len - a,
            (0, a, d) if           a <= 0 && d >= 0 => 4 * level_side_len + d,
            (h, 0, d) if h >= 0 &&           d >= 0 => 5 * level_side_len + h,
            _ => panic!("expected normalized self"),
        };

        level_start + index_in_level as usize
    }

    pub fn to_cartesian(&self, scale: f32, origin: Pos2) -> Pos2 {
        const SIN_60_DEG: f32 = 0.86602540378;
        const COS_60_DEG: f32 = 0.5;
        const UNIT_H_DIR: Vec2 = Vec2::new(1.0, 0.0);
        const UNIT_A_DIR: Vec2 = Vec2::new(COS_60_DEG, -SIN_60_DEG); //y-axis points down
        const UNIT_D_DIR: Vec2 = Vec2::new(-COS_60_DEG, SIN_60_DEG); //y-axis points down
        
        origin 
            + self.horizontal as f32 * scale * UNIT_H_DIR
            + self.ascending as f32 * scale * UNIT_A_DIR        
            + self.descending as f32 * scale * UNIT_D_DIR
    }
}

impl std::ops::Add<HexPos> for HexPos {
    type Output = HexPos;

    fn add(self, rhs: HexPos) -> Self::Output {
        HexPos::new(
            self.horizontal + rhs.horizontal, 
            self.ascending + rhs.ascending, 
            self.descending + rhs.descending).normalize()
    }
}

impl std::ops::Sub<HexPos> for HexPos {
    type Output = HexPos;

    fn sub(self, rhs: HexPos) -> Self::Output {
        HexPos::new(
            self.horizontal - rhs.horizontal, 
            self.ascending - rhs.ascending, 
            self.descending - rhs.descending).normalize()
    }
}

impl std::ops::Mul<isize> for HexPos {
    type Output = HexPos;

    fn mul(self, rhs: isize) -> Self::Output {
        HexPos::new(
            self.horizontal * rhs, 
            self.ascending * rhs, 
            self.descending * rhs)
    }
}
