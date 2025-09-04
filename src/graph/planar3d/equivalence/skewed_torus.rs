use super::grid::{self, Coords};

/// same system as in thesis:
/// `li` for `i` in `[1, 2, 3]` states that the point goes through the `li`th grid line in direction `i`.
/// Note, that movement along a grid line does *NOT* change the line coordinate of this direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LineCoords {
    l1: isize,
    l2: isize,
    l3: isize,
}

#[allow(dead_code)]
impl LineCoords {
    /// assumes x-axis to point from left to right and y-axis to point from lower-left to upper-right.
    /// (an additional redundant z-axis can be added pointing from the lower-right to the upper-left.)
    /// this is the same coordiante system as used in the master's thesis
    /// (NOT THE SAME AS USED IN THIS PROGRAM FOR THE OTHER GRIDS)
    #[inline(always)]
    fn new_from_thesis_system(x: isize, y: isize) -> Self {
        let l1 = y;
        let l2 = -x;
        let l3 = -x - y;
        Self { l1, l2, l3 }
    }

    pub fn new(coords: Coords) -> Self {
        let x = coords.x;
        let y = -coords.y;
        Self::new_from_thesis_system(x, y)
    }

    pub fn rot_60(&self) -> Self {
        let l1 = -self.l3;
        let l2 = self.l1;
        let l3 = self.l2;
        Self { l1, l2, l3 }
    }

    pub fn rot_neg60(&self) -> Self {
        let l1 = self.l2;
        let l2 = self.l3;
        let l3 = -self.l1;
        Self { l1, l2, l3 }
    }

    pub fn negate(&self) -> Self {
        let l1 = -self.l1;
        let l2 = -self.l2;
        let l3 = -self.l3;
        Self { l1, l2, l3 }
    }

    pub fn norm(&self) -> isize {
        self.l1.abs().max(self.l2.abs()).max(self.l3.abs())
    }

    #[inline(always)]
    fn as_thesis_sytem_xy(&self) -> (isize, isize) {
        let x = -self.l2;
        let y = self.l1;
        (x, y)
    }

    pub fn as_coords(&self) -> Coords {
        let (x, y_upwards) = self.as_thesis_sytem_xy();
        let y = -y_upwards;
        Coords { x, y }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Torus6 {
    /// shortest difference between two elements of the same equivalence class
    /// note: the same holds for (integer multiples of) 60-degree rotations of [`Self::eq_step`].
    pub eq_step: LineCoords,
}

impl Torus6 {
    pub fn new(dx: isize, dy: isize) -> Self {
        Self {
            eq_step: LineCoords::new_from_thesis_system(dx, dy),
        }
    }

    /// idea: map the parallelogram with one corner in the origin and
    /// directions `self.eq_step` and `self.eq_step.rot_60()` to a scaled version of the unit square `[0, 1]^2`.
    /// The scale is chosen such that integer coordinates are mapped to integer coordinates.
    /// apply this map to `vec`, shift `vec` into scaled unit square, apply map inverse to shifted `vec`.
    pub fn normalize(&self, vec_x: isize, vec_y: isize) -> (isize, isize) {
        let (u, v) = self.eq_step.as_thesis_sytem_xy();
        let to_square = |x, y| ((u + v) * x + v * y, -v * x + u * y);
        //determinant of `to_square` and side length of square
        let det = u * u + u * v + v * v;
        let from_square = |x, y| {
            let x_res = u * x - v * y;
            let y_res = v * x + (u + v) * y;
            debug_assert_eq!(x_res % det, 0);
            debug_assert_eq!(y_res % det, 0);
            (x_res / det, y_res / det)
        };
        debug_assert!({
            let og_botm = (u, v);
            let og_side = self.eq_step.rot_60().as_thesis_sytem_xy();
            let square_botm = to_square(og_botm.0, og_botm.1);
            let square_side = to_square(og_side.0, og_side.1);
            debug_assert_eq!(square_botm, (det, 0));
            debug_assert_eq!(square_side, (0, det));
            debug_assert_eq!(og_botm, from_square(square_botm.0, square_botm.1));
            debug_assert_eq!(og_side, from_square(square_side.0, square_side.1));
            true
        });

        let (mut sq_x, mut sq_y) = to_square(vec_x, vec_y);
        while sq_x >= det {
            sq_x -= det;
        }
        while sq_x < 0 {
            sq_x += det;
        }
        while sq_y >= det {
            sq_y -= det;
        }
        while sq_y < 0 {
            sq_y += det;
        }
        from_square(sq_x, sq_y)
    }

    pub fn neighbors_of(&self, v: Coords) -> impl Iterator<Item = Coords> + Clone {
        let base_steps = grid::Norm::Hex.unit_directions();
        base_steps.iter().map(move |&bs| {
            let Coords { x, y } = v + bs;
            let (x_thesis, y_thesis) = self.normalize(x, -y);
            Coords { x: x_thesis, y: -y_thesis }
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn rotations_work() {
        let v = LineCoords::new(Coords { x: 40, y: -5 });
        assert_eq!(v.as_coords(), Coords { x: 40, y: -5 });

        assert_eq!(v, v.rot_60().rot_neg60());
        assert_eq!(v, v.rot_neg60().rot_60());
        assert_eq!(v.rot_60().rot_60().rot_60(), v.negate());
        assert_eq!(v.rot_neg60().rot_neg60().rot_neg60(), v.negate());
        assert_eq!(v, v.negate().negate());
    }

    #[test]
    fn normalize_works() {
        // these are simple to see, as the torus is not actually rotated
        let t = Torus6 {
            eq_step: LineCoords::new_from_thesis_system(10, 0),
        };
        for (input, expected) in [
            ((1, 1), (1, 1)),
            ((10, 10), (0, 0)),
            ((-58, 2), (2, 2)),
            ((33, 602), (3, 2)),
            ((234, 72541), (4, 1)),
        ] {
            assert_eq!(t.normalize(input.0, input.1), expected);
        }

        // these are hard to see. to check for yourself, please draw a picture
        let t2 = Torus6 {
            eq_step: LineCoords::new_from_thesis_system(10, 2),
        };
        for (input, expected) in [
            ((1, 1), (1, 1)),
            ((10, 12), (0, 10)),
            ((13, 15), (5, 1)),
            ((15, 13), (5, 11)),
            ((15, 12), (5, 10)),
        ] {
            assert_eq!(t2.normalize(input.0, input.1), expected);
        }
    }
}
