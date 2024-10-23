use std::isize;

use grid::Norm;

use super::*;

/// map from `iproduct!(0..32, 0..32)` to `0..32`
/// where the pair stands for values overlapping in `EscapableNodes::escapable` and `i` beeing the bit set
/// in `self.escapable` to represent vertices reaching that overlap.
struct LazySafeToDilemma {
    /// this is sorted by the pair. it is a vec, because fast access is more important than fast insertion.
    /// (only up to 32 elements can be inserted anyway...)
    from: Vec<(u8, u8)>,
    /// `self.from[i]` maps to `self.to[i]`
    to: Vec<u16>,
    /// `self.from[i]` was encountered `self.number[i]` times
    number: Vec<usize>,
    /// `self.from[i]` was first encountered at vertex `self.first[i]`
    first: Vec<usize>,
}

impl LazySafeToDilemma {
    fn new() -> Self {
        Self {
            from: Vec::new(),
            to: Vec::new(),
            number: Vec::new(),
            first: Vec::new(),
        }
    }

    /// adds new configuration if none for that pair exists, else returns existing one.
    fn make_val(&mut self, overlap: (u8, u8), vertex: usize) -> u16 {
        let pos = self.from.binary_search(&overlap);
        match pos {
            Ok(index) => {
                self.number[index] += 1;
                self.to[index]
            },
            Err(index) => {
                let res = self.from.len() as u16;
                if res >= 32 {
                    return 31;
                }
                self.from.insert(index, overlap);
                self.number.insert(index, 1);
                self.first.insert(index, vertex);
                self.to.insert(index, res);
                res
            },
        }
    }

    fn clear(&mut self) {
        self.from.clear();
    }

    /// contains (from, to, number, first) in that order, where from is itself a pair.
    #[allow(dead_code)]
    fn iter(&self) -> impl Iterator<Item = ((u8, u8), u16, usize, usize)> + '_ {
        izip!(
            self.from.iter().copied(),
            self.to.iter().copied(),
            self.number.iter().copied(),
            self.first.iter().copied()
        )
    }
}

/// when two sets of escapable vertices overlap, they allow the robber to approach both at once and thus
/// only allow the police to move one such set away from the robber.
/// to get all the information we need, both the escapable vertices computed via directions
/// and the escapable vertices computed via sections of boundary are required for this
/// thing here to function
pub struct DilemmaNodes {
    to_this: LazySafeToDilemma,

    /// marks vertices, that neighbor (or are contained in) at least two regions of
    /// `EscapableNodes::escapable` at the same time. -> one entry per vertex.
    /// if bit `i` in entry `i` is set, then the bits `(ei, ej)` mapping in `self.to_this` to `i`
    /// represent the regions in `EscapableNodes::escapable`, that overlap here.
    overlapping: Vec<u32>,

    /// thing we are actually interested in. one entry per vertex, set bit's as in
    /// `Self::overlapping`, just now the regions are extended.
    dilemma: Vec<u32>,

    /// temporary data, only kept permanently to save on allocations.
    /// one entry ver vertex, starts as copy of [`Self::overlapping`].
    /// with every component of overlapping taken away, left are vertices yet to be handeled.
    overlapping_left: Vec<u32>,
}

impl DilemmaNodes {
    pub fn new() -> Self {
        Self {
            to_this: LazySafeToDilemma::new(),
            overlapping: Vec::new(),
            dilemma: Vec::new(),
            overlapping_left: Vec::new(),
        }
    }

    pub fn dilemma(&self) -> &[u32] {
        &self.dilemma
    }

    fn update_overlapping(&mut self, esc_components: &[u32], edges: &EdgeList) {
        self.to_this.clear();
        self.overlapping.clear();
        self.overlapping.resize(esc_components.len(), 0);
        let mut bits = smallvec::SmallVec::<[u8; 16]>::new();
        for (v, over, neighs, &ls) in izip!(
            0..,
            &mut self.overlapping,
            edges.neighbors(),
            esc_components
        ) {
            let union_with_neighs: u32 =
                neighs.map(|n| esc_components[n]).fold(ls, std::ops::BitOr::bitor);
            bits.clear();
            bits.extend((0..32).filter(|&i| union_with_neighs & (1 << i) != 0));
            for (index, &i) in izip!(0.., &bits) {
                for &j in &bits[(index + 1)..] {
                    let this_i = self.to_this.make_val((i, j), v);
                    *over |= 1 << this_i;
                }
            }
        }

        self.overlapping_left.resize(esc_components.len(), 0);
        self.overlapping_left.copy_from_slice(&self.overlapping);
    }

    fn update_dilemma(
        &mut self,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        esc_dirs: &EscapableDirections,
    ) {
        debug_assert_eq!(self.overlapping.len(), edges.nr_vertices());
        debug_assert_eq!(self.overlapping_left.len(), edges.nr_vertices());
        self.dilemma.clear();
        self.dilemma.resize(edges.nr_vertices(), 0);

        let g = esc_dirs.graph();
        if g.columns.nr_vertices() != edges.nr_vertices() || g.norm != Norm::Hex {
            return;
        }

        let mut curr_component = vec![0; edges.nr_vertices()];
        // we guarantee to find every component by iterating trough every vertex.
        // how far we have iterated previously, is remembered here.
        let mut search_start = 0;
        loop {
            // find first not-yet handled vertex of a component
            let Some(component_v) =
                (search_start..edges.nr_vertices()).find(|&v| self.overlapping_left[v] != 0)
            else {
                return;
            };
            // a single vertex can be part of multiple components -> no `+ 1`
            search_start = component_v;
            let component_bit = 1 << self.overlapping_left[component_v].trailing_zeros();
            debug_assert!(self.overlapping_left[component_v] & component_bit != 0);

            // find extreme vertices
            let extreme_points = {
                let fst_corner = {
                    let mut v = g.coordinates_of(component_v);
                    for &dir in g.norm.unit_directions() {
                        for _ in 0..g.columns.len {
                            let Some(v_step) = g.try_wrap(v + dir) else {
                                break;
                            };
                            let index_step = g.unchecked_index_of(v_step);
                            if self.overlapping_left[index_step] & component_bit == 0 {
                                break;
                            }
                            v = v_step;
                        }
                    }
                    debug_assert_eq!(Some(v), g.try_wrap(v));
                    v
                };
                let nr_neighs_inside = |v_coords| {
                    let v_index = g.unchecked_index_of(v_coords);
                    let neighs = edges.neighbors_of(v_index);
                    neighs.filter(|&n| self.overlapping_left[n] & component_bit != 0).count()
                };
                debug_assert!(g.wrap || nr_neighs_inside(fst_corner) <= 3);
                let mut res = [grid::Coords{ x: 0, y: 0 }; 6];
                let mut curr_corner = fst_corner;
                for (res_i, &dir) in izip!(&mut res, g.norm.unit_directions()) {
                    let mut v = curr_corner;
                    for _ in 0..g.columns.len {
                        let Some(v_step) = g.try_wrap(v + dir) else {
                            break;
                        };
                        let index_step = g.unchecked_index_of(v_step);
                        if self.overlapping_left[index_step] & component_bit == 0 {
                            break;
                        }
                        debug_assert!(g.wrap || nr_neighs_inside(v) <= 4);
                        v = v_step;
                    }
                    debug_assert!(g.wrap || nr_neighs_inside(v) <= 3);
                    *res_i = v;
                    curr_corner = v;
                }
                debug_assert_eq!(curr_corner, fst_corner);
                res
            };

            for &corner in &extreme_points {
                self.dilemma[g.unchecked_index_of(corner)] |= component_bit;
            }
            let thickness = {
                const ORDER: [usize; 3] = [1, 3, 2];
                const MID: usize = 3;
                // we evaluate how far apart in the relevant dirction two opposite corners are.
                // this is the reason we always add all six corners: 
                // we now now at which index an extreme point in a given direction is found.
                // the order in which the pairs are stored is verified here.
                debug_assert!({
                    let unit_dirs = g.norm.unit_directions();
                    let (xs, ys) = unit_dirs.split_at(MID);
                    for (ord, &x, &y) in izip!(ORDER, xs, ys) {
                        let d = g.norm.canonical_coords(x - y);
                        match ord {
                            1 => debug_assert!(d.e1 == 2 && d.e2 == 0 && d.e3 == 0),
                            2 => debug_assert!(d.e1 == 0 && d.e2 == 2 && d.e3 == 0),
                            3 => debug_assert!(d.e1 == 0 && d.e2 == 0 && d.e3 == -2),
                            _ => unreachable!(),
                        }
                    }
                    unit_dirs.len() == 2 * MID
                });
                let mut min_diff = isize::MAX;
                let (fst_half, snd_half) = extreme_points.split_at(MID);
                // access in same order as verified above
                for (ord, &ex_1, &ex_2) in izip!(ORDER, fst_half, snd_half) {
                    let diff = g.norm.canonical_coords(ex_1 - ex_2);
                    let relevant_index = match ord {
                        1 => diff.e1_line_index(),
                        2 => diff.e2_line_index(),
                        3 => diff.e3_line_index(),
                        _ => unreachable!(),
                    };
                    min_diff = isize::min(min_diff, relevant_index.abs());
                }
                min_diff + 1
            };
            println!("dicke {thickness}");

            // transfer component from [`Self::overlapping_left`] to [`curr_component`]
            self.overlapping_left[component_v] -= component_bit;
            curr_component[component_v] = component_bit;
            queue.push_back(component_v);
            while let Some(v) = queue.pop_front() {
                for n in edges.neighbors_of(v) {
                    if self.overlapping_left[n] & component_bit != 0 {
                        self.overlapping_left[n] -= component_bit;
                        curr_component[n] = component_bit;
                        queue.push_back(n);
                    }
                }
            }
        }
    }

    /// values is just some memory with length `edges.nr_vertices()`
    /// to be used as temporary space
    pub fn update(
        &mut self,
        edges: &EdgeList,
        esc_components: &[u32],
        esc_dirs: &EscapableDirections,
        queue: &mut VecDeque<usize>,
    ) {
        self.update_overlapping(esc_components, edges);
        self.update_dilemma(edges, queue, esc_dirs);
    }
}
