use std::isize;

use grid::Norm;

use super::*;

/// map from `iproduct!(0..32, 0..32)` to `0..32`
/// where the pair stands for values overlapping in `EscapableNodes::escapable` and `i` beeing the bit set
/// in `self.escapable` to represent vertices reaching that overlap.
struct MapToDilemmaBit {
    /// this is sorted by the pair. it is a vec, because fast access is more important than fast insertion.
    /// (only up to 32 elements can be inserted anyway...)
    from: Vec<(u8, u8)>,
    /// `self.from[i]` maps to `self.to[i]`
    to: Vec<u16>,
}

impl MapToDilemmaBit {
    fn new() -> Self {
        Self {
            from: Vec::new(),
            to: Vec::new(),
        }
    }

    /// adds new configuration if none for that pair exists, else returns existing one.
    fn make_val(&mut self, overlap: (u8, u8)) -> u16 {
        let pos = self.from.binary_search(&overlap);
        match pos {
            Ok(index) => self.to[index],
            Err(index) => {
                let res = self.from.len() as u16;
                if res >= 32 {
                    return 31;
                }
                self.from.insert(index, overlap);
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
    fn iter(&self) -> impl Iterator<Item = ((u8, u8), u16)> + '_ {
        izip!(self.from.iter().copied(), self.to.iter().copied(),)
    }
}

/// when two sets of escapable vertices overlap, they allow the robber to approach both at once and thus
/// only allow the police to move one such set away from the robber.
/// to get all the information we need, both the escapable vertices computed via directions
/// and the escapable vertices computed via sections of boundary are required for this
/// thing here to function
pub struct DilemmaNodes {
    to_this: MapToDilemmaBit,

    /// marks vertices, that neighbor (or are contained in) at least two regions of
    /// `EscapableNodes::escapable` at the same time. -> one entry per vertex.
    /// if bit `i` in entry `i` is set, then the bits `(ei, ej)` mapping in `self.to_this` to `i`
    /// represent the regions in `EscapableNodes::escapable`, that overlap here.
    dilemma: Vec<u32>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    allowed_steps: Vec<isize>,
}

impl DilemmaNodes {
    pub fn new() -> Self {
        Self {
            to_this: MapToDilemmaBit::new(),
            dilemma: Vec::new(),
            allowed_steps: Vec::new(),
        }
    }

    pub fn dilemma(&self) -> &[u32] {
        &self.dilemma
    }

    #[allow(dead_code)]
    pub fn allowed_steps(&self) -> &[isize] {
        &self.allowed_steps
    }

    fn mark_overlapping(&mut self, esc_components: &[u32], edges: &EdgeList) {
        self.to_this.clear();

        self.dilemma.clear();
        self.dilemma.resize(esc_components.len(), 0);

        self.allowed_steps.clear();
        self.allowed_steps.resize(esc_components.len(), 0);

        let mut bits = smallvec::SmallVec::<[u8; 16]>::new();
        for (over, neighs, &ls) in izip!(&mut self.dilemma, edges.neighbors(), esc_components) {
            let union_with_neighs: u32 =
                neighs.map(|n| esc_components[n]).fold(ls, std::ops::BitOr::bitor);
            bits.clear();
            bits.extend((0..32).filter(|&i| union_with_neighs & (1 << i) != 0));
            for (index, &i) in izip!(0.., &bits) {
                for &j in &bits[(index + 1)..] {
                    let this_i = self.to_this.make_val((i, j));
                    *over |= 1 << this_i;
                }
            }
        }
    }

    fn mark_dilemma(
        &mut self,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        esc_dirs: &EscapableDirections,
    ) {
        debug_assert_eq!(self.dilemma.len(), edges.nr_vertices());

        let g = esc_dirs.graph();
        if g.columns.nr_vertices() != edges.nr_vertices() || g.norm != Norm::Hex {
            return;
        }
        const UNIT_DIRS: &[grid::Coords] = Norm::Hex.unit_directions();

        let mut overlapping_left = self.dilemma.clone();
        let mut booundaries = [const { Vec::new() }; 6];

        // we guarantee to find every component by iterating trough every vertex.
        // how far we have iterated previously, is remembered here.
        let mut search_start = 0;
        loop {
            // find first not-yet handled vertex of a component
            let Some(component_v) =
                (search_start..edges.nr_vertices()).find(|&v| overlapping_left[v] != 0)
            else {
                return;
            };
            // a single vertex can be part of multiple components -> no `+ 1`
            search_start = component_v;
            let component_bit = 1 << overlapping_left[component_v].trailing_zeros();
            debug_assert!(overlapping_left[component_v] & component_bit != 0);

            self.allowed_steps.fill(-1);

            // find extreme vertices
            let extreme_points = {
                let fst_corner = {
                    let mut v = g.coordinates_of(component_v);
                    for &dir in UNIT_DIRS {
                        for _ in 0..g.columns.len {
                            let Some(v_step) = g.try_wrap(v + dir) else {
                                break;
                            };
                            let index_step = g.unchecked_index_of(v_step);
                            if overlapping_left[index_step] & component_bit == 0 {
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
                    neighs.filter(|&n| overlapping_left[n] & component_bit != 0).count()
                };
                debug_assert!(g.wrap || nr_neighs_inside(fst_corner) <= 3);
                let mut res = [grid::Coords { x: 0, y: 0 }; 6];
                let mut curr_corner = fst_corner;
                for boundary in &mut booundaries {
                    boundary.clear();
                }
                for (res_i, &dir) in izip!(&mut res, UNIT_DIRS) {
                    let mut v = curr_corner;
                    for _ in 0..g.columns.len {
                        let Some(v_step) = g.try_wrap(v + dir) else {
                            break;
                        };
                        let index_step = g.unchecked_index_of(v_step);
                        if overlapping_left[index_step] & component_bit == 0 {
                            break;
                        }
                        debug_assert!(g.wrap || nr_neighs_inside(v) <= 4);
                        for (&neigh_dir, boundary) in izip!(UNIT_DIRS, &mut booundaries) {
                            let Some(neigh_coords) = g.try_wrap(v + neigh_dir) else {
                                continue;
                            };
                            let neigh_index = g.unchecked_index_of(neigh_coords);
                            if overlapping_left[neigh_index] & component_bit == 0
                            {
                                boundary.push(v);
                            }
                        }
                        v = v_step;
                    }
                    debug_assert!(g.wrap || nr_neighs_inside(v) <= 3);
                    *res_i = v;
                    curr_corner = v;
                }
                debug_assert_eq!(curr_corner, fst_corner);
                res
            };

            let thickness = {
                const MID: usize = UNIT_DIRS.len() / 2;
                const ORDER: [usize; MID] = [1, 3, 2];
                // we evaluate how far apart in the relevant dirction two opposite corners are.
                // this is the reason we always add all six corners:
                // we now now at which index an extreme point in a given direction is found.
                // the order in which the pairs are stored is verified here.
                debug_assert!({
                    let (xs, ys) = UNIT_DIRS.split_at(MID);
                    for (ord, &x, &y) in izip!(ORDER, xs, ys) {
                        let d = g.norm.canonical_coords(x - y);
                        match ord {
                            1 => debug_assert!(d.e1 == 2 && d.e2 == 0 && d.e3 == 0),
                            2 => debug_assert!(d.e1 == 0 && d.e2 == 2 && d.e3 == 0),
                            3 => debug_assert!(d.e1 == 0 && d.e2 == 0 && d.e3 == -2),
                            _ => unreachable!(),
                        }
                    }
                    true
                });
                let mut min_diff = isize::MAX;
                let (fst_half, snd_half) = extreme_points.split_at(MID);
                // access in same order as in assert above
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
            println!("dicke {thickness}:");
            for bnd in &booundaries {
                for &v in bnd {
                    print!("{} ", g.unchecked_index_of(v));
                }
                print!("\n\n");
            }

            // transfer component from [`Self::overlapping_left`] to [`curr_component`]
            overlapping_left[component_v] -= component_bit;
            self.allowed_steps[component_v] = thickness;
            queue.push_back(component_v);
            while let Some(v) = queue.pop_front() {
                for n in edges.neighbors_of(v) {
                    if overlapping_left[n] & component_bit != 0 {
                        overlapping_left[n] -= component_bit;
                        self.allowed_steps[n] = thickness;
                        queue.push_back(n);
                    }
                }
            }
            for (i, &dir, boundary) in izip!(0.., UNIT_DIRS, &booundaries) {
                let dir_marker = 1 << i;
                for &v in boundary {
                    let mut in_region_which_overlaps = true;
                    for step_len in 1..(g.columns.len as isize) {
                        let Some(index) = g.index_of(v +  step_len * dir) else {
                            break;
                        };
                        if esc_dirs.escapable[index] & dir_marker != 0 {
                            in_region_which_overlaps = false;
                        }
                        if in_region_which_overlaps {
                            if esc_dirs.escapable[index] != 0 {
                                self.allowed_steps[index] = thickness;
                                self.dilemma[index] |= component_bit;
                            } else {
                                in_region_which_overlaps = false;
                                queue.push_back(index);
                            }
                        }
                        if !in_region_which_overlaps {
                            self.allowed_steps[index] = 0;
                        }
                    }
                }
            }
            
            let mut neigh_vals = smallvec::SmallVec::<[isize; 8]>::new();
            while let Some(v) = queue.pop_front() {
                if self.dilemma[v] & component_bit != 0 {
                    continue;
                }
                if self.allowed_steps[v] < 0 {
                    continue;
                }
                neigh_vals.clear();
                neigh_vals.extend(edges.neighbors_of(v).filter_map(|n| {
                    let val = self.allowed_steps[n];
                    (val > 0).then_some(val)
                }));
                neigh_vals.sort_by(|a, b| a.cmp(b).reverse());
                if let Some((_, &val)) = neigh_vals
                    .iter()
                    .dedup_by_with_count(|a, b| a == b)
                    .find(|(count, _)| count > &1)
                {
                    if self.allowed_steps[v] < val - 1 {
                        self.dilemma[v] |= component_bit;
                        self.allowed_steps[v] = val - 1;
                        queue.extend(edges.neighbors_of(v));
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
        self.mark_overlapping(esc_components, edges);
        self.mark_dilemma(edges, queue, esc_dirs);
    }
}
