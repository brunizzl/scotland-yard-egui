use grid::*;

use super::*;

/// when two sets of escapable vertices overlap, they allow the robber to approach both at once and thus
/// only allow the police to move one such set away from the robber.
/// to get all the information we need, both the escapable vertices computed via directions
/// and the escapable vertices computed via sections of boundary are required for this
/// thing here to function
pub struct DilemmaNodes {
    pub dilemma_dirs: Vec<u8>,

    pub dilemma_regions: Vec<u32>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub overlap: Vec<u32>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub allowed_steps: Vec<isize>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub allowed_dirs: Vec<u8>,
}

impl DilemmaNodes {
    pub fn new() -> Self {
        Self {
            dilemma_dirs: Vec::new(),
            dilemma_regions: Vec::new(),
            overlap: Vec::new(),
            allowed_steps: Vec::new(),
            allowed_dirs: Vec::new(),
        }
    }

    fn mark_overlapping(
        &mut self,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        cops_hull: &[InSet],
        esc_dirs: &[u8],
    ) {
        let combine_neighbor_dirs = |v: usize| {
            edges
                .neighbors_of(v)
                .map(|n| esc_dirs[n])
                .fold(esc_dirs[v], std::ops::BitOr::bitor)
        };
        let overlaps = |dirs: u8| dirs.count_ones() > 3;

        let mut search_start = 0;
        for overlap_nr in 0.. {
            // find first not-yet handled vertex of an overlap region
            let (region_v, region_dirs) = 'find_new_region: {
                let v0 = search_start;
                for (v, h, &marker) in izip!(v0.., &cops_hull[v0..], &self.overlap[v0..]) {
                    if h.contained() && marker == 0 {
                        let combined_dirs = combine_neighbor_dirs(v);
                        if overlaps(combined_dirs) {
                            search_start = v + 1;
                            break 'find_new_region (v, combined_dirs);
                        }
                    }
                }
                return;
            };
            let region_bit = 1u32 << (overlap_nr % 32);

            queue.clear();
            queue.push_back(region_v);
            while let Some(v) = queue.pop_front() {
                if cops_hull[v].contained() && (self.overlap[v] & region_bit == 0) {
                    let combined_dirs = combine_neighbor_dirs(v);
                    if overlaps(combined_dirs & region_dirs) {
                        self.overlap[v] |= region_bit;
                        queue.extend(edges.neighbors_of(v));
                    }
                }
            }
        }
    }

    fn mark_dilemma(
        &mut self,
        esc_dirs: &EscapableDirections,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        active_cops: &[&Character],
    ) {
        let g = esc_dirs.graph.data;
        if g.grid.nr_vertices() != edges.nr_vertices() || g.norm != Norm::Hex {
            return;
        }
        const UNIT_DIRS: &[grid::Coords] = Norm::Hex.unit_directions();

        let mut overlapping_left = self.overlap.clone();
        let mut boundaries = [const { Vec::new() }; 6];

        //self.dilemma_regions.clone_from_slice(&self.overlap);

        // we guarantee to find every component by iterating trough every vertex.
        // how far we have iterated previously, is remembered here.
        let mut search_start = 0;
        loop {
            // find first not-yet handled vertex of a component
            let Some(component_v) =
                (search_start..edges.nr_vertices()).find(|&v| overlapping_left[v] != 0)
            else {
                break;
            };
            // a single vertex can be part of multiple components -> no `+ 1`
            search_start = component_v;
            let component_bit = 1 << overlapping_left[component_v].trailing_zeros();
            debug_assert!(overlapping_left[component_v] & component_bit != 0);

            const NOT_IN_SHADOW: isize = isize::MIN;
            self.allowed_steps.fill(NOT_IN_SHADOW);
            self.allowed_dirs.fill(0);

            // find extreme vertices
            let extreme_points = {
                let fst_corner = {
                    let mut v = g.coordinates_of(component_v);
                    for &dir in UNIT_DIRS {
                        for _ in 0..g.grid.len {
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
                for boundary in &mut boundaries {
                    boundary.clear();
                }
                for (res_i, &dir) in izip!(&mut res, UNIT_DIRS) {
                    let mut v = curr_corner;
                    for _ in 0..g.grid.len {
                        let Some(v_step) = g.try_wrap(v + dir) else {
                            break;
                        };
                        let index_step = g.unchecked_index_of(v_step);
                        if overlapping_left[index_step] & component_bit == 0 {
                            break;
                        }
                        debug_assert!(g.wrap || nr_neighs_inside(v) <= 4);

                        for (&neigh_dir, boundary) in izip!(UNIT_DIRS, &mut boundaries) {
                            let Some(neigh_index) = g.index_of(v + neigh_dir) else {
                                continue;
                            };
                            if overlapping_left[neigh_index] & component_bit == 0 {
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

            overlapping_left[component_v] -= component_bit;
            self.allowed_steps[component_v] = thickness;
            self.allowed_dirs[component_v] = (1 << 6) - 1;
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

            for (i, &dir, boundary) in izip!(0.., UNIT_DIRS, &boundaries) {
                let shadow_marker = 1 << i;
                for &v in boundary {
                    let escape_marker = {
                        let mut res = 0;
                        let fst_step = v + dir;
                        for (i, &esc_dir) in izip!(0.., UNIT_DIRS) {
                            if let Some(index) = g.index_of(fst_step + esc_dir) {
                                if self.overlap[index] & component_bit != 0 {
                                    res |= 1 << i;
                                }
                            }
                        }
                        res
                    };
                    let mut in_region_which_overlaps = true;
                    let mut steps_left = thickness;
                    for step_len in 1..g.side_len() {
                        let Some(index) = g.index_of(v + step_len * dir) else {
                            break;
                        };
                        if in_region_which_overlaps {
                            if esc_dirs.esc_directions[index] & shadow_marker != 0 {
                                break;
                            }
                            if esc_dirs.esc_directions[index] != 0 {
                                self.allowed_steps[index] = thickness;
                            } else {
                                in_region_which_overlaps = false;
                                queue.push_back(index);
                            }
                        }
                        if !in_region_which_overlaps {
                            self.allowed_steps[index] = 0;
                            steps_left -= 1;
                            if steps_left == 0 {
                                break;
                            }
                        }
                        self.allowed_dirs[index] |= escape_marker;
                    }
                }
            }

            let mut neigh_vals = smallvec::SmallVec::<[isize; 6]>::new();
            while let Some(v) = queue.pop_front() {
                if self.allowed_steps[v] == NOT_IN_SHADOW {
                    continue;
                }
                let allowed_dirs_at_v = self.allowed_dirs[v];
                if self.dilemma_dirs[v] & allowed_dirs_at_v != 0 {
                    continue;
                }

                neigh_vals.clear();
                let v_coords = g.coordinates_of(v);
                for (_i, &esc_dir) in izip!(0.., UNIT_DIRS) {
                    if (1 << _i) & allowed_dirs_at_v == 0 {
                        continue;
                    }
                    let Some(neigh_index) = g.index_of(v_coords + esc_dir) else {
                        continue;
                        //panic!("a valid escape direction must lead to a valid vertex.");
                    };
                    let val = self.allowed_steps[neigh_index];
                    if val > 0 {
                        debug_assert_ne!(val, NOT_IN_SHADOW);
                        neigh_vals.push(val);
                    }
                }

                neigh_vals.sort_by(|a, b| a.cmp(b).reverse());
                if let Some((_, &val)) = neigh_vals
                    .iter()
                    .dedup_by_with_count(|a, b| a == b)
                    .find(|(count, _)| count > &1)
                {
                    if self.allowed_steps[v] < val - 1 {
                        self.dilemma_dirs[v] |= allowed_dirs_at_v;
                        self.dilemma_regions[v] |= component_bit;
                        self.allowed_steps[v] = val - 1;
                        queue.extend(edges.neighbors_of(v));
                    }
                }
            }
        }

        let relevant_cops = active_cops
            .iter()
            .copied()
            .filter(|c| g.disc_around(c.vertex(), 2).any(|v| self.dilemma_dirs[v] != 0))
            .collect_vec();
        esc_dirs.graph.remove_non_winning(&relevant_cops, &mut self.dilemma_dirs);

        for (&dir, &overlap, marker) in
            izip!(&self.dilemma_dirs, &self.overlap, &mut self.dilemma_regions)
        {
            if dir == 0 && overlap == 0 {
                *marker = 0;
            }
        }
    }

    pub fn update(
        &mut self,
        edges: &EdgeList,
        hull: &[InSet],
        esc_dirs: &EscapableDirections,
        queue: &mut VecDeque<usize>,
        active_cops: &[&Character],
    ) {
        assert_eq!(hull.len(), edges.nr_vertices());
        assert_eq!(esc_dirs.esc_directions.len(), edges.nr_vertices());

        self.overlap.clear();
        self.overlap.resize(edges.nr_vertices(), 0);

        self.dilemma_regions.clear();
        self.dilemma_regions.resize(edges.nr_vertices(), 0);

        self.allowed_steps.clear();
        self.allowed_steps.resize(edges.nr_vertices(), 0);

        self.allowed_dirs.clear();
        self.allowed_dirs.resize(edges.nr_vertices(), 0);

        self.dilemma_dirs.clear();
        self.dilemma_dirs.resize(edges.nr_vertices(), 0);

        if !esc_dirs.graph.represents_current_map {
            return;
        }

        self.mark_overlapping(edges, queue, hull, &esc_dirs.esc_directions);
        self.mark_dilemma(esc_dirs, edges, queue, active_cops);
    }
}
