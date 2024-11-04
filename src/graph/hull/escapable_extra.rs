use grid::*;

use super::*;

/// when two sets of escapable vertices overlap, they allow the robber to approach both at once and thus
/// only allow the police to move one such set away from the robber.
/// this is higly experimental and not yet correct.
pub struct DilemmaNodes {
    pub dilemma_dirs: Vec<Dirs>,

    pub dilemma_regions: Vec<u32>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub overlap: Vec<u32>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub taken_steps: Vec<isize>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub overlap_dirs: Vec<Dirs>,
}

impl DilemmaNodes {
    pub fn new() -> Self {
        Self {
            dilemma_dirs: Vec::new(),
            dilemma_regions: Vec::new(),
            overlap: Vec::new(),
            taken_steps: Vec::new(),
            overlap_dirs: Vec::new(),
        }
    }

    #[allow(dead_code)]
    fn mark_overlapping_dirs(
        &mut self,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        cops_hull: &[InSet],
        esc_dirs: &[Dirs],
    ) {
        let combine_neighbor_dirs =
            |v: usize| edges.neighbors_of(v).map(|n| esc_dirs[n]).fold(esc_dirs[v], Dirs::union);
        //|v: usize| esc_dirs[v];
        let overlaps = |dirs: Dirs| dirs.0.count_ones() > 3;

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
                    if overlaps(combined_dirs.intersection(region_dirs)) {
                        self.overlap[v] |= region_bit;
                        queue.extend(edges.neighbors_of(v));
                    }
                }
            }
        }
    }

    #[allow(dead_code)]
    fn mark_overlapping_components(
        &mut self,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        cops_hull: &[InSet],
        esc_components: &[u32],
    ) {
        let combine_neighbor_components = |v: usize| {
            edges
                .neighbors_of(v)
                .map(|n| esc_components[n])
                .fold(esc_components[v], std::ops::BitOr::bitor)
        };
        let overlaps = |components: u32| components.count_ones() == 2;

        let mut search_start = 0;
        for overlap_nr in 0.. {
            // find first not-yet handled vertex of an overlap region
            let (region_v, region_dirs) = 'find_new_region: {
                let v0 = search_start;
                for (v, h, &marker) in izip!(v0.., &cops_hull[v0..], &self.overlap[v0..]) {
                    if h.contained() && marker == 0 {
                        let combined_dirs = combine_neighbor_components(v);
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
                    let combined_dirs = combine_neighbor_components(v);
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
        escape_directions: &EscapableDirections,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        _active_cops: &[&Character],
    ) {
        let g = escape_directions.graph.data;
        if !escape_directions.graph.represents_current_map || g.norm != Norm::Hex {
            return;
        }
        let esc_dirs = &escape_directions.esc_directions[..];
        let esc_comps = &escape_directions.esc_components[..];

        let mut overlapping_left = self.overlap.clone();

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

            self.taken_steps.fill(isize::MAX);
            self.overlap_dirs.fill(Dirs::EMPTY);

            overlapping_left[component_v] -= component_bit;
            queue.push_back(component_v);

            // all extreme vertices in a given direction.
            let mut extreme_vertices = [g.coordinates_of(component_v); 6];
            while let Some(v) = queue.pop_front() {
                {
                    let combined =
                        edges.neighbors_of(v).map(|n| esc_dirs[n]).fold(esc_dirs[v], Dirs::union);
                    let mut options = Dirs::EMPTY;
                    for &dir in Dirs::unit_bits(Norm::Hex) {
                        let escape_options = dir.add_adjacent_on_hex().setminus(dir);
                        if combined.intersection(escape_options) == escape_options {
                            options.unionize(dir);
                        }
                    }
                    self.overlap_dirs[v].unionize(options);
                }
                for n in edges.neighbors_of(v) {
                    if overlapping_left[n] & component_bit != 0 {
                        overlapping_left[n] -= component_bit;
                        queue.push_back(n);

                        let n_coords = g.coordinates_of(n);
                        for (i, extreme) in izip!(0..6, &mut extreme_vertices) {
                            let diff = g.norm.canonical_coords(n_coords - *extreme);
                            let diff_dir = match i {
                                0 => diff.e3_line_index(),
                                1 => diff.e2_line_index(),
                                2 => -diff.e1_line_index(),
                                3 => -diff.e3_line_index(),
                                4 => -diff.e2_line_index(),
                                5 => diff.e1_line_index(),
                                _ => unreachable!(),
                            };
                            if diff_dir > 0 {
                                *extreme = n_coords;
                            }
                        }
                    }
                }
            }

            let thickness = {
                let mut min_diff = isize::MAX;
                let (fst_half, snd_half) = extreme_vertices.split_at(3);
                for (i, &ex_1, &ex_2) in izip!(0..3, fst_half, snd_half) {
                    let diff = g.norm.canonical_coords(ex_1 - ex_2);
                    let diff_dir = match i {
                        0 => diff.e3_line_index(),
                        1 => diff.e2_line_index(),
                        2 => diff.e1_line_index(),
                        _ => unreachable!(),
                    };
                    min_diff = isize::min(min_diff, diff_dir.abs());
                }
                min_diff + 1
            };

            queue.clear();
            for (v, neighs, &res) in izip!(0.., edges.neighbors(), &self.overlap_dirs) {
                let v_count = res.0.count_ones();
                if esc_dirs[v].is_empty() && v_count >= 2 {
                    self.dilemma_dirs[v].unionize(res);
                }
                for n in neighs {
                    let n_count = self.overlap_dirs[n].0.count_ones();
                    if n_count >= 2 && n_count > v_count {
                        self.dilemma_regions[v] |= 1;
                        self.taken_steps[n] = 0;
                        queue.push_back(v);
                        break;
                    }
                }
            }

            let escape_bits = edges
                .neighbors_of(component_v)
                .map(|n| esc_comps[n])
                .fold(esc_comps[component_v], std::ops::BitOr::bitor);

            'pop_queue: while let Some(v) = queue.pop_front() {
                let v_coords = g.coordinates_of(v);

                let in_safe_region = esc_comps[v] & escape_bits != 0;
                let continued_dirs = if self.taken_steps[v] == isize::MAX && in_safe_region {
                    Dirs::all_bits_and_directions(g.norm)
                        .filter_map(|(&dir, &coords)| {
                            g.index_of(v_coords + coords).map(|index| {
                                self.overlap_dirs[index].intersection(dir.add_adjacent_on_hex())
                            })
                        })
                        .fold(self.overlap_dirs[v], Dirs::union)
                } else {
                    Dirs::all_bits_and_directions(g.norm)
                        .filter_map(|(&dir, &coords)| {
                            g.index_of(v_coords + coords)
                                .map(|index| self.overlap_dirs[index].intersection(dir))
                        })
                        .fold(self.overlap_dirs[v], Dirs::union)
                };

                let neigh_steps = {
                    let allowed_dirs = if in_safe_region {
                        continued_dirs.add_adjacent_on_hex()
                    } else {
                        continued_dirs
                    };
                    let mut res = smallvec::SmallVec::<[_; 6]>::new();
                    for (&dir, &coords) in Dirs::all_bits_and_directions(g.norm) {
                        if allowed_dirs.intersection(dir).nonempty() {
                            if let Some(n) = g.index_of(v_coords + coords) {
                                if self.taken_steps[n] < isize::MAX {
                                    res.push(self.taken_steps[n])
                                }
                            }
                        }
                    }
                    res.sort();
                    res
                };

                for (&steps_1, &steps_2) in neigh_steps.iter().tuple_windows() {
                    if steps_1 == steps_2 {
                        let max_update_val = isize::min(thickness, self.taken_steps[v]);
                        let new_val = if in_safe_region { steps_1 } else { steps_1 + 1 };
                        if new_val < max_update_val {
                            if esc_dirs[v].is_empty() {
                                self.dilemma_dirs[v].unionize(continued_dirs);
                            }
                            self.overlap_dirs[v] = continued_dirs;
                            self.taken_steps[v] = new_val;
                            self.dilemma_regions[v] |= 8;
                            queue.extend(edges.neighbors_of(v));
                        }
                        continue 'pop_queue;
                    }
                }
            }
        }

        //esc_dirs.graph.remove_non_winning(active_cops, &mut self.dilemma_dirs);
        //for (&dir, &overlap, marker) in
        //    izip!(&self.dilemma_dirs, &self.overlap, &mut self.dilemma_regions)
        //{
        //    if dir.is_empty() && overlap == 0 {
        //        *marker = 0;
        //    }
        //}
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

        self.taken_steps.clear();
        self.taken_steps.resize(edges.nr_vertices(), 0);

        self.dilemma_dirs.clear();
        self.dilemma_dirs.resize(edges.nr_vertices(), Dirs::EMPTY);

        self.overlap_dirs.clear();
        self.overlap_dirs.resize(edges.nr_vertices(), Dirs::EMPTY);

        if !esc_dirs.graph.represents_current_map {
            return;
        }

        //self.mark_overlapping_dirs(edges, queue, hull, &esc_dirs.esc_directions);
        self.mark_overlapping_components(edges, queue, hull, &esc_dirs.esc_components);
        //self.mark_overlap_result_dirs(edges, esc_dirs, hull, queue);
        self.mark_dilemma(esc_dirs, edges, queue, active_cops);
    }
}
