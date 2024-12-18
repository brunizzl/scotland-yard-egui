use grid::*;

use super::*;

/// when two sets of escapable vertices overlap, they allow the robber to approach both at once and thus
/// only allow the police to move one such set away from the robber.
/// this is higly experimental and not yet correct.
pub struct DilemmaNodes {
    /// has as many entries as distinct overlap regions where found.
    /// each entry consists of (a vertex of the region, the region marker, the region directions)
    region_info: Vec<(usize, u32, Dirs)>,

    pub dilemma_dirs: Vec<Dirs>,

    pub dilemma_regions: Vec<u32>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub overlap: Vec<u32>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub energy: Vec<isize>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    pub energy_dirs: Vec<Dirs>,
}

/// stage 1: just consider cones for overlap.
/// stage: n + 1: consider cones and results of all previous n stages for overlap.
/// any value above 1 is higly experimental and not guaranteed to yield a continuous construct.
const MAX_STAGES: usize = 1;

impl DilemmaNodes {
    pub fn new() -> Self {
        Self {
            region_info: Vec::new(),
            dilemma_dirs: Vec::new(),
            dilemma_regions: Vec::new(),
            overlap: Vec::new(),
            energy: Vec::new(),
            energy_dirs: Vec::new(),
        }
    }

    fn mark_overlapping_dirs(
        &mut self,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        cops_hull: &[InSet],
    ) -> std::ops::Range<usize> {
        let esc_dirs = &self.dilemma_dirs[..];
        // guarentee only vertices where multiple escape options exist are marked as escapeable.
        // not tested here: every marked direction must lead on a straight line to
        // the boundary and every vertex in between must also have the direction marked.
        debug_assert!(esc_dirs.iter().all(|&dirs| dirs.count() != 1));

        let fst_new_region_index = self.region_info.len();

        let mut search_start = 0;
        'find_regions: for overlap_nr in fst_new_region_index.. {
            // find first not-yet handled vertex of an overlap region
            let (region_v, region_dirs) = 'find_new_region: {
                let v0 = search_start;
                for (v, h, &marker) in izip!(v0.., &cops_hull[v0..], &self.overlap[v0..]) {
                    if h.contained() && marker == 0 {
                        let dirs = esc_dirs[v];
                        if dirs.count() == 4 && dirs.connected_on(Norm::Hex) {
                            search_start = v + 1;
                            break 'find_new_region (v, dirs);
                        }
                    }
                }
                break 'find_regions;
            };
            let region_bit = 1u32 << (overlap_nr % 32);
            self.region_info.push((region_v, region_bit, region_dirs));

            queue.clear();
            queue.push_back(region_v);
            while let Some(v) = queue.pop_front() {
                if cops_hull[v].contained() && (self.overlap[v] & region_bit == 0) {
                    let dirs = esc_dirs[v];
                    if dirs.contains(region_dirs) {
                        self.overlap[v] |= region_bit;
                        queue.extend(edges.neighbors_of(v));
                    }
                }
            }
        }

        fst_new_region_index..self.region_info.len()
    }

    fn mark_dilemma(
        &mut self,
        escape_directions: &EscapableDirections,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        cops_hull: &[InSet],
        min_cop_dist: &[isize],
        new_regions: std::ops::Range<usize>,
    ) {
        let g = escape_directions.graph.data;
        if !escape_directions.graph.represents_current_map || g.norm != Norm::Hex {
            return;
        }
        let old_dirs = self.dilemma_dirs.clone();

        // helps to find regions.
        let mut overlapping_left = self.overlap.clone();

        for &(component_v, component_bit, component_dirs) in &self.region_info[new_regions] {
            debug_assert!(overlapping_left[component_v] & component_bit != 0);
            debug_assert!(component_dirs.connected_on(Norm::Hex));
            debug_assert_eq!(component_dirs.count(), 4);

            let escape_dirs = component_dirs.keep_inner_on_hex();
            debug_assert!(escape_dirs.connected_on(Norm::Hex));
            debug_assert_eq!(escape_dirs.count(), 2);

            overlapping_left[component_v] -= component_bit;
            queue.push_back(component_v);

            // one extreme vertex per direction.
            let mut extreme_vertices = [g.coordinates_of(component_v); 6];
            // find extreme vertices and remove current region from overlapping_left
            while let Some(v) = queue.pop_front() {
                for n in edges.neighbors_of(v) {
                    if overlapping_left[n] & component_bit != 0 {
                        overlapping_left[n] -= component_bit;
                        queue.push_back(n);

                        let n_coords = g.coordinates_of(n);
                        for (i, extreme) in izip!(0..6, &mut extreme_vertices) {
                            let diff = n_coords - *extreme;
                            let diff_dir = match i {
                                0 => diff.line_e1_index(),
                                1 => diff.line_e2_index(),
                                2 => diff.line_e3_index(),
                                3 => -diff.line_e1_index(),
                                4 => -diff.line_e2_index(),
                                5 => -diff.line_e3_index(),
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
                    let diff = ex_1 - ex_2;
                    let diff_dir = match i {
                        0 => diff.line_e1_index(),
                        1 => diff.line_e2_index(),
                        2 => diff.line_e3_index(),
                        _ => unreachable!(),
                    };
                    min_diff = isize::min(min_diff, diff_dir.abs());
                }
                min_diff
            };

            // these are the four directions of component_dirs as coords and
            // always ordered such that directions at neighboring indices are adjacent
            let all_relevant_coords = {
                let mut res = [Coords { x: isize::MAX, y: isize::MAX }; 4];
                let vals = Dirs::all_bits_and_directions(Norm::Hex)
                    .cycle()
                    .skip_while(|(&dir, _)| component_dirs.intersection(dir).nonempty())
                    .skip_while(|(&dir, _)| component_dirs.intersection(dir).is_empty());
                for (ptr, (&dir, &val)) in izip!(&mut res, vals) {
                    debug_assert!(component_dirs.intersection(dir).nonempty());
                    *ptr = val;
                }
                let adjacent = |(&d1, &d2)| Norm::Hex.apply(d1 - d2) == 1;
                debug_assert!(res.iter().tuple_windows().all(adjacent));
                res
            };
            // the coordinates of the two directions in escape_dirs
            let fwd_steps = [all_relevant_coords[1], all_relevant_coords[2]];
            let [left_step_fwd, right_step_fwd] = fwd_steps;
            let fwd_dirs = [
                left_step_fwd.dirs(Norm::Hex),
                right_step_fwd.dirs(Norm::Hex),
            ];
            let [left_dir_fwd, right_dir_fwd] = fwd_dirs;
            debug_assert_eq!(left_dir_fwd.count(), 1);
            debug_assert_eq!(right_dir_fwd.count(), 1);
            debug_assert_eq!(left_dir_fwd.union(right_dir_fwd), escape_dirs);
            // the two "outher" directions
            let [left_step, right_step] = [all_relevant_coords[0], all_relevant_coords[3]];
            debug_assert_eq!(left_step, -right_step);

            self.energy.fill(isize::MIN);
            self.energy_dirs.fill(Dirs::EMPTY);
            queue.push_back(component_v);
            'compute_energy: while let Some(v) = queue.pop_front() {
                if self.energy[v] != isize::MIN || min_cop_dist[v] < 2 || cops_hull[v].outside() {
                    continue 'compute_energy;
                }
                if old_dirs[v].contains(component_dirs) {
                    // v is part of the original overlap
                    self.energy[v] = thickness;
                    self.energy_dirs[v] = escape_dirs;
                    self.dilemma_regions[v] |= component_bit;
                    debug_assert!(self.dilemma_dirs[v].contains(component_dirs));

                    queue.extend(edges.neighbors_of(v));
                    continue 'compute_energy;
                }

                // the energy can only be as large as
                // the thinnest section connecting some vertex to vertices before it.
                // this row is thus the longest connected subset of the
                // grid line in parallel_dir_coords directions containing v,
                // where every vertex has a neighbor in a esc_dir_coords direction with positive energy.
                let (row_fst, row_len, row_dirs) = {
                    let v_coords = g.coordinates_of(v);
                    let mut v_left = v_coords;
                    let mut v_right = v_coords;
                    let mut row_len = 1;
                    let mut row_dirs = escape_dirs;
                    for _ in 0..g.grid.len {
                        let Some(next_left) = g.try_wrap(v_left + left_step) else {
                            break;
                        };
                        if min_cop_dist[g.unchecked_index_of(next_left)] < 2 {
                            row_dirs = row_dirs.setminus(right_dir_fwd);
                            debug_assert!(row_dirs.count() <= 1);
                            break;
                        }
                        let Some(index_fwd) = g.index_of(next_left + right_step_fwd) else {
                            break;
                        };
                        if self.energy[index_fwd] < 0 {
                            break;
                        }
                        row_dirs.intersect(self.energy_dirs[index_fwd]);
                        v_left = next_left;
                        row_len += 1;
                    }
                    for _ in 0..g.grid.len {
                        let Some(next_right) = g.try_wrap(v_right + right_step) else {
                            break;
                        };
                        if min_cop_dist[g.unchecked_index_of(next_right)] < 2 {
                            row_dirs = row_dirs.setminus(left_dir_fwd);
                            debug_assert!(row_dirs.count() <= 1);
                            break;
                        }
                        let Some(index_fwd) = g.index_of(next_right + left_step_fwd) else {
                            break;
                        };
                        if self.energy[index_fwd] < 0 {
                            break;
                        }
                        row_dirs.intersect(self.energy_dirs[index_fwd]);
                        v_right = next_right;
                        row_len += 1;
                    }
                    debug_assert_eq!(
                        g.try_wrap(v_left + (row_len - 1) * right_step),
                        Some(v_right)
                    );
                    (v_left, row_len, row_dirs)
                };

                // we update the energy of all vertices of the row at once,
                // because now we know the row width.
                'update_row: for i in 0..row_len {
                    let vi_coords = row_fst + i * right_step;
                    let Some(vi) = g.index_of(vi_coords) else {
                        continue 'update_row;
                    };
                    let in_old_cone = old_dirs[vi].intersection(component_dirs).count() >= 2;

                    let new_step_energy = {
                        let mut energies = [isize::MIN; 2];
                        for (step_fwd, step_dir, energy) in
                            izip!(fwd_steps, fwd_dirs, &mut energies)
                        {
                            if let Some(n) = g.index_of(vi_coords + step_fwd) {
                                if !in_old_cone || row_dirs.contains(step_dir) {
                                    *energy = self.energy[n];
                                }
                            }
                        }
                        if in_old_cone {
                            // in cone -> enough to have one neighbor with same function value
                            isize::max(energies[0], energies[1])
                        } else {
                            // outside cone -> both neighbors must have energy and we lose energy
                            isize::min(energies[0], energies[1]).saturating_sub(1)
                        }
                    };

                    if new_step_energy >= 0 {
                        self.energy_dirs[vi] = row_dirs;
                        self.dilemma_regions[vi] |= component_bit;
                        if MAX_STAGES == 1 || !in_old_cone {
                            self.dilemma_dirs[vi].unionize(escape_dirs);
                        }
                    }
                    if new_step_energy > self.energy[vi] {
                        self.energy[vi] = new_step_energy;
                        queue.extend(fwd_steps.iter().filter_map(|&c| g.index_of(vi_coords - c)));
                    }
                }
            }
        }
    }

    pub fn update(
        &mut self,
        edges: &EdgeList,
        hull: &[InSet],
        esc_dirs: &EscapableDirections,
        queue: &mut VecDeque<usize>,
        min_cop_dist: &[isize],
    ) {
        assert_eq!(hull.len(), edges.nr_vertices());
        assert_eq!(esc_dirs.esc_directions.len(), edges.nr_vertices());

        self.overlap.clear();
        self.overlap.resize(edges.nr_vertices(), 0);

        self.dilemma_regions.clear();
        self.dilemma_regions.resize(edges.nr_vertices(), 0);

        self.energy.clear();
        self.energy.resize(edges.nr_vertices(), isize::MIN);

        self.dilemma_dirs.resize(edges.nr_vertices(), Dirs::EMPTY);
        self.dilemma_dirs.copy_from_slice(&esc_dirs.cone_esc_directions);

        self.energy_dirs.clear();
        self.energy_dirs.resize(edges.nr_vertices(), Dirs::EMPTY);

        if !esc_dirs.graph.represents_current_map || esc_dirs.graph.data.norm != Norm::Hex {
            return;
        }

        self.region_info.clear();
        for _ in 0..MAX_STAGES {
            let new_regions = self.mark_overlapping_dirs(edges, queue, hull);
            if new_regions.is_empty() {
                break;
            }
            self.mark_dilemma(esc_dirs, edges, queue, hull, min_cop_dist, new_regions);
        }
    }
}
