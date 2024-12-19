use grid::*;

use super::*;

fn cone_contains(v: Coords, cone_corner: Coords, cone_dirs: Dirs) -> bool {
    cone_dirs.contains((v - cone_corner).dirs(Norm::Hex))
}

/// when two escapable cones overlap, they allow the robber to approach both at once and thus
/// only allow the police to move one such set away from the robber.
/// this struct tries to compute which vertices allow a robber escape to one of these two cones.
pub struct DilemmaNodes {
    /// has as many entries as distinct overlap regions where found.
    /// each entry consists of (a vertex of the region, the region marker, the region directions)
    region_info: Vec<(usize, u32, Dirs)>,

    /// union of [`EscapableDirections::cone_esc_directions`] and [`Self::dilemma_dirs`].
    pub all_dirs: Vec<Dirs>,

    /// has the same vertices marked as [`Self::dilemma_regions`], 
    /// but remembers the escape directions instead of the component number.
    pub dilemma_dirs: Vec<Dirs>,

    /// marks regions that allow escape to a given overlap.
    /// suberset of [`Self::overlap`].
    pub dilemma_regions: Vec<u32>,

    /// marks regions where two cones with fitting directions overlap.
    /// subset of [`Self::dilemma_regions`].
    pub overlap: Vec<u32>,

    /// temporary value, only stored here to allow visual debugging (and to save on allocations).
    /// this tells us at the time where a given region is found, 
    /// how many more steps away from a cone we can safely take.
    pub energy: Vec<isize>,
}

/// stage 1: just consider cones for overlap.
/// stage: n + 1: consider cones and results of all previous n stages for overlap.
/// any value above 1 is higly experimental and not guaranteed to yield a continuous construct.
const MAX_STAGES: usize = 100;

impl DilemmaNodes {
    pub fn new() -> Self {
        Self {
            region_info: Vec::new(),
            all_dirs: Vec::new(),
            dilemma_dirs: Vec::new(),
            dilemma_regions: Vec::new(),
            overlap: Vec::new(),
            energy: Vec::new(),
        }
    }

    fn mark_overlapping_dirs(
        &mut self,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        cops_hull: &[InSet],
    ) -> std::ops::Range<usize> {
        let all_dirs = &self.all_dirs[..];
        // guarentee only vertices where multiple escape options exist are marked as escapeable.
        // not tested here: every marked direction must lead on a straight line to
        // the boundary and every vertex in between must also have the direction marked.
        debug_assert!(all_dirs.iter().all(|&dirs| dirs.count() != 1));

        let fst_new_region_index = self.region_info.len();

        let mut search_start = 0;
        'find_regions: for overlap_nr in fst_new_region_index.. {
            // find first not-yet handled vertex of an overlap region
            let (region_v, region_dirs) = 'find_new_region: {
                let v0 = search_start;
                for (v, h, &marker) in izip!(v0.., &cops_hull[v0..], &self.overlap[v0..]) {
                    if h.contained() && marker == 0 {
                        let dirs = all_dirs[v];
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
                    let dirs = all_dirs[v];
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
        active_cops: &[&Character],
        new_regions: std::ops::Range<usize>,
    ) {
        let g = escape_directions.graph.data;
        if !escape_directions.graph.represents_current_map || g.norm != Norm::Hex {
            return;
        }

        for &(component_v, component_bit, component_dirs) in &self.region_info[new_regions] {
            debug_assert!(self.overlap[component_v] & component_bit != 0);
            debug_assert!(component_dirs.connected_on(Norm::Hex));
            debug_assert_eq!(component_dirs.count(), 4);

            let escape_dirs = component_dirs.keep_inner_on_hex();
            debug_assert!(escape_dirs.connected_on(Norm::Hex));
            debug_assert_eq!(escape_dirs.count(), 2);

            queue.push_back(component_v);

            // one extreme vertex per direction.
            let mut extreme_vertices = [g.coordinates_of(component_v); 6];
            // find extreme vertices of overlap.
            // note: this assumes the overlapping region to be convex.
            while let Some(v) = queue.pop_front() {
                for n in edges.neighbors_of(v) {
                    if self.overlap[n] & component_bit == 0 {
                        continue;
                    }
                    let n_coords = g.coordinates_of(n);
                    let mut new_extreme = false;
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
                            new_extreme = true;
                        }
                    }
                    if new_extreme {
                        queue.push_back(n);
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

            let cops_dirs = Dirs::all(Norm::Hex).setminus(component_dirs);
            let dangerous_cop_cone_corners = {
                let v0 = g.coordinates_of(component_v);
                debug_assert!(cone_contains(v0 + left_step_fwd, v0, escape_dirs));
                debug_assert!(cone_contains(v0 + right_step_fwd, v0, escape_dirs));
                debug_assert!(!cone_contains(v0 + left_step, v0, escape_dirs));
                debug_assert!(!cone_contains(v0 + right_step, v0, escape_dirs));

                debug_assert!(cone_contains(v0 - left_step_fwd, v0, cops_dirs));
                debug_assert!(cone_contains(v0 - right_step_fwd, v0, cops_dirs));
                debug_assert!(!cone_contains(v0 + left_step, v0, cops_dirs));
                debug_assert!(!cone_contains(v0 + right_step, v0, cops_dirs));

                debug_assert!(cone_contains(v0 + left_step, v0, component_dirs));
                debug_assert!(cone_contains(v0 + right_step, v0, component_dirs));
                debug_assert!(!cone_contains(v0 - left_step_fwd, v0, component_dirs));
                debug_assert!(!cone_contains(v0 - right_step_fwd, v0, component_dirs));

                let mut dangerous_cops = smallvec::SmallVec::<[_; 8]>::new();
                for &cop in active_cops {
                    let cop_coords = g.coordinates_of(cop.vertex());
                    let cop_not_dangerous = cone_contains(cop_coords, v0, escape_dirs);
                    if cop_not_dangerous {
                        continue;
                    }
                    if !dangerous_cops.iter().any(|&cc| cone_contains(cop_coords, cc, cops_dirs)) {
                        dangerous_cops.retain(|&mut cc| !cone_contains(cc, cop_coords, cops_dirs));
                        dangerous_cops.push(cop_coords);
                    }
                }
                // shift every cone by two units such that both boundaries now are in the interior.
                for v_coords in &mut dangerous_cops {
                    *v_coords = *v_coords + left_step_fwd + right_step_fwd;
                }
                dangerous_cops
            };
            let dangerous_cop_cone_corners = &dangerous_cop_cone_corners[..];
            let in_danger = |v: Coords| -> bool {
                dangerous_cop_cone_corners.iter().any(|&cone_corner| {
                    cone_contains(v, cone_corner, cops_dirs) && v != cone_corner
                })
            };

            self.energy.fill(isize::MIN);
            queue.push_back(component_v);
            'compute_energy: while let Some(v) = queue.pop_front() {
                if self.energy[v] != isize::MIN || cops_hull[v].outside() {
                    continue 'compute_energy;
                }
                if self.all_dirs[v].contains(component_dirs) {
                    // v is part of the original overlap
                    self.energy[v] = thickness;
                    self.dilemma_regions[v] |= component_bit;
                    self.dilemma_dirs[v].unionize(component_dirs);
                    debug_assert!(!in_danger(g.coordinates_of(v)));

                    queue.extend(edges.neighbors_of(v));
                    continue 'compute_energy;
                }

                let v_coords = g.coordinates_of(v);
                if in_danger(v_coords) {
                    continue 'compute_energy;
                }

                let in_old_cone = self.all_dirs[v].intersection(component_dirs).count() >= 2;
                let new_step_energy = {
                    let get = |n| g.index_of(n).map_or(isize::MIN, |n| self.energy[n]);
                    let e1 = get(v_coords + left_step_fwd);
                    let e2 = get(v_coords + right_step_fwd);
                    if in_old_cone {
                        // in cone -> enough to have one neighbor with same function value
                        isize::max(e1, e2)
                    } else {
                        // outside cone -> both neighbors must have energy and we lose energy
                        isize::min(e1, e2).saturating_sub(1)
                    }
                };
                if new_step_energy >= 0 {
                    self.dilemma_regions[v] |= component_bit;
                    if MAX_STAGES == 1 || !in_old_cone {
                        self.dilemma_dirs[v].unionize(escape_dirs);
                    }
                }
                if new_step_energy > self.energy[v] {
                    self.energy[v] = new_step_energy;
                    queue.extend(fwd_steps.iter().filter_map(|&s| g.index_of(v_coords - s)));
                }
            }
        }

        for (all, &dil) in izip!(&mut self.all_dirs, &self.dilemma_dirs) {
            all.unionize(dil);
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

        self.energy.clear();
        self.energy.resize(edges.nr_vertices(), isize::MIN);

        self.dilemma_dirs.clear();
        self.dilemma_dirs.resize(edges.nr_vertices(), Dirs::EMPTY);

        self.all_dirs.resize(edges.nr_vertices(), Dirs::EMPTY);
        self.all_dirs.copy_from_slice(&esc_dirs.cone_esc_directions);

        if !esc_dirs.graph.represents_current_map || esc_dirs.graph.data.norm != Norm::Hex {
            return;
        }

        self.region_info.clear();
        for _ in 0..MAX_STAGES {
            let new_regions = self.mark_overlapping_dirs(edges, queue, hull);
            if new_regions.is_empty() {
                break;
            }
            self.mark_dilemma(esc_dirs, edges, queue, hull, active_cops, new_regions);
        }

        // esc_dirs.cone_esc_directions are only marked inside hull -> add rest now
        for (all, h, &esc) in izip!(&mut self.all_dirs, hull, &esc_dirs.esc_directions,) {
            if !h.contained() {
                all.unionize(esc);
            }
        }
    }
}
