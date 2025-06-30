use grid::*;

use super::*;

struct Cone(Coords, Dirs);

impl Cone {
    #[inline(always)]
    fn contains(&self, v: Coords) -> bool {
        let Cone(corner, dirs) = *self;
        debug_assert!(dirs.connected_on(Norm::Hex));
        dirs.contains((v - corner).dirs(Norm::Hex))
    }
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
const MAX_STAGES: usize = 1;

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
                    if !h.contained() {
                        continue;
                    }
                    let dirs = all_dirs[v];
                    let dirs_count = dirs.count();
                    if dirs_count == 4 && marker == 0 && dirs.connected_on(Norm::Hex) {
                        search_start = v + 1;
                        break 'find_new_region (v, dirs);
                    }
                    if dirs_count == 5 {
                        // to handle this is only required in very niche cases:
                        // only if every vertex of a region is also contained in
                        // some other region.
                        let nr_markers = marker.count_ones();
                        let new_dirs = match nr_markers {
                            0 => dirs.intersection(dirs.rotate_left_hex()),
                            1 => {
                                let other_overlap_nr = marker.ilog2() as usize;
                                let other_dirs = self.region_info[other_overlap_nr].2;
                                debug_assert!(dirs.contains(other_dirs));
                                let left_dirs = dirs.intersection(dirs.rotate_left_hex());
                                let right_dirs = dirs.intersection(dirs.rotate_right_hex());
                                if other_dirs == left_dirs {
                                    right_dirs
                                } else {
                                    debug_assert_eq!(other_dirs, right_dirs);
                                    left_dirs
                                }
                            },
                            _ => continue,
                        };
                        debug_assert_eq!(new_dirs.count(), 4);
                        debug_assert!(new_dirs.connected_on(Norm::Hex));
                        search_start = v;
                        break 'find_new_region (v, new_dirs);
                    }
                    if dirs_count == 6 {
                        // in principle, one should also handle this case, however
                        // the case above is already highly unlikely.
                        // this case `dirs_count == 6` must only be considered if every vertex
                        // of a region is also contained in TWO other regions.
                        // i have a hard time imagining this to occur in an interesting cop setup.
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
        active_cops_coords: &[Coords],
        new_regions: std::ops::Range<usize>,
    ) {
        use Norm::Hex;
        let g = escape_directions.graph.data;
        if !escape_directions.graph.represents_current_map || g.norm != Hex || g.wrap {
            return;
        }

        for &(component_v, component_bit, component_dirs) in &self.region_info[new_regions] {
            debug_assert!(self.overlap[component_v] & component_bit != 0);
            debug_assert!(component_dirs.connected_on(Hex));
            debug_assert_eq!(component_dirs.count(), 4);

            let escape_dirs = component_dirs.keep_inner_on_hex();
            debug_assert!(escape_dirs.connected_on(Hex));
            debug_assert_eq!(escape_dirs.count(), 2);

            // one extreme vertex per direction.
            let mut extreme_vertices = [g.coordinates_of(component_v); 6];
            // find extreme vertices of overlap.
            // note: this procedure assumes the overlapping region to be convex.
            // (or at least not spiral-shaped or something like that.)
            queue.push_back(component_v);
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

            // a measure of how many police moves the overlap is guranteed to persist.
            let thickness = {
                let [ex1, ex2, ex3, ex4, ex5, ex6] = extreme_vertices;
                let diff_1_4 = (ex1 - ex4).line_e1_index();
                let diff_2_5 = (ex2 - ex5).line_e2_index();
                let diff_3_6 = (ex3 - ex6).line_e3_index();
                debug_assert!(diff_1_4 >= 0);
                debug_assert!(diff_2_5 >= 0);
                debug_assert!(diff_3_6 >= 0);
                isize::min(diff_1_4, diff_2_5).min(diff_3_6)
            };

            // these are the four directions of component_dirs as coords (called steps) and as direction sets
            let [
                (left_step, left_dir),
                (left_step_fwd, left_dir_fwd),
                (rght_step_fwd, rght_dir_fwd),
                (rght_step, rght_dir),
            ] = {
                let mut res = [(Coords::MAX, Dirs::EMPTY); 4];
                // gurantee to have all values ordered not by absolute direction index,
                // but such that two following values are also geometrically adjacent.
                let vals = Dirs::all_bits_and_directions(Hex)
                    .cycle()
                    .skip_while(|&(&dir, _)| component_dirs.intersection(dir).nonempty())
                    .skip_while(|&(&dir, _)| component_dirs.intersection(dir).is_empty());
                for (ptr, (&dir, &val)) in izip!(&mut res, vals) {
                    debug_assert!(component_dirs.intersection(dir).nonempty());
                    *ptr = (val, dir);
                }
                debug_assert!(res.iter().tuple_windows().all(|(&(s1, d1), &(s2, d2))| {
                    let steps_adjacent = Hex.apply(s1 - s2) == 1;
                    let dirs_adjacent = Dirs::union(d1, d2).connected_on(Hex);
                    steps_adjacent && dirs_adjacent
                }));
                res
            };
            debug_assert_eq!(left_step, -rght_step);
            let fwd_steps = [left_step_fwd, rght_step_fwd];

            // the two sets of connected directions of the original cones which caused the overlap.
            let left_cone_dirs = Dirs::union(left_dir, left_dir_fwd);
            let rght_cone_dirs = Dirs::union(rght_dir_fwd, rght_dir);
            debug_assert_eq!(Dirs::union(left_dir_fwd, rght_dir_fwd), escape_dirs);
            debug_assert_eq!(Dirs::union(left_cone_dirs, rght_cone_dirs), component_dirs);
            // the two directions not contained in the two cones (the opposite of escape_dirs)
            let cops_dirs = Dirs::all(Hex).setminus(component_dirs);
            debug_assert!(Dirs::intersection(left_cone_dirs, rght_cone_dirs).is_empty());
            debug_assert!(Dirs::intersection(left_cone_dirs, cops_dirs).is_empty());
            debug_assert!(Dirs::intersection(cops_dirs, rght_cone_dirs).is_empty());

            // these is the smallest subset of "close" (e.g. not beyond the overlap) cop positions,
            // such that every close cop is contained in the union
            // over the cones in cops_dirs from the subset vertices.
            // this subset however does not contain the cop positions directly,
            // but shifted in such a manner that also exactly the boundary outside each
            // cop cone is contained in the shifted cone.
            let dangerous_cop_cone_corners = {
                let v0 = g.coordinates_of(component_v);
                debug_assert!(Cone(v0, escape_dirs).contains(v0 + left_step_fwd));
                debug_assert!(Cone(v0, escape_dirs).contains(v0 + rght_step_fwd));
                debug_assert!(!Cone(v0, escape_dirs).contains(v0 + left_step));
                debug_assert!(!Cone(v0, escape_dirs).contains(v0 + rght_step));

                debug_assert!(Cone(v0, cops_dirs).contains(v0 - left_step_fwd));
                debug_assert!(Cone(v0, cops_dirs).contains(v0 - rght_step_fwd));
                debug_assert!(!Cone(v0, cops_dirs).contains(v0 + left_step));
                debug_assert!(!Cone(v0, cops_dirs).contains(v0 + rght_step));

                debug_assert!(Cone(v0, component_dirs).contains(v0 + left_step));
                debug_assert!(Cone(v0, component_dirs).contains(v0 + rght_step));
                debug_assert!(!Cone(v0, component_dirs).contains(v0 - left_step_fwd));
                debug_assert!(!Cone(v0, component_dirs).contains(v0 - rght_step_fwd));

                // find the minimum set of cops that create all dangerous cones
                let mut danger = smallvec::SmallVec::<[_; 8]>::new();
                for &new_cop in active_cops_coords {
                    let cop_not_dangerous = Cone(v0, escape_dirs).contains(new_cop);
                    if cop_not_dangerous {
                        continue;
                    }
                    if !danger.iter().any(|&old| Cone(old, cops_dirs).contains(new_cop)) {
                        danger.retain(|&mut old| !Cone(new_cop, cops_dirs).contains(old));
                        danger.push(new_cop);
                    }
                }
                // shift every cone by two units such that both boundaries are now in the interior.
                let shift = left_step_fwd + rght_step_fwd;
                debug_assert_eq!((-shift).dirs(Hex), cops_dirs);
                for v_coords in &mut danger {
                    *v_coords = *v_coords + shift;
                }
                danger
            };
            let dangerous_cop_cone_corners = &dangerous_cop_cone_corners[..];
            let in_danger = |v: Coords| -> bool {
                dangerous_cop_cone_corners.iter().any(|&cone_corner| {
                    // because every cone was shifted, we only need to test if v is in shifted cone,
                    // and is not identical to the very tip.
                    Cone(cone_corner, cops_dirs).contains(v) && v != cone_corner
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

                let in_old_cone = {
                    let dirs = self.all_dirs[v];
                    dirs.contains(left_cone_dirs) || dirs.contains(rght_cone_dirs)
                };
                let new_step_energy = {
                    let get = |n| g.index_of(n).map_or(isize::MIN, |n| self.energy[n]);
                    let e1 = get(v_coords + left_step_fwd);
                    let e2 = get(v_coords + rght_step_fwd);
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

    pub fn update<'a>(
        &mut self,
        edges: &EdgeList,
        hull: &[InSet],
        esc_dirs: &EscapableDirections,
        queue: &mut VecDeque<usize>,
        active_cops: impl Iterator<Item = &'a Character>,
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

        let active_cops_coords = active_cops
            .map(|c| esc_dirs.graph.data.coordinates_of(c.vertex()))
            .collect_vec();

        self.region_info.clear();
        for _ in 0..MAX_STAGES {
            let new_regions = self.mark_overlapping_dirs(edges, queue, hull);
            if new_regions.is_empty() {
                break;
            }
            self.mark_dilemma(
                esc_dirs,
                edges,
                queue,
                hull,
                &active_cops_coords,
                new_regions,
            );
        }

        // add the directions found via other means (and those outside hull)
        for (all, &esc) in izip!(&mut self.all_dirs, &esc_dirs.esc_directions,) {
            all.unionize(esc);
        }
    }
}
