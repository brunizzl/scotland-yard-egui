use super::*;
use grid::*;

/// should compute the same thing as [`hull::EscapableNodes`],
/// if graph in question is a grid (optionally wrapped to torus)
pub struct EscapableDirections {
    /// in quad grids the first four bits per entry store,
    /// wether it is safe to escape in the corresponding of the four directions.
    /// in hex grids, the first 6 bits are used.
    pub escapable: Vec<u8>,
    /// distance of every vertex to vertex `0`, which is also `Coords{ x: 0, y: 0 }`.
    /// used to compute arbitrary distances on tori
    dists_0: Vec<isize>,
    /// remembers graph of last update.
    /// if that is unchanged, [`Self::dists_0`] can be kept the same.
    graph: GridGraph,
}

impl EscapableDirections {
    pub fn new() -> Self {
        Self {
            escapable: Vec::new(),
            dists_0: Vec::new(),
            graph: GridGraph {
                columns: OrderedColWise { len: 0 },
                norm: Norm::Quad,
                wrap: false,
            },
        }
    }

    pub fn escape_dir_emojies(&self) -> &'static [char] {
        match self.graph.norm {
            Norm::Quad => &['➡', '⬇', '⬅', '⬆'],
            Norm::Hex => &['➡', '↘', '↙', '⬅', '↖', '↗'],
        }
    }

    fn update_dists_dirs(
        &mut self,
        map: &Embedding3D,
        queue: &mut VecDeque<usize>,
        hull_data: &ConvexHullData,
    ) -> bool {
        let Some(g) = GridGraph::try_from(map) else {
            return false;
        };

        // update distances
        debug_assert!(g.side_len() > 0);
        if g.wrap && self.graph != g {
            self.dists_0.clear();
            self.dists_0.resize(map.nr_vertices(), isize::MAX);
            self.dists_0[0] = 0;
            queue.clear();
            queue.push_back(0);
            map.edges().calc_distances_to(queue, &mut self.dists_0);
        }
        self.graph = g;

        // initialize escapable to have the complete hull inside marked
        self.escapable.clear();
        self.escapable.resize(map.nr_vertices(), 0);
        let all = (1u8 << self.graph.norm.unit_directions().len()) - 1;
        for (esc, inside) in izip!(&mut self.escapable, hull_data.hull()) {
            if inside.contained() {
                *esc = all;
            }
        }

        true
    }

    pub fn update<'a>(
        &mut self,
        map: &Embedding3D,
        queue: &mut VecDeque<usize>,
        hull_data: &ConvexHullData,
        active_cops: impl Iterator<Item = &'a Character>,
    ) {
        if !self.update_dists_dirs(map, queue, hull_data) {
            return;
        }
        let g = self.graph;

        let cops_vec = active_cops.collect_vec();
        let dist = |a, b| {
            if g.wrap {
                let Coords { x, y } = a - b;
                let wrapped = g.columns.pack_small_coordinates(x, y);
                self.dists_0[g.index_of(wrapped)]
            } else {
                g.norm.apply(a - b)
            }
        };
        let mut safe_boundaries = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
        'remove_pair_shadows: for (ch_cop1, ch_cop2) in cops_vec.iter().tuple_combinations() {
            let cop1 = g.coordinates_of(ch_cop1.vertex());
            let cop2 = g.coordinates_of(ch_cop2.vertex());
            let cops_dist = dist(cop1, cop2);
            // dist computed via norm must behave the same as dists computed via breadth-first-search
            debug_assert_eq!(ch_cop1.dists()[ch_cop2.vertex()], cops_dist);
            debug_assert_eq!(ch_cop2.dists()[ch_cop1.vertex()], cops_dist);
            if cops_dist < 4 {
                continue 'remove_pair_shadows;
            }
            let path_dirs = {
                // find all directions in which a shortest path from `cop1` to `cop2` starts
                let mut res = smallvec::SmallVec::<[_; 4]>::new();
                for &d in g.norm.unit_directions() {
                    if dist(cop1 + d, cop2) == cops_dist - 1 {
                        res.push(d);
                    }
                }
                // graph is connected -> there is always a shortest path -> .len() >= 1
                debug_assert!(g.wrap || (1..=2).contains(&res.len()));
                // guarantee the fixed storage of safe boundaries suffices
                debug_assert!((1..=safe_boundaries.len()).contains(&res.len()));
                res
            };

            for sb in &mut safe_boundaries {
                sb.clear();
            }
            for (bd, &d1) in izip!(&mut safe_boundaries, &path_dirs) {
                let mut curr = cop1;
                let mut add_if_safe = |curr| {
                    if dist(cop1, curr) >= 2 && dist(cop2, curr) >= 2 {
                        bd.push(curr);
                    }
                };
                for step1 in 0..cops_dist {
                    let plus_d1 = curr + d1;
                    // a single direction to walk from cop1 to cop2 only suffices,
                    // if both stand on a common grid line.
                    // we start walking in d1 until this no longer brings us closer to cop2.
                    if dist(cop2, plus_d1) == cops_dist - (step1 + 1) {
                        curr = g.try_wrap(plus_d1).unwrap();
                        add_if_safe(curr);
                    } else {
                        // we have reached the corner of the hull between cop1 and cop2 and must
                        // now follow the grid line to cop2.
                        let d2 = *path_dirs
                            .iter()
                            .find(|&&d2| dist(cop2, curr + d2) == cops_dist - (step1 + 1))
                            .unwrap();
                        // curr is already on same grid line as cop2
                        debug_assert_eq!(
                            g.try_wrap(curr + (cops_dist - step1) * d2).unwrap(),
                            cop2
                        );
                        for step2 in step1..cops_dist {
                            let plus_d2 = curr + d2;
                            debug_assert_eq!(dist(cop2, plus_d2), cops_dist - (step2 + 1));
                            curr = g.try_wrap(plus_d2).unwrap();
                            add_if_safe(curr);
                        }
                        // we have taken cops_dist many steps -> better have reached cop2
                        debug_assert_eq!(curr, cop2);
                        break;
                    }
                }
                debug_assert_eq!(curr, cop2);
            }

            // for the found safe boundaries, we remove all vertices not part of the induced
            // escapable region for every direction.
            // note: we only remove "behind" the safe boundary,
            // thus vertices "behind" cops / neighbors of cops are removed seperately below.
            for (i, &escape_dir) in izip!(0.., g.norm.unit_directions()) {
                let mask = 1u8 << i;
                for safe_boundary in &safe_boundaries[..path_dirs.len()] {
                    let safe_dist = safe_boundary.len() as isize - 1;
                    debug_assert_eq!(safe_dist, cops_dist - 4);
                    for &path_v in safe_boundary {
                        let mut in_safe_region = true;
                        for step in 0..g.side_len() {
                            let v = path_v - step * escape_dir;
                            let Some(v_repr) = g.try_wrap(v) else {
                                break;
                            };
                            let index = g.index_of(v_repr);
                            if self.escapable[index] & mask == 0 {
                                break;
                            }

                            let still_safe =
                                || safe_boundary.iter().all(|&b| dist(v_repr, b) <= safe_dist);
                            in_safe_region = in_safe_region && still_safe();
                            // on tori, continuously walking in the same direction will not result
                            // in a monotonous distance increase to a given fixed vertex.
                            // on flat grids with boundaries, it will.
                            // the theory will not (directly) consider tori anyway, thus
                            // we act as if we aren't on a torus => once outside the safe region,
                            // always (from there on) outside the safe region.
                            debug_assert!(g.wrap || in_safe_region || !still_safe());
                            if !in_safe_region {
                                self.escapable[index] -= mask;
                            }
                        }
                    }
                }
            }
        }

        // remove vertices directly shadowed by single cops
        for &ch_cop in &cops_vec {
            let cop = g.coordinates_of(ch_cop.vertex());
            self.escapable[g.index_of(cop)] = 0;
            for (i, &escape_dir) in izip!(0.., g.norm.unit_directions()) {
                let mask = 1u8 << i;
                for cop_neigh in g.neighbors_of(cop) {
                    for step in 0..g.side_len() {
                        let v = cop_neigh - step * escape_dir;
                        let Some(v_repr) = g.try_wrap(v) else {
                            break;
                        };
                        let index = g.index_of(v_repr);
                        if self.escapable[index] & mask == 0 {
                            break;
                        }
                        self.escapable[index] -= mask;
                    }
                }
            }
        }
    }
}
