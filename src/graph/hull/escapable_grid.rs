use super::*;
use grid::*;

/// should compute nearly the same thing as [`hull::EscapableNodes`],
/// if graph in question is a grid (optionally wrapped to torus).
/// Nothing is computed on other graphs.
/// If the graph is a non-wrapped (non-torus) grid,
/// also vertices outside the convex hull are marked.
/// This struct represents the thesis' conncept of "Winning Direction" sets.
pub struct EscapableDirections {
    /// one entry per vertex, each region (inside the cop hull and
    /// induced by a boundary segement) is marked by one bit.
    /// the hope is, that we never have more than 32 regions, otherwise
    /// regions share a bit and we hope they do not touch.
    pub esc_components: Vec<u32>,

    /// which directions are contained in region i.
    /// note: not every vertex of a region must have every direction of the region.
    component_directions: [Dirs; 32],

    /// stores escape directions for every vertex (if graph is grid).
    /// on tori, only vertices inside the convex hull may be marked.
    pub esc_directions: Vec<Dirs>,
    /// remembers graph of last update.
    pub graph: DistGridGraph,
}

impl EscapableDirections {
    pub fn new() -> Self {
        Self {
            esc_components: Vec::new(),
            component_directions: [Dirs::EMPTY; 32],
            esc_directions: Vec::new(),
            graph: DistGridGraph::new(),
        }
    }

    fn initialize_directions(&mut self, convex_cop_hull: &[InSet]) {
        assert!(self.graph.represents_current_map);
        assert_eq!(self.graph.nr_vertices(), convex_cop_hull.len());
        assert_eq!(self.graph.nr_vertices(), self.esc_directions.len());

        // initialize escapable to have the complete hull inside marked for doughnuts
        // and to have __everything__ marked for non-wrapping grids
        let all = Dirs::all(self.graph.data.norm);
        if !self.graph.data.wrap {
            self.esc_directions.fill(all);
        } else {
            for (esc, h) in izip!(&mut self.esc_directions, convex_cop_hull) {
                if h.contained() {
                    *esc = all;
                }
            }
        }
    }

    /// returns the number of found components
    fn find_components(&mut self, hull: &[InSet], queue: &mut VecDeque<usize>) -> usize {
        let g = self.graph.data;
        assert_eq!(hull.len(), self.esc_components.len());
        assert_eq!(hull.len(), self.esc_directions.len());

        let mut boundary_section = Vec::new();

        // we guarantee to find every component by iterating trough every vertex.
        // how far we have iterated previously, is remembered here.
        let mut search_start = 0;
        for component_nr in 0.. {
            let component_bit = 1u32 << (component_nr % 32);

            // find first not-yet handled vertex of a component
            let component_v = 'find_new_component: {
                let i0 = search_start;
                for (i, h, &esc, &marker) in izip!(
                    i0..,
                    &hull[i0..],
                    &self.esc_directions[i0..],
                    &self.esc_components[i0..]
                ) {
                    if h.on_boundary() && esc.nonempty() && marker == 0 {
                        search_start = i;
                        break 'find_new_component i;
                    }
                }
                return component_nr;
            };

            boundary_section.clear();
            queue.clear();

            // find the boundary section
            boundary_section.push(component_v);
            self.esc_components[component_v] |= component_bit;
            queue.push_back(component_v);
            while let Some(v) = queue.pop_front() {
                let v_coords = g.coordinates_of(v);
                for n in g.neighbor_indices_of(v_coords) {
                    if hull[n].on_boundary()
                        && self.esc_directions[n].nonempty()
                        && self.esc_components[n] & component_bit == 0
                    {
                        boundary_section.push(n);
                        self.esc_components[n] |= component_bit;
                        queue.push_back(n);
                    }
                }
            }

            // we could take the directions of any boundary vertex and are done,
            // except if a region from the other side overlaps into this boundary.
            let boundary_dirs = boundary_section
                .iter()
                .map(|&bv| self.esc_directions[bv])
                .fold(Dirs::all(g.norm), Dirs::intersection);
            self.component_directions[component_nr % 32] = boundary_dirs;
            debug_assert!(g.wrap || boundary_dirs.0.count_ones() > 0);

            // if a boundary has three escape directions,
            // it is sufficient to only walk (the opposite of) the center direction
            // to reach every vertex of the component from some boundary vertex.
            let center_dirs = if boundary_dirs.0.count_ones() <= 2 {
                boundary_dirs
            } else {
                // we do not consider what happens on tori, because on there,
                // what we computed as the convex hull is not even always convex.
                debug_assert!(g.wrap || g.norm == Norm::Hex);
                debug_assert!(g.wrap || boundary_dirs.keep_inner_on_hex().0 != 0);
                boundary_dirs.keep_inner_on_hex()
            };
            let inside_dirs = smallvec::SmallVec::<[_; 6]>::from_iter(
                center_dirs.directions(g.norm).map(std::ops::Neg::neg),
            );

            // mark the region
            for &inside_dir in &inside_dirs {
                for &bv in &boundary_section {
                    let bv_coords = g.coordinates_of(bv);
                    let mut dirs_left = center_dirs;
                    for step_len in 1..g.side_len() {
                        let Some(v) = g.index_of(bv_coords + step_len * inside_dir) else {
                            break;
                        };
                        dirs_left.intersect(self.esc_directions[v]);
                        if match g.norm {
                            Norm::Hex => dirs_left != center_dirs,
                            Norm::Quad => dirs_left.is_empty(),
                        } {
                            break;
                        }
                        self.esc_components[v] |= component_bit;
                    }
                }
            }
        }
        unreachable!()
    }

    /// sometimes we failed to unmark some escape directions on tori.
    /// we take a conservative approach here and only keep the directions of the corresponding components.
    /// this is extra conservative, because which directions belong to a boundary section on a torus,
    /// is somewhat ill-defined (due to the fact, that we do not actually compute a correct convex hull on tori).
    /// therefore some regions may be more shallow than expected, in some cases consisting of the boundary only.
    fn delete_leftover_on_torus(&mut self, nr_components: usize) {
        assert_eq!(self.esc_components.len(), self.graph.nr_vertices());
        assert_eq!(self.esc_directions.len(), self.graph.nr_vertices());
        if !self.graph.data.wrap {
            return;
        }
        let nr_used_bits = nr_components.clamp(0, 32);
        let used_bits_mask = (1 << nr_used_bits) - 1;
        for (dir, &comp) in izip!(&mut self.esc_directions, &self.esc_components) {
            debug_assert_eq!(comp & used_bits_mask, comp);
            let valid_dirs = izip!(0..nr_used_bits, &self.component_directions)
                .filter_map(|(i, &ds)| ((1 << i) & comp != 0).then_some(ds))
                .fold(Dirs::EMPTY, Dirs::union);
            dir.intersect(valid_dirs);
        }
    }

    pub fn update(
        &mut self,
        map: &Embedding3D,
        queue: &mut VecDeque<usize>,
        cop_hull: &[InSet],
        active_cops: &[&Character],
    ) {
        assert_eq!(map.nr_vertices(), cop_hull.len());

        self.esc_components.clear();
        self.esc_components.resize(map.nr_vertices(), 0);

        self.esc_directions.clear();
        self.esc_directions.resize(map.nr_vertices(), Dirs::EMPTY);

        self.graph.update(map, queue);
        if !self.graph.represents_current_map {
            return;
        }
        self.initialize_directions(cop_hull);
        self.graph.remove_non_winning(active_cops, &mut self.esc_directions);
        let nr_components = self.find_components(cop_hull, queue);
        self.delete_leftover_on_torus(nr_components);
    }
}

pub struct DistGridGraph {
    pub data: GridGraph,
    /// distance of every vertex to vertex `0`, which is also `Coords{ x: 0, y: 0 }`.
    /// used to compute arbitrary distances on tori
    pub dists_0: Vec<isize>,
    /// not all graphs in this program fit in the format of a GridGraph, hence this.
    pub represents_current_map: bool,
}

impl DistGridGraph {
    pub fn nr_vertices(&self) -> usize {
        self.data.grid.nr_vertices()
    }

    pub fn new() -> Self {
        Self {
            data: GridGraph {
                grid: OrderedColWise { len: 0 },
                norm: Norm::Quad,
                wrap: false,
            },
            dists_0: Vec::new(),
            represents_current_map: false,
        }
    }

    pub fn update(&mut self, map: &Embedding3D, queue: &mut VecDeque<usize>) {
        let Some(g) = GridGraph::try_from(map) else {
            self.represents_current_map = false;
            return;
        };

        // update distances
        debug_assert!(g.side_len() > 0);
        if g.wrap && self.data != g {
            self.dists_0.clear();
            self.dists_0.resize(map.nr_vertices(), isize::MAX);
            self.dists_0[0] = 0;
            queue.clear();
            queue.push_back(0);
            map.edges().calc_distances_to(queue, &mut self.dists_0);
        }
        self.data = g;

        self.represents_current_map = true;
    }

    /// computes distance of `v` to `Coords { x: 0, y: 0 }`.
    /// if `self.wrapped`, `dists_to_0` must contain the distance to 0 of every normalized vertex.
    pub fn dist_to_0(&self, v: Coords) -> isize {
        if self.data.wrap {
            let wrapped = self.data.grid.pack_small_coordinates(v.x, v.y);
            debug_assert_eq!(self.dists_0.len(), self.data.grid.nr_vertices());
            self.dists_0[self.data.unchecked_index_of(wrapped)]
        } else {
            self.data.norm.apply(v)
        }
    }

    /// computes all safe boundaries between cop1 and cop2.
    /// to be efficient with allocations, we pass the storage for the result as `safe_boundaries` parameter.
    /// returns how many safe boundaries where constructed.
    fn find_safe_boundaries(
        &self,
        cop1: Coords,
        cop2: Coords,
        safe_boundaries: &mut [Vec<Coords>; 6],
    ) -> usize {
        assert!(self.represents_current_map);
        let g = self.data;
        let dist = |a, b| self.dist_to_0(a - b);

        let cops_dist = dist(cop1, cop2);
        debug_assert!(cops_dist >= 4);

        let path_dirs = {
            // find all directions in which a shortest path from `cop1` to `cop2` starts
            let mut res = smallvec::SmallVec::<[_; 6]>::new();
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

        for sb in &mut safe_boundaries[..] {
            sb.clear();
        }
        for (bd, &d1) in izip!(safe_boundaries, &path_dirs) {
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
                    debug_assert_eq!(g.try_wrap(curr + (cops_dist - step1) * d2).unwrap(), cop2);
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

        path_dirs.len()
    }

    pub fn remove_non_winning(&self, cops_vec: &[&Character], winning: &mut [Dirs]) {
        assert!(self.represents_current_map);
        let g = self.data;
        let dist = |a, b| self.dist_to_0(a - b);

        let mut safe_boundaries = [const { Vec::new() }; 6];
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
            let nr_boundaries = self.find_safe_boundaries(cop1, cop2, &mut safe_boundaries);

            // for the found safe boundaries, we remove all vertices not part of the induced
            // escapable region for every direction.
            // note: we only remove "behind" the safe boundary,
            // thus vertices "behind" cops / neighbors of cops are removed seperately below.
            for (&mask, &escape_dir) in Dirs::unit_bits_and_directions(g.norm) {
                for safe_boundary in &safe_boundaries[..nr_boundaries] {
                    let safe_dist = safe_boundary.len() as isize - 1;
                    debug_assert_eq!(safe_dist, cops_dist - 4);
                    for &path_v in safe_boundary {
                        let mut in_safe_region = true;
                        for step in 0..g.side_len() {
                            let v = path_v - step * escape_dir;
                            let Some(v_repr) = g.try_wrap(v) else {
                                break;
                            };
                            let index = g.unchecked_index_of(v_repr);
                            if winning[index].intersection(mask).is_empty() {
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
                                winning[index].0 -= mask.0;
                            }
                        }
                    }
                }
            }
        }

        // remove vertices directly shadowed by single cops
        for &ch_cop in cops_vec {
            let cop = g.coordinates_of(ch_cop.vertex());
            winning[g.unchecked_index_of(cop)] = Dirs::EMPTY;
            for (&mask, &escape_dir) in Dirs::unit_bits_and_directions(g.norm) {
                for cop_neigh in g.neighbors_of(cop) {
                    for step in 0..g.side_len() {
                        let v = cop_neigh - step * escape_dir;
                        let Some(index) = g.index_of(v) else {
                            break;
                        };
                        if winning[index].intersection(mask).is_empty() {
                            break;
                        }
                        winning[index].0 -= mask.0;
                    }
                }
            }
        }
    }
}
