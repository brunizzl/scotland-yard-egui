use super::*;

/// all cops delimiting a Region are listed here
/// values are interpreted as indices in `cops` slice, e.g.
/// index 0 maps to the second position in all of the characters.
#[derive(Debug)]
struct DelimitingGroup {
    marker: u32,
    pair: (usize, usize),
    inner: smallvec::SmallVec<[usize; 6]>,
}

impl DelimitingGroup {
    fn new(marker: u32, pair: (usize, usize)) -> Self {
        Self {
            marker,
            pair,
            inner: smallvec::SmallVec::new(),
        }
    }

    fn marker_bit(&self) -> u32 {
        self.marker.ilog2()
    }

    #[allow(dead_code)]
    fn debug_string(&self, cops: &[Character]) -> String {
        let emoji = |v| cops.iter().find(|c| c.vertex() == v).unwrap().id().emoji();
        let inner = {
            let nr_seps = self.inner.len().saturating_sub(1);
            let seps = std::iter::repeat(", ").take(nr_seps);
            let mut res = String::new();
            res.extend(self.inner.iter().map(|&v| emoji(v)).interleave(seps));
            res
        };
        let (c0, c1) = self.pair;
        format!(
            "bit {}: ({}, {}) + [{}]",
            self.marker_bit(),
            emoji(c0),
            emoji(c1),
            inner
        )
    }
}

/// divides vertices in convex hull into subsets where a robber escape through some boundary segment
/// is possible
/// (each segment lies between two cops on boundary, cops in interior are not considered in computation)
pub struct EscapeableNodes {
    /// intermediary value, kept to reserve allocations etc.
    /// has one entry per vertex, stores distance of that vertex to some (unspecified) node on hull boundary.
    /// which node the distance is in respect to is remembered in `self.last_write_by`
    some_boundary_dist: Vec<isize>, //one entry per vertex

    /// intermediary value, kept to save on allocations and for convinience
    /// one entry per vertex, each boundary node has an index in `ConvexHull::boundary`.
    /// if `last_write_by[vertex_i] == j` then vertex_i was visited last, when distances to boundary vertex
    /// of (boundary) index j where computed.
    last_write_by: Vec<usize>, //one entry per vertex

    /// thing we are actually interested in.
    /// has one entry per vertex. interesting are only vertices in convex hull.
    /// if vertex `v` is escapable via segment `i`, the `(i % 32)`'th bit is set in `escapable[v]`
    escapable: Vec<u32>, //one entry per vertex

    /// intermediary value
    cop_pair_hull: CopPairHullData,

    /// intermediary value, kept to reserve allocations etc.
    /// lists vertices of cop_pair_hull(s) boundaries, but only side torward outher boundary.
    /// only kept permanently to minimize allocations (and for debugging output)
    some_inner_boundaries: [Vec<usize>; 3],

    /// with respect to some escapable region, contains information if a vertex
    /// is not only part of the original escapable, but also of the
    /// shrunk region, because interior cops could endanger some parts.
    keep_escapable: Vec<Keep>, //one entry per vertex

    /// a cop participates in those groups, where he is dangerous to the corresponding region.
    cop_groups: Vec<DelimitingGroup>,
}

/// determines which bit is set in [`EscapeableNodes::escapable`]
fn compute_marker(cop_pair: (usize, usize), cops: &[Character]) -> u32 {
    let cop1 = cop_pair.0;
    let cop1_index = cops.iter().position(|c| c.vertex() == cop1).unwrap();
    // on planar graphs taking the first index should always be sufficient.
    // (as each cop starts only up to one boundary segment)
    1u32 << (cop1_index % 32)
}

impl EscapeableNodes {
    pub fn escapable(&self) -> &[u32] {
        &self.escapable
    }

    #[allow(dead_code)]
    pub fn boundary_dist(&self) -> &[isize] {
        &self.some_boundary_dist
    }

    #[allow(dead_code)]
    pub fn inner_connecting_line(&self) -> impl Iterator<Item = &usize> {
        self.some_inner_boundaries.iter().flat_map(|b| b.iter())
    }

    #[allow(dead_code)]
    pub fn owners(&self) -> &[usize] {
        &self.last_write_by
    }

    pub fn new() -> Self {
        Self {
            some_boundary_dist: Vec::new(),
            last_write_by: Vec::new(),
            escapable: Vec::new(),
            cop_pair_hull: CopPairHullData::default(),
            some_inner_boundaries: Default::default(),
            keep_escapable: Vec::new(),
            cop_groups: Vec::new(),
        }
    }

    /// weird hybrid of region coloring and distance calculation:
    /// update distances of self.some_boundary_dist in neighborhood of queue entries,
    /// recolor region in self.last_write_by to be owner
    fn calc_small_dists(
        &mut self,
        mut select: impl FnMut(usize) -> bool,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
        max_dist: isize,
        last_owner: Option<usize>,
        owner: usize,
    ) {
        let is_fst = last_owner.is_none();
        let last_owner = last_owner.unwrap_or(usize::MAX);
        let mut local_queue = std::mem::take(queue);
        while let Some(v) = local_queue.pop_front() {
            let dist = self.some_boundary_dist[v];
            if dist >= max_dist {
                continue;
            }
            for n in edges.neighbors_of(v) {
                //can by specified by caller what region is selected as baseline (e.g. convex hull...)
                if !select(n) {
                    continue;
                }
                //the fist boundary vertex in a segment is allowed to mark all vertices in its reach.
                //all further vertices of that same segment can only mark hull vertices which have been reached by
                //all previous ones.
                //the vertices counted at the end are only those reached by all boundary vertices.
                if !is_fst && self.last_write_by[n] != last_owner {
                    continue;
                }
                //we have been here before -> no need to mark again
                if self.last_write_by[n] == owner {
                    continue;
                }
                //it is key to never check the distance we overwrite. we don't want the minimum distance to all
                //boundary nodes, we actually want the distance to the current boundary node.
                //we still guarantee to compute the correct distance, because we discover vertices via
                //breadh-first seach.
                self.some_boundary_dist[n] = dist + 1;
                local_queue.push_back(n);
                self.last_write_by[n] = owner;
            }
        }
        *queue = local_queue;
    }

    /// assumes segment was last colored by owner
    /// also assumes queue to only contain vertices of the region to add
    fn add_region_to_escapable(
        &mut self,
        owner: usize,
        marker: u32,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        debug_assert!(owner != 0);
        while let Some(v) = queue.pop_front() {
            debug_assert!(self.escapable[v] & marker != 0);
            debug_assert_eq!(self.last_write_by[v], owner);
            for n in edges.neighbors_of(v) {
                //already marked -> don't add to queue again
                if self.escapable[n] & marker != 0 {
                    debug_assert_eq!(self.last_write_by[n], owner);
                    continue;
                }
                if self.last_write_by[n] == owner {
                    self.escapable[n] |= marker;
                    queue.push_back(n);
                }
            }
        }
    }

    /// assumes escapable regions from boundary cops inwards have already been build.
    /// these regions will now be deleted in regions where third cops could otherwise interfere.
    fn consider_interior_cops(
        &mut self,
        cops: &[Character],
        hull_data: &ConvexHullData,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        self.keep_escapable.clear();
        self.keep_escapable.resize(edges.nr_vertices(), Keep::No);

        //owners of boundary vertices where chosen as what the vertice's index in this vector was
        //  -> new owners can start after all valid vector indices
        let mut owner = hull_data.boundary.len();
        queue.clear();
        let iter = izip!(
            hull_data.boundary_cop_vertices(),
            hull_data.safe_boundary_parts()
        );
        for ((v_left, v_right), safe_outher_boundary) in iter {
            let marker = compute_marker((v_left, v_right), cops);
            let left_cop = cops.iter().find(|c| c.vertex() == v_left).unwrap();
            let right_cop = cops.iter().find(|c| c.vertex() == v_right).unwrap();

            let endangers = |escapable: &[u32], c: &Character| {
                let v = c.vertex();
                let active = c.is_active();
                let distinct = v != v_left && v != v_right;
                //if cop is not at least next to region, then there must be some point on the region boundary
                //that can be reached faster by a robber starting in the region than by that outside cop.
                //this is by definition of the regions.
                let in_region = escapable[v] & marker != 0;
                let next_to_region = edges.neighbors_of(v).any(|n| escapable[n] & marker != 0);
                active && distinct && (in_region || next_to_region)
            };
            for (i, cop_i) in izip!(0.., cops) {
                if !endangers(&self.escapable, cop_i) {
                    continue;
                }
                //note how i == j at first:
                //we generally update self.escapable one cop pair at a time, but single cops are silently also
                //handled in this routine.
                for cop_j in cops.iter().skip(i) {
                    if !endangers(&self.escapable, cop_j) {
                        continue;
                    }
                    //compare with "<=" and ">" to not skip case where i == j
                    let i_closer_eq_to_left =
                        left_cop.dists()[cop_i.vertex()] <= left_cop.dists()[cop_j.vertex()];
                    let i_closer_to_right =
                        right_cop.dists()[cop_i.vertex()] < right_cop.dists()[cop_j.vertex()];
                    if i_closer_eq_to_left == i_closer_to_right {
                        //one cop stands in front of the other -> enough to consider that first cop on its own
                        continue;
                    }
                    let (left_inner, right_inner) = if i_closer_eq_to_left {
                        (cop_i, cop_j)
                    } else {
                        (cop_j, cop_i)
                    };

                    //idea
                    //1. fill each hull of pair with KeepVertex::Perhaps and build boundaries to pair hulls
                    //2. fill region from outher boundary to newly created borders of pair hulls with KeepVertex::Yes
                    //3. prune inner boundaries
                    //4. fill each region admitting escape through one cop pair with KeepVertex::Yes
                    //5. remove markers where not KeepVertex::Yes
                    //6. reset self.keep_in_escapable

                    //step 1: paint in the hull between each cop pair with KeepVertex::Perhaps
                    debug_assert!(self.keep_escapable.iter().all(|&k| k == Keep::No));
                    let cops_line = [left_cop, left_inner, right_inner, right_cop];
                    for ((&c1, &c2), boundary) in izip!(
                        cops_line.iter().tuple_windows(),
                        &mut self.some_inner_boundaries
                    ) {
                        self.cop_pair_hull.compute_hull([c1, c2], edges, queue);
                        boundary.clear();
                        boundary::find_unordered_boundary(
                            boundary,
                            &self.cop_pair_hull.hull,
                            edges,
                            &self.cop_pair_hull.all_vertices,
                        );
                        for &v in &self.cop_pair_hull.all_vertices {
                            self.keep_escapable[v] = Keep::Perhaps;
                        }
                        self.cop_pair_hull.reset_hull(c1, edges, queue);
                    }

                    //step 2: paint region between outher boundary and pair hulls with KeepVertex::Yes
                    debug_assert!(queue.is_empty());
                    for &v in safe_outher_boundary {
                        if self.keep_escapable[v] == Keep::No {
                            self.keep_escapable[v] = Keep::Yes;
                            queue.push_back(v);
                        } else {
                            //TODO: why is this assertion sometimes wrong?
                            //debug_assert_eq!(self.keep_escapable[v], Keep::Perhaps);
                            self.keep_escapable[v] = Keep::YesOnBoundary;
                        }
                    }
                    let paint =
                        |n: usize, keep: &[_]| keep[n] == Keep::No && hull_data.hull[n].contained();
                    edges.recolor_region_with(Keep::Yes, &mut self.keep_escapable, paint, queue);

                    //step 3: prune inner boundaries to only contain safe halfs reaching outside
                    for ((&c1, &c2), boundary) in izip!(
                        cops_line.iter().tuple_windows(),
                        &mut self.some_inner_boundaries
                    ) {
                        //get rid of vertices too close to cops at first, else retain_safe_inner_boundary can run into problems
                        boundary.retain(|&v| isize::min(c1.dists()[v], c2.dists()[v]) > 1);
                        retain_safe_inner_boundary(
                            c1.dists(),
                            boundary,
                            &self.keep_escapable,
                            edges,
                        );
                        debug_assert!(boundary.first().map_or(true, |&v| c1.dists()[v] == 2));
                        debug_assert!(boundary.last().map_or(true, |&v| c2.dists()[v] == 2));
                        debug_assert!(edges.has_path(boundary)); //somehow we can't assert this in tori :(
                    }

                    //step 4: find escapable region for each cop pair and paint it with KeepVertex::Yes
                    let mut escapable = std::mem::take(&mut self.escapable);
                    let inner_boundaries = std::mem::take(&mut self.some_inner_boundaries);
                    for ((&c1, &c2), safe_segment) in izip!(
                        cops_line.iter().tuple_windows(),
                        inner_boundaries.iter().map(|b| &b[..])
                    ) {
                        let max_escapable_dist = safe_segment.len() as isize - 1;
                        debug_assert_eq!(
                            max_escapable_dist.max(0),
                            (c1.dists()[c2.vertex()] - 4).max(0)
                        );
                        let mut last_owner = None;
                        for &v in safe_segment {
                            if last_owner.is_some() {
                                //algorithm is only correct if all vertices in
                                //safe_segment turn out to be actually safe.
                                debug_assert_eq!(Some(self.last_write_by[v]), last_owner);
                            }
                            debug_assert!(queue.is_empty());
                            queue.push_back(v);
                            self.some_boundary_dist[v] = 0;
                            self.last_write_by[v] = owner;
                            let in_hull = |n: usize| hull_data.hull[n].contained();
                            self.calc_small_dists(
                                in_hull,
                                edges,
                                queue,
                                max_escapable_dist,
                                last_owner,
                                owner,
                            );
                            last_owner = Some(owner);
                            owner += 1;
                        }

                        debug_assert!(queue.is_empty());
                        if let Some(last_owner) = last_owner {
                            let keep = &mut self.keep_escapable[..];
                            let is_owned = |&&v: &&_| self.last_write_by[v] == last_owner;
                            let init = safe_segment.iter().filter(is_owned);
                            let safe = |n, _: &[_]| self.last_write_by[n] == last_owner;
                            edges.recolor_with_init(Keep::Yes, keep, safe, queue, init);
                        }
                    }

                    //step 5: only keep markers with KeepVertex::Yes
                    for &v in safe_outher_boundary {
                        queue.push_back(v);
                        self.keep_escapable[v] = Keep::YesButAlreadyVisited;
                    }
                    while let Some(v) = queue.pop_front() {
                        use Keep::*;
                        debug_assert_ne!(self.keep_escapable[v], Yes);
                        for n in edges.neighbors_of(v) {
                            let keep = self.keep_escapable[n];
                            if keep == Yes {
                                self.keep_escapable[n] = YesButAlreadyVisited;
                                queue.push_back(n);
                            }
                            if matches!(keep, No | Perhaps) && escapable[n] & marker != 0 {
                                escapable[n] -= marker;
                                queue.push_back(n);
                            }
                        }
                    }

                    //step 6: reset self.keep_in_escapable (use recolor to not iterate over whole array)
                    let keep = &mut self.keep_escapable[..];
                    let init = safe_outher_boundary.iter();
                    edges.recolor_with_init(Keep::No, keep, |_, _| true, queue, init);
                    debug_assert!(keep.iter().all(|&k| k == Keep::No));

                    //finally: guarantee inner cops are still endangering curr region
                    escapable[left_inner.vertex()] |= marker;
                    escapable[right_inner.vertex()] |= marker;

                    self.escapable = escapable;
                    self.some_inner_boundaries = inner_boundaries;
                }
            }
        }

        //because of EdgeList::recolor_region_with reasons, we always included the vertices reset here as safe robber
        //positions. this is obv. wrong
        for cop in cops.iter().filter(|c| c.is_active()) {
            self.escapable[cop.vertex()] = 0;
            let mut regions = 0;
            for n in edges.neighbors_of(cop.vertex()) {
                regions |= std::mem::replace(&mut self.escapable[n], 0);
            }
            for group in &mut self.cop_groups {
                if regions & group.marker != 0 {
                    group.inner.push(cop.vertex())
                }
            }
        }
    }

    /// ignores cops in interior of hull, computes safe regions only with respect to boundary cops
    fn consider_boundary_cops(
        &mut self,
        cops: &[Character],
        hull_data: &ConvexHullData,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        queue.clear();
        let iter = izip!(
            hull_data.boundary_cop_vertices(),
            hull_data.safe_boundary_indices(),
            hull_data.safe_boundary_parts()
        );
        for (cop_pair, indices, vertices) in iter {
            let marker = compute_marker(cop_pair, cops);
            let max_escapable_dist = indices.len() as isize - 1;
            debug_assert_eq!(indices.len(), vertices.len());

            //we call calc_small_dists vor every vertex in our boundary segement.
            //to speed up computation, our first and second vertex are on opposing ends.
            //future boundary vertices thus only consider inner vertices reached by both extremes,
            //which should get us some constant factor in speedup.
            let mut last_owner = None;
            let mut last_v = usize::MAX;
            let mut compute_dists_to = |owner, v| {
                debug_assert!(owner != 0);
                queue.push_back(v);
                self.some_boundary_dist[v] = 0;
                self.last_write_by[v] = owner;
                let in_hull = |x: usize| hull_data.hull[x].contained();
                self.calc_small_dists(in_hull, edges, queue, max_escapable_dist, last_owner, owner);
                last_owner = Some(owner);
                last_v = v;
            };
            let mut iter = izip!(indices.clone(), vertices);
            if let Some((owner, &v)) = iter.next_back() {
                compute_dists_to(owner, v);
            }
            for (owner, &v) in iter {
                compute_dists_to(owner, v);
            }

            let owner = last_owner.unwrap();
            debug_assert!(queue.is_empty());
            queue.push_back(last_v);
            self.escapable[last_v] |= marker;
            self.add_region_to_escapable(owner, marker, edges, queue);
            self.cop_groups.push(DelimitingGroup::new(marker, cop_pair));
        }
    }

    pub fn update(
        &mut self,
        cops: &[Character],
        hull_data: &ConvexHullData,
        edges: &EdgeList,
        queue: &mut VecDeque<usize>,
    ) {
        let nr_vertices = edges.nr_vertices();
        self.escapable.clear();
        self.last_write_by.clear();
        self.some_boundary_dist.clear();
        self.escapable.resize(nr_vertices, 0);
        self.last_write_by.resize(nr_vertices, 0); //no owner is 0 (as a cop stands at pos 0 of hull boundary)
        self.some_boundary_dist.resize(nr_vertices, isize::MAX);
        self.cop_groups.clear();

        self.consider_boundary_cops(cops, hull_data, edges, queue);
        self.consider_interior_cops(cops, hull_data, edges, queue);

        println!("-------- groups in escapable ------------");
        for group in &self.cop_groups {
            println!("{}", group.debug_string(cops));
        }
    }
}
