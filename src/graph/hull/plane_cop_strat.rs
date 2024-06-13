use crate::app::character;
use crate::app::map;

use super::*;

/// in the infinite planar 4-regular or 6-regular graph if cops stand on a circle with constant distance
/// to the vertex with coordinates (0, 0), this struct aims to find all points from which the robber
/// could attack a corner, e.g. force a cop to move away from an extreme vertex on the circle.
pub struct PlaneCopStat {
    /// thing we are actually interested in.
    /// has one entry per vertex. interesting are only vertices in convex hull.
    /// if vertex `v` is in danger one `i`, the `(i % 32)`'th bit is set in `danger_zones[v]`
    danger_zones: Vec<u32>, //one entry per vertex
}

/// in a regula planar graph, most vertices on a hull boundary would have the same number of
/// neighbors beeing interior vertices, as beeing vertices outside the hull.
/// a corner is defined as having fewer interior neighbors as outside neighbors.
fn on_corner(v: usize, hull: &[InSet], edges: &EdgeList) -> bool {
    hull[v].on_boundary() && {
        let mut nr_inside = 0;
        let mut nr_outside = 0;
        for n in edges.neighbors_of(v) {
            if hull[n].in_interieur() {
                nr_inside += 1;
            }
            if hull[n].outside() {
                nr_outside += 1;
            }
        }
        nr_inside < nr_outside
    }
}

/// the distance to the next corner is at most 1
fn near_corner(v: usize, hull: &[InSet], edges: &EdgeList) -> bool {
    on_corner(v, hull, edges) || edges.neighbors_of(v).any(|n| on_corner(n, hull, edges))
}

impl PlaneCopStat {
    pub fn new() -> Self {
        Self { danger_zones: Vec::new() }
    }

    pub fn danger_zones(&self) -> &[u32] {
        &self.danger_zones
    }

    pub fn update(
        &mut self,
        shape: map::Shape,
        edges: &EdgeList,
        hull: &[InSet],
        escapable: &[u32],
        characters: &character::State,
        queue: &mut VecDeque<usize>,
    ) {
        self.danger_zones.clear();
        self.danger_zones.resize(edges.nr_vertices(), 0);
        let min_outside_at_corner = match shape {
            map::Shape::RegularPolygon2D(6) | map::Shape::TriangTorus => 3,
            map::Shape::SquareTorus => 2,
            _ => {
                // currently i only thought about a strategy for 4-regular or 6-regular graphs.
                return;
            },
        };
        //if the robber finds himself next to one of these points,
        //a corner guarding cop is forced out of his position, which the police must avoid.
        let mut danger_points = Vec::new();
        //will store dists to `danger_points`
        let mut dists = vec![isize::MAX; edges.nr_vertices()];
        let mut this_marker_i = 0;
        for (i, cop_i) in izip!(0.., characters.active_cops()) {
            let cop_i_v = cop_i.vertex();
            if !near_corner(cop_i_v, hull, edges) {
                continue;
            }
            // this is the number of police moves required at minimum to get a second cop to help
            // cop_i guard the corner.
            let closest_cop_dist = {
                let mut res = isize::MAX;
                for (j, cop_j) in izip!(0.., characters.active_cops()) {
                    let dist = cop_j.dists()[cop_i_v];
                    if i != j && dist < res {
                        res = dist;
                    }
                }
                res
            };

            // the closest a lazy-safe (e.g. escapable) vertex is to any cop is 2.
            // thus the markers "next to" the cop_i are found at neighbors of neighbors.
            // note: to get the k-neighbors this way has exponential complexity.
            let esc_marker = edges.neighbors_of(cop_i_v).fold(0, |m, n| {
                edges.neighbors_of(n).fold(m, |mm, nn| mm | escapable[nn])
            });
            let marked_esc = |v: usize| escapable[v] & esc_marker != 0;
            let i_dists = cop_i.dists();
            let valid_next = |v: usize, n: usize| {
                let on_boundary = edges.neighbors_of(n).any(|nn| !marked_esc(nn));
                let is_further = i_dists[v] < i_dists[n];
                marked_esc(n) && on_boundary && is_further
            };

            queue.clear();
            let valid_nns = |ni| edges.neighbors_of(ni).filter(move |&nni| valid_next(ni, nni));
            queue.extend(edges.neighbors_of(cop_i_v).flat_map(valid_nns));

            // add all those lazy-safe vertices to danger_points that can only become non-lazy-safe
            // by a move of cop_i away from his precious corner
            danger_points.clear();
            while let Some(v) = queue.pop_front() {
                danger_points.push(v);
                let mut found_corner = false;
                let valid_ns = || edges.neighbors_of(v).filter(|&n| valid_next(v, n));
                for n in valid_ns() {
                    let is_outside = |&nn: &_| !marked_esc(nn);
                    let nr_outside = edges.neighbors_of(n).filter(is_outside).count();
                    found_corner |= nr_outside >= min_outside_at_corner;
                }
                if !found_corner {
                    queue.extend(valid_ns().filter(|&n| hull[n].in_interieur()));
                }
            }

            // mark all points closer to a danger_point than the next cop is to help guarding the corner.
            queue.extend(danger_points.iter());
            for &v in &danger_points {
                queue.push_back(v);
                dists[v] = 0;
            }
            let this_marker = 1u32 << (this_marker_i % 32);
            this_marker_i += 1;
            let select = |n, ds: &[_], new_dist| {
                let res = new_dist <= closest_cop_dist && ds[n] > new_dist;
                if res {
                    self.danger_zones[n] |= this_marker;
                }
                res
            };
            edges.calc_distances_to_with(&mut dists, select, queue);

            //clean up dists to be used to find the next danger region
            let init = danger_points.iter();
            edges.recolor_with_init(isize::MAX, &mut dists, |_, _| true, queue, init);
            debug_assert!(dists.iter().all(|&v| v == isize::MAX));
        }
    }
}
