use super::*;

/// tries to find the boundary of `set` as if `edges` belong to a triangulation.
/// returns wether the process was successful.
/// on failure the boundary may be partially constructed.
fn find_hull_boundary_in_triangulation(
    boundary: &mut Vec<usize>,
    hull: &[InSet],
    edges: &EdgeList,
) -> Result<(), ()> {
    debug_assert_eq!(boundary.len(), 1);
    let fst_inside = boundary[0];
    debug_assert!(hull[fst_inside].on_boundary());
    let mut curr_inside = fst_inside;
    let mut last_inside = fst_inside;
    // similar deal to snd_last_outside below
    let mut snd_last_inside = fst_inside;

    let neighbors_of = |v: usize| edges.neighbors_of(v);

    // snd_last_outside is almost not needed: all vertices inside the hull
    // neighboring any given vertex outside must be a clique,
    // else the path connecting them through the outside vertex is a shortest path
    // and thus the vertex not outside the hull.
    // if our (assumed to be locally planar) graph somehow contains a four-clique or
    // the hull is not actually a convex hull, this variable can safe us.
    let mut snd_last_outside = usize::MAX;
    let fst_outside = neighbors_of(fst_inside).find(|&n| hull[n].outside()).unwrap();
    let mut last_outside = fst_outside;
    //can fail if object we want to find the boundary of is not actually a convex hull.
    //or if the graph in question is not (locally) planar
    let mut curr_outside = neighbors_of(fst_inside)
        .find(|&n| hull[n].outside() && edges.has_edge(n, fst_outside))
        .ok_or(())?;

    // idea: curr_outside must neighbor curr_inside at all time. this way we are guaranteed
    // to not switch sides on a hull with local thickness <= 2.
    // if neighter of curr_outside and curr_inside can be advanced, while keeping
    // the neighbor relation, we have something other than a triangulation and exit.
    let mut change = true;
    // in a graph of unknown shape, it can not be ruled out, that an up to unbounded number of
    // vertices are neighbors of the last boundary vertex. in that case we return if nothing in
    // the boundary changes after some maximum number of consecutive outside steps.
    let mut outside_streak = 0;
    while change && curr_outside != fst_outside {
        change = false;

        // step outside the hull but keep curr_inside as neighbor
        let outside_step = neighbors_of(curr_outside).find(|&n| {
            ![last_outside, snd_last_outside].contains(&n)
                && hull[n].outside()
                && edges.has_edge(curr_inside, n)
        });
        if let Some(next_outside) = outside_step {
            snd_last_outside = last_outside;
            last_outside = curr_outside;
            curr_outside = next_outside;
            change = true;
            outside_streak += 1;
        }

        // step on hull boundary but keep curr_outside as neighbor
        let inside_step = neighbors_of(curr_inside).find(|&n| {
            ![last_inside, snd_last_inside].contains(&n)
                && hull[n].on_boundary()
                && edges.has_edge(curr_outside, n)
        });
        if let Some(next_inside) = inside_step.or_else(|| {
            // special case if hull has thickness 1 and curr_inside is an extreme point
            (!change
                && edges.has_edge(curr_outside, last_inside)
                && edges.has_edge(last_outside, curr_inside))
            .then_some(last_inside)
        }) {
            boundary.push(next_inside);
            snd_last_inside = last_inside;
            last_inside = curr_inside;
            curr_inside = next_inside;
            change = true;
            outside_streak = 0;
        }

        debug_assert!(edges.has_edge(curr_inside, curr_outside));
        if boundary.len() == edges.nr_vertices() || outside_streak > 32 {
            return Err(());
        }
    }
    // we always try to take a step in the hull after we step outside.
    // if we reach the outside start with an outside step, we are guaranteed to find the
    // inside start with the following step inside.
    // the only special case is where the boundary has length 2 or 3, because then fst_inside will
    // not be readded, as it is also last_inside or snd_last_inside.
    // (TODO: how does this hold up in non-planar graphs?)
    debug_assert!(!change || Some(&fst_inside) == boundary.last() || boundary.len() < 10);

    change.then_some(()).ok_or(())
}

/// build to return something often resembling a hull boundary if the graph in question is
/// not a triangulation.
/// if the function returns "success", it is however not guaranteed that the result is actually correct.
#[allow(dead_code)]
fn find_hull_boundary_fallback(
    boundary: &mut Vec<usize>,
    dist_to_boundary_start: &[isize],
    hull: &[InSet],
    edges: &EdgeList,
    queue: &mut VecDeque<usize>,
) -> Result<(), ()> {
    debug_assert_eq!(boundary.len(), 1);
    let fst_inside = boundary[0];
    debug_assert_eq!(dist_to_boundary_start[fst_inside], 0);
    debug_assert!(hull[fst_inside].on_boundary());

    let mut potential_next = std::mem::take(queue);
    let mut curr_inside = fst_inside;
    let mut last_inside = fst_inside;
    'find_boundary: loop {
        potential_next.clear();
        potential_next.extend(
            edges
                .neighbors_of(curr_inside)
                .filter(|&n| n != last_inside && hull[n].on_boundary()),
        );
        if potential_next.len() > 1 {
            // if we have choice,
            // only keep the options visited the longest time ago / not at all
            for v in boundary.iter().rev() {
                if potential_next.contains(v) {
                    potential_next.retain(|next| next != v);
                    debug_assert!(!potential_next.is_empty());
                    if potential_next.len() == 1 {
                        break;
                    }
                }
            }
        }

        let next_inside = match potential_next.len() {
            1 => potential_next[0], //common case
            0 => {
                // if the hull has some pointy corner where the boundary visits the same vertices twice,
                // once before visiting the corner, once after leaving,
                // we need to take last_inside again when curr_inside is corner.
                // note: the case where the hull consists of only a single vertex is also covered here,
                // as last_inside (like curr_inside) starts beeing equal to fst_inside.
                last_inside
            },
            _ => {
                // if there are multiple options not yet visited,
                // we take the one leading further away from the search start.
                // this is no exact science but should at least be somewhat
                // reliable at the start of construction.
                let max_dist = potential_next
                    .iter()
                    .fold(0, |max, &v| isize::max(max, dist_to_boundary_start[v]));
                potential_next
                    .iter()
                    .copied()
                    .find(|&v| dist_to_boundary_start[v] == max_dist)
                    .unwrap()
            },
        };
        last_inside = curr_inside;
        curr_inside = next_inside;
        boundary.push(next_inside);
        if next_inside == fst_inside {
            break 'find_boundary; //happy case :)
        }
        if boundary.len() > edges.nr_vertices() {
            //emergency exit. something went wrong and we failed to find the boundary :(
            boundary.clear();
            break 'find_boundary;
        }
    }
    potential_next.clear();
    *queue = potential_next;

    (boundary.last() == Some(&fst_inside)).then_some(()).ok_or(())
}

/// assumes boundary has first vertex already inserted as starting point
/// and that hull is up-to date, including having all boundary vertices marked
pub fn find_convex_hull_boundary(
    boundary: &mut Vec<usize>,
    dist_to_boundary_start: &[isize],
    hull: &[InSet],
    edges: &EdgeList,
    queue: &mut VecDeque<usize>,
) -> Result<(), ()> {
    let start = boundary[0];
    let res = find_hull_boundary_in_triangulation(boundary, hull, edges);
    if res.is_ok() {
        return res;
    }
    boundary.clear();
    boundary.push(start);
    find_hull_boundary_fallback(boundary, dist_to_boundary_start, hull, edges, queue)
}

/// relies just on [`InSet::No`] and [`InSet::Interieur`], doesn't need [`InSet::OnBoundary`] for `hull` to function.
/// `all_elements` contains every element of `hull`.
pub fn find_unordered_boundary(
    boundary: &mut Vec<usize>,
    hull: &[InSet],
    edges: &EdgeList,
    all_elements: &[usize],
) {
    debug_assert!(boundary.is_empty());
    debug_assert_eq!(edges.nr_vertices(), hull.len());

    for &v in all_elements {
        debug_assert!(hull[v].contained());
        if edges.neighbors_of(v).any(|n| hull[n].outside()) {
            boundary.push(v);
        }
    }
}
