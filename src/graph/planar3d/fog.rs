//! in this setting, cops and robber have the same roles, which is cleaning the fog.
//! they are not exactly equivalent though: the robber causes a cleaning + fog update when moved.

use itertools::{Itertools, izip};

use crate::{
    app::character::{self},
    graph::EdgeList,
};

pub struct GameSates {
    /// last entry is the present.
    history: Vec<Vec<bool>>,
    /// from how far can vertices be cleaned
    cleaning_range: isize,
    /// if fog is only shown after some moves where made,
    /// we need to know how many moves happened beforehand.
    /// this variable stores the number of overall rounds (with and without fog)
    nr_rounds: usize,
}

/// returns for each vertex wether it currently lies outside the cleaners
/// visibility (further than `range` from the closest cleaner, aka _is in the dark_) or not.
fn dark_vertices(edges: &EdgeList, cleaners: &character::State, range: isize) -> Vec<bool> {
    let mut result = vec![true; edges.nr_vertices()];
    let mut visited = std::collections::HashSet::new();
    let mut queue = std::collections::VecDeque::new();
    for cleaner in cleaners.all() {
        if !cleaner.is_active() {
            continue;
        }
        queue.push_back(cleaner.last_resting_vertex());
        visited.insert(cleaner.last_resting_vertex());
        while let Some(v) = queue.pop_front() {
            for n in edges.neighbors_of(v) {
                if cleaner.dists()[n] <= range && !visited.contains(&n) {
                    queue.push_back(n);
                    visited.insert(n);
                }
            }
        }
        for v in visited.drain() {
            result[v] = false;
        }
    }
    result
}

impl GameSates {
    pub fn new() -> Self {
        Self {
            history: Vec::new(),
            cleaning_range: 1,
            nr_rounds: usize::MAX / 2,
        }
    }

    pub fn curr_fog(&self) -> Option<&[bool]> {
        self.history.last().map(|fog| &fog[..])
    }

    pub fn update(&mut self, edges: &EdgeList, cleaners: &character::State, cleaning_range: isize) {
        if self.cleaning_range != cleaning_range {
            self.history.clear();
            self.cleaning_range = cleaning_range;
        }
        if self.history.is_empty() {
            self.history.push(dark_vertices(edges, cleaners, cleaning_range));
        }

        // for simplicity, moving the robber determines when a cleaning move is over.
        if let Some(robber) = cleaners.active_robber() {
            let new_nr_rounds = robber.past_vertices().len();
            let rounds_diff = new_nr_rounds as isize - self.nr_rounds as isize;
            self.nr_rounds = new_nr_rounds;
            match rounds_diff {
                -1 => {
                    // the last move was undone -> no updates required after also undoing last move here
                    self.history.pop();
                    return;
                },
                0 => return,               // no change -> nothing to do here
                1 => {},                   // new move is completed -> update below
                _ => self.history.clear(), // unknow state -> compute fresh state below.
            }
        } else {
            return;
        }

        // determine what cleaners currently can't see
        let dark = dark_vertices(edges, cleaners, cleaning_range);

        // only keep fog if cleaners can't see it
        let mut reduced_fog = self.history.last().unwrap_or(&dark).clone();
        for (foggy, &is_dark) in izip!(&mut reduced_fog, &dark) {
            *foggy &= is_dark;
        }

        // spread fog where possible.
        let new_fog = izip!(0.., edges.neighbors())
            .map(|(v, ns)| dark[v] && std::iter::once(v).chain(ns).any(|n| reduced_fog[n]))
            .collect_vec();

        self.history.push(new_fog);
    }
}
