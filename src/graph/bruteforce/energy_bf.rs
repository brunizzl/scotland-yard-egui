//! this file is dedicated to the variant of the game, where the robber is granted units of energy per round,
//! which can either be used directly to walk or can be stored in a bank.
//! the bank can also be robbed to walk further.
//!
//! the cop movement rules are decided by [`crate::graph::bruteforce::rules`]. the cops have no energy bank.

use super::*;
// turns out the fog spreading logic is the same as the robber-can-many-edges-at-once logic. who would have thought?
use super::fog_util as fog;

/// to the outside we handle energy as a ratio, where one unit is used up when the robber walks one edge.
/// internally, the bank capacity and the allowance are brought to the same denominator.
/// it is thus easier to say a step takes *denominator* much energy and every energy value becoming an integer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct EnergyParams {
    /// internal unit scaling: this is the denominator [`Self::allowance`] and
    /// [`Self::bank_capacity`] are divided by when one step takes one unit of energy.
    pub energy_per_step: usize,
    /// the energy the robber is additionally given each round
    pub allowance: usize,
    /// the maximum amount of energy that can be carried over a round boundary
    pub bank_capacity: usize,
}

impl EnergyParams {
    pub const STANDARD_GAME: Self = EnergyParams {
        energy_per_step: 1,
        allowance: 1,
        bank_capacity: 0,
    };

    pub fn print_compact(self) -> String {
        let Self {
            energy_per_step: d,
            allowance: a,
            bank_capacity: b,
        } = self;
        if d == 1 {
            format!("a = {a}, b = {b}")
        } else {
            format!("a = {a}/{d}, b = {b}/{d}")
        }
    }
}

impl Default for EnergyParams {
    fn default() -> Self {
        Self::STANDARD_GAME
    }
}

pub struct EnergyRobberStrat {
    pub params: EnergyParams,
    pub symmetry: ExplicitClasses,
    /// for each possible bank level, this maps to which vertices are safe
    /// (depending on the cop positions)
    pub safe: Vec<SafeRobberPositions>,
    pub cop_moves: CopConfigurations,
    pub robber_wins: bool,
}

impl EnergyRobberStrat {
    /// the equivalent of [`RobberWinData::safe_vertices`]
    pub fn safe_vertex_energies(&self, mut cops: RawCops) -> impl ExactSizeIterator<Item = usize> {
        let (autos, cop_positions) = self.cop_moves.pack(&self.symmetry, &mut cops);
        let safe_lvls_at_cops =
            Vec::from_iter(self.safe.iter().map(|lvl| lvl.robber_safe_when(cop_positions)));
        autos[0].forward().map(move |v| {
            let min_safe_bank = safe_lvls_at_cops.iter().position(|lvl| lvl[v]);
            min_safe_bank.unwrap_or(usize::MAX)
        })
    }
}

/// all arguments also taken by [`super::compute_safe_robber_positions`] do the same as they do there.
#[allow(dead_code)]
pub fn compute_robber_energy_strat<R, S>(
    rules: R,
    params: EnergyParams,
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    manager: &thread_manager::LocalManager,
) -> Result<EnergyRobberStrat, String>
where
    S: SymmetryGroup + Serialize,
    R: CopRules,
{
    let EnergyParams {
        energy_per_step,
        allowance,
        bank_capacity,
    } = params;

    if energy_per_step == 0 {
        return Err("Unendliche Räuberenergie ist im Programm ausgeschlossen".to_string());
    }
    if allowance < energy_per_step {
        return Err(format!(
            "Räuber hat {allowance}/{energy_per_step} < 1 Schritte pro Runde."
        ));
    }

    let nr_map_vertices = edges.nr_vertices();
    if nr_map_vertices == 0 {
        return Err("Spielfeld darf nicht leer sein".to_string());
    }
    if !edges.is_connected() {
        return Err("Spielfeld muss zusammenhängend sein".to_string());
    }

    manager.update("liste Polizeipositionen")?;
    let cop_moves = CopConfigurations::new(&edges, &sym, nr_cops, manager)?;

    manager.update("reserviere Speicher für Queue")?;
    let Some(mut queue) = RobberStratQueue::new(&cop_moves) else {
        return Err("Zu wenig Speicherplatz (initiale Queue zu lang)".to_owned());
    };

    // a game state is a tuple (C, r, e), where C is a cop positions multiset (the cop state),
    // r the robber position and e the current robber energy.
    // the usual SafeRobberPositions stores for each usual game state (C, r),
    // wether the robber is safe in this situation (e.g. has a winning strategy).
    // we thus have all these tuples for each possible energy level.
    // note: it may be unoptimal cache-wise to do it in this order,
    // but this was the easiest to hack together for now.
    let mut safe_lvls = Vec::new();
    safe_lvls.reserve_exact(bank_capacity + 1);
    {
        let err = || "Zu wenig Speicherplatz (Räuberstrategiefunktion zu groß)".to_string();
        let mut safe_lvl = SafeRobberPositions::new(nr_map_vertices, &cop_moves).ok_or_else(err)?;
        for (i, index) in izip!(0.., cop_moves.all_positions()) {
            if i % 4096 == 0 {
                let percent = 100.0 * (i as f32) / (cop_moves.nr_configurations() as f32);
                let msg = format!("initialisiere Räuberstrategiefunktion: {percent:.2}%");
                manager.update(msg)?;
            }

            let robber_range = safe_lvl.robber_indices_at(index);
            for cop_pos in cop_moves.unpack(index) {
                safe_lvl.mark_robber_at(robber_range.at(cop_pos), false);
                for n in edges.neighbors_of(cop_pos) {
                    safe_lvl.mark_robber_at(robber_range.at(n), false);
                }
            }
        }
        for _ in 0..bank_capacity {
            let cloned_lvl = safe_lvl.try_clone().ok_or_else(err)?;
            safe_lvls.push(cloned_lvl);
        }
        safe_lvls.push(safe_lvl);
    }

    // as in the fog case, given some set of safe robber positions (or fog),
    // this can compute all positions reached in at most the given number of steps.
    let visible = EdgeList::from_iter((0..nr_map_vertices).map(std::iter::once), 1);
    let mut robber_step_computation = fog::FogStepComputation::new(&edges, &visible, 1);

    // same role as in the standard bruteforce algorithm, except a copy for each possible bank level exists.
    // role of a single entry (e.g. the role of the thing for a given energy level):
    // if the current game state has cop configuration `curr`, this contains all robber positions that
    // where safe last game state, given that the cops move to `curr`.
    let mut safe_should_cops_move_to_curr =
        vec![fog::Fog::new_filled(nr_map_vertices); bank_capacity + 1];

    // only used deep inside the following loops, but defined here to not be recreated below for each loop iteration
    let mut prev_intersect_to_curr = vec![false; nr_map_vertices];

    let mut time_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            let nr_safe = safe_lvls[0].robber_safe_when(curr_cop_positions).count_ones();
            manager.update(format!(
                "berechne Räuberstrategie:\n{:.2}% in Queue ({}), Runde {}, {:.2}% sicher",
                100.0 * (queue.len() as f32) / (cop_moves.nr_configurations() as f32),
                queue.len(),
                queue.rounds_complete(),
                100.0 * (nr_safe as f32) / (nr_map_vertices as f32),
            ))?;
            time_until_log_refresh = 10_000;
        }

        // iterate through all cops states possibly preceeding the current one and intersect
        // what is stored as safe then with the states marked safe when cops move to curr.
        let curr_cops = cop_moves.eager_unpack(curr_cop_positions);
        for mut prev_cops in rules.raw_cop_moves_from(&edges, curr_cops) {
            // update safe_should_cops_move_to_curr:
            // in the highest energy state, the robber could only have gotten here by walking at most the allowance,
            // the energy state below can only be reached by allowance + 1 and so on,
            // down to the lowest energy state, which perhaps the robber reached
            // from the (close to) highest energy state previously.
            // the annoying thing: a given energy e can be reached by any other e' > e
            // where (e' - e) - allowance is a multiple of energy_per_step.
            // thus, compared to the standard algorithm, we need to do roughly a factor (bank_capacity / energy_per_step) more.
            // note: in the standard algorithm this is only computed once for all neighbors of curr_cops.
            // we can't do that here, because prev_cops is required to do so in the general case.
            {
                for prev in &mut safe_should_cops_move_to_curr {
                    prev.set_cleared();
                }
                let mut curr_safe = fog::Fog::new_filled(nr_map_vertices);
                for (curr_balance, safe_lvl_all) in izip!(0.., &safe_lvls) {
                    // i am saddened that i chose two different integer types to store bits in fog vs SafeRobberPositions.
                    // one should fix this when writing the data structure with better cache locality for the current problem.
                    curr_safe
                        .as_mut_slice(nr_map_vertices)
                        .clone_from_bitslice(safe_lvl_all.robber_safe_when(curr_cop_positions));

                    let max_steps = (bank_capacity - curr_balance + allowance) / energy_per_step;
                    for taken_steps in 0..=max_steps {
                        let used_energy = taken_steps * energy_per_step;

                        let prev_balance = if used_energy + curr_balance >= allowance {
                            used_energy + curr_balance - allowance
                        } else {
                            // assume last round the robber had balance 0.
                            // he then got the allowance and used some of it to move (maybe 0).
                            // at least the rest must now be found in the bank. we thus have
                            // used_energy + curr_balance >= allowance in every possible scenario.
                            // the current case can thus be skipped.
                            continue;
                        };
                        let prev = &mut safe_should_cops_move_to_curr[prev_balance];

                        // this is where we need prev_cops:
                        // note that this step is performed backwards in time. we want to answer the question
                        // "given the current set of assumed safe vertices, from which positions could the robber reach these?"
                        // if the maximum number of steps the robber can do in a round is less than i guess 3,
                        // then prev_cops is not actually required to get the correct intersection below.
                        robber_step_computation.fog_speed = taken_steps as isize;
                        let prev_to_curr =
                            robber_step_computation.compute_step(&curr_safe, &prev_cops);
                        prev.or_assign(&prev_to_curr);
                    }
                }
            }

            let (autos_prev_to_repr, prev_cops_repr) = cop_moves.pack(&sym, &mut prev_cops);
            // except for additionally looping over the different balances,
            // this structure is the same as in the standard alrorithm.
            let mut change_at_any_auto_any_balance = false;
            for auto_prev_to_repr in autos_prev_to_repr {
                for (_balance, prev_safe_to_curr, all_with_balance) in
                    izip!(0.., &safe_should_cops_move_to_curr, &mut safe_lvls)
                {
                    let mut change_at_this_auto_this_balance = false;
                    for (v, v_safe_so_far) in izip!(
                        auto_prev_to_repr.backward(),
                        all_with_balance.robber_safe_when(prev_cops_repr)
                    ) {
                        let intersect = prev_safe_to_curr.is_foggy_at(v) && *v_safe_so_far;
                        change_at_this_auto_this_balance |= intersect != *v_safe_so_far;
                        prev_intersect_to_curr[v] = intersect;
                    }

                    if change_at_this_auto_this_balance {
                        change_at_any_auto_any_balance = true;

                        let range = all_with_balance.robber_indices_at(prev_cops_repr);
                        for (v, &val) in izip!(auto_prev_to_repr.forward(), &prev_intersect_to_curr)
                        {
                            all_with_balance.mark_robber_at(range.at(v), val);
                        }
                    }
                }
            }
            if change_at_any_auto_any_balance {
                queue.push(prev_cops_repr);
            }
        }
    }

    // the robber has a winning strategy iff he has safe positions for every cop state.
    // assuming connectivity, the cops can always walk to cop state 0 -> suffices to check that
    let cops_at_0 = CompactCopsIndex { fst_index: 0, rest_index: 0 };
    let robber_wins = safe_lvls[0].robber_safe_when(cops_at_0).any();

    Ok(EnergyRobberStrat {
        params,
        symmetry: ExplicitClasses::from(&sym),
        safe: safe_lvls,
        cop_moves,
        robber_wins,
    })
}

#[cfg(test)]
pub fn cop_number(rules: impl CopRules + Clone, p: EnergyParams, g: &Embedding3D) -> Option<usize> {
    let mut nr = 1;
    let sym = g.sym_group().to_explicit();
    let (_, manager) = thread_manager::build_managers();
    loop {
        let rs = rules.clone();
        let es = g.edges().clone();
        let strat = compute_robber_energy_strat(rs, p, nr, es, sym.clone(), &manager).ok()?;
        if !strat.robber_wins {
            return Some(nr);
        }
        nr += 1;
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn subdiv_hexagon_speed() {
        let rules = rules::GeneralEagerCops(10);
        let (_, manager) = thread_manager::build_managers();
        for n in 1..=4 {
            dbg!(n);
            let shape = shape::Shape::RegularPolygon2D(6);
            let map = Embedding3D::new_map_from(&shape, n);
            let sym = NoSymmetry::new(map.nr_vertices());
            assert_eq!(map.nr_vertices(), 1 + 6 * (((n + 1) * (n + 2)) / 2));

            // happy case: robber wins
            {
                let win_params = EnergyParams {
                    energy_per_step: n,
                    allowance: n + 1,
                    bank_capacity: n - 1,
                };
                let edges = map.edges().clone();
                let win_outcome =
                    compute_robber_energy_strat(rules, win_params, 1, edges, sym, &manager);
                assert!(win_outcome.unwrap().robber_wins);
            }

            // sad case: robber loses
            // note that the graph in question sometimes still allows for quite some interesting
            // robber strategies if the allowance is just slightly above 1 but still below 1 + 1/n.
            {
                let lose_params = EnergyParams::STANDARD_GAME;
                let edges = map.edges().clone();
                let lose_outcome =
                    compute_robber_energy_strat(rules, lose_params, 1, edges, sym, &manager);
                assert!(!lose_outcome.unwrap().robber_wins);
            }
        }
    }
}
