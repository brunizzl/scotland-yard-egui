//! this file is dedicated to the variant of the game, where the robber is granted units of energy per round,
//! which can either be used directly to walk or can be stored in a bank.
//! the bank can also be robbed to walk further.
//!
//! the cop movement rules are decided by [`crate::graph::bruteforce::rules`]. the cops have no energy bank.

use super::*;
// turns out the fog spreading logic is the same as the robber-can-many-edges-at-once logic. who would have thought?
use super::fog_util as fog;

pub type EnergyRatio = num::rational::Ratio<usize>;

#[derive(Debug, Clone, Copy)]
pub struct EnergyParams {
    /// the energy the robber is additionally given each round
    pub allowance: EnergyRatio,
    /// the maximum amount of energy that can be carried over a round boundary
    pub bank_capacity: EnergyRatio,
}

impl EnergyParams {
    #[cfg(test)]
    pub const STANDARD_GAME: Self = EnergyParams {
        allowance: EnergyRatio::ONE,
        bank_capacity: EnergyRatio::ZERO,
    };
}

/// returns two rational numbers with same ratios but shared denominator.
pub fn to_common_denom(x: EnergyRatio, y: EnergyRatio) -> (EnergyRatio, EnergyRatio) {
    let (x_numer, x_denom) = x.reduced().into_raw();
    let (y_numer, y_denom) = y.reduced().into_raw();
    let common_denom = num::integer::lcm(x_denom, y_denom);
    let new_x = EnergyRatio::new_raw(x_numer * common_denom / x_denom, common_denom);
    let new_y = EnergyRatio::new_raw(y_numer * common_denom / y_denom, common_denom);
    (new_x, new_y)
}

#[allow(dead_code)]
pub struct EnergyRobberStrat {
    pub params: EnergyParams,
    pub symmetry: ExplicitClasses,
    /// for each possible bank level, this maps to which vertices are safe
    /// (depending on the cop positions)
    pub safe: Vec<SafeRobberPositions>,
    pub cop_moves: CopConfigurations,
    pub robber_wins: bool,
}

/// all arguments also taken by [`super::compute_safe_robber_positions`] do the same as they do there.
#[allow(dead_code)]
pub fn compute_robber_energy_strat<R, S>(
    rules: R,
    raw_params: EnergyParams,
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    manager: &thread_manager::LocalManager,
) -> Result<EnergyRobberStrat, String>
where
    S: SymmetryGroup + Serialize,
    R: Rules,
{
    if raw_params.allowance.denom() == &0 {
        return Err("allowance is NaN (division by 0)".to_string());
    }
    if raw_params.bank_capacity.denom() == &0 {
        return Err("bank capacity is NaN (division by 0)".to_string());
    }
    if raw_params.allowance < EnergyRatio::ONE {
        return Err(format!(
            "allowance must at least be 1, but is {}.",
            raw_params.allowance
        ));
    }

    // to the outside we handle energy as a ratio, where one unit is used up when the robber walks one edge.
    // internally, the bank capacity and the allowance are brought to the same denominator.
    // it is thus easier to say a step takes *denominator* much energy and every energy value becoming an integer.
    let params = {
        let (a, b) = to_common_denom(raw_params.allowance, raw_params.bank_capacity);
        assert_eq!(a.denom(), b.denom());
        assert_eq!(a, raw_params.allowance);
        assert_eq!(b, raw_params.bank_capacity);
        EnergyParams { allowance: a, bank_capacity: b }
    };
    let energy_per_step = *params.allowance.denom();
    let allowance = *params.allowance.numer();
    let bank_capacity = *params.bank_capacity.numer();

    let nr_map_vertices = edges.nr_vertices();
    if nr_map_vertices == 0 {
        return Err("Spielfeld darf nicht leer sein".to_string());
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

    // only used deep inside the following loops, but defined here to not be recreated here each loop iteration
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

        // update safe_should_cops_move_to_curr:
        // in the highest energy state, the robber could only have gotten here by walking at most the allowance,
        // the energy state below can only be reached by allowance + 1 and so on,
        // down to the lowest energy state, which perhaps the robber reached
        // from the (close to) highest energy state previously.
        // the annoying thing: a given energy e can be reached by any other e' > e
        // where (e' - e) - allowance is a multiple of energy_per_step.
        // thus, compared to the standard algorithm, we need to do roughly a factor (bank_capacity / energy_per_step) more.
        //
        // note: this value computed as it is now is somewhat sketchy:
        // we want to know from which robber states (e.g. energy and position), it is possible to reach the
        // robber states marked as safe corrensponding to the current cop state.
        // TODO: is it oke to compute this using the current cop positions as blockers
        // that must be circumvented by the robber? really, one would want to use the previous cop state.
        // the sad part is, that there are many previous cop states for a single current cop state.
        // is it possible to change the perspective to not walk backwards, but forwards?
        // e.g. we would not look at the previous, but at future cop states?
        // anyway, i feel that in the current form, this is likely a bug.
        {
            let curr_cops = cop_moves.eager_unpack(curr_cop_positions);
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
                        // if the current balance is less than the allowance,
                        // we must have taken a step to get here.
                        // -> this is an impossible combination of taken_steps and curr_balance.
                        debug_assert_eq!(taken_steps, 0);
                        debug_assert!(curr_balance < allowance);
                        continue;
                    };
                    let prev = &mut safe_should_cops_move_to_curr[prev_balance];

                    robber_step_computation.fog_speed = taken_steps as isize;
                    // this is the sketchy part: do &curr_cops suffice here? TODO: think this through.
                    let prev_to_curr = robber_step_computation.compute_step(&curr_safe, &curr_cops);
                    prev.or_assign(&prev_to_curr);
                }
            }
        }

        // iterate through all cops states possibly preceeding the current one and intersect
        // what is stored as safe then with the states marked safe when cops move to curr.
        for (autos_prev_to_repr, prev_cop_positions_reprs) in
            rules.cop_moves_from(&cop_moves, &edges, &sym, curr_cop_positions)
        {
            let mut change_at_any_auto_any_balance = false;
            for auto_prev_to_repr in autos_prev_to_repr {
                for (_balance, prev_safe_to_curr, all_with_balance) in
                    izip!(0.., &safe_should_cops_move_to_curr, &mut safe_lvls)
                {
                    let mut change_at_this_auto_this_balance = false;
                    for (v, v_safe_so_far) in izip!(
                        auto_prev_to_repr.backward(),
                        all_with_balance.robber_safe_when(prev_cop_positions_reprs)
                    ) {
                        let intersect = prev_safe_to_curr.is_foggy_at(v) && *v_safe_so_far;
                        change_at_this_auto_this_balance |= intersect != *v_safe_so_far;
                        prev_intersect_to_curr[v] = intersect;
                    }

                    if change_at_this_auto_this_balance {
                        change_at_any_auto_any_balance = true;

                        let range = all_with_balance.robber_indices_at(prev_cop_positions_reprs);
                        for (v, &val) in izip!(auto_prev_to_repr.forward(), &prev_intersect_to_curr)
                        {
                            all_with_balance.mark_robber_at(range.at(v), val);
                        }
                    }
                }
            }
            if change_at_any_auto_any_balance {
                queue.push(prev_cop_positions_reprs);
            }
        }
    }

    let cops_at_0 = CompactCopsIndex { fst_index: 0, rest_index: 0 };
    let robber_wins = safe_lvls.iter().any(|lvl| lvl.robber_safe_when(cops_at_0).any());

    Ok(EnergyRobberStrat {
        params,
        symmetry: ExplicitClasses::from(&sym),
        safe: safe_lvls,
        cop_moves,
        robber_wins,
    })
}

#[cfg(test)]
pub fn cop_number(rules: impl Rules + Clone, p: EnergyParams, g: &Embedding3D) -> Option<usize> {
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
