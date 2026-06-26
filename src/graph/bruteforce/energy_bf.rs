//! this file is dedicated to the variant of the game, where the robber is granted units of energy per round,
//! which can either be used directly to walk or can be stored in a bank.
//! the bank can also be robbed to walk further.
//!
//! the cop movement rules are decided by [`crate::graph::bruteforce::rules`]. the cops have no energy bank.

use super::*;
// turns out the fog spreading logic is the same as the robber-can-many-edges-at-once logic. who would have thought?
use super::fog_util as fog;

pub type EnergyRatio = num::rational::Ratio<isize>;

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
    /// the energy the robber is additionally given each round
    pub allowance: EnergyRatio,
    /// the maximum amount of energy that can be carried over a round boundary
    pub bank_capacity: EnergyRatio,
    pub symmetry: ExplicitClasses,
    /// for each possible bank level, this maps to which vertices are safe
    /// (depending on the cop positions)
    pub safe: Vec<SafeRobberPositions>,
    pub cop_moves: CopConfigurations,
    pub robber_wins: bool,
}

/// all arguments also taken by [`super::compute_safe_robber_positions`] do the same as they do there.
#[allow(dead_code)]
pub fn compute_robber_energy_strat<R>(
    rules: R,
    nr_cops: usize,
    raw_allowance: EnergyRatio,
    raw_bank_capacity: EnergyRatio,
    edges: EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<EnergyRobberStrat, String>
where
    R: Rules,
{
    if raw_allowance.denom() == &0 {
        return Err("allowance is NaN (division by 0)".to_string());
    }
    if raw_bank_capacity.denom() == &0 {
        return Err("bank capacity is NaN (division by 0)".to_string());
    }
    if raw_allowance < EnergyRatio::ONE {
        return Err(format!(
            "allowance must at least be 1, but is {raw_allowance}."
        ));
    }

    // to the outside we handle energy as a ratio, where one unit is used up when the robber walks one edge.
    // internally, the bank capacity and the allowance are brought to the same denominator.
    // it is thus easier to say a step takes *denominator* much energy and every energy value becoming an integer.
    let (allowance_ratio, bank_capacity_ratio) = to_common_denom(raw_allowance, raw_bank_capacity);
    assert_eq!(allowance_ratio.denom(), bank_capacity_ratio.denom());
    assert_eq!(allowance_ratio, raw_allowance);
    assert_eq!(bank_capacity_ratio, raw_bank_capacity);
    let energy_per_step = *allowance_ratio.denom();
    let allowance = *allowance_ratio.numer();
    let bank_capacity = *bank_capacity_ratio.numer();

    let nr_map_vertices = edges.nr_vertices();
    if nr_map_vertices == 0 {
        return Err("Spielfeld darf nicht leer sein".to_string());
    }

    manager.update("liste Polizeipositionen")?;
    let sym = NoSymmetry::new(nr_map_vertices);
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
    safe_lvls.reserve_exact(bank_capacity as usize);
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
        for _ in 0..(bank_capacity - 1) {
            let cloned_lvl = safe_lvl.try_clone().ok_or_else(err)?;
            safe_lvls.push(cloned_lvl);
        }
        safe_lvls.push(safe_lvl);
    }

    // as in the fog case, given some set of safe robber positions (or fog),
    // this can compute all positions reached in at most the given number of steps.
    let mut robber_step_computation = fog::FogStepComputation::new(&edges, &edges, 1);

    // same role as in the standard bruteforce algorithm, except a copy for each possible bank level exists.
    // role of a single entry (e.g. the role of the thing for a given energy level):
    // if the current game state has cop configuration `curr`, this contains all robber positions that
    // where safe last game state, given that the cops move to `curr`.
    let mut safe_should_cops_move_to_curr =
        vec![fog::Fog::new_filled(nr_map_vertices); bank_capacity as usize];

    // only used deep inside the following loops, but defined here to not be recreated here each loop iteration
    let mut prev_intersect_to_curr = vec![false; nr_map_vertices];

    let mut time_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        let curr_cops = cop_moves.eager_unpack(curr_cop_positions);

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

        {
            for prev in &mut safe_should_cops_move_to_curr {
                prev.clear();
            }
            let mut curr_safe = fog::Fog::new_filled(nr_map_vertices);
            for (curr_balance, safe_lvl_all) in izip!(0.., &safe_lvls) {
                // i am saddened that i chose two different integer types to store bits in fog vs SafeRobberPositions.
                // one fix this when (possibly) writing the data structure with better cache locality for the current problem.
                curr_safe
                    .as_mut_slice()
                    .clone_from_bitslice(safe_lvl_all.robber_safe_when(curr_cop_positions));

                let max_steps = (bank_capacity - curr_balance + allowance) / energy_per_step;
                for taken_steps in 0..=max_steps {
                    let used_energy = taken_steps * energy_per_step;
                    let prev_balance = used_energy + curr_balance - allowance;
                    debug_assert!(prev_balance >= 0);
                    let prev = &mut safe_should_cops_move_to_curr[prev_balance as usize];

                    robber_step_computation.fog_speed = taken_steps;
                    let prev_to_curr = robber_step_computation.compute_step(&curr_safe, &curr_cops);
                    prev.or_assign(&prev_to_curr);
                }
            }
        }

        // iterate through all cops states possibly preceeding the current one and intersect
        // what is stored as safe then with the states marked safe when cops move to curr.
        let all_prev_cop_states = rules.raw_cop_moves_from(&edges, curr_cops);
        for mut prev_cops in all_prev_cop_states.map(RawCops::sorted) {
            let (_, prev_cops_index) = cop_moves.pack(&sym, &mut prev_cops);
            let mut change_at_any_balance = false;
            for (prev_safe_to_curr, prev_all) in
                izip!(&safe_should_cops_move_to_curr, &mut safe_lvls)
            {
                let mut change_at_this_balance = false;
                let prev_at_cops = prev_all.robber_safe_when(prev_cops_index);
                for (v, intersect, old_prev_safe) in
                    izip!(0.., &mut prev_intersect_to_curr, prev_at_cops)
                {
                    let v_safe_so_far = prev_safe_to_curr.is_foggy_at(v);
                    *intersect = *old_prev_safe && v_safe_so_far;
                    change_at_this_balance |= *intersect != v_safe_so_far;
                }
                if change_at_this_balance {
                    change_at_any_balance = true;

                    let range = prev_all.robber_indices_at(prev_cops_index);
                    for (v, &val) in izip!(0.., &prev_intersect_to_curr) {
                        prev_all.mark_robber_at(range.at(v), val);
                    }
                }
            }
            if change_at_any_balance {
                queue.push(prev_cops_index);
            }
        }
    }

    let cops_at_0 = CompactCopsIndex { fst_index: 0, rest_index: 0 };
    let robber_wins = safe_lvls.iter().any(|lvl| lvl.robber_safe_when(cops_at_0).any());

    Ok(EnergyRobberStrat {
        allowance: allowance_ratio,
        bank_capacity: bank_capacity_ratio,
        symmetry: ExplicitClasses::from(&sym),
        safe: safe_lvls,
        cop_moves,
        robber_wins,
    })
}
