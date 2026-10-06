//! this file is dedicated to the variant of the game, where the robber is granted units of energy per round,
//! which can either be used directly to walk or can be stored in a bank.
//! the bank can also be robbed to walk further.
//!
//! the cop movement rules are decided by [`crate::graph::bruteforce::rules`]. the cops have no energy bank.

use serde::{Deserialize, Serialize};

use super::*;

/// to the outside we handle energy as a ratio, where one unit is used up when the robber walks one edge.
/// internally, the bank capacity and the allowance are brought to the same denominator.
/// it is thus easier to say a step takes *denominator* much energy and every energy value becoming an integer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
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
            bank_capacity: c,
        } = self;
        if d == 1 {
            format!("a = {a}, c = {c}")
        } else {
            format!("a = {a}/{d}, c = {c}/{d}")
        }
    }
}

impl Default for EnergyParams {
    fn default() -> Self {
        Self::STANDARD_GAME
    }
}

pub type UEnergy = u8;
pub const INFINITY: UEnergy = UEnergy::MAX;

/// for each cop configuration in [`CopStates`] this struct stores for each map vertex,
/// how much energy the robber would need at minimum to win this game state.
#[derive(Serialize, Deserialize, PartialEq, Eq)]
pub struct MinSafeEnergy {
    data: BTreeMap<usize, Vec<UEnergy>>,
    nr_map_vertices: usize,
}

impl MinSafeEnergy {
    /// construct self if enough memory is available
    fn new(nr_map_vertices: usize, cop_states: &CopStates, init: UEnergy) -> Option<Self> {
        let mut time = BTreeMap::new();
        for (&fst_index, indices) in &cop_states.configurations {
            let nr_entries = indices.len().checked_mul(nr_map_vertices)?;

            let mut data = Vec::new();
            data.try_reserve(nr_entries).ok()?;
            data.resize(nr_entries, init);
            let old = time.insert(fst_index, data);
            debug_assert!(old.is_none());
        }

        Some(Self { data: time, nr_map_vertices })
    }

    pub fn nr_map_vertices(&self) -> usize {
        self.nr_map_vertices
    }
}

impl std::ops::Index<CompactCopsIndex> for MinSafeEnergy {
    type Output = [UEnergy];
    /// returns current number for each vertex in graph given cops placed like `index`
    fn index(&self, index: CompactCopsIndex) -> &Self::Output {
        let start = index.rest_index * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        &self.data.get(&index.fst_index).unwrap()[start..stop]
    }
}
impl std::ops::IndexMut<CompactCopsIndex> for MinSafeEnergy {
    /// returns current number for each vertex in graph given cops placed like `index`
    fn index_mut(&mut self, index: CompactCopsIndex) -> &mut Self::Output {
        let start = index.rest_index * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        &mut self.data.get_mut(&index.fst_index).unwrap()[start..stop]
    }
}

pub struct EnergyRobberStrat {
    pub params: EnergyParams,
    pub symmetry: ExplicitClasses,
    /// for each cop arrangement, this stores the minimum energy at each vertex,
    /// that the robber requires to survive.
    pub min_safe_energy: MinSafeEnergy,
    pub cop_states: CopStates,
    /// what energy does the robber need in his bank in order to win?
    /// if the robber has no winning strategy, this holds value [`INFINITY`].
    pub min_initial_robber_energy: UEnergy,
}

impl EnergyRobberStrat {
    fn new(
        params: EnergyParams,
        symmetry: ExplicitClasses,
        min_safe_energy: MinSafeEnergy,
        cop_states: CopStates,
    ) -> Self {
        let min_against = |cops| min_safe_energy[cops].iter().copied().min().unwrap();
        let min_initial_robber_energy = cop_states.all_positions().map(min_against).max().unwrap();
        Self {
            params,
            symmetry,
            min_safe_energy,
            cop_states,
            min_initial_robber_energy,
        }
    }

    /// the equivalent of [`RobberWinData::safe_vertices`]
    pub fn safe_vertex_energies(
        &self,
        mut cops: RawCops,
    ) -> impl ExactSizeIterator<Item = UEnergy> {
        let (autos, cop_positions) = self.cop_states.pack(&self.symmetry, &mut cops);
        let energies = &self.min_safe_energy[cop_positions];
        autos[0].forward().map(|v| energies[v])
    }
}

/// all arguments also taken by [`super::compute_safe_robber_positions`] do the same as they do there.
/// this function differs from [`compute_robber_energy_strat`], by reserving one bit for every
/// combination of piece positions and bank level between 0 and [`EnergyParams::bank_capacity`],
/// whereas the other only stores the minimum energy required for the robber to win for every combination of pieces.
/// we keep this implementation around to verify the improved version.
/// also: the the police strategy is found in the same manner.
#[cfg(test)]
fn compute_robber_energy_strat_naive<R, S>(
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
    // turns out the fog spreading logic is the same as the robber-can-walk-many-edges-at-once logic. who would have thought?
    use super::fog_util as fog;

    let EnergyParams {
        energy_per_step,
        allowance,
        bank_capacity,
    } = params;

    if energy_per_step == 0 {
        return Err("infinite robber energy is not considered.".to_string());
    }
    if allowance < energy_per_step {
        return Err(format!(
            "robber has {allowance}/{energy_per_step} < 1 steps per round."
        ));
    }
    if bank_capacity >= INFINITY as usize {
        let u_energy = std::any::type_name::<UEnergy>();
        return Err(format!("bank capacity (+1) must fit in {u_energy}"));
    }

    let nr_map_vertices = edges.nr_vertices();
    if nr_map_vertices == 0 {
        return Err("map must be nonempty".to_string());
    }
    if !edges.is_connected() {
        return Err("map must be connected".to_string());
    }

    manager.update("list cop positions")?;
    let cop_states = CopStates::new(&edges, &sym, nr_cops, manager)?;

    manager.update("reserve storage for queue")?;
    let Some(mut queue) = RobberStratQueue::new(&cop_states) else {
        return Err("not enough RAM (initial queue too long)".to_owned());
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
        let err = || "not enough RAM (robber strategy function too large)".to_string();
        let mut safe_lvl =
            SafeRobberPositions::new(nr_map_vertices, &cop_states).ok_or_else(err)?;
        for (i, index) in izip!(0.., cop_states.all_positions()) {
            if i % 4096 == 0 {
                let percent = 100.0 * (i as f32) / (cop_states.nr_states() as f32);
                let msg = format!("initialise robber strategy function: {percent:.2}%");
                manager.update(msg)?;
            }

            let robber_range = safe_lvl.robber_indices_at(index);
            for v in rules.vertices_in_reach(&edges, cop_states.unpack(index)) {
                safe_lvl.mark_robber_at(robber_range.at(v), false);
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
                "compute robber strategy:\n{:.2}% in queue ({}), round {}, {:.2}% safe",
                100.0 * (queue.len() as f32) / (cop_states.nr_states() as f32),
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
                prev.set_cleared();
            }
            let curr_cops = cop_states.eager_unpack(curr_cop_positions);
            let mut curr_safe = fog::Fog::new_filled(nr_map_vertices);
            for (curr_balance, safe_lvl_all) in izip!(0.., &safe_lvls) {
                // i am saddened that i chose two different integer types to store bits in fog vs SafeRobberPositions.
                // one should fix this when writing the data structure with better cache locality for the current problem.
                curr_safe
                    .as_mut_slice(nr_map_vertices)
                    .clone_from_bitslice(safe_lvl_all.robber_safe_when(curr_cop_positions));

                let max_steps = (bank_capacity + allowance - curr_balance) / energy_per_step;
                for taken_steps in 0..=max_steps {
                    let used_energy = taken_steps * energy_per_step;

                    if bank_capacity >= allowance && used_energy + curr_balance < allowance {
                        // assume last round the robber had balance 0.
                        // he then got the allowance and used some of it to move (maybe 0).
                        // this branch assumes the bank can hold at least the allowance.
                        // at least the rest must now be found in the bank. we thus have
                        // used_energy + curr_balance >= allowance in every possible scenario.
                        // the current case can thus be skipped.
                        continue;
                    }
                    let prev_balance = (used_energy + curr_balance).saturating_sub(allowance);
                    let prev = &mut safe_should_cops_move_to_curr[prev_balance];

                    // note that this step is performed backwards in time. we want to answer the question
                    // "given the current set of assumed safe vertices, from which positions could the robber reach these?"
                    robber_step_computation.fog_speed = taken_steps as isize;
                    let prev_to_curr = robber_step_computation.compute_step(&curr_safe, &curr_cops);
                    prev.or_assign(&prev_to_curr);
                }
            }

            // whenever an energy is safe for the robber, all energy levels above should be as well.
            debug_assert!((0..nr_map_vertices).all(|v| {
                let to_curr = safe_should_cops_move_to_curr.iter();
                to_curr.map(|fog| fog.is_foggy_at(v)).is_sorted()
            }));
        }

        // iterate through all cops states possibly preceeding the current one and intersect
        // what is stored as safe then with the states marked safe when cops move to curr.
        let all_prev_cops = rules.cop_moves_from(&cop_states, &edges, &sym, curr_cop_positions);
        for (autos_prev_to_repr, prev_cops_repr) in all_prev_cops {
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

    drop(queue);
    let Some(mut min_energy) = MinSafeEnergy::new(nr_map_vertices, &cop_states, INFINITY) else {
        return Err("not enough RAM (energy function too big)".to_owned());
    };

    manager.update("write energy function")?;
    let mut lvls_at_index = Vec::new();
    for index in cop_states.all_positions() {
        let energy = &mut min_energy[index];
        lvls_at_index.clear();
        lvls_at_index.extend(safe_lvls.iter().map(|lvl| lvl.robber_safe_when(index)));
        for (e, v) in izip!(energy, 0..nr_map_vertices) {
            if let Some(lvl) = lvls_at_index.iter().position(|lvl| lvl[v]) {
                *e = lvl as UEnergy;
            }
        }
    }

    Ok(EnergyRobberStrat::new(
        params,
        ExplicitClasses::from(&sym),
        min_energy,
        cop_states,
    ))
}

/// all arguments also taken by [`super::compute_safe_robber_positions`] do the same as they do there.
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
    let energy_per_step = params.energy_per_step as isize;
    let allowance = params.allowance as isize;
    let bank_capacity = params.bank_capacity as isize;

    if energy_per_step == 0 {
        return Err("infinite robber energy is not considered.".to_string());
    }
    if allowance < energy_per_step {
        return Err(format!(
            "robber has {allowance}/{energy_per_step} < 1 steps per round."
        ));
    }
    if bank_capacity >= INFINITY as isize {
        return Err(format!(
            "bank capacity (+1) must fit in {}",
            std::any::type_name::<UEnergy>()
        ));
    }

    let nr_map_vertices = edges.nr_vertices();
    if nr_map_vertices == 0 {
        return Err("graph must be nonempty".to_string());
    }
    if !edges.is_connected() {
        return Err("graph must be connected".to_string());
    }

    manager.update("list police positions")?;
    let cop_states = CopStates::new(&edges, &sym, nr_cops, manager)?;

    manager.update("reserve storage for energy function")?;
    let Some(mut min_energy) = MinSafeEnergy::new(edges.nr_vertices(), &cop_states, 0) else {
        return Err("not enough RAM (energy function too large)".to_owned());
    };

    manager.update("initialise queue")?;
    let Some(mut queue) = RobberStratQueue::new(&cop_states) else {
        return Err("not enough RAM (initial queue too long)".to_owned());
    };

    // used at multiple places whenever we want to do a breath-first search.
    let mut vertex_queue = std::collections::VecDeque::new();

    manager.update("initialise energy function")?;
    for (i, index) in izip!(0.., cop_states.all_positions()) {
        if i % 4096 == 0 {
            manager.recieve()?;
        }

        let energies = &mut min_energy[index];
        for v in rules.vertices_in_reach(&edges, cop_states.unpack(index)) {
            energies[v] = INFINITY;
        }
    }

    // in the loop below, this buffer is used to store the computed energy,
    // that the robber would need in the case that the cops moved to curr_cop_positions.
    let mut energy_should_cops_move_to_curr = vec![isize::MAX; nr_map_vertices];

    let mut time_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            let nr_safe = min_energy[curr_cop_positions]
                .iter()
                .filter(|&&e| e < INFINITY)
                .count();
            manager.update(format!(
                "compute robber strategy:\n{:.2}% in queue ({}), round {}, {:.2}% safe",
                100.0 * (queue.len() as f32) / (cop_states.nr_states() as f32),
                queue.len(),
                queue.rounds_complete(),
                100.0 * (nr_safe as f32) / (nr_map_vertices as f32),
            ))?;
            time_until_log_refresh = 10_000;
        }

        // update energy_should_cops_move_to_curr:
        // in the highest energy state, the robber could only have gotten here by walking at most the allowance,
        // the energy state below can only be reached by allowance + 1 and so on,
        // down to the lowest energy state, which perhaps the robber reached
        // from the (close to) highest energy state previously.
        {
            debug_assert!(vertex_queue.is_empty());
            for (v, &curr_at_v, prev_at_v) in izip!(
                0..,
                &min_energy[curr_cop_positions],
                &mut energy_should_cops_move_to_curr
            ) {
                // initialise each energy value with the required previous energy,
                // assuming the robber was already at v in the last round.
                *prev_at_v = match curr_at_v {
                    // infinity - allowance is still infinity.
                    INFINITY => INFINITY as isize,
                    // note that we must allow negative values here.
                    // in the case where the value is negative, the interpretation is that this
                    // is not the final computed value, but v is the destination with
                    // the robber previously standing sufficiently far away.
                    // these far previous robber positions are discovered in the while loop below.
                    _ => curr_at_v as isize - allowance,
                };
                // only enter v into queue if the current value could have been attained by moving to this position.
                if *prev_at_v + energy_per_step <= bank_capacity {
                    vertex_queue.push_back(v);
                }
            }
            let curr_cops = cop_states.eager_unpack(curr_cop_positions);
            while let Some(v) = vertex_queue.pop_front() {
                let prev_at_v = energy_should_cops_move_to_curr[v];
                debug_assert!(prev_at_v <= bank_capacity);
                let prev_to_v = prev_at_v + energy_per_step;
                if prev_to_v > bank_capacity {
                    continue;
                }
                // check for each neighbor n of v, whether it would be better to not end a move there as robber,
                // but instead continue the move and walk to v.
                // (with the precondition that it makes sense to walk to / start in n in the first place.)
                for n in edges.neighbors_of(v) {
                    let prev_at_n = &mut energy_should_cops_move_to_curr[n];
                    if *prev_at_n > prev_to_v && !curr_cops.contains(&n) {
                        *prev_at_n = prev_to_v;
                        vertex_queue.push_back(n);
                    }
                }
            }
            let energy_is_inf = |&c| energy_should_cops_move_to_curr[c] == INFINITY as isize;
            debug_assert!(curr_cops.iter().all(energy_is_inf));
        }

        let all_prev_cops = rules.cop_moves_from(&cop_states, &edges, &sym, curr_cop_positions);
        for (autos_prev_to_repr, prev_cops_repr) in all_prev_cops {
            let mut prev_energy_changed = false;
            for auto_prev_to_repr in autos_prev_to_repr {
                for (v, prev_min_energy) in izip!(
                    auto_prev_to_repr.backward(),
                    &mut min_energy[prev_cops_repr]
                ) {
                    let to_curr = energy_should_cops_move_to_curr[v];
                    if (*prev_min_energy as isize) < to_curr {
                        prev_energy_changed = true;

                        debug_assert!(to_curr <= bank_capacity || to_curr == INFINITY as isize);
                        *prev_min_energy = to_curr as UEnergy;
                    }
                }
            }
            if prev_energy_changed {
                queue.push(prev_cops_repr);
            }
        }
    }

    Ok(EnergyRobberStrat::new(
        params,
        ExplicitClasses::from(&sym),
        min_energy,
        cop_states,
    ))
}

pub struct EnergyCopStrat {
    pub params: EnergyParams,
    pub symmetry: ExplicitClasses,
    /// for each possible energy state of the robber, this holds the usual [`TimeToWin`] of [`CopStrategy`].
    pub times_to_live: Vec<TimeToWin>,
    pub cop_states: CopStates,
    /// what energy does the robber need in his bank in order to win?
    /// if the robber has no winning strategy, this holds value [`INFINITY`].
    pub min_initial_robber_energy: UEnergy,
}

impl EnergyCopStrat {
    fn new(
        params: EnergyParams,
        symmetry: ExplicitClasses,
        times_to_live: Vec<TimeToWin>,
        cop_states: CopStates,
    ) -> Self {
        let min_against = |cops: CompactCopsIndex| {
            (0..=params.bank_capacity)
                .map(|b: usize| times_to_live[b][cops].iter().max().unwrap())
                .position(|&ttl| ttl == UTime::MAX)
                .map_or(UEnergy::MAX, |energy| energy as UEnergy)
        };
        let min_initial_robber_energy = cop_states.all_positions().map(min_against).max().unwrap();
        Self {
            params,
            symmetry,
            times_to_live,
            cop_states,
            min_initial_robber_energy,
        }
    }

    pub fn as_ref<'a>(&'a self) -> CopStrategyRef<'a> {
        CopStrategyRef {
            symmetry: &self.symmetry,
            cop_states: &self.cop_states,
            time_to_win: &self.times_to_live,
        }
    }
}

type USteps = UTime;
/// this is stored per vertex.
#[derive(Clone, Copy)]
struct BestRobberMove {
    /// to vertex of what time to live can robber go in this round from current vertex
    time_to_live: UTime,
    /// how far away is the reachable vertex of highest time to live
    nr_steps: USteps,
}
impl BestRobberMove {
    const fn new(time_to_live: UTime, nr_steps: USteps) -> Self {
        Self { time_to_live, nr_steps }
    }
}

/// keeps all memory required to compute the (relevant parameters of) an ideal robber move for each vertex.
/// inputs are a given assignment of time to live to each vertex and a maximum number of robber steps allowed in the current move.
struct BatchedBestRobberMoves {
    data: Vec<BestRobberMove>,
    queue: std::collections::VecDeque<usize>,
}

impl BatchedBestRobberMoves {
    fn new(nr_map_vertices: usize, ps: EnergyParams) -> Result<Self, String> {
        if (ps.bank_capacity + ps.allowance) / ps.energy_per_step >= USteps::MAX as usize {
            let max = USteps::MAX;
            let msg = format!("robber may only do fewer than {max} steps per round");
            return Err(msg);
        }
        let data = vec![BestRobberMove::new(0, USteps::MAX); nr_map_vertices];
        let queue = std::collections::VecDeque::new();
        Ok(Self { data, queue })
    }

    /// returns for each vertex v, what the highest time to live reachable in at most `max_robber_steps` in `curr_times` is.
    /// assumes every entry in curr_times to be either [`UTime::MAX`] or at most `max_ttl`.
    fn compute(
        &mut self,
        edges: &EdgeList,
        curr_cops: &RawCops,
        curr_times: &[UTime],
        max_robber_steps: usize,
        max_ttl: UTime,
    ) -> &[BestRobberMove] {
        debug_assert_eq!(edges.nr_vertices(), self.data.len());
        debug_assert_eq!(edges.nr_vertices(), curr_times.len());
        debug_assert!(curr_times.iter().all(|&ttl| ttl <= max_ttl || ttl == UTime::MAX));
        self.data.fill(BestRobberMove::new(0, USteps::MAX));

        // we spread each time to live to all reachable vertices.
        // by doing this with increasing ttl's, we ensure the lower ttl's where already spread
        // when spreading the current one above them.
        for time_to_live in (1..=max_ttl).chain(std::iter::once(UTime::MAX)) {
            debug_assert!(self.queue.is_empty());
            // maybe TODO: it is potentially wastefull to iterate over every vertex for every time_to_live.
            for (v, robber_step, &v_curr_ttl) in izip!(0.., &mut self.data, curr_times) {
                if v_curr_ttl == time_to_live {
                    *robber_step = BestRobberMove { time_to_live, nr_steps: 0 };
                    self.queue.push_back(v);
                }
            }
            while let Some(v) = self.queue.pop_front() {
                let curr = &self.data[v];
                debug_assert_eq!(curr.time_to_live, time_to_live);
                debug_assert!((curr.nr_steps as usize) < max_robber_steps);
                let next_steps = curr.nr_steps + 1;
                for neigh_v in edges.neighbors_of(v) {
                    if curr_cops.contains(&neigh_v) {
                        // don't allow robber to walk through cops.
                        continue;
                    }
                    let neigh = &self.data[neigh_v];
                    debug_assert!(neigh.time_to_live <= time_to_live);
                    if neigh.time_to_live < time_to_live
                        || (neigh.time_to_live == time_to_live && neigh.nr_steps > next_steps)
                    {
                        self.data[neigh_v] = BestRobberMove::new(time_to_live, next_steps);
                        if (next_steps as usize) < max_robber_steps {
                            self.queue.push_back(neigh_v);
                        }
                    }
                }
            }
        }
        &self.data
    }
}

pub fn compute_cop_energy_strat<R, S>(
    rules: R,
    params: EnergyParams,
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    manager: &thread_manager::LocalManager,
) -> Result<EnergyCopStrat, String>
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
        return Err("infinite robber energy is not considered.".to_string());
    }
    if allowance < energy_per_step {
        return Err(format!(
            "robber has {allowance}/{energy_per_step} < 1 steps per round."
        ));
    }
    if bank_capacity >= INFINITY as usize {
        let u_energy = std::any::type_name::<UEnergy>();
        return Err(format!("bank capacity (+1) must fit in {u_energy}"));
    }

    let nr_map_vertices = edges.nr_vertices();
    if nr_map_vertices == 0 {
        return Err("map must be nonempty".to_string());
    }
    if !edges.is_connected() {
        return Err("map must be connected".to_string());
    }

    // local variable used deep down in the big loop.
    // initialised here to check the preconditions before more complex things are build.
    // initialised outside any loop, because this is only it's own type to recycle heap memory.
    let mut best_robber_moves = BatchedBestRobberMoves::new(nr_map_vertices, params)?;

    manager.update("list cop positions")?;
    let cop_states = CopStates::new(&edges, &sym, nr_cops, manager)?;

    manager.update("reserve storage for queue")?;
    let Some(mut queue) = CopStratQueue::new(&cop_states) else {
        return Err("not enough RAM (initial queue too long)".to_owned());
    };

    // a game state is a tuple (C, r, e), where C is a cop positions multiset (the cop state),
    // r the robber position and e the current robber energy.
    // the usual TimeToWin stores for each usual game state (C, r),
    // how many more rounds the cops need at least to capture the robber.
    // we thus store this number for each possible energy level.
    // note: it may be unoptimal cache-wise to do it in this order,
    // but this was the easiest to hack together for now.
    let mut times_to_live = Vec::new();
    times_to_live.reserve_exact(bank_capacity + 1);
    {
        let err = || "not enough RAM (time-to-live function too large)".to_string();
        let mut time_to_live = TimeToWin::new(nr_map_vertices, &cop_states).ok_or_else(err)?;
        for (i, cops_index) in izip!(0.., cop_states.all_positions()) {
            if i % 4096 == 0 {
                let percent = 100.0 * (i as f32) / (cop_states.nr_states() as f32);
                let msg = format!("initialise time-to-live function: {percent:.2}%");
                manager.update(msg)?;
            }

            let times_at_cops = &mut time_to_live[cops_index];
            for v in rules.vertices_in_reach(&edges, cop_states.unpack(cops_index)) {
                times_at_cops[v] = 0;
            }
        }
        for _ in 0..bank_capacity {
            let cloned_time_to_live = time_to_live.try_clone().ok_or_else(err)?;
            times_to_live.push(cloned_time_to_live);
        }
        times_to_live.push(time_to_live);
    }

    // same role as in the standard bruteforce algorithm, except a copy for each possible bank level exists.
    // role of a single entry (e.g. the role of the thing for a given energy level):
    // if the current game state has cop configuration `curr`, this contains the time for cops to win
    // for each possible robber position, given that the cops move to `curr`.
    let mut times_should_cops_move_to_curr_storage =
        vec![UTime::MAX; nr_map_vertices * (bank_capacity + 1)];
    let mut times_should_cops_move_to_curr = times_should_cops_move_to_curr_storage
        .chunks_mut(nr_map_vertices)
        .collect_vec();

    let mut iters_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        iters_until_log_refresh -= 1;
        if iters_until_log_refresh == 0 {
            let nr_safe = |ttw: &[_]| ttw.iter().filter(|&&t| t == UTime::MAX).count() as f32;
            manager.update(format!(
                "compute cop strategy:\n{:.2}% in queue ({}), round {}, {:.2}% unreached",
                100.0 * (queue.len() as f32) / (cop_states.nr_states() as f32),
                queue.len(),
                queue.curr_max(),
                100.0 * nr_safe(&times_to_live[0][curr_cop_positions]) / (nr_map_vertices as f32),
            ))?;
            iters_until_log_refresh = 1_000;
        }

        // update times_should_cops_move_to_curr:
        // in the highest energy state, the robber could only have gotten here by walking at most the allowance,
        // the energy state below can only be reached by allowance + 1 and so on,
        // down to the lowest energy state, which perhaps the robber reached
        // from the (close to) highest energy state previously.
        // the annoying thing: a given energy e can be reached by any other e' > e
        // where (e' - e) - allowance is a multiple of energy_per_step.
        // thus, compared to the standard algorithm, we need to do roughly a factor (bank_capacity / energy_per_step) more.
        {
            for time_to_curr in &mut times_should_cops_move_to_curr {
                time_to_curr.fill(0);
            }
            let curr_cops = cop_states.eager_unpack(curr_cop_positions);
            for (curr_balance, time_to_live) in izip!(0.., &times_to_live) {
                let max_robber_steps = (bank_capacity + allowance - curr_balance) / energy_per_step;
                let robber_moves = best_robber_moves.compute(
                    &edges,
                    &curr_cops,
                    &time_to_live[curr_cop_positions],
                    max_robber_steps,
                    queue.curr_max(),
                );

                for (v, robber_move) in izip!(0.., robber_moves) {
                    if robber_move.nr_steps as usize > max_robber_steps {
                        debug_assert_eq!(robber_move.nr_steps, USteps::MAX);
                        continue;
                    }
                    let used_energy = robber_move.nr_steps as usize * energy_per_step;
                    if bank_capacity >= allowance && used_energy + curr_balance < allowance {
                        // assume last round the robber had balance 0.
                        // he then got the allowance and used some of it to move (maybe 0).
                        // this branch assumes the bank can hold at least the allowance.
                        // at least the rest must now be found in the bank. we thus have
                        // used_energy + curr_balance >= allowance in every possible scenario.
                        // the current case can thus be skipped.
                        continue;
                    }
                    let prev_balance = (used_energy + curr_balance).saturating_sub(allowance);
                    for time_to_curr in &mut times_should_cops_move_to_curr[prev_balance..] {
                        time_to_curr[v] = UTime::max(time_to_curr[v], robber_move.time_to_live);
                    }
                }
            }

            // whenever an energy is safe for the robber, all energy levels above should be as well.
            // more generally, whenever the robber can live ttl many more rounds with a given energy,
            // he can live at least as many rounds with a higher energy.
            debug_assert!((0..nr_map_vertices).all(|v| {
                let to_curr = times_should_cops_move_to_curr.iter();
                to_curr.map(|time| time[v]).is_sorted()
            }));
        }
        {
            let mut curr_is_at_max = false;
            for time_to_curr in &mut times_should_cops_move_to_curr {
                for ttl in time_to_curr.iter_mut() {
                    let (new_time, at_max) = queue.clamp_successor(*ttl)?;
                    *ttl = new_time;
                    curr_is_at_max |= at_max;
                }
            }
            if curr_is_at_max {
                queue.mark_as_at_max(curr_cop_positions);
            }
        }

        // iterate through all cops states possibly preceeding the current one and intersect
        // what is stored as safe then with the states marked safe when cops move to curr.
        for (autos_prev_to_repr, prev_cops_repr) in
            rules.cop_moves_from(&cop_states, &edges, &sym, curr_cop_positions)
        {
            // except for additionally looping over the different balances,
            // this structure is the same as in the standard alrorithm.
            let mut change_at_any_auto_any_balance = false;
            for auto_prev_to_repr in autos_prev_to_repr {
                for (_balance, time_to_curr, time_to_live) in
                    izip!(0.., &times_should_cops_move_to_curr, &mut times_to_live)
                {
                    for (v, neigh_time) in izip!(
                        auto_prev_to_repr.backward(),
                        &mut time_to_live[prev_cops_repr]
                    ) {
                        let to_curr_time = time_to_curr[v];
                        if *neigh_time > to_curr_time {
                            debug_assert!(to_curr_time < queue.curr_max());
                            change_at_any_auto_any_balance = true;
                            *neigh_time = to_curr_time;
                        }
                    }
                }
            }
            if change_at_any_auto_any_balance {
                queue.push(prev_cops_repr);
            }
        }
    }

    Ok(EnergyCopStrat::new(
        params,
        ExplicitClasses::from(&sym),
        times_to_live,
        cop_states,
    ))
}

#[cfg(test)]
fn cmp_results(cop_strat: &EnergyCopStrat, robber_strat: &EnergyRobberStrat) -> Result<(), String> {
    assert_eq!(cop_strat.params, robber_strat.params);

    for cops in cop_strat.cop_states.all_positions() {
        for v in 0..cop_strat.symmetry.nr_vertices() {
            let robber_min_robber_energy = robber_strat.min_safe_energy[cops][v];
            let cops_min_robber_energy = (0..=(cop_strat.params.bank_capacity))
                .position(|bank| cop_strat.times_to_live[bank][cops][v] == UTime::MAX)
                .map_or(UEnergy::MAX, |energy| energy as UEnergy);

            if robber_min_robber_energy != cops_min_robber_energy {
                let raw_cops = cop_strat.cop_states.eager_unpack(cops);
                return Err(format!(
                    "difference at cops {raw_cops:?} with vertex {v}. \
                    cop strat: {cops_min_robber_energy}, robber strat: {robber_min_robber_energy}"
                ));
            }
        }
    }
    if cop_strat.min_initial_robber_energy != robber_strat.min_initial_robber_energy {
        return Err(format!(
            "computed min inital robber energy via cops: {}, via robber: {}",
            cop_strat.min_initial_robber_energy, robber_strat.min_initial_robber_energy
        ));
    }
    Ok(())
}

#[cfg(test)]
pub fn cop_number(rules: impl CopRules + Clone, p: EnergyParams, g: &Embedding3D) -> Option<usize> {
    let mut nr = 1;
    let sym = g.sym_group().to_explicit();
    let (_, manager) = thread_manager::build_managers();
    loop {
        let rs = rules.clone();
        let es = g.edges().clone();
        let strat_v1 = compute_robber_energy_strat(rs, p, nr, es, sym.clone(), &manager).ok()?;

        let rs = rules.clone();
        let es = g.edges().clone();
        let strat_v2 =
            compute_robber_energy_strat_naive(rs, p, nr, es, sym.clone(), &manager).unwrap();

        let rs = rules.clone();
        let es = g.edges().clone();
        let strat_v3 = compute_cop_energy_strat(rs, p, nr, es, sym.clone(), &manager).unwrap();

        assert!(strat_v1.min_safe_energy == strat_v2.min_safe_energy);
        assert!(strat_v1.min_initial_robber_energy == strat_v2.min_initial_robber_energy);
        assert_eq!(cmp_results(&strat_v3, &strat_v1), Ok(()));
        if strat_v1.min_initial_robber_energy == INFINITY {
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
            let shape = Shape::RegularPolygon2D(Resolution(n), 6);
            let map = Embedding3D::new_map_from(shape);
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
                    compute_robber_energy_strat(rules, win_params, 1, edges, sym, &manager)
                        .unwrap();

                let edges = map.edges().clone();
                let win_outcome_naive =
                    compute_robber_energy_strat_naive(rules, win_params, 1, edges, sym, &manager)
                        .unwrap();

                assert!(win_outcome.min_safe_energy == win_outcome_naive.min_safe_energy);
                assert!(win_outcome_naive.min_initial_robber_energy == 0);
                assert!(win_outcome.min_initial_robber_energy == 0);

                let edges = map.edges().clone();
                let cop_strat =
                    compute_cop_energy_strat(rules, win_params, 1, edges, sym, &manager).unwrap();
                assert_eq!(cmp_results(&cop_strat, &win_outcome), Ok(()));
            }

            // sad case: robber loses
            // note that the graph in question sometimes still allows for quite some interesting
            // robber strategies if the allowance is just slightly above 1 but still below 1 + 1/n.
            {
                let lose_params = EnergyParams::STANDARD_GAME;
                let edges = map.edges().clone();
                let lose_outcome =
                    compute_robber_energy_strat(rules, lose_params, 1, edges, sym, &manager)
                        .unwrap();

                let edges = map.edges().clone();
                let lose_outcome_naive =
                    compute_robber_energy_strat_naive(rules, lose_params, 1, edges, sym, &manager)
                        .unwrap();

                assert!(lose_outcome.min_safe_energy == lose_outcome_naive.min_safe_energy);
                assert!(lose_outcome_naive.min_initial_robber_energy == INFINITY);
                assert!(lose_outcome.min_initial_robber_energy == INFINITY);

                let edges = map.edges().clone();
                let cop_strat =
                    compute_cop_energy_strat(rules, lose_params, 1, edges, sym, &manager).unwrap();
                assert_eq!(cmp_results(&cop_strat, &lose_outcome), Ok(()));
            }
        }
    }
}
