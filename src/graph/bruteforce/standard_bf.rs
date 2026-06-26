//! this file implements the bruteforce algorithms for the "usual" game.
//! this means the rules have freedom to decide how the police can move,
//! but the robber is allowed to move either 0 or 1 steps per round.
use super::*;

#[derive(Serialize, Deserialize)]
pub struct RobberWinData {
    #[serde(deserialize_with = "deserialize_explicit")]
    pub symmetry: ExplicitClasses,
    pub safe: SafeRobberPositions,
    pub cop_moves: CopConfigurations,
}

/// the field [`RobberWinData::symmetry`] changed type to [`ExplicitClasses`] from [`SymGroup`].
/// we thus want both versions to be correctly deserialized.
pub fn deserialize_explicit<'de, D>(deserializer: D) -> Result<ExplicitClasses, D::Error>
where
    D: serde::Deserializer<'de>,
{
    //SymGroup::deserialize(deserializer).map(|s| ExplicitClasses::from(s))
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum ExplicitAsMaybeEnum {
        Enum(SymGroup),
        Direct(ExplicitClasses),
    }

    Ok(match ExplicitAsMaybeEnum::deserialize(deserializer)? {
        ExplicitAsMaybeEnum::Enum(e) => e.into(),
        ExplicitAsMaybeEnum::Direct(expl) => expl,
    })
}

impl RobberWinData {
    /// does all the automorphism stuff for one and just returns for each map vertex wether it is safe,
    /// given the passed cop state.
    pub fn safe_vertices(&self, mut cops: RawCops) -> impl ExactSizeIterator<Item = bool> {
        let (autos, cop_positions) = self.cop_moves.pack(&self.symmetry, &mut cops);
        let safe_vertices = self.safe.robber_safe_when(cop_positions);
        autos[0].forward().map(|v| safe_vertices[v])
    }
}

#[derive(Serialize, Deserialize)]
pub enum Outcome {
    /// stores for how many cops result was computed and for a graph over how many vertices of what shape
    CopsWin,
    RobberWins(RobberWinData),
}

/// algorithm 2.2 of Fabian Hamann's masters thesis.
///
/// idea: store for each cop configuration and each vertex of the game map, if this vertex is safe for the robber to be on.
/// (this storage is [`SafeRobberPositions::safe`]).
///
/// start with every vertex marked as safe, that is not a cop's position and not a neighbor of a cop's position.
/// put every cop state in the queue to update.
///
/// for any cop state `c`:
/// if the cops can arrive at `c` in a single move (e.g. all positions except one are identical and that one neighbors the old position),
/// the robber was only safe on a vertex with respect to a previous cop state `c'`, if he can move to a vertex marked as safe for `c`.
///
/// thus the main loop of the algorithm takes a cop state `c` out of the queue,
/// marks the neighborhood of the vertices marked safe for state `c` as *"safe should the cops move to `c`"* and goes though all
/// potential previous cop states `c'`. a vertex will now lose it's safety status w.r.t. `c'`, if it is not also contained
/// in the freshly marked neighborhood of the vertices marked safe for state `c`.
/// should at least one vertex lose it's safe status for cop configuration `c'`, we need to test cop states `c''` which could turn into `c'`
/// via a single move. hence `c'` is entered into the queue.
///
/// the whole thing is somewhat more complicated by the fact, that neighboring cop states `c` and `c'` may not both be stored in the same
/// rotation. therefore one needs to constantly rotate between the two.
pub fn compute_safe_robber_positions<S, R>(
    rules: R,
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    manager: &thread_manager::LocalManager,
) -> Result<Outcome, String>
where
    S: SymmetryGroup + Serialize,
    R: Rules,
{
    if nr_cops == 0 {
        return Err("Mindestens ein Cop muss auf Spielfeld sein.".to_owned());
    }
    if nr_cops > MAX_COPS {
        return Err(format!(
            "Rechnung kann für höchstens {MAX_COPS} Cops durchgeführt werden."
        ));
    }

    manager.update("liste Polizeipositionen")?;
    let cop_moves = CopConfigurations::new(&edges, &sym, nr_cops, manager)?;

    manager.update("reserviere Speicher für Räuberstrategiefunktion")?;
    let Some(mut f) = SafeRobberPositions::new(edges.nr_vertices(), &cop_moves) else {
        return Err("Zu wenig Speicherplatz (Räuberstrategiefunktion zu groß)".to_owned());
    };

    manager.update("reserviere Speicher für Queue")?;
    let Some(mut queue) = RobberStratQueue::new(&cop_moves) else {
        return Err("Zu wenig Speicherplatz (initiale Queue zu lang)".to_owned());
    };

    let max_degree_less_than_nr_cops = edges.max_degree() < nr_cops;

    for (i, index) in izip!(0.., cop_moves.all_positions()) {
        if i % 4096 == 0 {
            let percent = 100.0 * (i as f32) / (cop_moves.nr_configurations() as f32);
            let msg = format!("initialisiere Räuberstrategiefunktion: {percent:.2}%");
            manager.update(msg)?;
        }

        // test round trip
        debug_assert!({
            let mut unpacked = cop_moves.eager_unpack(index);
            let (_, packed) = cop_moves.pack(&sym, &mut unpacked);
            debug_assert_eq!(index, packed);
            true
        });

        //line 2
        let robber_range = f.robber_indices_at(index);
        for cop_pos in cop_moves.unpack(index) {
            f.mark_robber_at(robber_range.at(cop_pos), false);
            for n in edges.neighbors_of(cop_pos) {
                f.mark_robber_at(robber_range.at(n), false);
            }
        }

        // special case treatment: if nr cops is larger than nr neighbors of vertex,
        // vertex is not marked cop win and all neighbors are marked, then the robber has lost.
        // This is, because at least one cop is uninvolved in the current stalemate
        // and can come and capture the robber.
        // note: this is REALLY conservative and assumes no cop can guard two of
        // the robber's neighbors at once. only in this case however will this procedure
        // be of advantage.
        if max_degree_less_than_nr_cops {
            for (v, mut neighs) in izip!(0.., edges.neighbors()) {
                let safe_at = |v| f.robber_safe_at(robber_range.at(v));
                if safe_at(v) && neighs.all(|n| !safe_at(n)) {
                    f.mark_robber_at(robber_range.at(v), false);
                }
            }
        }
    }

    let nr_map_vertices = edges.nr_vertices();
    //if the current game state has cop configuration `curr`, this contains all robber positions that
    //where safe last game state, given that the cops move to `curr`.
    let mut safe_should_cops_move_to_curr = vec![false; nr_map_vertices];
    //intersection of `safe_should_cops_move_to_curr` and the vertices previously marked as safe for gamestate bevor curr
    let mut f_temp = vec![false; nr_map_vertices];

    //lines 4 + 5
    let mut time_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            let nr_safe = f.robber_safe_when(curr_cop_positions).count_ones();
            manager.update(format!(
                "berechne Räuberstrategie:\n{:.2}% in Queue ({}), Runde {}, {:.2}% sicher",
                100.0 * (queue.len() as f32) / (cop_moves.nr_configurations() as f32),
                queue.len(),
                queue.rounds_complete(),
                100.0 * (nr_safe as f32) / (nr_map_vertices as f32),
            ))?;
            time_until_log_refresh = 10_000;
        }

        //line 6
        for (safe_before, safe_now) in izip!(
            &mut safe_should_cops_move_to_curr,
            f.robber_safe_when(curr_cop_positions)
        ) {
            *safe_before = *safe_now;
        }
        for (v, safe_now) in izip!(0.., f.robber_safe_when(curr_cop_positions)) {
            if *safe_now {
                for n in edges.neighbors_of(v) {
                    safe_should_cops_move_to_curr[n] = true;
                }
            }
        }

        //line 7
        for (neigh_rotations, rotated_neigh_cop_positions) in
            rules.cop_moves_from(&cop_moves, &edges, &sym, curr_cop_positions)
        {
            //if it only takes a single move to go from curr_cop_positions to rotated_neigh_cop_positions, so
            //should the other direction.
            debug_assert!(
                rules
                    .cop_moves_from(&cop_moves, &edges, &sym, rotated_neigh_cop_positions)
                    .any(|(_, pos)| pos == curr_cop_positions)
            );
            debug_assert!(!neigh_rotations.is_empty());

            let mut f_neighbor_changed_some_rotation = false;
            for neigh_rotate in neigh_rotations {
                //line 8
                let mut f_neighbor_changed_this_rotation = false;
                //guarantee that rotations do the right thing
                debug_assert!(
                    !R::IS_LAZY || {
                        let mut unpacked_curr = cop_moves.eager_unpack(curr_cop_positions);
                        //all positions of current cop configuration.
                        let mut moved_cop_pos = usize::MAX;
                        for rotated_neigh_pos in cop_moves.unpack(rotated_neigh_cop_positions) {
                            let unrotated = neigh_rotate.apply_backward(rotated_neigh_pos);
                            let rerotated = neigh_rotate.apply_forward(unrotated);
                            debug_assert_eq!(rerotated, rotated_neigh_pos);
                            //if the neighbor configuration's position is found in curr configuration,
                            //remove entry from curr configuration.
                            if let Some(i) = unpacked_curr.iter().position(|&c| c == unrotated) {
                                unpacked_curr[i] = usize::MAX;
                            } else {
                                //position is not found -> that must have been the cop that moved.
                                //remember for later.
                                debug_assert!(moved_cop_pos == usize::MAX);
                                moved_cop_pos = unrotated;
                            }
                        }
                        //now only the curr position of the cop which just moved there should be left in unpacked_curr.
                        debug_assert!(moved_cop_pos != usize::MAX);
                        unpacked_curr.iter().all(|&c| {
                            c == usize::MAX || edges.neighbors_of(c).contains(&moved_cop_pos)
                        })
                    }
                );

                for (v, marked_safe_for_neigh) in izip!(
                    neigh_rotate.backward(),
                    f.robber_safe_when(rotated_neigh_cop_positions)
                ) {
                    f_neighbor_changed_this_rotation |=
                        *marked_safe_for_neigh && !safe_should_cops_move_to_curr[v];
                    f_temp[v] = *marked_safe_for_neigh && safe_should_cops_move_to_curr[v];
                }

                //line 9
                if f_neighbor_changed_this_rotation {
                    f_neighbor_changed_some_rotation = true;
                    //line 10
                    let range = f.robber_indices_at(rotated_neigh_cop_positions);
                    for (v, &val) in izip!(neigh_rotate.forward(), &f_temp) {
                        f.mark_robber_at(range.at(v), val);
                    }
                }
            }
            //if we changed the robber win function for some rotation, it is irrelevant which rotation caused the change.
            //we thus only enqueue each neighbor up to once, not up to once per our rotation.
            if f_neighbor_changed_some_rotation {
                //line 11
                queue.push(rotated_neigh_cop_positions);
            }

            //lines 13 + 14 + 15
            if f.robber_safe_when(rotated_neigh_cop_positions).not_any() {
                //it is redundant to return `f` (or `cop_moves`), because no vertex would be marked anyway.
                return Ok(Outcome::CopsWin);
            }
        }
    }

    let result = RobberWinData {
        symmetry: sym.into_enum().into(),
        safe: f,
        cop_moves,
    };
    debug_assert!(verify_continuity_robber(rules, &result, &edges, manager).is_ok());

    Ok(Outcome::RobberWins(result))
}

/// [`data.safe`] is called continuous, if it is Lipschitz-continuous with constant 1,
/// where the distance measurement is the Haussdorff distance induced by the graph distance function.
///
/// Said differently: for every move the police can take, if the robber's vertex is currently marked safe,
/// the robber must be neighboring a safe vertex next move.
/// Further: there must exist a safe vertex for every police arrangement.
pub fn verify_continuity_robber(
    rules: impl Rules,
    data: &RobberWinData,
    edges: &EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<(), String> {
    // logging things
    let nr_configs = data.cop_moves.nr_configurations();
    let mut i_config = 0;
    let mut time_until_log_refresh = 1;
    let log_refresh_interval = (nr_configs / 10_000).clamp(1000, 100_000);

    let nr_map_vertices = data.cop_moves.nr_map_vertices();
    let mut safe_should_cops_move_to_curr = vec![false; nr_map_vertices];
    for cops_index in data.cop_moves.all_positions() {
        let cops = data.cop_moves.eager_unpack(cops_index);
        // logging things
        time_until_log_refresh -= 1;
        i_config += 1;
        if time_until_log_refresh == 0 {
            let done_percent = (i_config as f64) / (nr_configs as f64) * 100.0;
            manager.update(format!("verifiziere Kontinuität: {done_percent:.2}%"))?;
            time_until_log_refresh = log_refresh_interval;
        }

        let mut some_safe = false;
        safe_should_cops_move_to_curr.fill(false);
        for (v, neighs, safe) in izip!(
            0..,
            edges.neighbors(),
            data.safe.robber_safe_when(cops_index)
        ) {
            if *safe {
                some_safe = true;
                safe_should_cops_move_to_curr[v] = true;
                for n in neighs {
                    safe_should_cops_move_to_curr[n] = true;
                }
            }
        }

        // verify some safe vertex exists for every cop arrangement
        if !some_safe {
            return Err(format!(
                "Räuber ist nicht sicher, wenn Cops {cops:?} besetzen.",
            ));
        }

        // verify for every possible previous cop arrangement,
        // a vertex marked safe there still has a safe neighbor here (safe neighbors precomputed).
        for cops_neighs in rules.raw_cop_moves_from(edges, cops) {
            for (v, &safe_before_curr, safe_last_move) in izip!(
                0..,
                &safe_should_cops_move_to_curr,
                data.safe_vertices(cops_neighs)
            ) {
                if safe_last_move && !safe_before_curr {
                    return Err(format!(
                        "Räuberstrategie nicht kontinuierlich an Knoten {v},\
                        wenn Cops von {cops_neighs:?} zu {cops:?} ziehen."
                    ));
                }
            }
        }
    }

    Ok(())
}

pub type UTime = u8;

/// for each cop configuration in [`CopConfigurations`] this struct stores for each map vertex,
/// how many more moves the police need at most to catch the robber.
#[derive(Serialize, Deserialize)]
pub struct TimeToWin {
    time: BTreeMap<usize, Vec<UTime>>,
    nr_map_vertices: usize,
}

impl TimeToWin {
    /// returns [`Self`] if enough memory is available
    fn new(nr_map_vertices: usize, cop_moves: &CopConfigurations) -> Option<Self> {
        let mut time = BTreeMap::new();
        for (&fst_index, indices) in &cop_moves.configurations {
            let nr_entries = indices.len().checked_mul(nr_map_vertices)?;

            let mut data = Vec::new();
            data.try_reserve(nr_entries).ok()?;
            data.resize(nr_entries, UTime::MAX);
            let old = time.insert(fst_index, data);
            debug_assert!(old.is_none());
        }

        Some(Self { time, nr_map_vertices })
    }

    pub fn nr_map_vertices(&self) -> usize {
        self.nr_map_vertices
    }

    /// returns current number for each vertex in graph given cops placed like `index`
    pub fn nr_moves_left(&self, index: CompactCopsIndex) -> &[UTime] {
        let start = index.rest_index * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        &self.time.get(&index.fst_index).unwrap()[start..stop]
    }

    /// returns current number for each vertex in graph given cops placed like `index`
    pub fn nr_moves_left_mut(&mut self, index: CompactCopsIndex) -> &mut [UTime] {
        let start = index.rest_index * self.nr_map_vertices();
        let stop = start + self.nr_map_vertices();
        &mut self.time.get_mut(&index.fst_index).unwrap()[start..stop]
    }
}

#[derive(Serialize, Deserialize)]
pub struct CopStrategy {
    #[serde(deserialize_with = "deserialize_explicit")]
    pub symmetry: ExplicitClasses,
    pub time_to_win: TimeToWin,
    pub cop_moves: CopConfigurations,
    pub max_moves: usize,
    /// selection of at most 20 cop configurations
    /// where there is a vertex which let's the robber loose in maximal many moves
    #[serde(skip)] // marked skip to stay backwards compatible.
    pub extreme_positions: Vec<Vec<usize>>,
    pub cops_win: bool,
}

impl CopStrategy {
    /// values not stored because that would break the format are computed here
    pub fn compute_serde_skipped(&mut self) {
        if self.extreme_positions.is_empty() {
            let max_moves = self.max_moves as UTime;
            for index in self.cop_moves.all_positions() {
                if self.time_to_win.nr_moves_left(index).contains(&max_moves) {
                    self.extreme_positions
                        .push(self.cop_moves.unpack(index).collect_vec());
                    if self.extreme_positions.len() == 20 {
                        break;
                    }
                }
            }
        }
    }

    /// the equivalent of [`RobberWinData::safe_vertices`]
    pub fn times_for(&self, mut cops: RawCops) -> impl ExactSizeIterator<Item = UTime> {
        let (autos, cop_positions) = self.cop_moves.pack(&self.symmetry, &mut cops);
        let time_left = self.time_to_win.nr_moves_left(cop_positions);
        autos[0].forward().map(|v| time_left[v])
    }
}

pub fn compute_cop_strategy<S, R>(
    rules: R,
    nr_cops: usize,
    edges: EdgeList,
    sym: S,
    manager: &thread_manager::LocalManager,
) -> Result<CopStrategy, String>
where
    S: SymmetryGroup + Serialize,
    R: Rules,
{
    if nr_cops == 0 {
        return Err("Mindestens ein Cop muss auf Spielfeld sein.".to_owned());
    }
    if nr_cops > MAX_COPS {
        return Err(format!(
            "Rechnung kann für höchstens {MAX_COPS} Cops durchgeführt werden."
        ));
    }

    manager.update("liste Polizeipositionen")?;
    let cop_moves = CopConfigurations::new(&edges, &sym, nr_cops, manager)?;

    manager.update("reserviere Speicher für Cop Startegie")?;
    let Some(mut f) = TimeToWin::new(edges.nr_vertices(), &cop_moves) else {
        return Err("Zu wenig Speicherplatz (Copstrat zu groß)".to_owned());
    };

    manager.update("reserviere Speicher für Queue")?;
    let Some(mut queue) = CopStratQueue::new(&cop_moves) else {
        return Err("Zu wenig Speicherplatz (initiale Queue zu lang)".to_owned());
    };

    manager.update("initialisiere Queue")?;
    for (i, index) in izip!(0.., cop_moves.all_positions()) {
        if i % 4096 == 0 {
            manager.recieve()?;
        }

        let robber_positions = f.nr_moves_left_mut(index);
        for cop_pos in cop_moves.unpack(index) {
            robber_positions[cop_pos] = 0;
            for n in edges.neighbors_of(cop_pos) {
                robber_positions[n] = 0;
            }
        }
    }

    let nr_map_vertices = edges.nr_vertices();
    //if the current game state has cop configuration `curr`, this contains the time for cops to win
    //for each possible robber position, given that the cops move to `curr`.
    let mut times_should_cops_move_to_curr = vec![UTime::MAX; nr_map_vertices];

    let mut time_until_log_refresh: usize = 1;
    while let Some(curr_cop_positions) = queue.pop() {
        time_until_log_refresh -= 1;
        if time_until_log_refresh == 0 {
            manager.update(format!(
                "berechne Copstrategie:\n{:.2}% in Queue ({}), max {}",
                100.0 * (queue.len() as f32) / (cop_moves.nr_configurations() as f32),
                queue.len(),
                queue.curr_max()
            ))?;
            time_until_log_refresh = 2000;
        }

        let curr_times = f.nr_moves_left(curr_cop_positions);
        let mut curr_is_at_max = false;
        for (v, neighs) in izip!(0.., edges.neighbors()) {
            let max_neigh_time = neighs.fold(curr_times[v], |acc, n| acc.max(curr_times[n]));
            const OVERFLOW_NEIGH_TIME: UTime = UTime::MAX - 1;
            if max_neigh_time == OVERFLOW_NEIGH_TIME {
                return Err(format!(
                    "Cops brauchen mehr Züge als in {} passen",
                    std::any::type_name::<UTime>()
                ));
            }
            let new_time = max_neigh_time.saturating_add(1);
            times_should_cops_move_to_curr[v] = if new_time == queue.curr_max() {
                curr_is_at_max = true;
                UTime::MAX
            } else {
                debug_assert!(new_time < queue.curr_max() || new_time == UTime::MAX);
                new_time
            };
        }
        if curr_is_at_max {
            queue.mark_as_at_max(curr_cop_positions);
        }

        for (neigh_rotations, rotated_neigh_cop_positions) in
            rules.cop_moves_from(&cop_moves, &edges, &sym, curr_cop_positions)
        {
            let mut f_neighbor_changed = false;
            for neigh_rotate in neigh_rotations {
                for (v, neigh_time) in izip!(
                    neigh_rotate.backward(),
                    f.nr_moves_left_mut(rotated_neigh_cop_positions)
                ) {
                    let this_time = times_should_cops_move_to_curr[v];
                    if *neigh_time > this_time {
                        debug_assert!(this_time < queue.curr_max());
                        f_neighbor_changed = true;
                        *neigh_time = this_time;
                    }
                }
            }
            if f_neighbor_changed {
                queue.push(rotated_neigh_cop_positions);
            }
        }
    }

    manager.update("berechne Fun Facts")?;
    let mut max_moves = 0;
    let mut cops_win = true;
    for vals in f.time.values() {
        for &val in vals {
            if val == UTime::MAX {
                cops_win = false;
            } else if val > max_moves {
                max_moves = val;
            }
        }
    }

    let mut res = CopStrategy {
        symmetry: sym.into_enum().into(),
        time_to_win: f,
        cop_moves,
        max_moves: max_moves as usize,
        extreme_positions: Vec::new(),
        cops_win,
    };
    res.compute_serde_skipped();

    debug_assert!(verify_continuity_cops(rules, &res, &edges, manager).is_ok());

    Ok(res)
}

/// similar to [`verify_continuity_robber`], but this continuity has (to my knowledge)
/// not as simple a name as "Lippschitz-continuous with respect to the Haussdorff norm".
///
/// Note: in [`verify_continuity_robber`], one has to check for every game state that is
/// a winning position for the robber, wether the robber can reach a new winning game state.
/// This is equivalent to _for each cop move, check if the robber can react_.
///
/// Here, the situation is slightly different. We don't want to keep the game running forever
/// (that is a robber win), thus we need to quantify what _winning_ actually means:
/// _for each robber move, there must be a cop move that decreases the number of rounds left_.
fn verify_continuity_cops(
    rules: impl Rules,
    data: &CopStrategy,
    edges: &EdgeList,
    manager: &thread_manager::LocalManager,
) -> Result<(), String> {
    // logging things
    let nr_configs = data.cop_moves.nr_configurations();
    let mut i_config = 0;
    let mut time_until_log_refresh = 1;
    let log_refresh_interval = (nr_configs / 10_000).clamp(1000, 100_000);

    let nr_map_vertices = data.cop_moves.nr_map_vertices();
    let mut max_rounds_left_cops_to_neigh = vec![UTime::MAX; nr_map_vertices];
    let mut robber_rounds_decrease = vec![false; nr_map_vertices];
    for cops_index in data.cop_moves.all_positions() {
        let cops = data.cop_moves.eager_unpack(cops_index);
        // logging things
        time_until_log_refresh -= 1;
        i_config += 1;
        if time_until_log_refresh == 0 {
            let done_percent = (i_config as f64) / (nr_configs as f64) * 100.0;
            manager.update(format!("verifiziere Kontinuität: {done_percent:.2}%"))?;
            time_until_log_refresh = log_refresh_interval;
        }

        robber_rounds_decrease.fill(false);
        for cops_neigh in rules.raw_cop_moves_from(edges, cops) {
            max_rounds_left_cops_to_neigh.fill(0);
            for (v, robber_neighs, cop_neigh_time) in
                izip!(0.., edges.neighbors(), data.times_for(cops_neigh))
            {
                let set_max_eq = |a: &mut UTime, b: UTime| *a = UTime::max(*a, b);
                set_max_eq(&mut max_rounds_left_cops_to_neigh[v], cop_neigh_time);
                for n in robber_neighs {
                    set_max_eq(&mut max_rounds_left_cops_to_neigh[n], cop_neigh_time);
                }
            }

            for (v, &robber_neigh_max, curr_rounds_left) in
                izip!(0.., &max_rounds_left_cops_to_neigh, data.times_for(cops))
            {
                // assuming police state `cops` occurs before police state `cops_neigh`,
                // the found number of moves left in the current position is not optimal.
                if curr_rounds_left.saturating_sub(1) > robber_neigh_max {
                    return Err(format!(
                        "Copstrategie nicht optimal an Knoten {v},\
                        wenn Cops von {cops_neigh:?} zu {cops:?} ziehen."
                    ));
                }
                // assuming police state `cops` occurs before police state `cops_neigh`,
                // the moves to `cops_neigh` is actually optimal for the police.
                if curr_rounds_left.saturating_sub(1) == robber_neigh_max {
                    robber_rounds_decrease[v] = true;
                }
            }
        }

        if data.cops_win
            && let Some(v) = robber_rounds_decrease.iter().position(|&x| !x)
        {
            return Err(format!(
                "Copstrategie hat keinen Zug für Räuber auf {v}, Cops auf {cops:?}."
            ));
        }
    }

    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::graph::{Embedding3D, Shape};

    /// produces same result as [`cop_number`], but uses [`compute_cop_strategy`]
    /// instead of [`compute_safe_robber_positions`] to get there.
    fn cop_number_cop_strat(rules: impl Rules + Clone, g: &Embedding3D) -> Option<usize> {
        let mut nr = 1;
        let sym = g.sym_group().to_explicit();
        let (_, manager) = thread_manager::build_managers();
        loop {
            let rs = rules.clone();
            let es = g.edges().clone();
            let cops_strat = compute_cop_strategy(rs, nr, es, sym.clone(), &manager).ok()?;
            if cops_strat.cops_win {
                return Some(nr);
            }
            nr += 1;
        }
    }

    fn cop_number(rules: impl Rules + Clone, g: Embedding3D) -> Option<usize> {
        let mut nr = 1;
        let sym = g.sym_group().to_explicit();
        let (_, manager) = thread_manager::build_managers();
        loop {
            let robber_strat = compute_safe_robber_positions(
                rules.clone(),
                nr,
                g.edges().clone(),
                sym.clone(),
                &manager,
            )
            .ok()?;
            if let Outcome::CopsWin = robber_strat {
                let p = energy_bf::EnergyParams::STANDARD_GAME;
                assert_eq!(energy_bf::cop_number(rules.clone(), p, &g), Some(nr));
                assert_eq!(cop_number_cop_strat(rules.clone(), &g), Some(nr));
                return Some(nr);
            }
            nr += 1;
        }
    }

    #[test]
    fn test_cop_numbers() {
        let eager = GeneralEagerCops(MAX_COPS as u32);
        let c_e = |s, res| cop_number(eager, Embedding3D::new_map_from(&s, res));
        let c_l = |s, res| cop_number(LazyCops, Embedding3D::new_map_from(&s, res));

        assert_eq!(c_l(Shape::Cube, 0), Some(2));
        assert_eq!(c_l(Shape::Dodecahedron, 0), Some(3));
        assert_eq!(c_e(Shape::Dodecahedron, 0), Some(3));
        assert_eq!(c_l(Shape::Icosahedron, 1), Some(3));
        assert_eq!(c_e(Shape::Icosahedron, 1), Some(2));
        assert_eq!(c_l(Shape::TriangTorus, 3), Some(2));
        assert_eq!(c_l(Shape::TriangTorus, 4), Some(3));
        assert_eq!(c_e(Shape::TriangTorus, 4), Some(2));
        assert_eq!(c_l(Shape::SquareGrid, 5), Some(2));
    }
}
