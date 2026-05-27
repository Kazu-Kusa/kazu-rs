use bdmc_rs::controller::CloseLoopController;
use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};

use crate::state::MovingState;
use crate::transition::{BreakerResult, MovingTransition};

mod graph;

/// Main Botix struct for managing states and transitions.
///
/// Stores states and transitions in registries keyed by their IDs.
/// The graph is built from a flat list of transitions via `build_full()`,
/// which validates structure and computes adjacency maps.
pub struct Botix {
    /// The bot's controller.
    controller: CloseLoopController,
    /// State registry: state_id → MovingState.
    states: HashMap<usize, MovingState>,
    /// Transition registry: transition_id → MovingTransition.
    transitions: HashMap<usize, MovingTransition>,
    /// Forward adjacency: state_id → transition_id (unique per state).
    forward_edge: HashMap<usize, usize>,
    /// Reverse adjacency: state_id ← [transition_ids] that target it.
    incoming_edges: HashMap<usize, Vec<usize>>,
    /// The unique start state ID.
    start_state: usize,
}

impl Botix {
    /// Build a Botix graph from controller, states, and transitions.
    ///
    /// Validates:
    /// - Each state appears in at most one transition's `from_states`.
    /// - Exactly one start state (indegree 0).
    /// - All states are reachable from the start state.
    /// - All referenced state IDs exist in the state registry.
    pub fn build_full(
        controller: CloseLoopController,
        states: Vec<MovingState>,
        transitions: Vec<MovingTransition>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut state_map: HashMap<usize, MovingState> = HashMap::new();
        let mut forward_edge: HashMap<usize, usize> = HashMap::new();
        let mut incoming_edges: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut trans_map: HashMap<usize, MovingTransition> = HashMap::new();
        let mut state_forward_count: HashMap<usize, usize> = HashMap::new();

        // Index states.
        for state in states {
            let sid = state.id();
            if state_map.contains_key(&sid) {
                return Err(format!("Duplicate state ID: {}", sid).into());
            }
            incoming_edges.entry(sid).or_default();
            state_map.insert(sid, state);
        }

        // First pass: read adjacency info from transitions (by reference).
        for t in &transitions {
            let tid = t.id();
            if trans_map.contains_key(&tid) {
                return Err(format!("Duplicate transition ID: {}", tid).into());
            }

            // Validate all referenced state IDs exist.
            for &from_id in &t.from_states {
                if !state_map.contains_key(&from_id) {
                    return Err(format!(
                        "Transition {} references unknown from_state {}",
                        tid, from_id
                    )
                    .into());
                }
                *state_forward_count.entry(from_id).or_insert(0) += 1;
                if state_forward_count[&from_id] > 1 {
                    return Err(format!(
                        "State {} connects to multiple forward transitions. \
                         Branching must be inside a single MovingTransition.",
                        from_id
                    )
                    .into());
                }
                forward_edge.insert(from_id, tid);
            }

            for &to_id in t.to_states.values() {
                if !state_map.contains_key(&to_id) {
                    return Err(format!(
                        "Transition {} references unknown to_state {}",
                        tid, to_id
                    )
                    .into());
                }
                incoming_edges.entry(to_id).or_default().push(tid);
            }
        }

        // Determine start state(s).
        let start_candidates: Vec<usize> = state_map
            .keys()
            .filter(|id| incoming_edges.get(id).is_none_or(|v| v.is_empty()))
            .copied()
            .collect();

        if start_candidates.len() != 1 {
            return Err(format!(
                "Must have exactly one start state (indegree 0), found {}: {:?}",
                start_candidates.len(),
                start_candidates
            )
            .into());
        }

        let start_state = start_candidates[0];

        // Second pass: move transitions into the registry.
        for t in transitions {
            trans_map.insert(t.id(), t);
        }

        // Verify accessibility: all states reachable from start.
        let reachable =
            Self::compute_reachable_set(&state_map, &forward_edge, &trans_map, start_state);
        let all_ids: HashSet<usize> = state_map.keys().copied().collect();
        let unreachable: Vec<usize> = all_ids.difference(&reachable).copied().collect();
        if !unreachable.is_empty() {
            return Err(format!(
                "States not reachable from start state {}: {:?}",
                start_state, unreachable
            )
            .into());
        }

        Ok(Self {
            controller,
            states: state_map,
            transitions: trans_map,
            forward_edge,
            incoming_edges,
            start_state,
        })
    }

    /// Execute the state machine directly — no JIT, no codegen.
    ///
    /// Walks the graph starting from `start_state`, calling controller methods,
    /// hooks, and evaluating breakers at each step. Loops until an end state
    /// is reached (no forward edge).
    pub fn execute(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let mut current = self.start_state;

        loop {
            let outcome = self.execute_one_state(current)?;

            match outcome {
                TransitionOutcome::NextState(next) => current = next,
                TransitionOutcome::End => break,
            }
        }

        Ok(())
    }

    /// Execute a single state and its forward transition.
    /// Returns the outcome (next state or end).
    fn execute_one_state(
        &mut self,
        state_id: usize,
    ) -> Result<TransitionOutcome, Box<dyn std::error::Error>> {
        // We need to access both states and transitions. Since execute()
        // takes &mut self, we can't borrow self.states and self.transitions
        // simultaneously. We use indices to avoid the borrow conflict.

        // Call before_entering hooks.
        if let Some(state) = self.states.get(&state_id) {
            for hook in state.before_entering() {
                hook();
            }
        }

        // Resolve and set speeds.
        let speeds = {
            let state = self
                .states
                .get(&state_id)
                .ok_or_else(|| format!("State {} not found in registry", state_id))?;
            state.resolve_speeds(self.controller.context())
        };
        let speeds_f64: Vec<f64> = speeds.iter().map(|&s| s as f64).collect();
        self.controller.set_motors_speed(&speeds_f64)?;

        // Call after_exiting hooks.
        if let Some(state) = self.states.get(&state_id) {
            for hook in state.after_exiting() {
                hook();
            }
        }

        // Determine next state.
        match self.forward_edge.get(&state_id) {
            None => Ok(TransitionOutcome::End),
            Some(&trans_id) => {
                // Clone the necessary info to avoid borrow issues with breaker closures.
                let duration;
                let check_interval;
                let to_states: HashMap<BreakerResult, usize>;
                let has_breaker: bool;

                {
                    let trans = self
                        .transitions
                        .get(&trans_id)
                        .ok_or_else(|| format!("Transition {} not found in registry", trans_id))?;
                    duration = trans.duration;
                    check_interval = trans.check_interval;
                    to_states = trans.to_states.clone();
                    has_breaker = trans.breaker.is_some();
                }

                if !has_breaker {
                    // Simple delay.
                    std::thread::sleep(Duration::from_secs_f64(duration));
                    let next = to_states
                        .values()
                        .next()
                        .copied()
                        .ok_or_else(|| format!("Transition {} has no to_states", trans_id))?;
                    Ok(TransitionOutcome::NextState(next))
                } else {
                    // Poll breaker until duration expires or non-placeholder result.
                    let start = Instant::now();
                    let max_dur = Duration::from_secs_f64(duration);
                    let check_dur = Duration::from_secs_f64(check_interval.max(0.001));

                    // We call the breaker directly through the stored reference.
                    let _breaker_fn = self
                        .transitions
                        .get(&trans_id)
                        .and_then(|t| t.breaker.as_ref())
                        .ok_or_else(|| format!("Breaker not found for transition {}", trans_id))?;

                    // We need to call the breaker, but we can't hold a reference
                    // while also needing to access other fields. Since breaker is
                    // Fn() (not FnMut), we can call it through the reference.
                    // But we can't hold the immutable borrow of self.transitions
                    // across the loop while also calling the breaker.

                    // Strategy: extract the breaker result through repeated calls.
                    // Since the breaker is behind a shared reference in the HashMap,
                    // and we need to call it multiple times, we use a different approach:
                    // we call the breaker, drop the borrow, sleep, repeat.

                    let last_result;
                    loop {
                        let result = {
                            let trans = self.transitions.get(&trans_id).unwrap();
                            trans.breaker.as_ref().unwrap()()
                        };
                        if result != BreakerResult::Placeholder {
                            last_result = result;
                            break;
                        }
                        if start.elapsed() >= max_dur {
                            last_result = result;
                            break;
                        }
                        let remaining = max_dur.saturating_sub(start.elapsed());
                        std::thread::sleep(check_dur.min(remaining));
                    }

                    let next = to_states.get(&last_result).copied().ok_or_else(|| {
                        format!(
                            "Transition {}: no matching to_state for breaker result {:?}",
                            trans_id, last_result
                        )
                    })?;
                    Ok(TransitionOutcome::NextState(next))
                }
            }
        }
    }

    /// Wait for `duration` seconds, polling `breaker` at `check_interval`.
    /// Returns the first non-Placeholder breaker result, or the last result
    /// if the duration elapses without a break.
    pub fn wait_with_breaker(
        duration_sec: f64,
        check_interval: f64,
        breaker: &(dyn Fn() -> BreakerResult + Send + Sync),
    ) -> BreakerResult {
        let start = Instant::now();
        let max_duration = Duration::from_secs_f64(duration_sec);
        let check_dur = Duration::from_secs_f64(check_interval.max(0.001));

        // Initial check.
        let mut last_result = breaker();
        if last_result != BreakerResult::Placeholder {
            return last_result;
        }

        while start.elapsed() < max_duration {
            let remaining = max_duration.saturating_sub(start.elapsed());
            std::thread::sleep(check_dur.min(remaining));
            last_result = breaker();
            if last_result != BreakerResult::Placeholder {
                return last_result;
            }
        }

        last_result
    }

    /// Get a reference to the controller.
    pub fn controller(&self) -> &CloseLoopController {
        &self.controller
    }

    /// Get a mutable reference to the controller.
    pub fn controller_mut(&mut self) -> &mut CloseLoopController {
        &mut self.controller
    }

    /// Get the start state ID.
    pub fn start_state_id(&self) -> usize {
        self.start_state
    }

    /// Get a reference to a state by ID.
    pub fn get_state(&self, id: usize) -> Option<&MovingState> {
        self.states.get(&id)
    }

    /// Get a reference to a transition by ID.
    pub fn get_transition(&self, id: usize) -> Option<&MovingTransition> {
        self.transitions.get(&id)
    }

    /// Get total number of states.
    pub fn state_count(&self) -> usize {
        self.states.len()
    }

    /// Get total number of transitions.
    pub fn transition_count(&self) -> usize {
        self.transitions.len()
    }
}

enum TransitionOutcome {
    NextState(usize),
    End,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::MovingState;
    use crate::transition::{BreakerResult, MovingTransition};

    fn make_linear_chain() -> (Vec<MovingState>, Vec<MovingTransition>) {
        let s0 = MovingState::straight(100);
        let s1 = MovingState::straight(200);
        let s2 = MovingState::halt();

        let s0_id = s0.id();
        let s1_id = s1.id();
        let s2_id = s2.id();

        let t0 = MovingTransition::new(0.5)
            .unwrap()
            .with_from_state(s0_id)
            .with_single_to_state(s1_id);

        let t1 = MovingTransition::new(0.3)
            .unwrap()
            .with_from_state(s1_id)
            .with_single_to_state(s2_id);

        (vec![s0, s1, s2], vec![t0, t1])
    }

    #[test]
    fn test_build_linear_chain() {
        let (states, transitions) = make_linear_chain();
        let controller = CloseLoopController::new(None, None, None, None).unwrap();
        let botix = Botix::build_full(controller, states, transitions);
        assert!(botix.is_ok(), "Build failed: {:?}", botix.err());
        let botix = botix.unwrap();
        assert_eq!(botix.start_states().len(), 1);
        assert_eq!(botix.end_states().len(), 1);
        assert_eq!(botix.state_count(), 3);
        assert_eq!(botix.transition_count(), 2);
    }

    #[test]
    fn test_build_duplicate_from_state() {
        let s0 = MovingState::straight(100);
        let s1 = MovingState::halt();
        let s2 = MovingState::straight(200);

        let s0_id = s0.id();
        let s1_id = s1.id();
        let s2_id = s2.id();

        let t0 = MovingTransition::new(0.5)
            .unwrap()
            .with_from_state(s0_id)
            .with_single_to_state(s1_id);

        let t1 = MovingTransition::new(0.5)
            .unwrap()
            .with_from_state(s0_id)
            .with_single_to_state(s2_id);

        let controller = CloseLoopController::new(None, None, None, None).unwrap();
        let result = Botix::build_full(controller, vec![s0, s1, s2], vec![t0, t1]);
        assert!(result.is_err());
    }

    #[test]
    fn test_build_multiple_starts() {
        let s0 = MovingState::straight(100);
        let s1 = MovingState::straight(200);

        let controller = CloseLoopController::new(None, None, None, None).unwrap();
        let result = Botix::build_full(controller, vec![s0, s1], vec![]);
        assert!(result.is_err());
    }

    #[test]
    fn test_wait_with_breaker_immediate() {
        let breaker = || BreakerResult::Bool(true);
        let result = Botix::wait_with_breaker(1.0, 0.01, &breaker);
        assert_eq!(result, BreakerResult::Bool(true));
    }

    #[test]
    fn test_wait_with_breaker_timeout() {
        let breaker = || BreakerResult::Placeholder;
        let result = Botix::wait_with_breaker(0.05, 0.01, &breaker);
        assert_eq!(result, BreakerResult::Placeholder);
    }

    #[test]
    fn test_find_loops_no_loop() {
        let (states, transitions) = make_linear_chain();
        let controller = CloseLoopController::new(None, None, None, None).unwrap();
        let botix = Botix::build_full(controller, states, transitions).unwrap();
        let loops = botix.find_loops();
        assert!(loops.is_empty(), "Expected no loops, got {:?}", loops);
    }

    #[test]
    fn test_loop_rejected() {
        let s0 = MovingState::straight(100);
        let s1 = MovingState::straight(200);
        let s2 = MovingState::straight(300);

        let s0_id = s0.id();
        let s1_id = s1.id();
        let s2_id = s2.id();

        let t0 = MovingTransition::new(0.5)
            .unwrap()
            .with_from_state(s0_id)
            .with_to_state(BreakerResult::Bool(true), s1_id)
            .with_to_state(BreakerResult::Bool(false), s2_id);

        // s2 points back to s0 creating a loop — rejected.
        let t1 = MovingTransition::new(0.3)
            .unwrap()
            .with_from_state(s2_id)
            .with_single_to_state(s0_id);

        let controller = CloseLoopController::new(None, None, None, None).unwrap();
        // Build should fail because s0 has incoming edge from t1.
        let result = Botix::build_full(controller, vec![s0, s1, s2], vec![t0, t1]);
        assert!(result.is_err());
    }
}
