use crate::state::MovingState;
use crate::transition::{BreakerResult, MovingTransition};

/// Builder for constructing state chains programmatically.
///
/// Maintains a flip-flop between adding states and transitions,
/// auto-connecting them via `from_states`/`to_states`.
pub struct MovingChainComposer {
    states: Vec<MovingState>,
    transitions: Vec<MovingTransition>,
    /// When true, the next `add` expects a MovingState; when false, a MovingTransition.
    next_is_state: bool,
}

impl MovingChainComposer {
    /// Create a new empty composer.
    pub fn new() -> Self {
        Self {
            states: Vec::new(),
            transitions: Vec::new(),
            next_is_state: true,
        }
    }

    /// Get the last state added, if any.
    pub fn last_state(&self) -> Option<&MovingState> {
        self.states.last()
    }

    /// Get the last transition added, if any.
    pub fn last_transition(&self) -> Option<&MovingTransition> {
        self.transitions.last()
    }

    /// Add a state. Panics if a transition was expected.
    pub fn add_state(&mut self, state: MovingState) -> &mut Self {
        assert!(
            self.next_is_state,
            "Expected MovingState, but next need is MovingTransition"
        );

        let state_id = state.id();
        if let Some(last_trans) = self.transitions.last_mut()
            && !last_trans.to_states.values().any(|&id| id == state_id)
        {
            last_trans
                .to_states
                .insert(BreakerResult::Placeholder, state_id);
        }

        self.states.push(state);
        self.next_is_state = false;
        self
    }

    /// Add a transition. Panics if a state was expected.
    pub fn add_transition(&mut self, mut trans: MovingTransition) -> &mut Self {
        assert!(
            !self.next_is_state,
            "Expected MovingTransition, but next need is MovingState"
        );

        if let Some(last_state) = self.states.last() {
            let last_id = last_state.id();
            if !trans.from_states.contains(&last_id) {
                trans.from_states.push(last_id);
            }
        }

        self.transitions.push(trans);
        self.next_is_state = true;
        self
    }

    /// Convenience: add a state and transition pair.
    pub fn add_pair(&mut self, state: MovingState, transition: MovingTransition) -> &mut Self {
        self.add_state(state);
        self.add_transition(transition);
        self
    }

    /// Concatenate a pre-built chain of states and transitions.
    pub fn concat(
        &mut self,
        states: Vec<MovingState>,
        transitions: Vec<MovingTransition>,
    ) -> &mut Self {
        let state_len = states.len();
        let trans_len = transitions.len();

        if states.is_empty() && transitions.is_empty() {
            return self;
        }

        if (state_len as isize - trans_len as isize).abs() > 1 {
            panic!(
                "State/transition count mismatch: {} states, {} transitions",
                state_len, trans_len
            );
        }

        // If the head transition already has from_states, pop and add the
        // first state before the transition (matches Python concat behavior).
        let has_start_state = transitions
            .first()
            .is_some_and(|t| !t.from_states.is_empty());

        let mut states_iter = states.into_iter();
        let mut trans_iter = transitions.into_iter();

        if has_start_state && let Some(start_state) = states_iter.next() {
            self.add_state(start_state);
        }

        // Add the head transition.
        if let Some(trans) = trans_iter.next() {
            let trans = if trans.from_states.is_empty() {
                let mut t = trans;
                if let Some(last) = self.states.last() {
                    t.from_states.push(last.id());
                }
                t
            } else {
                trans
            };
            self.add_transition(trans);
        }

        // Interleave: transition first, then state (matching Python zip_longest).
        loop {
            match (trans_iter.next(), states_iter.next()) {
                (Some(trans), Some(state)) => {
                    self.add_transition(trans);
                    self.add_state(state);
                }
                (Some(trans), None) => {
                    self.add_transition(trans);
                }
                (None, Some(state)) => {
                    self.add_state(state);
                }
                (None, None) => break,
            }
        }

        self
    }

    /// Reset the composer to initial state.
    pub fn init_container(&mut self) -> &mut Self {
        self.states.clear();
        self.transitions.clear();
        self.next_is_state = true;
        self
    }

    /// Export the built states and transitions.
    pub fn export(self) -> (Vec<MovingState>, Vec<MovingTransition>) {
        (self.states, self.transitions)
    }
}

impl Default for MovingChainComposer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_chain() {
        let mut comp = MovingChainComposer::new();

        let s0 = MovingState::straight(100);
        let s1 = MovingState::halt();
        let t0 = MovingTransition::new(1.0).unwrap();

        comp.add_state(s0).add_transition(t0).add_state(s1);

        let (states, transitions) = comp.export();
        assert_eq!(states.len(), 2);
        assert_eq!(transitions.len(), 1);

        assert!(transitions[0].from_states.contains(&states[0].id()));
        assert!(
            transitions[0]
                .to_states
                .values()
                .any(|&id| id == states[1].id())
        );
    }

    #[test]
    fn test_concat_one_state_one_trans() {
        let mut comp = MovingChainComposer::new();

        let s0 = MovingState::straight(100);
        comp.add_state(s0);

        let s1 = MovingState::straight(200);
        let t0 = MovingTransition::new(0.5).unwrap();

        comp.concat(vec![s1], vec![t0]);

        let (states, transitions) = comp.export();
        assert_eq!(states.len(), 2);
        assert_eq!(transitions.len(), 1);
        assert!(transitions[0].from_states.contains(&states[0].id()));
        assert!(
            transitions[0]
                .to_states
                .values()
                .any(|&id| id == states[1].id())
        );
    }

    #[test]
    #[should_panic]
    fn test_wrong_order_panics() {
        let mut comp = MovingChainComposer::new();
        comp.add_transition(MovingTransition::new(1.0).unwrap());
    }
}
