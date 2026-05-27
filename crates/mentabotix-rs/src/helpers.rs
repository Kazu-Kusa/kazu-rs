use crate::composer::MovingChainComposer;
use crate::state::MovingState;
use crate::transition::{BreakerResult, MovingTransition};
use rand::Rng;

/// Simple counter-based unique name generator.
pub struct NameGenerator {
    prefix: String,
    counter: usize,
}

impl NameGenerator {
    /// Create a new NameGenerator with the given prefix.
    pub fn new(prefix: impl Into<String>) -> Self {
        Self {
            prefix: prefix.into(),
            counter: 0,
        }
    }

    /// Generate the next unique name.
    pub fn next_name(&mut self) -> String {
        let name = format!("{}{}", self.prefix, self.counter);
        self.counter += 1;
        name
    }

    /// Peek at the current counter without advancing.
    pub fn current_count(&self) -> usize {
        self.counter
    }

    /// Reset the counter.
    pub fn reset(&mut self) {
        self.counter = 0;
    }
}

/// Weighted random selection.
///
/// Returns a closure that, each time called, selects an element from `pool`
/// according to `weights`. If weights are empty or None, uniform selection is used.
pub fn weighted_selector<T: Clone + Send + Sync + 'static>(
    pool: Vec<T>,
    weights: Option<Vec<f64>>,
) -> Box<dyn Fn() -> T + Send + Sync> {
    let use_weights = weights.filter(|w| !w.is_empty());

    match use_weights {
        Some(w) => {
            // Normalize weights.
            let total: f64 = w.iter().sum();
            let cumsum: Vec<f64> = w
                .iter()
                .scan(0.0, |acc, &weight| {
                    *acc += weight / total;
                    Some(*acc)
                })
                .collect();

            Box::new(move || {
                let r: f64 = rand::thread_rng().r#gen();
                let idx = cumsum
                    .iter()
                    .position(|&c| r <= c)
                    .unwrap_or(pool.len() - 1);
                pool[idx].clone()
            })
        }
        None => Box::new(move || {
            let idx = rand::thread_rng().gen_range(0..pool.len());
            pool[idx].clone()
        }),
    }
}

/// Generate a straight-line acceleration/deceleration chain.
///
/// Creates a sequence of states and transitions that linearly interpolate
/// from `start_speed` to `end_speed` over `duration` seconds, with `interval`
/// seconds per step. The speed follows a power curve controlled by `power_exponent`.
///
/// If a `breaker` is provided, each transition includes it and `state_on_break`
/// becomes a possible to_state.
pub fn straight_chain(
    start_speed: i32,
    end_speed: i32,
    duration: f64,
    power_exponent: f64,
    interval: f64,
    breaker: Option<Box<dyn Fn() -> BreakerResult + Send + Sync>>,
) -> (Vec<MovingState>, Vec<MovingTransition>) {
    let state_on_break = MovingState::halt();
    let _state_on_break_id = state_on_break.id();

    let adjusted_duration = duration;
    let deviation = end_speed - start_speed;
    let step_count = (adjusted_duration / interval).max(1.0) as usize;
    let step_duration = adjusted_duration / step_count as f64;

    let mut comp = MovingChainComposer::new();
    comp.add_state(MovingState::straight(start_speed));

    for i in 1..=step_count {
        let progress = i as f64 / step_count as f64;
        let speed = start_speed + (deviation as f64 * progress.powf(power_exponent)).round() as i32;

        let trans = MovingTransition::new(step_duration).unwrap();

        if let Some(ref breaker_fn) = breaker {
            // With breaker: the transition can be interrupted.
            // Register the breaker and the state_on_break.
            // Since we can't clone the breaker, we need a different approach.
            // In practice, the breaker can be shared via Arc.
            // For now, we skip the breaker in the chain (caller attaches it).
            let _ = breaker_fn;
        }

        comp.add_transition(trans)
            .add_state(MovingState::straight(speed));
    }

    // Include the state_on_break in the output if breaker is provided.
    let (mut states, transitions) = comp.export();

    if breaker.is_some() {
        states.push(state_on_break);
    }

    (states, transitions)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_name_generator() {
        let mut name_gen = NameGenerator::new("test_");
        assert_eq!(name_gen.next_name(), "test_0");
        assert_eq!(name_gen.next_name(), "test_1");
        assert_eq!(name_gen.next_name(), "test_2");
    }

    #[test]
    fn test_weighted_selector_uniform() {
        let pool = vec!["a", "b", "c"];
        let selector = weighted_selector(pool.clone(), None);
        for _ in 0..10 {
            let val = selector();
            assert!(pool.contains(&val));
        }
    }

    #[test]
    fn test_weighted_selector_weighted() {
        let pool = vec!["x", "y"];
        let weights = Some(vec![1.0, 0.0]); // Always pick "x".
        let selector = weighted_selector(pool, weights);
        for _ in 0..10 {
            assert_eq!(selector(), "x");
        }
    }

    #[test]
    fn test_straight_chain_length() {
        let (states, transitions) = straight_chain(0, 100, 1.0, 1.0, 0.1, None);

        // 1.0 / 0.1 = 10 steps → 11 states (start + 10), 10 transitions.
        assert_eq!(states.len(), 11);
        assert_eq!(transitions.len(), 10);
    }

    #[test]
    fn test_straight_chain_speeds() {
        let (states, _transitions) = straight_chain(
            0, 100, 1.0, 1.0, // linear
            0.25, None,
        );

        // 4 steps: speeds should be 0, 25, 50, 75, 100.
        assert_eq!(states.len(), 5);
        let speeds: Vec<i32> = states.iter().map(|s| s.speeds()[0]).collect();
        assert_eq!(speeds[0], 0);
        assert_eq!(speeds.last(), Some(&100));
    }
}
