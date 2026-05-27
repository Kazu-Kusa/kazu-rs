use std::collections::HashMap;
use std::fmt;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Typed breaker result — replaces Python's arbitrary KT type variable.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum BreakerResult {
    Bool(bool),
    Int(i64),
    Str(String),
    /// Sentinel for branchless transitions: only one to_state.
    Placeholder,
}

impl fmt::Display for BreakerResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BreakerResult::Bool(b) => write!(f, "{}", b),
            BreakerResult::Int(i) => write!(f, "{}", i),
            BreakerResult::Str(s) => write!(f, "{}", s),
            BreakerResult::Placeholder => write!(f, "_"),
        }
    }
}

impl From<bool> for BreakerResult {
    fn from(b: bool) -> Self {
        BreakerResult::Bool(b)
    }
}

impl From<i64> for BreakerResult {
    fn from(i: i64) -> Self {
        BreakerResult::Int(i)
    }
}

impl From<i32> for BreakerResult {
    fn from(i: i32) -> Self {
        BreakerResult::Int(i as i64)
    }
}

impl From<&str> for BreakerResult {
    fn from(s: &str) -> Self {
        BreakerResult::Str(s.to_string())
    }
}

impl From<String> for BreakerResult {
    fn from(s: String) -> Self {
        BreakerResult::Str(s)
    }
}

/// Counter for generating unique transition IDs.
static TRANSITION_ID_COUNTER: AtomicUsize = AtomicUsize::new(0);

/// Represents a transition between movement states.
///
/// Stores state IDs (indices into Botix's state registry), not owned
/// `MovingState` objects. This eliminates cloning and makes the graph
/// lightweight.
pub struct MovingTransition {
    /// Unique transition identifier.
    id: usize,
    /// Transition duration in seconds.
    pub duration: f64,
    /// Optional breaker function to interrupt the transition.
    pub breaker: Option<std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync>>,
    /// Frequency to check for state transition (seconds).
    pub check_interval: f64,
    /// Starting state IDs for the transition.
    pub from_states: Vec<usize>,
    /// Destination state IDs mapped by breaker result.
    pub to_states: HashMap<BreakerResult, usize>,
}

impl MovingTransition {
    /// Create a new MovingTransition with required duration.
    pub fn new(duration: f64) -> Result<Self, &'static str> {
        if duration < 0.0 {
            return Err("Duration cannot be negative");
        }

        Ok(Self {
            id: TRANSITION_ID_COUNTER.fetch_add(1, Ordering::SeqCst),
            duration,
            breaker: None,
            check_interval: 0.01,
            from_states: Vec::new(),
            to_states: HashMap::new(),
        })
    }

    /// Set the breaker function.
    pub fn with_breaker<F>(mut self, breaker: F) -> Self
    where
        F: Fn() -> BreakerResult + Send + Sync + 'static,
    {
        self.breaker = Some(std::sync::Arc::new(breaker));
        self
    }

    /// Set the breaker from an existing Arc.
    pub fn with_arc_breaker(
        mut self,
        breaker: std::sync::Arc<dyn Fn() -> BreakerResult + Send + Sync>,
    ) -> Self {
        self.breaker = Some(breaker);
        self
    }

    /// Set the breaker function returning bool (convenience).
    pub fn with_bool_breaker<F>(mut self, breaker: F) -> Self
    where
        F: Fn() -> bool + Send + Sync + 'static,
    {
        self.breaker = Some(std::sync::Arc::new(move || BreakerResult::Bool(breaker())));
        self
    }

    /// Set the check interval.
    pub fn with_check_interval(mut self, interval: f64) -> Self {
        self.check_interval = interval;
        self
    }

    /// Add a from state by ID.
    pub fn with_from_state(mut self, state_id: usize) -> Self {
        self.from_states.push(state_id);
        self
    }

    /// Add a to state with a breaker result key.
    pub fn with_to_state<K: Into<BreakerResult>>(mut self, key: K, state_id: usize) -> Self {
        self.to_states.insert(key.into(), state_id);
        self
    }

    /// Set a single to_state (branchless transition).
    pub fn with_single_to_state(mut self, state_id: usize) -> Self {
        self.to_states.insert(BreakerResult::Placeholder, state_id);
        self
    }

    /// Get the transition identifier.
    pub fn id(&self) -> usize {
        self.id
    }

    /// Check if this transition has branching (multiple to_states).
    pub fn is_branching(&self) -> bool {
        self.to_states.len() > 1
    }

    /// Check if this transition has a breaker function.
    pub fn has_breaker(&self) -> bool {
        self.breaker.is_some()
    }
}

impl fmt::Display for MovingTransition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Transition{}({:.3}s, {} branches)",
            self.id,
            self.duration,
            self.to_states.len()
        )
    }
}

impl fmt::Debug for MovingTransition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MovingTransition")
            .field("id", &self.id)
            .field("duration", &self.duration)
            .field("check_interval", &self.check_interval)
            .field("from_states", &self.from_states)
            .field("to_states", &self.to_states)
            .finish()
    }
}

impl PartialEq for MovingTransition {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for MovingTransition {}

impl std::hash::Hash for MovingTransition {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_transition() {
        let t = MovingTransition::new(1.0).unwrap();
        assert_eq!(t.duration, 1.0);
        assert!(t.breaker.is_none());
        assert!(t.from_states.is_empty());
        assert!(t.to_states.is_empty());
    }

    #[test]
    fn test_new_transition_negative_duration() {
        assert!(MovingTransition::new(-0.1).is_err());
    }

    #[test]
    fn test_with_breaker() {
        let t = MovingTransition::new(1.0)
            .unwrap()
            .with_breaker(|| BreakerResult::Bool(true));
        assert!(t.has_breaker());
    }

    #[test]
    fn test_branchless_transition() {
        let t = MovingTransition::new(1.0)
            .unwrap()
            .with_from_state(0)
            .with_single_to_state(1);
        assert!(!t.is_branching());
        assert_eq!(t.to_states.len(), 1);
    }

    #[test]
    fn test_branching_transition() {
        let t = MovingTransition::new(1.0)
            .unwrap()
            .with_from_state(0)
            .with_to_state(BreakerResult::Bool(true), 1)
            .with_to_state(BreakerResult::Bool(false), 2);
        assert!(t.is_branching());
    }

    #[test]
    fn test_breaker_result_from() {
        assert_eq!(BreakerResult::from(true), BreakerResult::Bool(true));
        assert_eq!(BreakerResult::from(42i32), BreakerResult::Int(42));
        assert_eq!(
            BreakerResult::from("hello"),
            BreakerResult::Str("hello".into())
        );
    }
}
