use bdmc_rs::controller::CloseLoopController;
use std::collections::{HashMap, HashSet};
use std::fmt;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Motor speed configuration for different control patterns
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpeedPattern {
    /// All wheels same speed
    Full(i32),
    /// Left and right wheels different speeds
    LeftRight { left: i32, right: i32 },
    /// Individual wheel control
    Individual {
        front_left: i32,
        rear_left: i32,
        front_right: i32,
        rear_right: i32,
    },
}

impl SpeedPattern {
    /// Convert to array of individual wheel speeds
    pub fn to_array(&self) -> [i32; 4] {
        match *self {
            SpeedPattern::Full(speed) => [speed; 4],
            SpeedPattern::LeftRight { left, right } => [left, left, right, right],
            SpeedPattern::Individual {
                front_left,
                rear_left,
                front_right,
                rear_right,
            } => [front_left, rear_left, front_right, rear_right],
        }
    }

    /// Get the pattern type
    pub fn pattern_type(&self) -> PatternType {
        match self {
            SpeedPattern::Full(_) => PatternType::Full,
            SpeedPattern::LeftRight { .. } => PatternType::LeftRight,
            SpeedPattern::Individual { .. } => PatternType::Individual,
        }
    }
}

/// Three types of control commands
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatternType {
    Full,
    LeftRight,
    Individual,
}

/// Arrow styles for UML generation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrowStyle {
    Down,
    Left,
    Right,
    Up,
}

impl ArrowStyle {
    /// Create a new ArrowStyle from string
    pub fn from_str(direction: &str) -> Result<Self, &'static str> {
        match direction {
            "up" => Ok(ArrowStyle::Up),
            "down" => Ok(ArrowStyle::Down),
            "left" => Ok(ArrowStyle::Left),
            "right" => Ok(ArrowStyle::Right),
            _ => Err("Must be one of [up, down, left, right]"),
        }
    }

    /// Get the string representation of the arrow
    pub const fn as_str(self) -> &'static str {
        match self {
            ArrowStyle::Down => "-->",
            ArrowStyle::Left => "-left->",
            ArrowStyle::Right => "-right->",
            ArrowStyle::Up => "-up->",
        }
    }
}

impl Default for ArrowStyle {
    fn default() -> Self {
        ArrowStyle::Down
    }
}

impl fmt::Display for ArrowStyle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

/// Configuration for movement calculations
#[derive(Debug, Clone)]
pub struct MovementConfig {
    /// The width of the track (distance between wheels with same axis)
    pub track_width: f64,
    /// The multiplier for diagonal speeds (designed for drift movement)
    pub diagonal_multiplier: f64,
}

impl Default for MovementConfig {
    fn default() -> Self {
        Self {
            track_width: 100.0,
            diagonal_multiplier: 1.53,
        }
    }
}

/// Turn direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TurnDirection {
    Left,
    Right,
}

/// Fixed axis for drift movement
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixedAxis {
    FrontLeft,
    RearLeft,
    RearRight,
    FrontRight,
}

/// Counter for generating unique state IDs
static STATE_ID_COUNTER: AtomicUsize = AtomicUsize::new(0);

/// Represents a movement state of the robot
pub struct MovingState {
    /// Unique state identifier
    id: usize,
    /// Speed configuration for this state
    speed_pattern: SpeedPattern,
    /// Functions to call before entering the state
    before_entering: Vec<Box<dyn Fn() + Send + Sync>>,
    /// Functions to call after exiting the state
    after_exiting: Vec<Box<dyn Fn() + Send + Sync>>,
}

impl MovingState {
    /// Create a new moving state with the given speed pattern
    pub fn new(speed_pattern: SpeedPattern) -> Self {
        Self {
            id: STATE_ID_COUNTER.fetch_add(1, Ordering::SeqCst),
            speed_pattern,
            before_entering: Vec::new(),
            after_exiting: Vec::new(),
        }
    }

    /// Create a halted state (all wheels stop)
    pub fn halt() -> Self {
        Self::new(SpeedPattern::Full(0))
    }

    /// Create a straight movement state
    pub fn straight(speed: i32) -> Self {
        Self::new(SpeedPattern::Full(speed))
    }

    /// Create a turn state
    pub fn turn(direction: TurnDirection, speed: i32) -> Self {
        match direction {
            TurnDirection::Left => Self::new(SpeedPattern::LeftRight {
                left: -speed,
                right: speed,
            }),
            TurnDirection::Right => Self::new(SpeedPattern::LeftRight {
                left: speed,
                right: -speed,
            }),
        }
    }

    /// Create a differential movement state
    pub fn differential(direction: TurnDirection, radius: f64, outer_speed: i32) -> Self {
        let config = MovementConfig::default();
        let inner_speed = (radius / (radius + config.track_width) * outer_speed as f64) as i32;

        match direction {
            TurnDirection::Left => Self::new(SpeedPattern::LeftRight {
                left: inner_speed,
                right: outer_speed,
            }),
            TurnDirection::Right => Self::new(SpeedPattern::LeftRight {
                left: outer_speed,
                right: inner_speed,
            }),
        }
    }

    /// Create a drift state
    pub fn drift(fixed_axis: FixedAxis, speed: i32) -> Self {
        let config = MovementConfig::default();
        let diagonal_speed = (speed as f64 * config.diagonal_multiplier) as i32;

        let pattern = match fixed_axis {
            FixedAxis::FrontLeft => SpeedPattern::Individual {
                front_left: 0,
                rear_left: speed,
                front_right: diagonal_speed,
                rear_right: speed,
            },
            FixedAxis::RearLeft => SpeedPattern::Individual {
                front_left: speed,
                rear_left: 0,
                front_right: speed,
                rear_right: diagonal_speed,
            },
            FixedAxis::RearRight => SpeedPattern::Individual {
                front_left: diagonal_speed,
                rear_left: speed,
                front_right: 0,
                rear_right: speed,
            },
            FixedAxis::FrontRight => SpeedPattern::Individual {
                front_left: speed,
                rear_left: diagonal_speed,
                front_right: speed,
                rear_right: 0,
            },
        };

        Self::new(pattern)
    }

    /// Get the state identifier
    pub fn id(&self) -> usize {
        self.id
    }

    /// Get the speed pattern
    pub fn speed_pattern(&self) -> SpeedPattern {
        self.speed_pattern
    }

    /// Get the pattern type
    pub fn pattern_type(&self) -> PatternType {
        self.speed_pattern.pattern_type()
    }

    /// Get speed values as an array
    pub fn speeds(&self) -> [i32; 4] {
        self.speed_pattern.to_array()
    }

    /// Apply a multiplier to the speeds
    pub fn with_multiplier(mut self, multiplier: f64) -> Self {
        self.speed_pattern = match self.speed_pattern {
            SpeedPattern::Full(speed) => SpeedPattern::Full((speed as f64 * multiplier) as i32),
            SpeedPattern::LeftRight { left, right } => SpeedPattern::LeftRight {
                left: (left as f64 * multiplier) as i32,
                right: (right as f64 * multiplier) as i32,
            },
            SpeedPattern::Individual {
                front_left,
                rear_left,
                front_right,
                rear_right,
            } => SpeedPattern::Individual {
                front_left: (front_left as f64 * multiplier) as i32,
                rear_left: (rear_left as f64 * multiplier) as i32,
                front_right: (front_right as f64 * multiplier) as i32,
                rear_right: (rear_right as f64 * multiplier) as i32,
            },
        };
        self
    }
}

impl fmt::Display for MovingState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.speed_pattern {
            SpeedPattern::Full(speed) => write!(f, "State{}({})", self.id, speed),
            SpeedPattern::LeftRight { left, right } => {
                write!(f, "State{}({}, {})", self.id, left, right)
            }
            SpeedPattern::Individual {
                front_left,
                rear_left,
                front_right,
                rear_right,
            } => write!(
                f,
                "State{}([{}, {}, {}, {}])",
                self.id, front_left, rear_left, front_right, rear_right
            ),
        }
    }
}

impl PartialEq for MovingState {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for MovingState {}

impl std::hash::Hash for MovingState {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}

/// Counter for generating unique transition IDs
static TRANSITION_ID_COUNTER: AtomicUsize = AtomicUsize::new(0);

/// Represents a transition between movement states
pub struct MovingTransition {
    /// Unique transition identifier
    id: usize,
    /// Transition duration in seconds
    pub duration: f64,
    /// Optional breaker function to interrupt transition
    pub breaker: Option<Box<dyn Fn() -> bool + Send + Sync>>,
    /// Frequency to check for state transition
    pub check_interval: f64,
    /// Starting states for the transition
    pub from_states: Vec<MovingState>,
    /// Destination states mapped to keys
    pub to_states: HashMap<String, MovingState>,
}

impl MovingTransition {
    /// Create a new MovingTransition
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

    /// Set the breaker function
    pub fn with_breaker<F>(mut self, breaker: F) -> Self
    where
        F: Fn() -> bool + Send + Sync + 'static,
    {
        self.breaker = Some(Box::new(breaker));
        self
    }

    /// Set the check interval
    pub fn with_check_interval(mut self, interval: f64) -> Self {
        self.check_interval = interval;
        self
    }

    /// Add a from state
    pub fn with_from_state(mut self, state: MovingState) -> Self {
        self.from_states.push(state);
        self
    }

    /// Add a to state with key
    pub fn with_to_state(mut self, key: impl Into<String>, state: MovingState) -> Self {
        self.to_states.insert(key.into(), state);
        self
    }

    /// Get the transition identifier
    pub fn id(&self) -> usize {
        self.id
    }
}

impl fmt::Display for MovingTransition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Transition{}({:.3}s)", self.id, self.duration)
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

/// Main Botix struct for managing states and transitions
pub struct Botix {
    /// The bot's controller
    controller: CloseLoopController,
    /// Token pool containing transitions
    transitions: Vec<MovingTransition>,
}

impl Botix {
    /// Create a new Botix instance
    pub fn new(controller: CloseLoopController) -> Self {
        Self {
            controller,
            transitions: Vec::new(),
        }
    }

    /// Get a reference to the controller
    pub fn controller(&self) -> &CloseLoopController {
        &self.controller
    }

    /// Get a mutable reference to the controller
    pub fn controller_mut(&mut self) -> &mut CloseLoopController {
        &mut self.controller
    }

    /// Add a transition to the pool
    pub fn add_transition(&mut self, transition: MovingTransition) -> &mut Self {
        self.transitions.push(transition);
        self
    }

    /// Remove a transition from the pool
    pub fn remove_transition(&mut self, transition_id: usize) -> &mut Self {
        self.transitions.retain(|t| t.id() != transition_id);
        self
    }

    /// Extend the pool with multiple transitions
    pub fn extend_transitions(&mut self, transitions: Vec<MovingTransition>) -> &mut Self {
        self.transitions.extend(transitions);
        self
    }

    /// Clear all transitions from the pool
    pub fn clear_transitions(&mut self) -> &mut Self {
        self.transitions.clear();
        self
    }

    /// Get start states from transition pool
    pub fn start_states(&self) -> HashSet<usize> {
        let mut states_indegree: HashMap<usize, i32> = HashMap::new();

        for transition in &self.transitions {
            for state in &transition.from_states {
                states_indegree.entry(state.id()).or_insert(0);
            }
            for state in transition.to_states.values() {
                *states_indegree.entry(state.id()).or_insert(0) += 1;
            }
        }

        states_indegree
            .iter()
            .filter(|&(_, &indegree)| indegree == 0)
            .map(|(&state_id, _)| state_id)
            .collect()
    }

    /// Get end states from transition pool
    pub fn end_states(&self) -> HashSet<usize> {
        let mut states_outdegree: HashMap<usize, i32> = HashMap::new();

        for transition in &self.transitions {
            for state in &transition.from_states {
                *states_outdegree.entry(state.id()).or_insert(0) += 1;
            }
            for state in transition.to_states.values() {
                states_outdegree.entry(state.id()).or_insert(0);
            }
        }

        states_outdegree
            .iter()
            .filter(|&(_, &outdegree)| outdegree == 0)
            .map(|(&state_id, _)| state_id)
            .collect()
    }

    /// Validate the transition pool structure
    pub fn validate(&self) -> Result<(), &'static str> {
        let start_states = self.start_states();
        if start_states.len() != 1 {
            return Err("Must have exactly one start state");
        }
        Ok(())
    }
}