use std::collections::HashMap;
use std::fmt;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

mod movement;
pub use movement::{ArrowStyle, MovementConfig, TurnDirection, FixedAxis};

/// Shared context for runtime evaluation of dynamic speed expressions.
pub type Context = HashMap<String, serde_json::Value>;

/// Motor speed configuration for different control patterns.
#[derive(Debug, Clone)]
pub enum SpeedPattern {
    /// All wheels same speed (concrete).
    Full(i32),
    /// Left and right wheels different speeds (concrete).
    LeftRight { left: i32, right: i32 },
    /// Individual wheel control (concrete).
    Individual {
        front_left: i32,
        rear_left: i32,
        front_right: i32,
        rear_right: i32,
    },
    /// Expression-based: evaluated with context at runtime.
    Dynamic {
        pattern_type: PatternType,
        expressions: [SpeedExpr; 4],
    },
}

/// A single speed expression — either constant or closure-evaluated.
pub enum SpeedExpr {
    Const(i32),
    Fn(std::sync::Arc<dyn Fn(&Context) -> i32 + Send + Sync>),
}

impl Clone for SpeedExpr {
    fn clone(&self) -> Self {
        match self {
            SpeedExpr::Const(v) => SpeedExpr::Const(*v),
            SpeedExpr::Fn(f) => SpeedExpr::Fn(std::sync::Arc::clone(f)),
        }
    }
}

impl fmt::Debug for SpeedExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SpeedExpr::Const(v) => write!(f, "Const({})", v),
            SpeedExpr::Fn(_) => write!(f, "Fn(..)"),
        }
    }
}

/// Three types of control commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatternType {
    Full,
    LeftRight,
    Individual,
}

impl SpeedPattern {
    /// Convert to array of individual wheel speeds.
    /// For Dynamic patterns, this returns zeros — use resolve_speeds() instead.
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
            SpeedPattern::Dynamic { .. } => [0; 4],
        }
    }

    /// Resolve speeds at runtime given a context.
    /// For concrete patterns, context is ignored.
    pub fn resolve_speeds(&self, ctx: &Context) -> [i32; 4] {
        match self {
            SpeedPattern::Full(s) => [*s; 4],
            SpeedPattern::LeftRight { left, right } => [*left, *left, *right, *right],
            SpeedPattern::Individual {
                front_left,
                rear_left,
                front_right,
                rear_right,
            } => [*front_left, *rear_left, *front_right, *rear_right],
            SpeedPattern::Dynamic { expressions, .. } => {
                let resolve = |expr: &SpeedExpr| -> i32 {
                    match expr {
                        SpeedExpr::Const(v) => *v,
                        SpeedExpr::Fn(f) => f(ctx),
                    }
                };
                [
                    resolve(&expressions[0]),
                    resolve(&expressions[1]),
                    resolve(&expressions[2]),
                    resolve(&expressions[3]),
                ]
            }
        }
    }

    /// Get the pattern type.
    pub fn pattern_type(&self) -> PatternType {
        match self {
            SpeedPattern::Full(_) => PatternType::Full,
            SpeedPattern::LeftRight { .. } => PatternType::LeftRight,
            SpeedPattern::Individual { .. } => PatternType::Individual,
            SpeedPattern::Dynamic { pattern_type, .. } => *pattern_type,
        }
    }

    /// Check if this pattern uses dynamic expressions.
    pub fn is_dynamic(&self) -> bool {
        matches!(self, SpeedPattern::Dynamic { .. })
    }
}

/// Counter for generating unique state IDs.
static STATE_ID_COUNTER: AtomicUsize = AtomicUsize::new(0);
/// Global registry: state ID → human-readable speed label.
/// Populated on every `MovingState::new()` so that `export_structure`
/// can recover labels from state IDs alone (transitions store IDs only).
static STATE_LABELS: Mutex<Option<HashMap<usize, String>>> = Mutex::new(None);

/// Register a state's speed label for later lookup by ID.
pub fn register_state_label(id: usize, label: String) {
    if let Ok(mut guard) = STATE_LABELS.lock() {
        guard.get_or_insert_with(HashMap::new).insert(id, label);
    }
}

/// Look up a registered state label by ID. Returns `None` if not found.
pub fn lookup_state_label(id: usize) -> Option<String> {
    STATE_LABELS.lock().ok().and_then(|guard| {
        guard.as_ref().and_then(|m| m.get(&id).cloned())
    })
}

/// Clear the global label registry (used between export runs).
pub fn clear_state_labels() {
    if let Ok(mut guard) = STATE_LABELS.lock() {
        *guard = None;
    }
}

/// Reset the state ID counter (only used in tests).
pub fn reset_state_id_counter() {
    STATE_ID_COUNTER.store(0, Ordering::SeqCst);
}

/// Represents a movement state of the robot.
#[derive(Clone)]
pub struct MovingState {
    /// Unique state identifier.
    id: usize,
    /// Speed configuration for this state.
    speed_pattern: SpeedPattern,
    /// Functions to call before entering the state.
    before_entering: Vec<std::sync::Arc<dyn Fn() + Send + Sync>>,
    /// Functions to call after exiting the state.
    after_exiting: Vec<std::sync::Arc<dyn Fn() + Send + Sync>>,
    /// Names of context variables used in dynamic speed expressions.
    used_context_vars: Vec<String>,
}

impl MovingState {
    fn compute_speed_label(sp: &SpeedPattern) -> String {
        match sp {
            SpeedPattern::Full(speed) => {
                if *speed == 0 {
                    "halt".to_string()
                } else {
                    format!("straight({})", speed)
                }
            }
            SpeedPattern::LeftRight { left, right } => {
                format!("turn(l={}, r={})", left, right)
            }
            SpeedPattern::Individual {
                front_left,
                rear_left,
                front_right,
                rear_right,
            } => {
                if front_left == rear_left && front_left == front_right && front_left == rear_right {
                    if *front_left == 0 {
                        "halt".to_string()
                    } else {
                        format!("straight({})", front_left)
                    }
                } else if front_left == rear_left && front_right == rear_right {
                    format!("turn(l={}, r={})", front_left, front_right)
                } else {
                    format!(
                        "indiv({},{},{},{})",
                        front_left, rear_left, front_right, rear_right
                    )
                }
            }
            SpeedPattern::Dynamic { .. } => "dynamic".to_string(),
        }
    }

    /// Create a new moving state with the given speed pattern.
    pub fn new(speed_pattern: SpeedPattern) -> Self {
        let label = Self::compute_speed_label(&speed_pattern);
        let id = STATE_ID_COUNTER.fetch_add(1, Ordering::SeqCst);
        register_state_label(id, label);
        Self {
            id,
            speed_pattern,
            before_entering: Vec::new(),
            after_exiting: Vec::new(),
            used_context_vars: Vec::new(),
        }
    }

    /// Create a halted state (all wheels stop).
    pub fn halt() -> Self {
        Self::new(SpeedPattern::Full(0))
    }

    /// Create a straight movement state.
    pub fn straight(speed: i32) -> Self {
        Self::new(SpeedPattern::Full(speed))
    }

    /// Create a turn state.
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

    /// Create a differential movement state.
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

    /// Create a drift state.
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

    /// Create a state with dynamic speed expressions.
    /// `expressions` is a 4-element array of SpeedExpr.
    /// `pattern_type` indicates how the expressions are interpreted.
    /// `used_context_vars` lists context variable names needed.
    pub fn dynamic(
        expressions: [SpeedExpr; 4],
        pattern_type: PatternType,
        used_context_vars: Vec<String>,
    ) -> Self {
        let id = STATE_ID_COUNTER.fetch_add(1, Ordering::SeqCst);
        let label = Self::compute_speed_label(&SpeedPattern::Dynamic {
            pattern_type,
            expressions: expressions.clone(),
        });
        register_state_label(id, label);
        Self {
            id,
            speed_pattern: SpeedPattern::Dynamic {
                pattern_type,
                expressions,
            },
            before_entering: Vec::new(),
            after_exiting: Vec::new(),
            used_context_vars,
        }
    }

    /// Get the state identifier.
    pub fn id(&self) -> usize {
        self.id
    }

    /// Get the speed pattern.
    pub fn speed_pattern(&self) -> &SpeedPattern {
        &self.speed_pattern
    }

    /// Get the pattern type.
    pub fn pattern_type(&self) -> PatternType {
        self.speed_pattern.pattern_type()
    }

    /// Get speed values as an array (zero for dynamic patterns).
    pub fn speeds(&self) -> [i32; 4] {
        self.speed_pattern.to_array()
    }

    /// Resolve speeds at runtime given a context.
    pub fn resolve_speeds(&self, ctx: &Context) -> [i32; 4] {
        self.speed_pattern.resolve_speeds(ctx)
    }

    /// Get used context variable names.
    pub fn used_context_vars(&self) -> &[String] {
        &self.used_context_vars
    }

    /// Check if this state uses dynamic speed expressions.
    pub fn is_dynamic(&self) -> bool {
        self.speed_pattern.is_dynamic()
    }

    /// Apply a multiplier to the speeds.
    /// Panics if called on a dynamic pattern.
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
            SpeedPattern::Dynamic { .. } => {
                panic!("Cannot apply multiplier to a dynamic speed pattern")
            }
        };
        self
    }

    /// Add a hook to be called before entering the state.
    pub fn with_before_entering<F: Fn() + Send + Sync + 'static>(mut self, hook: F) -> Self {
        self.before_entering.push(std::sync::Arc::new(hook));
        self
    }

    /// Add a hook to be called after exiting the state.
    pub fn with_after_exiting<F: Fn() + Send + Sync + 'static>(mut self, hook: F) -> Self {
        self.after_exiting.push(std::sync::Arc::new(hook));
        self
    }

    /// Get references to before-entering hooks.
    pub fn before_entering(&self) -> &[std::sync::Arc<dyn Fn() + Send + Sync>] {
        &self.before_entering
    }

    /// Get references to after-exiting hooks.
    pub fn after_exiting(&self) -> &[std::sync::Arc<dyn Fn() + Send + Sync>] {
        &self.after_exiting
    }
}

impl fmt::Display for MovingState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.speed_pattern {
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
            SpeedPattern::Dynamic { .. } => write!(f, "State{}(dynamic)", self.id),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_resolve_speeds_const() {
        let state = MovingState::straight(100);
        let ctx = Context::new();
        assert_eq!(state.resolve_speeds(&ctx), [100, 100, 100, 100]);
    }

    #[test]
    fn test_resolve_speeds_leftright() {
        let state = MovingState::turn(TurnDirection::Left, 50);
        let ctx = Context::new();
        assert_eq!(state.resolve_speeds(&ctx), [-50, -50, 50, 50]);
    }

    #[test]
    fn test_resolve_speeds_dynamic() {
        let mut ctx = Context::new();
        ctx.insert("v".to_string(), serde_json::Value::Number(serde_json::Number::from(42)));

        let expressions = [
            SpeedExpr::Fn(std::sync::Arc::new(|c: &Context| {
                c.get("v")
                    .and_then(|v| v.as_i64())
                    .unwrap_or(0) as i32
            })),
            SpeedExpr::Fn(std::sync::Arc::new(|c: &Context| {
                c.get("v")
                    .and_then(|v| v.as_i64())
                    .unwrap_or(0) as i32
            })),
            SpeedExpr::Const(10),
            SpeedExpr::Const(10),
        ];

        let state = MovingState::dynamic(expressions, PatternType::Individual, vec!["v".into()]);
        assert_eq!(state.resolve_speeds(&ctx), [42, 42, 10, 10]);
    }

    #[test]
    fn test_is_dynamic() {
        assert!(!MovingState::straight(100).is_dynamic());
        let dyn_state = MovingState::dynamic(
            [
                SpeedExpr::Const(1),
                SpeedExpr::Const(2),
                SpeedExpr::Const(3),
                SpeedExpr::Const(4),
            ],
            PatternType::Individual,
            vec![],
        );
        assert!(dyn_state.is_dynamic());
    }
}
