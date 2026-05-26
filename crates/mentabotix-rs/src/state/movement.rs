use std::fmt;

/// Arrow styles for UML generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ArrowStyle {
    #[default]
    Down,
    Left,
    Right,
    Up,
}

impl ArrowStyle {
    /// Create a new ArrowStyle from a direction string.
    pub fn from_direction(direction: &str) -> Result<Self, &'static str> {
        match direction {
            "up" => Ok(ArrowStyle::Up),
            "down" => Ok(ArrowStyle::Down),
            "left" => Ok(ArrowStyle::Left),
            "right" => Ok(ArrowStyle::Right),
            _ => Err("Must be one of [up, down, left, right]"),
        }
    }

    /// Get the string representation of the arrow.
    pub const fn as_str(self) -> &'static str {
        match self {
            ArrowStyle::Down => "-->",
            ArrowStyle::Left => "-left->",
            ArrowStyle::Right => "-right->",
            ArrowStyle::Up => "-up->",
        }
    }
}


impl fmt::Display for ArrowStyle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

/// Configuration for movement calculations.
#[derive(Debug, Clone)]
pub struct MovementConfig {
    /// The width of the track (distance between wheels with same axis).
    pub track_width: f64,
    /// The multiplier for diagonal speeds (designed for drift movement).
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

/// Turn direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TurnDirection {
    Left,
    Right,
}

/// Fixed axis for drift movement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixedAxis {
    FrontLeft,
    RearLeft,
    RearRight,
    FrontRight,
}
