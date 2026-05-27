pub mod botix;
pub mod composer;
pub mod export;
pub mod helpers;
pub mod menta;
pub mod registry;
pub mod state;
pub mod transition;

// Re-exports for convenience.
pub use botix::Botix;
pub use composer::MovingChainComposer;
pub use export::export_structure;
pub use helpers::{NameGenerator, straight_chain, weighted_selector};
pub use menta::{Menta, Sampler, SamplerType, SamplerUsage};
pub use registry::CaseRegistry;
pub use state::{
    ArrowStyle, Context, FixedAxis, MovementConfig, MovingState, PatternType, SpeedExpr,
    SpeedPattern, TurnDirection, clear_state_labels, lookup_state_label, register_state_label,
    reset_state_id_counter,
};
pub use transition::{BreakerResult, MovingTransition};
