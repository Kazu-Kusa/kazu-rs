# ── Rust workspace tasks for kazu-rs ──────────────────────────────

# Run clippy with auto-fix (apply suggestions)
clippy-fix *args="": fmt-check
    cargo clippy --fix --allow-dirty --allow-staged {{ args }}

# Run clippy (lint check only, no fix)
clippy *args="":
    cargo clippy {{ args }}

# Check compilation (quick)
check *args="":
    cargo check {{ args }}

# Build all (debug)
build *args="":
    cargo build {{ args }}

# Build release
release *args="":
    cargo build --release {{ args }}

# Run all tests
test *args="":
    cargo test --workspace {{ args }}

# Run tests for a specific package
test-pkg pkg *args="":
    cargo test -p {{ pkg }} {{ args }}

# Format code
fmt *args="":
    cargo fmt {{ args }}

# Format check (CI mode)
fmt-check:
    cargo fmt --check

# Full CI pipeline (fmt check + clippy + test)
ci:
    cargo fmt --check
    cargo clippy --workspace -- -D warnings
    cargo test --workspace
