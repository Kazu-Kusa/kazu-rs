# Porting TODO

Tracking undone portation work and review items comparing the Rust port (`kazu-rs`) against the Python `kazu` reference (`repo/`).

## Status Key

| Mark | Meaning |
|------|---------|
| ✅ | Fully ported |
| ⚠️ | Ported but incomplete/blocked |
| 🚫 | Deferred (Python-only) |
| ❌ | Not ported |
| 🔍 | Needs review |

---

## Compile Handlers (`compile.py` → `src/compile/`)

| Handler | Rust status | Notes |
|---------|-------------|-------|
| `make_edge_handler` | ✅ | `edge.rs` |
| `make_surrounding_handler` | ✅ | `surrounding.rs` |
| `make_scan_handler` | ✅ | `scan.rs` |
| `make_rand_turn_handler` | ⚠️ | `navigation.rs`, `#[allow(dead_code)]` — not registered in dispatch table or mission assembly |
| `make_gradient_move` | ❌ | Not ported. Python creates a dynamic speed state using Menta runtime codegen + controller context executors. Rust has `GradientConfig` and `use_gradient_move` config fields but no actual handler. |
| `make_search_handler` | ⚠️ | `scan.rs` — Rust version delegates to `make_scan_handler` only. Python version builds a weighted selector across gradient_move, rand_turn, and scan_move. |
| `make_fence_handler` | ✅ | `fence.rs` |
| `make_align_direction_handler` | ✅ | `navigation.rs` |
| `make_back_to_stage_handler` | ✅ | `navigation.rs` |
| `make_reboot_handler` | ✅ | `navigation.rs` |
| `make_rand_walk_handler` | ✅ | `navigation.rs` |
| `make_std_battle_handler` | ✅ | `battle.rs` |
| `make_on_stage_handler` | ✅ | `stage.rs` |
| `make_always_on_stage_battle_handler` | ✅ | `battle.rs` |
| `make_always_off_stage_battle_handler` | ✅ | `battle.rs` |
| `make_salvo_end_state` | ❌ | Not ported. Python uses `@lru_cache` + `controller.register_context_executor` to zero `prev_salvo_speed` on state entry. Rust `scan.rs` substitutes `continues_state()` — no salvo speed reset. |
| `make_unclear_zone_handler` | ⚠️ | `stage.rs` — exists but blocked on live ADC sensor access (gray ADC reads 0.0 without hardware). Not registered in `get_handler_full()` dispatch table. |

## Dispatch Tables (`src/compile/mod.rs`)

| Entry | `get_handler` | `get_handler_full` | Notes |
|-------|---------------|--------------------|-------|
| `edge` | ✅ | ✅ | |
| `surr` | ✅ | ✅ | |
| `scan` | ✅ | ✅ | |
| `search` | ✅ | ✅ | |
| `fence` | ✅ | ✅ | |
| `boot` (reboot) | ✅ | ✅ | |
| `bkstage` | ✅ | ✅ | |
| `rdwalk` | ✅ | ✅ | |
| `align` | ✅ | ✅ | |
| `stdbat` | ✅ | ✅ | |
| `onstage` | ✅ | ✅ | Actually maps to `make_always_on_stage_battle_handler` |
| `angbat` | ✅ | ✅ | |
| `afgbat` | ✅ | ✅ | |
| `randturn` | ❌ | ❌ | `make_rand_turn_handler` exists but not in either dispatch. Should it be exposed? |
| `unclear` | ❌ | ❌ | `make_unclear_zone_handler` exists but not in dispatch. Only used internally by battle handlers. |

## CLI Commands (`cli.py` → `src/commands/`)

| Command | Status | Notes |
|---------|--------|-------|
| `config` | ✅ | |
| `run` | ✅ | |
| `check` | ✅ | |
| `read` | ✅ | |
| `viz` | ✅ | |
| `cmd` | ✅ | |
| `ports` | ✅ | |
| `msg` | ✅ | |
| `light` | ⚠️ | Shell + color parse work; LED output blocked on native library |
| `tag` | ✅ | Requires `vision` feature |
| `breaker` | ✅ | |
| `bench` | ✅ | |
| `record` | ✅ | |
| `trac` | 🚫 | Relies on Python `viztracer` |
| `view` | 🚫 | Relies on Python `vizviewer` |

## Judgers (`judgers.py` → `src/judgers/`)

All 21 breaker methods from Python are ported ✅.

| Breaker | Rust location | Notes |
|---------|---------------|-------|
| `make_std_edge_rear_breaker` | `mod.rs` | ✅ |
| `make_std_edge_front_breaker` | `mod.rs` | ✅ |
| `make_std_edge_full_breaker` | `mod.rs` | ✅ |
| `make_std_turn_to_front_breaker` | `mod.rs` | ✅ |
| `make_std_atk_breaker` | `mod.rs` | ✅ |
| `make_atk_breaker_with_edge_sensors` | `mod.rs` | ✅ |
| `make_surr_breaker` | `mod.rs` | ✅ |
| `make_std_fence_breaker` | `mod.rs` | ✅ |
| `make_lr_sides_blocked_breaker` | `mod.rs` | ✅ |
| `make_std_stage_align_breaker` | `mod.rs` | ✅ |
| `make_stage_align_breaker_mpu` | `mod.rs` | ⚠️ `#[allow(dead_code)]` — blocked on MPU sensor |
| `make_align_direction_breaker_mpu` | `mod.rs` | ⚠️ `#[allow(dead_code)]` — blocked on MPU sensor |
| `make_std_align_direction_breaker` | `mod.rs` | ✅ |
| `make_std_stage_breaker` | `mod.rs` | ✅ |
| `make_always_on_stage_breaker` | `mod.rs` | ✅ |
| `make_always_off_stage_breaker` | `mod.rs` | ✅ |
| `make_is_on_stage_breaker` | `mod.rs` | ✅ |
| `make_std_scan_breaker` | `mod.rs` | ✅ |
| `make_reboot_button_pressed_breaker` | `mod.rs` | ✅ |
| `make_check_gray_adc_for_scan_breaker` | `mod.rs` | ✅ |
| `make_back_stage_side_away_breaker` | `misc.rs` / `mod.rs` | ✅ |

## Config Types (`config.py` → `src/config/`)

All config structs match between Python and Rust. Field-level defaults verified.

| Config | Status | Notes |
|--------|--------|-------|
| `EdgeConfig` | ✅ | |
| `SurroundingConfig` | ✅ | |
| `GradientConfig` | ✅ | Exists but not wired into any handler |
| `ScanConfig` | ✅ | |
| `RandTurn` | ✅ | |
| `SearchConfig` | ✅ | |
| `RandWalk` | ✅ | |
| `FenceConfig` | ✅ | |
| `StrategyConfig` | ✅ | |
| `PerformanceConfig` | ✅ | |
| `BootConfig` | ✅ | |
| `BackStageConfig` | ✅ | |
| `StageConfig` | ✅ | |
| `MotionConfig` | ✅ | |
| `VisionConfig` | ✅ | |
| `DebugConfig` | ✅ | |
| `SensorConfig` | ✅ | |
| `RunConfig` | ✅ | |
| `AppConfig` | ✅ | |
| `ContextVar` | ✅ | Rust adds `IsAligned`, `RecordedPack`; drops `adc_all`/`io_all`/`acc_all` (Menta sampler indexes — not needed without Menta runtime) |

## Hardware-Blocked Items

These compile as stubs/no-ops on non-Linux or without the physical robot.

| Item | Location | Blocked on |
|------|----------|------------|
| MPU stage alignment breaker | `judgers/align.rs` | Real MPU sensor (Raspberry Pi + `libuptech.so`) |
| MPU direction alignment breaker | `judgers/align.rs` | Real MPU sensor |
| `SensorData::mpu_yaw()` | `judgers/mod.rs` | Real MPU sensor |
| Fence handler MPU align path | `compile/fence.rs` | Real MPU sensor |
| Unclear zone handler (gray ADC) | `compile/stage.rs` | Live ADC sensor (gray ADC reads 0.0 via NullSensor) |
| Signal light LED output | `signal_light.rs` | Native LCD/LED hardware |

---

## 🔍 Review Items — Not Suitable for Direct Port

These Python features depend on the `Menta` runtime codegen system (`menta.construct_inlined_function`) and `controller.register_context_executor` / `register_context_getter`. Rust doesn't have equivalent runtime codegen — evaluate whether to implement a static equivalent or accept the simplification.

### 1. `make_gradient_move` — ❌ Not ported

**Python behavior:**
- Uses `menta.construct_inlined_function` with `SamplerUsage(adc_all, [gray_adc_index])` to generate a closure at runtime
- Formula: `speed = ±(min_speed + int(range * (s0 - lower) / (upper - lower)))`
- Registers a context executor on the controller to update `ContextVar.gradient_speed`
- Optionally registers a salvo-end speed updater
- Appends `before_entering` hooks with the updaters

**Rust situation:**
- `GradientConfig` fields are defined in config
- `ContextVar::GradientSpeed` exists
- No handler uses it; `make_search_handler` skips gradient move entirely

**Options:**
a. Implement a static gradient formula in the search handler that doesn't need Menta runtime codegen
b. Leave as dead config (current state) — gradient move is only useful with live ADC feedback
c. Implement a `SpeedExpr::Fn` closure that reads from a shared `SensorData` trait

### 2. `make_salvo_end_state` — ❌ Not ported

**Python behavior:**
- Cached factory returning `MovingState.halt()` with a `before_entering` hook
- Hook registers a context executor that zeros `prev_salvo_speed` via `controller.register_context_executor`

**Rust situation:**
- `scan.rs` uses `continues_state()` instead (no salvo speed tracking)
- `ContextVar::PrevSalvoSpeed` exists but nothing writes to it

**Options:**
a. Implement as a `MovingState::halt()` with a static `before_entering` callback that writes zeros
b. Accept the simplification — salvo speed reset is only relevant with real motor controller feedback

### 3. `make_rand_turn_handler` — ⚠️ Dead code

**Python behavior:**
- Creates `MovingState.rand_dir_turn(controller, speed, turn_left_prob)` — a state that picks left/right randomly at runtime via the controller
- Optionally uses `Breakers.make_std_turn_to_front_breaker`
- Used as one branch of `make_search_handler`'s weighted selector

**Rust situation:**
- Implemented in `navigation.rs` with `#[allow(dead_code)]`
- Not in any dispatch table or assembly
- No equivalent of `rand_dir_turn` — uses `make_turn_l` with fixed direction

**Options:**
a. Wire into dispatch as `"randturn"` pack
b. Wire into `make_search_handler` as a weighted option
c. Accept that random turning can be achieved via `rdwalk` instead

### 4. `callbacks.py` — 🚫 Deferred

Python click-specific CLI validation callbacks (`export_default_app_config`, `disable_cam_callback`, etc.). These are tightly coupled to the `click` framework and `_InternalConfig` pattern. Rust uses `clap` with `#[arg(...)]` attributes — validation is done declaratively. Not suitable for direct port.
