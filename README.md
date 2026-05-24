# kazu-rs

Rust port of [kazu](https://github.com/kcirJxek/kazu) — A Dedicated Robots Control System for competition robots.

## Architecture

```
kazu-rs/                          CLI + config + compile + commands (15 subcommands)
├── src/
│   ├── main.rs                    Entry point
│   ├── cli.rs                     Argument parsing (clap)
│   ├── constant.rs                Sensor weights, code signs, run-mode enums
│   ├── assembly.rs                Mission schema assembly (FGS, NGS, AFG, ANG, FGDL)
│   ├── signal_light.rs            LED color registry
│   ├── static_utils.rs            Tag query table, timestamp
│   ├── compile/                   12 behavior handlers (split from compile.rs)
│   │   ├── mod.rs                 Helpers, HandlerOutput, dispatch table
│   │   ├── edge.rs                Edge detection + escape chains
│   │   ├── surrounding.rs         Obstacle classification + targeting
│   │   ├── scan.rs                Scan + search behaviors
│   │   ├── fence.rs               Fence boundary response
│   │   ├── navigation.rs          Reboot, back-to-stage, rand walk, align direction
│   │   ├── battle.rs              Standard + always-on/off stage battle
│   │   └── stage.rs               On-stage loop + unclear zone recovery
│   ├── config/                    Configuration types (split from config.rs)
│   │   ├── mod.rs                 AppConfig, RunConfig, ContextVar, I/O loaders
│   │   ├── behavior.rs            Edge, Surrounding, Scan, Fence, Gradient, Rand*, TagGroup
│   │   ├── stage.rs               Stage, BackStage, Boot, Strategy, Performance
│   │   └── hardware.rs            Sensor, Motion, Vision, Debug
│   ├── judgers/                   Breaker predicate factories (split from judgers.rs)
│   │   ├── mod.rs                 SensorData trait, Breakers struct, delegation
│   │   ├── edge.rs                Edge detection breakers (rear, front, full)
│   │   ├── surrounding.rs         Attack + surrounding object detection
│   │   ├── fence.rs               Fence proximity breakers
│   │   ├── align.rs               Stage/direction alignment breakers (ADC + MPU)
│   │   ├── stage.rs               On/off stage + unclear zone detection
│   │   └── misc.rs                Scan, reboot button, gray ADC, back-stage
│   └── commands/                  14 CLI subcommand implementations
├── crates/
│   ├── bdmc-rs/                   Close-loop motor controller + serial I/O
│   ├── mentabotix-rs/             State-machine graph engine + PlantUML export
│   ├── upic-rs/                   AprilTag detection (opencv + apriltag)
│   └── uptechstar-rs/             HW bindings: sensors, LCD, LEDs (native FFI)
└── repo/                          Original Python sources (reference)
```

## Porting Status

### CLI Commands

| Command   | Status | Notes |
|-----------|--------|-------|
| `config`  | ✅ | Read/write app + run TOML configs |
| `run`     | ✅ | Missions resolved from run config via compile handlers + assembly |
| `check`   | ✅ | Tests mot/adc/io/mpu/cam |
| `read`    | ✅ | Stream sensor data to terminal/screen |
| `viz`     | ✅ | All 12 handlers produce state-machine graphs; PlantUML export |
| `cmd`     | ✅ | Raw serial command shell |
| `ports`   | ✅ | Scan + probe COM ports |
| `msg`     | ✅ | Serial message streaming |
| `light`   | ⚠️  | Shell mode + color parse; no real LED output without native lib |
| `tag`     | ✅ | AprilTag detection via upic-rs |
| `breaker` | ✅ | Emergency motor stop |
| `bench`   | ✅ | Siglight, Sleep, Adc, App benchmarks |
| `record`  | ✅ | Sensor data to CSV |
| `trac`    | 🚫 | Requires `viztracer` (Python-only) — deferred |
| `view`    | 🚫 | Requires `vizviewer` (Python-only) — deferred |

### Library Crates

| Python module | Rust crate | Status |
|---------------|-----------|--------|
| `mentabotix/` (158KB) | `mentabotix-rs/` | ✅ Core graph engine: Botix, MovingState, MovingTransition, Menta, Composer, PlantUML export |
| `bdmc/` (17KB) | `bdmc-rs/` | ✅ CloseLoopController, serial I/O, port discovery |
| `upic/` (31KB) | `upic-rs/` | ✅ TagDetector with native bindings |
| `pyuptech/` (37KB) | `uptechstar-rs/` | ✅ Sensors (MPU, ADC, IO), LCD screen, FFI loader (stub on non-Linux) |

### kazu Application Layer

| Module (KB)         | Rust location | Status | Notes |
|---------------------|---------------|--------|-------|
| `cli.py` (68)       | `cli.rs` + `commands/` | ✅ | 13/15 commands ported; 2 deferred (Python-only tools) |
| `compile.py` (68)   | `compile/` (8 files) | ✅ | 12 handlers; 2 partially blocked (see below) |
| `config.py` (32)    | `config/` (4 files) | ✅ | All config types + ContextVar + default_baudrate |
| `judgers.py` (47)   | `judgers/` (7 files) | ✅ | 21 breaker methods; 2 MPU-dependent deferred |
| `constant.py` (10)  | `constant.rs` | ✅ | CodeSign enums, Axis, Attitude, weight tables |
| `signal_light.py` (6) | `signal_light.rs` | ⚠️ | Registry ported; LED output blocked on HW |
| `hardwares.py` (4)  | partial | — | `SamplerIndexes` in uptechstar-rs; singleton wiring not ported |
| `assembly.py` (5)   | `assembly.rs` | ✅ | All 5 schema functions (FGS, NGS, AFG, ANG, FGDL) |
| `callbacks.py` (10) | ❌ | — | CLI validation callbacks (click-rs specific) |
| `checkers.py` (3)   | partial | — | Inline in `cmd_check` |
| `static.py` (2)     | `static_utils.rs` | ✅ | `make_query_table` wired into surr_breaker |
| `logger.py` (1)     | N/A | ✅ | Replaced by `log` crate |
| `visualize.py` (1)  | N/A | ✅ | Inline in `cmd_viz` |

## Blocked / Pending

### Blocked on hardware (`libuptech.so`)
These require the native C shared library (Raspberry Pi ARM HF). On other platforms they compile as no-ops or return fixed values.

| Location | Item | Effect |
|----------|------|--------|
| `judgers/align.rs` | `make_stage_align_breaker_mpu` | Stage alignment uses ADC fallback only; MPU yaw comparison dead code |
| `judgers/align.rs` | `make_align_direction_breaker_mpu` | Direction alignment uses ADC fallback only; MPU yaw comparison dead code |
| `judgers/mod.rs` | `SensorData::mpu_yaw()` | Trait method exists but no real implementation |
| `compile/fence.rs` | MPU-based stage align path | `use_mpu_align_stage` branch returns `None` breaker |
| `compile/stage.rs` | `make_unclear_zone_handler` | Gray ADC always reads 0.0; unclear zone detection is a no-op |
| `signal_light.rs` | `SigLightRegistry` | Registry + API ported; actual `lcd_set` / `adc_led_set` calls need real hardware |

### Deferred (Python-only tools)
| Item | Reason |
|------|--------|
| `trac` subcommand | Relies on `viztracer` (Python tracing library) |
| `view` subcommand | Relies on `vizviewer` (Python visualization tool) |
| `callbacks.py` | CLI validation pattern tied to `click` (Python CLI framework) |

### Available but not wired
| Location | Item | Notes |
|----------|------|-------|
| `compile/navigation.rs` | `make_rand_turn_handler` | Implemented; not in dispatch table or any mission schema |
| `static_utils.rs` | `get_timestamp()` | Stub returns fixed string; replace with `chrono` for production |

## Key Differences (Python → Rust)

| Concept | Python | Rust |
|---------|--------|------|
| Codegen | `exec()` string generation | Direct graph walk via `HashMap<usize, _>` adjacency |
| Closures | `Callable[[], bool]` lambdas | `Arc<dyn Fn() -> BreakerResult>` — cloneable, shareable |
| Transition conditions | `KT` generic typevar | `BreakerResult` enum (`Bool/Int/Str/Placeholder`) |
| Speed expressions | `exec()`-evaluated Python strings | `SpeedExpr::Fn(Arc<...>)` closures |
| State IDs | Object references | `usize` IDs (Clone derived on MovingState) |

## Installation

```bash
git clone <repository-url>
cd kazu-rs
cargo build --release
```

## Usage

```bash
kazu-rs config init                  # write default configs
kazu-rs viz                          # export all behavior diagrams
kazu-rs viz edge scan -d ./diagrams  # export specific packs
kazu-rs check mot -p COM1            # test motor controller
kazu-rs read adc io -i 0.5           # stream sensor data
kazu-rs ports -c                     # scan available serial ports
kazu-rs cmd -p COM1                  # raw serial command shell
kazu-rs breaker                      # emergency motor stop
kazu-rs msg                          # serial message streaming
kazu-rs light -s                     # interactive light shell
kazu-rs bench siglight               # benchmark signal light
```

For full help: `kazu-rs --help`

## Development

```bash
cargo build                     # build workspace
cargo test                      # run all tests
cargo build -p mentabotix-rs   # build specific crate
cargo run -- viz                # run CLI
```

## License

MIT — see [LICENSE](LICENSE).
