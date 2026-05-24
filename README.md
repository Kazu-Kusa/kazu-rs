# kazu-rs

Rust port of [kazu](https://github.com/kcirJxek/kazu) — A Dedicated Robots Control System for competition robots.

## Architecture

```
kazu-rs/                  CLI + config + compile (15 CLI commands)
├── bdmc-rs/              Close-loop motor controller + serial I/O
├── mentabotix-rs/        State-machine graph engine + PlantUML export
├── upic-rs/              AprilTag detection (bindings to native vision lib)
├── uptechstar-rs/        HW bindings: sensors, screen, LEDs (native FFI)
```

## Porting Status

### CLI Commands (Python `cli.py` 68KB → Rust `src/{cli,config,commands/}` modularized)

| Command | Status | Notes |
|---------|--------|-------|
| `config` | ✅ | Read/write app + run TOML configs |
| `run`    | ⚠️  | Execution loop works; missions are `Vec<String>` stubs (no DSL parser) |
| `check`  | ✅ | Tests mot/adc/io/mpu/cam (matches Python) |
| `read`   | ✅ | Stream sensor data to terminal/screen |
| `viz`    | ⚠️  | Dispatch table wired (12 packs); `edge` handler has real graph; 11 stubs |
| `cmd`    | ✅ | Raw serial command shell |
| `ports`  | ✅ | Scan + probe COM ports |
| `msg`    | ✅ | Serial message streaming shell |
| `light`  | ⚠️  | Shell mode only; no real LED output without native lib |
| `tag`    | ✅ | AprilTag camera detection via upic-rs |
| `breaker`| ✅ | Emergency motor stop |
| `bench`  | ⚠️  | Siglight/Sleep only; missing Adc/App benchmarks |
| `trac`   | 🚫 | Requires `viztracer` (Python-only) — deferred |
| `view`   | 🚫 | Requires `vizviewer` (Python-only) — deferred |
| `record` | ✅ | Sensor data to CSV; Enter to start/stop |

### Library Crates

| Python module | Rust crate | Status | Notes |
|---------------|-----------|--------|-------|
| `mentabotix/` (158KB) | `mentabotix-rs/` | ✅ | Core graph engine: Botix, MovingState, MovingTransition, Menta, Composer, PlantUML export. No codegen — direct graph walk instead of Python `exec()`. |
| `bdmc/` (17KB) | `bdmc-rs/` | ✅ | CloseLoopController, serial I/O, port discovery |
| `upic/` (31KB) | `upic-rs/` | ✅ | TagDetector with native bindings |
| `pyuptech/` (37KB) | `uptechstar-rs/` | ✅ | Sensors (MPU, ADC, IO), LCD screen, FFI loader |

### kazu Application Layer (Python `kazu/` 15 modules, ~300KB)

| Module (KB) | Rust status | Notes |
|-------------|-------------|-------|
| `cli.py` (68) | `src/{cli,config,commands/}` | 13/15 commands ported; 2 deferred (Python-only tools) |
| `compile.py` (68) | `compile.rs` (4) | 1/14 handlers real (`edge`); 13 stubs |
| `config.py` (32) | `config.rs` | `AppConfig`/`RunConfig` partial; `ContextVar`, `TagGroup`, `EdgeConfig`, `ScanConfig`, etc. not ported |
| `judgers.py` (47) | ❌ | 40+ `Breakers.*` predicate factories — none ported |
| `constant.py` (10) | `constant.rs` | ✅ | CodeSign enums, Axis, Attitude, weight tables |
| `signal_light.py` (6) | ❌ | LED registry + lifecycle |
| `hardwares.py` (4) | partial | `SamplerIndexes` partially in `uptechstar-rs`; singleton wiring not ported |
| `assembly.py` (5) | ❌ | Mission assembly: `assembly_AFG_schema`, `assembly_standard_schema` |
| `callbacks.py` (10) | ❌ | CLI validation callbacks |
| `checkers.py` (3) | partial | Inline in `cmd_check` |
| `static.py` (2) | ❌ | `continues_state`, `make_query_table` |
| `logger.py` (1) | N/A | Replaced by `log` crate |
| `visualize.py` (1) | N/A | Inline in `cmd_viz` |

### Key Differences (Python → Rust)

| Concept | Python | Rust |
|---------|--------|------|
| Codegen | `exec()` string generation | Direct graph walk via `HashMap<usize, _>` adjacency |
| Closures | `Callable[[], bool]` lambdas | `Arc<dyn Fn(&Context) -> i32>` |
| Transition conditions | `KT` generic typevar | `BreakerResult` enum (`Bool/Int/Str/Placeholder`) |
| Speed expressions | `exec()`-evaluated Python strings | `SpeedExpr::Fn(Arc<...>)` closures |
| State IDs | Object references | `usize` IDs (no cloning) |

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
cargo test                      # run all tests (31 suites, 0s)
cargo build -p mentabotix-rs   # build specific crate
cargo run -- viz                # run CLI
```

## License

MIT — see [LICENSE](LICENSE).
