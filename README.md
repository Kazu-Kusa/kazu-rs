# kazu-rs

A Rust workspace project containing multiple utility crates for various operations.

## Overview

kazu-rs is a multi-crate workspace that provides functionality through several specialized libraries:

- **bdmc-rs** - Core functionality crate
- **upic-rs** - Image/picture utilities
- **mentabotix-rs** - Additional utility functions

## Features

- Command-line interface with `clap`
- Configuration management with `figment` (supports TOML and environment variables)
- Progress indicators with `indicatif`
- Parallel processing with `rayon`
- Comprehensive logging with `log`

## Installation

```bash
git clone <repository-url>
cd kazu-rs
cargo build --release
```

## Usage

```bash
cargo run -- [OPTIONS]
```

For detailed usage information:

```bash
cargo run -- --help
```

## Configuration

The application supports configuration through:

- TOML configuration files
- Environment variables

## Development

This project uses a Cargo workspace structure. To work on individual crates:

```bash
# Build all crates
cargo build

# Run tests
cargo test

# Build specific crate
cargo build -p bdmc-rs
```

## License

this project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.