//! Build script for uptechstar-rs.
//!
//! The bundled `libuptech.so` is embedded into the binary at compile time via
//! `include_bytes!` in `ffi.rs`. No further copy steps are needed — the binary
//! is self-contained and extracts the `.so` to a temp cache on first use.

fn main() {
    println!("cargo:rerun-if-changed=lib/libuptech.so");
}
