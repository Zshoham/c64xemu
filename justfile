set shell := ["bash", "-euc"]

# List the supported developer commands.
default:
    @just --list

# Build the optimized emulator used by the conformance suite.
build *args:
    cargo build --release --locked {{args}}

# Build the firmware images the conformance suites load (provisions tests/build/ and builds
# the EABI NDK libraries on demand; see tests/tools/fetch_deps.sh and tests/tools/build_ndk_libs.sh).
firmware:
    ./tests/firmware/bios_kernel/build.sh
    ./tests/firmware/ndk_tcp/build.sh

# Run the emulator without spelling out Cargo's release separator.
run *args:
    cargo run --release --locked -- {{args}}

# Run native unit and component conformance tests with nextest.
test-rust *args:
    cargo nextest run --all-targets --all-features --locked {{args}}

# Run the complete emulator conformance suite with pytest.
test-conformance *args:
    python3 -m pytest {{args}}

# Run every Rust and emulator conformance test.
test: test-rust test-conformance

# Run the tests intended for the edit-compile-test loop.
test-fast: test-rust

# Format Rust sources.
fmt:
    cargo fmt --all

# Verify formatting without changing files.
fmt-check:
    cargo fmt --all -- --check

# Treat every Clippy diagnostic as a build failure.
lint:
    cargo clippy --all-targets --all-features --locked -- -D warnings

# Fast local quality gate.
check: fmt-check lint test-fast

# Complete quality gate used before merging.
ci: fmt-check lint test

# Remove Cargo build artifacts.
clean:
    cargo clean
