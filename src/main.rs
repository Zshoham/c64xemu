mod board;
mod boot;
mod bootctl;
mod cache;
mod chipcfg;
mod cio;
#[cfg(test)]
mod component_conformance;
#[cfg(test)]
include!(concat!(env!("OUT_DIR"), "/component_case_tests.rs"));
mod control;
mod cpu;
mod ddr2;
mod debugger;
mod decode_cli;
mod decoder;
mod devcfg;
mod device;
mod edma;
mod edma_tc;
mod elf;
mod emac;
mod emic;
mod emifa;
mod events;
mod gdb;
mod gpio;
mod host;
mod hpi;
mod hpi_replay;
mod i2c;
mod i2cboot;
mod idma;
mod machine;
mod mcbsp;
mod mdio;
mod memory;
mod netio;
mod pcap;
mod pdma;
mod pll;
mod port;
mod psc;
mod romboot;
mod runner;
mod smc;
mod space;
mod sploop;
mod sysclk3;
mod system;
mod timer;
mod trace;
mod ui;
mod utopia;
mod utopiaboot;
mod view;
mod writeback;

use clap::{Parser, Subcommand};
use std::process::ExitCode;

/// C64x+ DSP emulator: batch execution, tracing, an interactive debugger with a browser
/// dashboard, and a GDB remote stub.
#[derive(Parser)]
#[command(name = "c64xemu", version, propagate_version = true)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

// The subcommand tables differ in size because `run` carries the most options; the value
// exists for one `match` at startup, so boxing it would buy nothing. clap's `Subcommand`
// derive cannot see through a `Box` in any case.
#[allow(clippy::large_enum_variant)]
#[derive(Subcommand)]
enum Command {
    /// Decode instruction words without loading an image
    Decode(decode_cli::DecodeArgs),
    /// Load an ELF image and execute it
    Run(runner::RunArgs),
    /// Debug an ELF image: a console REPL, or the browser dashboard or GDB stub it can serve
    Debug(debugger::DebugArgs),
}

/// Numbers on the command line: decimal, or hexadecimal with a `0x` prefix.
pub fn parse_address(text: &str) -> Result<u32, String> {
    match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u32::from_str_radix(hex, 16).map_err(|_| format!("invalid address {text}")),
        None => text.parse().map_err(|_| format!("invalid address {text}")),
    }
}

/// SIGTERM asks the batch runner to stop at the next batch boundary and report its counters
/// before exiting, so a run that is cut short still produces its perf report. Only the batch
/// runner installs this; the interactive debugger keeps its own control of the terminal.
pub(crate) fn install_termination_handler() {
    let flag = crate::machine::termination_flag();
    signal_hook::flag::register(signal_hook::consts::SIGTERM, flag)
        .expect("install SIGTERM handler");
    // SIGINT stays at its default so Ctrl-C keeps its meaning in every other mode.
}

fn main() -> ExitCode {
    let result = match Cli::parse().command {
        Command::Decode(args) => decode_cli::run(args),
        Command::Run(args) => runner::run(args),
        Command::Debug(args) => debugger::debug(args),
    };
    match result {
        Ok(code) => ExitCode::from(code),
        Err(error) => {
            eprintln!("c64xemu: {error}");
            ExitCode::FAILURE
        }
    }
}
