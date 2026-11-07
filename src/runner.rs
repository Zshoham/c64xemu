//! Batch execution: load an image, run it to its exit point, report what the machine did.
//!
//! Everything about the machine itself - how memory is loaded, the Ethernet and HPI backends,
//! tracing - comes from `MachineArgs`, so this module is only the loop and the report.

use crate::machine::{Machine, MachineArgs, StopCondition, StopReason};
use clap::Args;
use std::collections::VecDeque;
use std::fs;
use std::path::PathBuf;

#[derive(Args)]
pub struct RunArgs {
    /// ELF image to execute
    pub image: PathBuf,
    #[command(flatten)]
    pub machine: MachineArgs,
    /// Give up after this many simulated cycles; 0 runs without a limit
    #[arg(
        long,
        default_value_t = 10_000_000,
        value_name = "CYCLES",
        help_heading = "Batch run"
    )]
    pub max_cycles: u64,
    /// Write the final machine state here instead of on stdout
    #[arg(long, value_name = "FILE", help_heading = "Batch run")]
    pub state_out: Option<PathBuf>,
    /// Print this memory window when the run ends; repeatable
    #[arg(long, value_parser = parse_dump, value_name = "ADDR:LEN", help_heading = "Batch run")]
    pub dump: Vec<(u32, usize)>,
    /// Observe B15 and committed stores within START:END without writing a guest watermark
    #[arg(long, value_parser = parse_stack_window, value_name = "START:END", help_heading = "Batch run")]
    pub stack_window: Option<(u32, u32)>,
    /// Observe B15 and committed stores on every core within START:END
    #[arg(long, value_parser = parse_stack_window, value_name = "START:END", conflicts_with = "stack_window", help_heading = "Batch run")]
    pub stack_window_all: Option<(u32, u32)>,
    /// Keep this many recent execute-packet addresses for fault diagnosis
    #[arg(
        long,
        default_value_t = 0,
        value_name = "N",
        help_heading = "Batch run"
    )]
    pub pc_trail: usize,
    /// Stop as soon as the program counter leaves every loaded segment
    #[arg(long, help_heading = "Batch run")]
    pub trap_oob: bool,
    /// Print an emulated-CPU performance report when the run ends
    #[arg(long, help_heading = "Batch run")]
    pub perf: bool,
}

pub fn run(args: RunArgs) -> Result<u8, String> {
    crate::install_termination_handler();
    let mut machine = Machine::load(&args.image, &args.machine)?;
    if let Some((start, end)) = args.stack_window {
        machine.configure_stack_observation(start, end)?;
    }
    if let Some((start, end)) = args.stack_window_all {
        machine.configure_stack_observation_all(start, end)?;
    }
    if machine.stop_address().is_none() {
        return Err("ELF has no C$$EXIT symbol; pass --stop-pc".into());
    }
    let breakpoints = machine.initial_breakpoints().to_vec();
    let loaded_ranges = machine.loaded_ranges();
    // A short history of executed packet addresses, so a fault can be traced back through the
    // call path that reached it. Only kept when asked for: it costs a push per cycle. The
    // check closure runs once per cycle inside `run`, so it records the trail and watches
    // for a fetch leaving the image in the same hook.
    let mut pc_trail: VecDeque<u32> = VecDeque::new();
    machine.arm_halt(&breakpoints);
    let outcome = if args.pc_trail == 0 && !args.trap_oob {
        // The normal batch path only has a cycle budget. Do not dispatch an inert diagnostic
        // closure on every emulated cycle.
        let mut conditions = [StopCondition::Budget(args.max_cycles)];
        machine.run(&mut conditions)
    } else {
        let mut conditions = [
            StopCondition::Budget(args.max_cycles),
            StopCondition::Check(&mut |machine: &Machine| {
                if args.pc_trail > 0 {
                    if pc_trail.len() == args.pc_trail {
                        pc_trail.pop_front();
                    }
                    let pc = machine.pc();
                    if pc_trail.back() != Some(&pc) {
                        pc_trail.push_back(pc);
                    }
                }
                if args.trap_oob {
                    let pc = machine.pc();
                    let inside = loaded_ranges
                        .iter()
                        .any(|(start, end)| pc >= *start && pc < *end);
                    if !inside {
                        eprintln!("fetch left the image at {}", machine.describe(pc));
                        if let Some(previous) = pc_trail.back() {
                            eprintln!("  came from {}", machine.describe(*previous));
                        }
                        for entry in pc_trail.iter().rev().take(12) {
                            eprintln!("  trail {}", machine.describe(*entry));
                        }
                        return Some(StopReason::OutOfBounds);
                    }
                }
                None
            }),
        ];
        machine.run(&mut conditions)
    };
    report(&mut machine, &args, outcome, &pc_trail)
}

/// Everything after the run, whatever ended it: the dumps, the fault, the captures `finish`
/// writes and the state report are all produced even when the run itself failed or `finish`
/// did, because those are what tell a failed run apart. Only then is the one error returned
/// chosen, first that applies: the run's own error, a latched fault, `finish`'s error, the
/// state write's, the stop reason's message, and last the replay file's exit-before-EOF
/// verdict. Every other error is printed rather than lost.
fn report(
    machine: &mut Machine,
    args: &RunArgs,
    outcome: Result<StopReason, String>,
    pc_trail: &VecDeque<u32>,
) -> Result<u8, String> {
    for (address, length) in &args.dump {
        let bytes = machine.read_memory(*address, *length);
        eprintln!("dump 0x{address:08x}:");
        for (index, chunk) in bytes.chunks(16).enumerate() {
            let words: Vec<String> = chunk
                .chunks(4)
                .map(|word| {
                    let mut value = 0u32;
                    for (position, byte) in word.iter().enumerate() {
                        value |= u32::from(*byte) << (8 * position);
                    }
                    format!("{value:08x}")
                })
                .collect();
            eprintln!(
                "  0x{:08x}  {}",
                address + (index * 16) as u32,
                words.join(" ")
            );
        }
    }
    let fault = machine.fault().map(str::to_owned);
    if let Some(reason) = &fault {
        eprintln!("fault: {reason}");
        if !pc_trail.is_empty() {
            eprintln!("last {} execute packets:", pc_trail.len());
            for pc in pc_trail {
                eprintln!("  {}", machine.describe(*pc));
            }
        }
    }
    let exit_code = machine.exit_code();
    // Rendered after `finish`, so the report reflects the flushed TX archive.
    let finished = machine.finish();
    let written = write_state(machine, args);
    if args.perf {
        eprintln!("{}", machine.perf_report());
    }
    let mut errors = Vec::new();
    let reason = match outcome {
        Ok(reason) => Some(reason),
        Err(error) => {
            errors.push(error);
            None
        }
    };
    errors.extend(fault.clone());
    errors.extend(finished.err());
    errors.extend(written.err());
    // A fault already says why the run stopped; a run error leaves no reason to explain.
    if let (Some(reason), None) = (reason, &fault) {
        errors.extend(stop_verdict(machine, args, reason));
    }
    let mut errors = errors.into_iter();
    match errors.next() {
        Some(first) => {
            for secondary in errors {
                eprintln!("c64xemu: {secondary}");
            }
            Err(first)
        }
        None => Ok(exit_code),
    }
}

/// What a stop that was not a fault says about the run: nothing for a clean exit that took the
/// whole replay file (if any), or a SIGTERM; otherwise why it is a failure. The state dump is
/// already written, so the message only has to say why it stopped.
fn stop_verdict(machine: &Machine, args: &RunArgs, reason: StopReason) -> Option<String> {
    match reason {
        // The program finished, but the host it was given had more to say: a batch run that
        // did not reach the end of its replay file did not test what the file describes.
        StopReason::Exit => machine
            .hpi_replay_pending_line()
            .map(|line| format!("hpi replay: program exited before line {line}")),
        StopReason::Breakpoint(hit) => Some(format!(
            "stopped at breakpoint {} on core {}",
            machine.describe(hit.address),
            hit.core
        )),
        StopReason::OutOfBounds => {
            Some(format!("fetch left the image at PC {:#010x}", machine.pc()))
        }
        StopReason::Terminated => {
            eprintln!("c64xemu: terminated by SIGTERM");
            None
        }
        StopReason::Fault => None,
        StopReason::Budget
        | StopReason::Packets
        | StopReason::Interrupted
        | StopReason::Watchpoint => Some(format!(
            "cycle limit {} reached at PC {:#010x}",
            args.max_cycles,
            machine.pc()
        )),
    }
}

/// The state report, to `--state-out` or stdout.
fn write_state(machine: &mut Machine, args: &RunArgs) -> Result<(), String> {
    let state = machine.state_json(args.machine.profile);
    let rendered = serde_json::to_vec_pretty(&state).map_err(|error| error.to_string())?;
    if let Some(path) = &args.state_out {
        fs::write(path, rendered).map_err(|error| format!("{}: {error}", path.display()))
    } else {
        println!("{}", String::from_utf8(rendered).unwrap());
        Ok(())
    }
}

/// `ADDR:LEN`: a memory window to print when the run ends.
fn parse_dump(text: &str) -> Result<(u32, usize), String> {
    let (address, length) = text
        .split_once(':')
        .ok_or_else(|| "expected ADDR:LEN".to_string())?;
    Ok((
        crate::parse_address(address)?,
        length
            .parse()
            .map_err(|_| format!("invalid length {length}"))?,
    ))
}

fn parse_stack_window(text: &str) -> Result<(u32, u32), String> {
    let (start, end) = text
        .split_once(':')
        .ok_or_else(|| "expected START:END".to_string())?;
    let start = crate::parse_address(start)?;
    let end = crate::parse_address(end)?;
    crate::stack_observer::StackMemoryObservation::new(start, end)?;
    Ok((start, end))
}

#[cfg(test)]
mod tests {
    use super::report;
    use crate::cpu::Cpu;
    use crate::host::{HostConnections, Wiring};
    use crate::machine::{Machine, StopCondition, StopReason};
    use crate::netio::EthernetFiles;
    use crate::{Cli, Command};
    use clap::Parser;
    use std::collections::VecDeque;
    use std::path::{Path, PathBuf};

    /// A scratch directory for one test's state file and captures.
    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let path =
                std::env::temp_dir().join(format!("c64xemu-runner-{}-{name}", std::process::id()));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).unwrap();
            Self(path)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// A machine whose `finish` fails: its `--pcap-out` names a directory.
    fn machine_whose_finish_fails(directory: &Path) -> Machine {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.attach_host(
            HostConnections::open(Wiring {
                net_poll: 1,
                net_port: 0,
                net: EthernetFiles {
                    pcap_out: Some(directory),
                    ..EthernetFiles::default()
                },
                hpi: None,
                utopia: None,
            })
            .unwrap(),
        );
        machine
    }

    fn state_args(scratch: &Scratch) -> (super::RunArgs, PathBuf) {
        let state = scratch.0.join("state.json");
        (
            run_args(&["--state-out", state.to_str().unwrap()]).unwrap(),
            state,
        )
    }

    /// A run that failed still finishes and writes its state, and its error is the one returned.
    #[test]
    fn a_run_error_still_writes_the_state_and_wins() {
        let scratch = Scratch::new("run-error");
        let (args, state) = state_args(&scratch);
        let mut machine = machine_whose_finish_fails(&scratch.0);
        let outcome = Err("unsupported CIO command 0xee".to_string());
        assert_eq!(
            report(&mut machine, &args, outcome, &VecDeque::new()),
            Err("unsupported CIO command 0xee".to_string())
        );
        assert!(state.is_file());
    }

    /// A `finish` failure no longer loses the state report, and a fault outranks it.
    #[test]
    fn a_finish_error_still_writes_the_state_and_a_fault_outranks_it() {
        let scratch = Scratch::new("finish-error");
        let (args, state) = state_args(&scratch);
        let mut machine = machine_whose_finish_fails(&scratch.0);
        machine.device.cpu_mut().write_words(0, &[0; 8]);
        let reason = machine.run(&mut [StopCondition::Budget(4)]);
        let error = report(&mut machine, &args, reason, &VecDeque::new()).unwrap_err();
        assert!(
            error.starts_with(&scratch.0.display().to_string()),
            "{error}"
        );
        assert!(state.is_file());

        std::fs::remove_file(&state).unwrap();
        let mut machine = machine_whose_finish_fails(&scratch.0);
        // An instruction word the device does not have: the model refuses to run it.
        machine.device.cpu_mut().write_words(0, &[0, 0x5645_4314]);
        let reason = machine.run(&mut []);
        assert_eq!(reason, Ok(StopReason::Fault));
        let fault = machine.fault().unwrap().to_owned();
        assert_eq!(
            report(&mut machine, &args, reason, &VecDeque::new()),
            Err(fault)
        );
        let written: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&state).unwrap()).unwrap();
        assert!(written["fault"].is_string());
    }

    fn run_args(extra: &[&str]) -> Result<super::RunArgs, clap::Error> {
        let mut argv = vec!["c64xemu", "run", "image.out"];
        argv.extend_from_slice(extra);
        match Cli::try_parse_from(argv)?.command {
            Command::Run(args) => Ok(args),
            _ => unreachable!(),
        }
    }

    /// The replay file is the HPI host, so it conflicts with every option of the socket host;
    /// `--hpi-poll`'s default is not an explicit option and does not.
    #[test]
    fn an_hpi_replay_file_excludes_the_socket_host() {
        let replay = run_args(&["--hpi-replay", "host.csv"]).unwrap();
        assert_eq!(
            replay.machine.hpi_replay.as_deref(),
            Some(std::path::Path::new("host.csv"))
        );
        for socket in [
            ["--hpi-unix", "hpi.sock"],
            ["--hpi-peer", "peer.sock"],
            ["--hpi-poll", "1024"],
        ] {
            let mut argv = vec!["--hpi-replay", "host.csv"];
            argv.extend(socket);
            assert!(run_args(&argv).is_err(), "{socket:?}");
        }
    }

    #[test]
    fn stack_windows_parse_and_are_mutually_exclusive() {
        let single = run_args(&["--stack-window", "0x00801000:0x00802000"]).unwrap();
        assert_eq!(single.stack_window, Some((0x0080_1000, 0x0080_2000)));
        assert_eq!(single.stack_window_all, None);
        let all = run_args(&["--stack-window-all", "4096:8192"]).unwrap();
        assert_eq!(all.stack_window_all, Some((0x1000, 0x2000)));
        assert_eq!(all.stack_window, None);
        assert!(run_args(&[
            "--stack-window",
            "0x1000:0x2000",
            "--stack-window-all",
            "0x1000:0x2000",
        ])
        .is_err());
        for bad in [
            "0x2000:0x1000",
            "0x1000:0x1000",
            "0x1002:0x2000",
            "0x1000:0x2002",
            "0x1000",
        ] {
            assert!(run_args(&["--stack-window", bad]).is_err(), "{bad}");
            assert!(run_args(&["--stack-window-all", bad]).is_err(), "{bad}");
        }
    }
}
