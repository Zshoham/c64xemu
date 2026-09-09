//! Interactive debugging: a console REPL, the browser dashboard, or the GDB stub - three front
//! ends onto one machine, selected by flag rather than by subcommand, because the session they
//! drive is identical.
//!
//! Breakpoints halt in the CPU at a fetch boundary - the same contract the runner uses for
//! C$$EXIT and the CIO handshake - so a branch that resolves straight onto one still stops.
//! `stepi` advances one dispatched execute packet, `stepc` one cycle, and every inspection
//! reads through the host view, so looking at memory neither fills a cache line nor moves a
//! replacement clock.

use crate::machine::{BreakpointHit, Machine, MachineArgs, StopCondition, StopReason};
use crate::{gdb, ui};
use clap::Args;
use std::io::{self, Write};
use std::path::Path;
use std::path::PathBuf;

#[derive(Args)]
pub struct DebugArgs {
    /// ELF image to debug
    pub image: PathBuf,
    #[command(flatten)]
    pub machine: MachineArgs,
    /// Cycle budget for one `continue`; 0 runs without a limit
    #[arg(
        long,
        default_value_t = 1_000_000_000,
        value_name = "CYCLES",
        help_heading = "Session"
    )]
    pub max_cycles: u64,
    /// Serve the browser dashboard instead of the console REPL
    #[arg(
        long,
        num_args = 0..=1,
        default_missing_value = "127.0.0.1:8080",
        value_name = "ADDR",
        conflicts_with = "gdb",
        help_heading = "Session"
    )]
    pub ui: Option<String>,
    /// Serve the GDB remote serial protocol instead of the console REPL
    #[arg(
        long,
        num_args = 0..=1,
        default_missing_value = "127.0.0.1:1234",
        value_name = "ADDR",
        help_heading = "Session"
    )]
    pub gdb: Option<String>,
}

pub fn debug(args: DebugArgs) -> Result<u8, String> {
    let machine = Machine::load(&args.image, &args.machine)?;
    // One session, three front ends. They are exclusive rather than concurrent: both the
    // dashboard and the stub own the breakpoint set and the run loop, and sharing those
    // between two clients would need a design of its own.
    if let Some(listen) = &args.ui {
        return ui::serve(machine, listen);
    }
    if let Some(listen) = &args.gdb {
        return gdb::serve(machine, listen, args.max_cycles);
    }
    let mut machine = machine;
    machine.set_issue_view(true);
    let breakpoints = machine
        .initial_breakpoints()
        .iter()
        .map(|address| Breakpoint {
            address: *address,
            label: machine.symbol_at(*address),
        })
        .collect();
    let mut debugger = Debugger {
        machine,
        breakpoints,
        watchpoints: Vec::new(),
        max_cycles: args.max_cycles,
        profile: args.machine.profile,
        quit: false,
    };
    debugger.sync_halt_addresses();
    debugger.repl();
    debugger.machine.finish()?;
    Ok(0)
}

struct Breakpoint {
    address: u32,
    label: Option<String>,
}

struct Watchpoint {
    bank: char,
    index: usize,
    last: u32,
}

struct Debugger {
    machine: Machine,
    breakpoints: Vec<Breakpoint>,
    watchpoints: Vec<Watchpoint>,
    max_cycles: u64,
    /// Whether the hotspot histogram is being kept, so `state` can report it.
    profile: bool,
    quit: bool,
}

impl Debugger {
    /// The halt list the CPU enforces: the user's breakpoints plus the program's exit point,
    /// so `continue` stops when the program ends rather than running into whatever follows.
    fn sync_halt_addresses(&mut self) {
        let breakpoints: Vec<u32> = self
            .breakpoints
            .iter()
            .map(|breakpoint| breakpoint.address)
            .collect();
        self.machine.arm_halt(&breakpoints);
    }

    fn repl(&mut self) {
        let stdin = io::stdin();
        println!("c64xemu debugger - 'help' for commands");
        self.where_now();
        while !self.quit {
            print!("(c64xemu) ");
            let _ = io::stdout().flush();
            let mut line = String::new();
            match stdin.read_line(&mut line) {
                Ok(0) => break,
                Ok(_) => {}
                Err(error) => {
                    eprintln!("read error: {error}");
                    break;
                }
            }
            let tokens = line.split_whitespace().collect::<Vec<_>>();
            if tokens.is_empty() {
                continue;
            }
            if let Err(error) = self.dispatch(&tokens) {
                println!("error: {error}");
            }
        }
    }

    fn dispatch(&mut self, tokens: &[&str]) -> Result<(), String> {
        match tokens[0] {
            "help" | "?" => self.help(),
            "quit" | "q" => self.quit = true,
            "info" | "i" => self.info(),
            "core" => self.core(tokens.get(1).copied())?,
            "regs" => self.regs(tokens.get(1).copied()),
            "reg" => self.set_register(tokens)?,
            "mem" | "x" => self.mem(tokens)?,
            "disas" | "d" => self.disas(tokens)?,
            "syms" => self.syms(tokens.get(1).copied()),
            "break" | "b" => self.break_command(tokens)?,
            "delete" => self.delete(tokens)?,
            "watch" => self.watch(tokens)?,
            "unwatch" => self.unwatch(tokens)?,
            "continue" | "c" => self.continue_run(tokens)?,
            "stepc" => self.step_cycles(tokens)?,
            "stepi" | "s" => self.step_packets(tokens)?,
            "cache" => self.cache(),
            "state" => self.state(tokens)?,
            "reset" => {
                self.machine.reload()?;
                self.where_now();
            }
            "trace" => self.trace_command(tokens)?,
            "write" => self.write_memory(tokens)?,
            other => return Err(format!("unknown command {other}; try 'help'")),
        }
        Ok(())
    }

    /// Which core the session is driving. Every other command - registers, memory, stepping,
    /// breakpoints - acts on it, so a six-core device is inspected one core at a time
    /// (SPRS612G section 2.4.1: the cores are independent, including which of them is running).
    fn core(&mut self, argument: Option<&str>) -> Result<(), String> {
        if let Some(token) = argument {
            let core: usize = token
                .parse()
                .map_err(|_| format!("expected a core number, got {token}"))?;
            if core >= self.machine.cores() {
                return Err(format!(
                    "core {core} does not exist; this device has {}",
                    self.machine.cores()
                ));
            }
            self.machine.set_focus(core);
        }
        for core in self.machine.snapshot().cores {
            let marker = if core.focused { '*' } else { ' ' };
            let state = if core.held_in_reset {
                "held in reset".to_string()
            } else if core.halted {
                "halted".to_string()
            } else {
                format!("running, {} cycles", core.cycles)
            };
            println!("{marker} core {}  pc {:#010x}  {state}", core.core, core.pc);
        }
        Ok(())
    }

    fn help(&self) {
        println!(
            "info                     machine state summary\n\
             core [N]                 show the device's cores, or drive core N\n\
             regs [a|b|c]             register dump (default both banks)\n\
             reg NAME VALUE           set an A or B register\n\
             mem ADDR [LEN]           hexdump memory (host view)\n\
             disas [ADDR] [N]         disassemble N instructions (default from the PC)\n\
             syms [PATTERN]           list symbols\n\
             break [ADDR|SYM]         set or list breakpoints (halt at fetch)\n\
             delete ID|all            remove breakpoints\n\
             watch REG                stop when a register changes\n\
             unwatch ID|all           remove watchpoints\n\
             continue [CYCLES]        run until breakpoint, watch, fault or budget\n\
             stepc [N]                run N cycles\n\
             stepi [N]                dispatch N execute packets\n\
             cache                    cache geometry and resident lines\n\
             state [FILE]             the JSON machine state `run` reports\n\
             reset                    reload the image and start over\n\
             trace on FILE [json]     start tracing to FILE\n\
             trace off                stop tracing (flushes)\n\
             write ADDR HEXBYTES      patch memory\n\
             quit"
        );
    }

    fn info(&mut self) {
        // One snapshot: everything printed below describes the same instant, which it could not
        // be relied on to do while each line asked the machine a fresh question.
        let state = self.machine.snapshot();
        let pc = state.core.pc;
        println!(
            "pc=0x{pc:08x}{} cycle={} packets={} fault={:?}",
            self.machine
                .symbol_at(pc)
                .map(|name| format!(" <{name}>"))
                .unwrap_or_default(),
            state.cycles,
            state.packets_executed,
            state.fault.as_deref()
        );
        let csr = state.core.control[0x01];
        println!(
            "csr=0x{csr:08x} (gie={}) ierr=0x{:08x} ifr=0x{:04x} ier=0x{:04x}",
            csr & 1,
            state.core.control[0x1f],
            state.core.control[0x02],
            state.core.control[0x04]
        );
        match &state.sploop {
            Some(dump) => println!(
                "sploop ii={} {} started={} total={}",
                dump.ii,
                if dump.filling { "filling" } else { "replaying" },
                dump.started,
                dump.total
                    .map_or_else(|| "?".into(), |total| total.to_string())
            ),
            None => println!("sploop inactive"),
        }
        let stats = state.stats;
        println!(
            "packets={} retired={} replays={} branches={} interrupts={} stores={} faults={}",
            stats.packets,
            stats.retired,
            stats.replay_packets,
            stats.branches_taken,
            stats.interrupts,
            stats.stores,
            stats.faults
        );
        for issue in &state.last_issue {
            // An instruction that claims no unit - NOP, SPMASK, SPKERNEL - has nothing to
            // name, so it is listed without a bogus `.?1`.
            let unit = if issue.unit == '?' {
                String::new()
            } else {
                format!(
                    ".{}{}{} ",
                    issue.unit,
                    if issue.side_a { 1 } else { 2 },
                    if issue.cross { "x" } else { "" }
                )
            };
            let path = match issue.mem_side_a {
                Some(true) => "t1 ",
                Some(false) => "t2 ",
                None => "",
            };
            let replayed = if issue.replayed { "(replay) " } else { "" };
            println!("  {unit}{path}{replayed}{}", issue.text);
        }
        if !self.breakpoints.is_empty() {
            println!("breakpoints:");
            for (id, breakpoint) in self.breakpoints.iter().enumerate() {
                println!("  {id}: {}", describe(breakpoint));
            }
        }
        for (id, watch) in self.watchpoints.iter().enumerate() {
            println!("watch {id}: {}{}", watch.bank, watch.index);
        }
    }

    fn regs(&mut self, which: Option<&str>) {
        match which.unwrap_or("all") {
            "a" | "A" => self.dump_bank('A'),
            "b" | "B" => self.dump_bank('B'),
            "c" | "control" => self.dump_control(),
            "all" => {
                self.dump_bank('A');
                self.dump_bank('B');
            }
            other => println!("unknown register set {other}"),
        }
    }

    fn dump_bank(&mut self, bank: char) {
        let snapshot = self.machine.snapshot();
        let file = if bank == 'A' {
            snapshot.core.a
        } else {
            snapshot.core.b
        };
        for row in 0..8 {
            print!("{}{:>2}:", bank.to_ascii_lowercase(), row * 4);
            for column in 0..4 {
                let index = row * 4 + column;
                print!(" {index:>2}={:08x}", file[index]);
            }
            println!();
        }
    }

    fn dump_control(&mut self) {
        let snapshot = self.machine.snapshot();
        for register in 0..=0x1fu8 {
            if let Some(name) = crate::control::register_name(register) {
                println!(
                    "{name:>6} = 0x{:08x}",
                    snapshot.core.control[register as usize]
                );
            }
        }
    }

    fn set_register(&mut self, tokens: &[&str]) -> Result<(), String> {
        let [_, name, value] = tokens else {
            return Err("usage: reg NAME VALUE".into());
        };
        let value = parse_u32(value)?;
        self.machine.set_register(name, value)?;
        println!("{name} = 0x{value:08x}");
        Ok(())
    }

    fn mem(&mut self, tokens: &[&str]) -> Result<(), String> {
        let address = match tokens.get(1) {
            Some(token) => self.machine.resolve(token)?,
            None => self.machine.pc() & !0xf,
        };
        let length = match tokens.get(2) {
            Some(text) => text.parse::<usize>().map_err(|_| "invalid LEN")?,
            None => 64,
        }
        .min(4096);
        let bytes = self.machine.read_memory(address, length);
        for (row, chunk) in bytes.chunks(16).enumerate() {
            print!("{:08x} ", address.wrapping_add((row * 16) as u32));
            for byte in chunk {
                print!("{byte:02x} ");
            }
            for _ in chunk.len()..16 {
                print!("   ");
            }
            print!(" ");
            for byte in chunk {
                print!(
                    "{}",
                    if (0x20..0x7f).contains(byte) {
                        *byte as char
                    } else {
                        '.'
                    }
                );
            }
            println!();
        }
        Ok(())
    }

    fn disas(&mut self, tokens: &[&str]) -> Result<(), String> {
        let mut address = match tokens.get(1) {
            Some(token) => self.machine.resolve(token)?,
            None => self.machine.pc(),
        };
        let count = match tokens.get(2) {
            Some(text) => text.parse::<usize>().map_err(|_| "invalid N")?,
            None => 12,
        }
        .min(256);
        let pc = self.machine.pc();
        for _ in 0..count {
            let (text, next) = self.machine.disassemble(address);
            let label = self
                .machine
                .symbols_by_address()
                .iter()
                .find(|(symbol, _)| *symbol == address)
                .map(|(_, name)| format!(" <{name}>"))
                .unwrap_or_default();
            let marker = if address == pc { "=>" } else { "  " };
            println!("{marker} {address:08x}{label}: {text}");
            if next <= address {
                break;
            }
            address = next;
        }
        Ok(())
    }

    fn cache(&mut self) {
        for view in self.machine.cache_views() {
            let bytes = view.capacity_lines * view.line_size as usize;
            println!(
                "{}: {} {} lines of {} B in {} ways ({} resident){}",
                view.name,
                if view.enabled { "enabled" } else { "disabled" },
                view.capacity_lines,
                view.line_size,
                view.ways,
                view.lines.len(),
                if view.frozen { ", frozen" } else { "" }
            );
            println!("     {bytes} B total");
            for line in view.lines.iter().take(16) {
                println!(
                    "     set {:>4}  0x{:08x}  {}",
                    line.set,
                    line.address,
                    if line.dirty { "dirty" } else { "clean" }
                );
            }
            if view.lines.len() > 16 {
                println!("     ... {} more", view.lines.len() - 16);
            }
        }
    }

    fn syms(&self, pattern: Option<&str>) {
        for (address, name) in self.machine.symbols_by_address() {
            if pattern.map_or(true, |pattern| name.contains(pattern)) {
                println!("{address:08x} {name}");
            }
        }
    }

    /// Writes the JSON state a batch run reports, to a file or to the terminal.
    fn state(&mut self, tokens: &[&str]) -> Result<(), String> {
        let profile = self.profile;
        let state = self.machine.state_json(profile);
        let rendered = serde_json::to_vec_pretty(&state).map_err(|error| error.to_string())?;
        match tokens.get(1) {
            Some(path) => {
                std::fs::write(path, rendered).map_err(|error| format!("{path}: {error}"))?;
                println!("wrote {path}");
            }
            None => println!("{}", String::from_utf8(rendered).unwrap()),
        }
        Ok(())
    }

    fn break_command(&mut self, tokens: &[&str]) -> Result<(), String> {
        if tokens.len() == 1 {
            for (id, breakpoint) in self.breakpoints.iter().enumerate() {
                println!("{id}: {}", describe(breakpoint));
            }
            return Ok(());
        }
        let addresses = tokens[1..]
            .iter()
            .map(|token| self.machine.resolve(token))
            .collect::<Result<Vec<_>, _>>()?;
        for address in addresses {
            let label = self.machine.symbol_at(address);
            self.breakpoints.push(Breakpoint { address, label });
            println!(
                "breakpoint {}: {}",
                self.breakpoints.len() - 1,
                describe(self.breakpoints.last().unwrap())
            );
        }
        self.sync_halt_addresses();
        Ok(())
    }

    fn delete(&mut self, tokens: &[&str]) -> Result<(), String> {
        let target = tokens.get(1).ok_or("usage: delete ID|all")?;
        if *target == "all" {
            let changed = !self.breakpoints.is_empty();
            self.breakpoints.clear();
            println!("breakpoints cleared");
            if changed {
                self.sync_halt_addresses();
            }
            return Ok(());
        }
        let id: usize = target.parse().map_err(|_| "invalid breakpoint id")?;
        if id >= self.breakpoints.len() {
            return Err(format!("no breakpoint {id}"));
        }
        let removed = self.breakpoints.remove(id);
        println!("removed breakpoint at 0x{:08x}", removed.address);
        self.sync_halt_addresses();
        Ok(())
    }

    fn watch(&mut self, tokens: &[&str]) -> Result<(), String> {
        let name = tokens.get(1).ok_or("usage: watch REG")?;
        let (bank, index) = parse_register_name(name)?;
        let last = self.machine.register(bank, index);
        self.watchpoints.push(Watchpoint { bank, index, last });
        println!(
            "watch {}: {bank}{index} = 0x{last:08x}",
            self.watchpoints.len() - 1
        );
        Ok(())
    }

    fn unwatch(&mut self, tokens: &[&str]) -> Result<(), String> {
        let target = tokens.get(1).ok_or("usage: unwatch ID|all")?;
        if *target == "all" {
            self.watchpoints.clear();
            println!("watchpoints cleared");
            return Ok(());
        }
        let id: usize = target.parse().map_err(|_| "invalid watchpoint id")?;
        if id >= self.watchpoints.len() {
            return Err(format!("no watchpoint {id}"));
        }
        self.watchpoints.remove(id);
        println!("removed watchpoint {id}");
        Ok(())
    }

    /// A program that has reached its exit point is over; running on would execute whatever
    /// follows it. False means the command should not move the machine.
    fn still_running(&mut self) -> bool {
        if self.machine.at_stop() {
            println!(
                "program has exited with code {}; 'reset' to run it again",
                self.machine.exit_code()
            );
            return false;
        }
        true
    }

    fn continue_run(&mut self, tokens: &[&str]) -> Result<(), String> {
        let budget = match tokens.get(1) {
            Some(text) => text.parse::<u64>().map_err(|_| "invalid cycle count")?,
            None => self.max_cycles,
        };
        if !self.still_running() {
            return Ok(());
        }
        let watch = &mut self.watchpoints;
        let mut conditions = [
            StopCondition::Budget(budget),
            StopCondition::Check(&mut |machine: &Machine| {
                hit_watchpoints(machine, watch).then_some(StopReason::Watchpoint)
            }),
        ];
        match self.machine.resume(&mut conditions) {
            Ok(StopReason::Breakpoint(hit)) => self.report_breakpoint(hit),
            Ok(StopReason::Exit) => println!(
                "program exited with code {} (cycle {})",
                self.machine.exit_code(),
                self.machine.cycles()
            ),
            Ok(StopReason::Fault) => {
                if let Some(fault) = self.machine.fault() {
                    println!("fault: {fault}");
                }
            }
            Ok(StopReason::Budget) => {
                println!("cycle budget exhausted at pc=0x{:08x}", self.machine.pc())
            }
            Ok(_) => println!("halted at pc=0x{:08x}", self.machine.pc()),
            Err(error) => return Err(error),
        }
        self.where_now();
        Ok(())
    }

    fn step_cycles(&mut self, tokens: &[&str]) -> Result<(), String> {
        let count: u64 = match tokens.get(1) {
            Some(text) => text.parse().map_err(|_| "invalid cycle count")?,
            None => 1,
        };
        if !self.still_running() {
            return Ok(());
        }
        let watch = &mut self.watchpoints;
        let mut conditions = [
            StopCondition::Budget(count),
            StopCondition::Check(&mut |machine: &Machine| {
                hit_watchpoints(machine, watch).then_some(StopReason::Watchpoint)
            }),
        ];
        match self.machine.resume(&mut conditions)? {
            StopReason::Breakpoint(hit) => self.report_breakpoint(hit),
            StopReason::Fault => {
                if let Some(fault) = self.machine.fault() {
                    println!("fault: {fault}");
                }
            }
            _ => {}
        }
        self.where_now();
        Ok(())
    }

    fn step_packets(&mut self, tokens: &[&str]) -> Result<(), String> {
        let count: u64 = match tokens.get(1) {
            Some(text) => text.parse().map_err(|_| "invalid packet count")?,
            None => 1,
        };
        if !self.still_running() {
            return Ok(());
        }
        let mut conditions = [StopCondition::Packets(count)];
        match self.machine.resume(&mut conditions) {
            Ok(StopReason::Breakpoint(hit)) => self.report_breakpoint(hit),
            Ok(StopReason::Fault) => {
                if let Some(fault) = self.machine.fault() {
                    println!("fault: {fault}");
                }
            }
            Ok(_) => {}
            Err(error) => return Err(error),
        }
        hit_watchpoints(&self.machine, &mut self.watchpoints);
        self.where_now();
        Ok(())
    }

    fn report_breakpoint(&self, hit: BreakpointHit) {
        if let Some((id, breakpoint)) = self
            .breakpoints
            .iter()
            .enumerate()
            .find(|(_, breakpoint)| breakpoint.address == hit.address)
        {
            println!(
                "breakpoint {id} on core {} at {} (cycle {})",
                hit.core,
                describe(breakpoint),
                self.machine.cycles()
            );
        } else {
            println!(
                "breakpoint on core {} at 0x{:08x} (cycle {})",
                hit.core,
                hit.address,
                self.machine.cycles()
            );
        }
    }

    fn where_now(&mut self) {
        let pc = self.machine.pc();
        let (text, _) = self.machine.disassemble(pc);
        println!(
            "pc=0x{pc:08x}{} cycle={}: {text}",
            self.machine
                .symbol_at(pc)
                .map(|name| format!(" <{name}>"))
                .unwrap_or_default(),
            self.machine.cycles()
        );
    }

    fn trace_command(&mut self, tokens: &[&str]) -> Result<(), String> {
        match tokens.get(1).copied() {
            Some("on") => {
                let path = tokens.get(2).ok_or("usage: trace on FILE [json]")?;
                let json = tokens.get(3).is_some_and(|format| *format == "json");
                self.machine
                    .start_trace(Path::new(path), json, None)
                    .map_err(|error| format!("{path}: {error}"))?;
                println!("tracing to {path}");
            }
            Some("off") => {
                self.machine.stop_trace();
                println!("tracing off");
            }
            _ => return Err("usage: trace on FILE [json] | trace off".into()),
        }
        Ok(())
    }

    fn write_memory(&mut self, tokens: &[&str]) -> Result<(), String> {
        let [_, address, hexbytes] = tokens else {
            return Err("usage: write ADDR HEXBYTES".into());
        };
        let address = self.machine.resolve(address)?;
        let bytes = decode_hex(hexbytes)?;
        self.machine.write_memory(address, &bytes);
        println!("wrote {} bytes at 0x{address:08x}", bytes.len());
        Ok(())
    }
}

/// Reports every watched register whose value moved, and says whether any did.
fn hit_watchpoints(machine: &Machine, watchpoints: &mut [Watchpoint]) -> bool {
    let mut hit = false;
    for (id, watch) in watchpoints.iter_mut().enumerate() {
        let current = match watch.bank {
            'A' => machine.register('A', watch.index),
            _ => machine.register('B', watch.index),
        };
        if current != watch.last {
            println!(
                "watch {id}: {}{} changed 0x{:08x} -> 0x{current:08x}",
                watch.bank, watch.index, watch.last
            );
            watch.last = current;
            hit = true;
        }
    }
    hit
}

fn describe(breakpoint: &Breakpoint) -> String {
    match &breakpoint.label {
        Some(label) => format!("0x{:08x} <{label}>", breakpoint.address),
        None => format!("0x{:08x}", breakpoint.address),
    }
}

fn parse_register_name(name: &str) -> Result<(char, usize), String> {
    let bank = name
        .chars()
        .next()
        .map(|letter| letter.to_ascii_uppercase())
        .filter(|letter| *letter == 'A' || *letter == 'B')
        .ok_or_else(|| format!("invalid register {name}"))?;
    let index: usize = name[1..]
        .parse()
        .map_err(|_| format!("invalid register {name}"))?;
    if index >= 32 {
        return Err(format!("invalid register {name}"));
    }
    Ok((bank, index))
}

fn parse_u32(text: &str) -> Result<u32, String> {
    crate::parse_address(text)
}

fn decode_hex(text: &str) -> Result<Vec<u8>, String> {
    if text.len() % 2 != 0 {
        return Err("hex byte string must have an even length".into());
    }
    (0..text.len())
        .step_by(2)
        .map(|index| {
            u8::from_str_radix(&text[index..index + 2], 16)
                .map_err(|_| format!("invalid hex byte {}", &text[index..index + 2]))
        })
        .collect()
}
