//! The machine every front end drives, and the command-line options that build it.
//!
//! A loaded image, a CPU, and the host connections the device needs - the CIO handshake, the
//! Ethernet backend, the HPI host side, the trace sink - are the same thing whether the caller
//! is the batch runner, the console debugger, the browser dashboard or the GDB stub. They live
//! here once, so `--pcap-in`, `--net-unix`, `--hpi-unix`, `--boot-table` and `--trace` mean the
//! same in every subcommand. Drivers update their breakpoint addresses when the list changes,
//! then run or resume with stop conditions and read the snapshot; halt lifecycle stays here.

use crate::cache::CacheView;
use crate::cio;
#[cfg(test)]
use crate::cpu::Cpu;
use crate::cpu::{CpuSnapshot, StatsCounters};
use crate::device::{Device, DeviceProfile};
use crate::elf::{Image, Segment};
use crate::netio::{hex, Ethernet, EthernetFiles, HostLink, PollSchedule};
use crate::romboot::BootParameters;
use crate::system::{Endian, Fabric};
use crate::trace::Trace;
use crate::view::{SploopDump, UnitIssue};
use clap::{Args, ValueEnum};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::fs;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, LazyLock};

/// The flag `signal-hook` sets on SIGTERM, shared with the batch runner so it stops at the
/// next batch boundary and prints its counters before exiting, like `perf stat` reporting on
/// termination. `LazyLock` keeps the initializer with the static, so the flag exists once the
/// handler is installed and reads as false before then.
static TERMINATION_REQUESTED: LazyLock<Arc<AtomicBool>> =
    LazyLock::new(|| Arc::new(AtomicBool::new(false)));

/// The flag for `signal-hook::flag::register`.
pub(crate) fn termination_flag() -> Arc<AtomicBool> {
    TERMINATION_REQUESTED.clone()
}

/// Whether SIGTERM has asked the run to stop.
pub(crate) fn termination_requested() -> bool {
    TERMINATION_REQUESTED.load(Ordering::Relaxed)
}

/// The machine and its host connections: how memory is loaded, what the device is wired to,
/// and what is observed while it runs. Shared by every subcommand that executes code, so an
/// option means the same thing wherever it appears.
#[derive(Args)]
#[command(next_help_heading = "Machine")]
pub struct MachineArgs {
    /// Which device to emulate: the single-core c6455, or the six-core c6472
    #[arg(long, default_value = "c6455", value_name = "NAME")]
    pub device: String,
    /// The BOOTMODE pins: mode 1 boots through --hpi-unix on either device; C6455 mode 5
    /// reads --i2c-eeprom. C6472 modes 9 and 10 boot over MAC port 0 or 1. Mode 0 starts
    /// every core; other C6472 ROM modes start core 0, which releases the remaining cores
    #[arg(long, default_value_t = 0, value_name = "MODE")]
    pub boot_mode: u8,
    /// The serial EEPROM image on the I2C bus, which boot mode 5 reads its boot parameters and
    /// its boot table out of
    #[arg(long, value_name = "FILE")]
    pub i2c_eeprom: Option<PathBuf>,
    /// Which of the eight boot parameter blocks an I2C boot loads: the configuration pins
    /// select it, and the block sits at 0x80 times this index
    #[arg(long, default_value_t = 0, value_name = "N")]
    pub boot_param_index: u32,
    /// The Ethernet address fused into the MAC ID words, which an Ethernet boot names itself by
    #[arg(long, default_value = "02:00:00:00:64:72", value_parser = parse_mac, value_name = "MAC")]
    pub mac_id: [u8; 6],
    /// The UDP port an Ethernet boot accepts boot-table frames on
    #[arg(long, default_value_t = 9, value_name = "PORT")]
    pub boot_udp_port: u16,
    /// Stop when the PC reaches this address (default: C$$EXIT, then abort)
    #[arg(long, value_name = "ADDR|SYM")]
    pub stop_pc: Option<String>,
    /// Halt before this address executes; repeatable, accepts a symbol name
    #[arg(long = "break", value_name = "ADDR|SYM")]
    pub breakpoints: Vec<String>,
    /// Load memory from a ROM boot table at FILE:ADDR instead of from the ELF segments
    #[arg(long, value_parser = parse_file_address, value_name = "FILE:ADDR")]
    pub boot_table: Option<(PathBuf, u32)>,
    /// Apply a boot configuration table at FILE:ADDR before the boot table
    #[arg(long, value_parser = parse_file_address, value_name = "FILE:ADDR")]
    pub boot_config: Option<(PathBuf, u32)>,
    /// Inject received Ethernet frames from this pcap file
    #[arg(long, value_name = "FILE")]
    pub pcap_in: Option<PathBuf>,
    /// Capture transmitted Ethernet frames into this pcap file
    #[arg(long, value_name = "FILE")]
    pub pcap_out: Option<PathBuf>,
    /// Unix datagram socket the emulator binds for live Ethernet
    #[arg(long, value_name = "PATH")]
    pub net_unix: Option<PathBuf>,
    /// Unix datagram socket transmitted frames are sent to
    #[arg(long, value_name = "PATH")]
    pub net_peer: Option<PathBuf>,
    /// How often, in cycles, the run loop looks at the Ethernet socket
    #[arg(long, default_value_t = 1_024, value_parser = non_zero_cycles, value_name = "CYCLES")]
    pub net_poll: u64,
    /// Which Ethernet MAC received frames are delivered to (default: MAC 0, or the port an
    /// Ethernet boot was strapped to)
    #[arg(long, value_name = "PORT")]
    pub net_port: Option<usize>,
    /// Unix datagram socket the emulator binds for the HPI host side
    #[arg(long, value_name = "PATH")]
    pub hpi_unix: Option<PathBuf>,
    /// Unix datagram socket HPI replies are sent to
    #[arg(long, value_name = "PATH")]
    pub hpi_peer: Option<PathBuf>,
    /// How often, in cycles, the run loop looks for host accesses
    #[arg(long, default_value_t = 1_024, value_parser = non_zero_cycles, value_name = "CYCLES")]
    pub hpi_poll: u64,
    /// Write an execution trace to this path
    #[arg(long, value_name = "FILE")]
    pub trace: Option<PathBuf>,
    /// Trace rendering: aligned text, or JSON Lines
    #[arg(long, value_enum, default_value_t = TraceFormat::Text)]
    pub trace_format: TraceFormat,
    /// Keep only the last N trace events, flushed at exit: a post-mortem window
    #[arg(long, value_name = "N")]
    pub trace_ring: Option<usize>,
    /// Record trace events only at or after this cycle
    #[arg(long, default_value_t = 0, value_name = "CYCLE")]
    pub trace_start: u64,
    /// Report the cause of every internal exception the firmware is left to handle
    #[arg(long)]
    pub trace_exceptions: bool,
    /// Keep the per-address packet histogram, which the hotspot views read
    #[arg(long)]
    pub profile: bool,
    /// Program every core's caches before the image runs: 32 KB of L1P, 32 KB of L1D and
    /// 256 KB of L2, the most a C64x+ megamodule has
    #[arg(long)]
    pub cache: bool,
}

#[derive(Clone, Copy, Default, PartialEq, ValueEnum)]
pub enum TraceFormat {
    #[default]
    Text,
    Json,
}

/// How memory is loaded, kept so a front end can restart the program without re-parsing
/// anything: the ELF's own segments, a ROM boot table the bootloader walks, or nothing at all.
enum Load {
    Segments(Vec<Segment>),
    /// The table bytes, where they sit, and the entry point the bootloader reported.
    BootTable(Vec<u8>, u32, u32),
    /// An external boot loads nothing from the ELF: the host, bus or network supplies the image,
    /// and the ELF on the command line supplies only symbols.
    Wire,
}

/// One core's state as the device sees it from outside, for a front end listing the cores.
#[derive(Clone, serde::Serialize)]
pub struct CoreSummary {
    pub core: usize,
    pub pc: u32,
    pub cycles: u64,
    pub halted: bool,
    pub held_in_reset: bool,
    pub fault: Option<String>,
    /// The core every front end drives, which the fabric follows.
    pub focused: bool,
}

/// Everything a front end displays, read at one instant.
///
/// The machine builds it; the batch runner's JSON, the console debugger's dump, the dashboard
/// and the GDB stub's register map are all projections of it, so none of them can report a
/// different machine than the others.
pub struct MachineSnapshot {
    /// The focused core's register files, program counter and control registers. Which core
    /// that is comes from the `focused` flag on the summaries below.
    pub core: CpuSnapshot,
    pub cycles: u64,
    pub packets_executed: u64,
    pub halted: bool,
    /// An armed user breakpoint on any running core; inspecting it never changes focus.
    pub breakpoint: Option<BreakpointHit>,
    /// Parked on the program's exit point, which is what "the program finished" means.
    pub at_stop: bool,
    pub fault: Option<String>,
    /// The SPLOOP buffer is live, which `TSR.SPLX` mirrors.
    pub loop_active: bool,
    pub stats: StatsCounters,
    pub last_issue: Vec<UnitIssue>,
    pub sploop: Option<SploopDump>,
    /// The exit code the program left in A4, the EABI return register.
    pub exit_code: u8,
    pub device: &'static str,
    pub cores: Vec<CoreSummary>,
}

pub struct Machine {
    /// The device: one core on a C6455, six on a C6472, and the fabric they share.
    pub device: Device,
    entry: u32,
    load: Load,
    /// A boot configuration table applied before anything else, as the ROM would.
    boot_config: Option<(Vec<u8>, u32)>,
    symbols: BTreeMap<String, u32>,
    /// The same symbols by address, for naming a PC.
    by_address: Vec<(u32, String)>,
    /// The CIO handshake breakpoint and buffer, when the image carries the symbols.
    cio: Option<(u32, u32)>,
    /// Everything the program has written through CIO, for the state dump.
    cio_output: Vec<u8>,
    ethernet: Ethernet,
    host_link: Option<HostLink>,
    hpi_poll: PollSchedule,
    trace_exceptions: bool,
    profiling: bool,
    trace: Option<(PathBuf, bool, Option<usize>, u64)>,
    /// Where the program ends: `--stop-pc`, or C$$EXIT, or abort.
    stop: Option<u32>,
    /// Driver-owned breakpoint addresses retained for arming, resume and reload.
    breakpoints: Vec<u32>,
    /// Breakpoints named on the command line, resolved once.
    initial_breakpoints: Vec<u32>,
    /// Which device this is, so a reload rebuilds the same one.
    profile: &'static DeviceProfile,
    boot_mode: u8,
    /// The address fused into the MAC ID words, and the UDP port an Ethernet boot listens on.
    mac_id: [u8; 6],
    boot_udp_port: u16,
    /// The serial EEPROM a board would have put on the I2C bus, and which of the eight boot
    /// parameter blocks the configuration pins select. Held so a reload straps the same board.
    i2c_eeprom: Option<Vec<u8>>,
    boot_param_index: u32,
}

/// One member of the stop-condition set `resume` evaluates on every cycle. The machine-native
/// conditions - a fault, the exit point and the armed halt list - are always members and are
/// checked first; everything else the driver cares about is named here.
pub enum StopCondition<'a> {
    /// Stop after `budget` cycles from the call; zero runs without a limit.
    Budget(u64),
    /// Stop after `packets` execute packets from the call.
    Packets(u64),
    /// A driver-supplied stop check, run once per cycle before the cycle runs. It sees the
    /// machine read-only and returns the reason when its condition holds; a driver can also
    /// use it as a per-cycle hook (the runner records its PC trail inside one).
    Check(&'a mut dyn FnMut(&Machine) -> Option<StopReason>),
}

/// The core and address caught by the halt rule. Simultaneous hits use core-index order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub struct BreakpointHit {
    pub core: usize,
    pub address: u32,
}

/// Why `resume` returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    /// The armed halt list caught the fetch - a user breakpoint, not the exit point.
    Breakpoint(BreakpointHit),
    /// The program reached its exit point (`--stop-pc`, C$$EXIT, or abort).
    Exit,
    /// The CPU refused to execute something.
    Fault,
    /// A `Budget` condition was satisfied.
    Budget,
    /// A `Packets` condition was satisfied.
    Packets,
    /// A driver check reported `Interrupted`.
    Interrupted,
    /// A driver check reported `Watchpoint`.
    Watchpoint,
    /// A driver check reported `OutOfBounds`.
    OutOfBounds,
    /// SIGTERM asked the emulator to stop and report its counters.
    Terminated,
}

impl Machine {
    pub fn load(path: &Path, args: &MachineArgs) -> Result<Self, String> {
        let image = Image::load(path)?;
        let cio = cio_handshake(&image);
        let mut by_address: Vec<(u32, String)> = image
            .symbols
            .iter()
            .map(|(name, address)| (*address, name.clone()))
            .collect();
        by_address.sort_unstable();
        let boot_config = match &args.boot_config {
            Some((path, address)) => Some((
                fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?,
                *address,
            )),
            None => None,
        };
        let boot_table = match &args.boot_table {
            Some((path, address)) => Some((
                fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?,
                *address,
            )),
            None => None,
        };
        let profile = DeviceProfile::named(&args.device)
            .ok_or_else(|| format!("unknown device {}", args.device))?;
        let hpi_boot = args.boot_mode == crate::hpi::BOOTMODE_HPI;
        if hpi_boot {
            if args.hpi_unix.is_none() {
                return Err(format!(
                    "boot mode {} is the {}'s HPI host boot; give it the host with --hpi-unix",
                    args.boot_mode, profile.name
                ));
            }
            if args.i2c_eeprom.is_some() {
                return Err("--i2c-eeprom and HPI host boot are two boot sources; use one".into());
            }
            if args.boot_config.is_some() {
                return Err(
                    "--boot-config cannot be used with HPI host boot; the host loads memory directly"
                        .into(),
                );
            }
        }
        let i2c_eeprom = match &args.i2c_eeprom {
            Some(path) => {
                Some(std::fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?)
            }
            None => None,
        };
        if args.boot_mode == crate::i2cboot::BOOTMODE_I2C_EEPROM && i2c_eeprom.is_none() {
            return Err(format!(
                "boot mode {} is the ROM's I2C EEPROM boot; give it the EEPROM with \
                 --i2c-eeprom",
                crate::i2cboot::BOOTMODE_I2C_EEPROM
            ));
        }
        // A boot builds the memory image from the host, wire or bus, so the ELF supplies only
        // symbols (SPRS276M section 2.4, SPRUEC6G sections 2.3.4 and 4.3.5).
        let ethernet_boot = match ethernet_boot_port(profile, args.boot_mode) {
            Some(Ok(_)) => true,
            Some(Err(message)) => return Err(message),
            None => false,
        };
        let load = if ethernet_boot || hpi_boot || i2c_eeprom.is_some() {
            Load::Wire
        } else {
            Load::Segments(image.segments)
        };
        // One wire, and it is attached to one of the device's MACs: MAC 0 unless the command
        // line says otherwise, or the MAC an Ethernet boot was strapped to, because that is the
        // port the board would have cabled.
        let net_port = args
            .net_port
            .unwrap_or_else(|| crate::romboot::mac_boot_port(args.boot_mode).unwrap_or(0));
        if net_port >= profile.emac_bases.len() {
            return Err(format!(
                "the {} has no Ethernet MAC {net_port}",
                profile.name
            ));
        }
        let mut machine = Self {
            device: Device::with_boot_mode(profile, image.entry, image.endian, args.boot_mode),
            profile,
            boot_mode: args.boot_mode,
            entry: image.entry,
            load,
            boot_config,
            symbols: image.symbols,
            by_address,
            cio,
            cio_output: Vec::new(),
            ethernet: Ethernet::open(
                args.net_poll,
                net_port,
                EthernetFiles {
                    pcap_in: args.pcap_in.as_deref(),
                    pcap_out: args.pcap_out.as_deref(),
                    socket: args.net_unix.as_deref(),
                    peer: args.net_peer.as_deref(),
                },
            )?,
            host_link: match &args.hpi_unix {
                Some(path) => Some(HostLink::bind(path, args.hpi_peer.as_deref())?),
                None => None,
            },
            hpi_poll: PollSchedule::new(args.hpi_poll),
            trace_exceptions: args.trace_exceptions,
            profiling: args.profile,
            trace: args.trace.as_ref().map(|path| {
                (
                    path.clone(),
                    args.trace_format == TraceFormat::Json,
                    args.trace_ring,
                    args.trace_start,
                )
            }),
            stop: None,
            breakpoints: Vec::new(),
            initial_breakpoints: Vec::new(),
            mac_id: args.mac_id,
            boot_udp_port: args.boot_udp_port,
            i2c_eeprom,
            boot_param_index: args.boot_param_index,
        };
        if matches!(machine.load, Load::Wire) {
            // Two boot sources are one too many.
            if boot_table.is_some() {
                return Err("--boot-table and a ROM boot are two boot sources; use one".into());
            }
            // A capture file cannot be the Ethernet boot's source: the boot code resets the MAC
            // before it configures it, so frames handed over before the announcement are gone.
            if ethernet_boot && machine.ethernet.has_preload() {
                return Err(
                    "--pcap-in frames arrive before an Ethernet boot has configured the \
                            MAC and are lost with its reset; drive the boot through --net-unix"
                        .into(),
                );
            }
        }
        // A boot table replaces the ELF's memory image entirely: the ELF is only there for the
        // symbols a debugger and the CIO handshake need (SPRUEC6G section 6.2.2).
        if let Some((table, address)) = boot_table {
            machine.install()?;
            machine.device.cpu_mut().write_bytes(address, &table);
            let entry =
                crate::boot::load_boot_table(&mut machine.device.cpu_mut().bus_mut(), address)?;
            machine.load = Load::BootTable(table, address, entry);
            machine.entry = entry;
        }
        machine.install()?;
        if args.cache {
            machine.enable_caches();
        }
        machine.stop = match &args.stop_pc {
            Some(token) => Some(machine.resolve(token)?),
            None => machine
                .symbols
                .get("C$$EXIT")
                .or_else(|| machine.symbols.get("abort"))
                .copied(),
        };
        machine.initial_breakpoints = args
            .breakpoints
            .iter()
            .map(|token| machine.resolve(token))
            .collect::<Result<_, _>>()?;
        Ok(machine)
    }

    /// Program every core's caches to the most a C64x+ megamodule has: 32 KB of L1P, 32 KB of
    /// L1D and 256 KB of L2, written through the configuration registers firmware uses rather
    /// than by reaching into the model (SPRU871K L1PCFG, L1DCFG and L2CFG, mode 4 in each).
    ///
    /// Nothing else changes. The MAR bits stay at their reset values, so which external memory
    /// an image makes cacheable is still the image's own business, and the caches come up cold:
    /// a program run this way answers the same but takes more cycles to do it.
    fn enable_caches(&mut self) {
        let focused = self.focus();
        for core in 0..self.cores() {
            self.set_focus(core);
            let mut bus = self.device.cpu_mut().bus_mut();
            bus.write32(crate::memory::L1PCFG, 4);
            bus.write32(crate::memory::L1DCFG, 4);
            bus.write32(crate::memory::L2CFG, 4);
        }
        self.set_focus(focused);
    }

    /// A machine with no image behind it, used by native tests that write instructions directly
    /// into memory before driving a debugger session.
    #[cfg(test)]
    pub fn bare(cpu: Cpu) -> Self {
        Self {
            device: Device::from_cpu(cpu),
            profile: &crate::device::C6455,
            boot_mode: 0,
            entry: 0,
            load: Load::Segments(Vec::new()),
            boot_config: None,
            symbols: BTreeMap::new(),
            by_address: Vec::new(),
            cio: None,
            cio_output: Vec::new(),
            ethernet: Ethernet::offline(),
            host_link: None,
            hpi_poll: PollSchedule::new(1),
            trace_exceptions: false,
            profiling: false,
            trace: None,
            stop: None,
            breakpoints: Vec::new(),
            initial_breakpoints: Vec::new(),
            mac_id: [0; 6],
            boot_udp_port: 9,
            i2c_eeprom: None,
            boot_param_index: 0,
        }
    }

    /// How many cores the device has, and which one the front ends are looking at.
    pub fn cores(&self) -> usize {
        self.device.cores()
    }

    pub fn focus(&self) -> usize {
        self.device.focus()
    }

    pub fn set_focus(&mut self, core: usize) {
        self.device.set_focus(core);
    }

    pub fn core_held_in_reset(&mut self, core: usize) -> bool {
        self.device.fabric().held_in_reset(core)
    }

    /// Puts the machine back in its reset state: a fresh CPU, memory loaded the way the command
    /// line asked, and the trace sink carried across so a restart keeps recording.
    pub fn reload(&mut self) -> Result<(), String> {
        let trace = std::mem::replace(self.device.cpu_mut().trace_mut(), Trace::off());
        // Frames the old device transmitted belong in the TX archive before it is rebuilt.
        self.with_ethernet(Ethernet::collect);
        self.ethernet.reset_history();
        // Rebuilding the device also rebuilds its CPUs, including their fetch-halt sets.
        let result = self.install();
        self.install_halt(false);
        // A reinstalled trace keeps writing to the same file rather than truncating it.
        if trace.enabled() {
            self.device.cpu_mut().set_trace(trace);
        }
        self.cio_output.clear();
        result?;
        Ok(())
    }

    fn install(&mut self) -> Result<(), String> {
        let endian = self.device.cpu().endian();
        // A reload straps the device the way the command line did: same profile, same
        // BOOTMODE pins, so the same cores come out of reset.
        self.device = Device::with_boot_mode(self.profile, self.entry, endian, self.boot_mode);
        let profiling = self.profiling;
        self.device.cpu_mut().set_profiling(profiling);
        self.ethernet.reset_poll();
        self.hpi_poll.reset();
        // The board's fused Ethernet address: what the ROM's Ethernet boot names itself by, and
        // what any firmware reads out of the MAC ID words (SPRS612G Table 3-2).
        self.device.fabric().fuse_mac_id(self.mac_id);
        if let Some(Ok(base)) = ethernet_boot_port(self.profile, self.boot_mode) {
            self.device.begin_ethernet_boot(
                base,
                BootParameters {
                    udp_destination_port: self.boot_udp_port,
                    ..BootParameters::default()
                },
            );
        }
        if let Some(contents) = &self.i2c_eeprom {
            let contents = contents.clone();
            self.device.begin_i2c_boot(self.boot_param_index, contents);
        }
        if let Some((table, address)) = &self.boot_config {
            // The configuration table runs before the boot table, which is the order the ROM
            // uses when a boot parameter block asks for one (SPRUEC6G sections 2.3.4, 6.2.3).
            let table = table.clone();
            let address = *address;
            self.device.cpu_mut().write_bytes(address, &table);
            let calls = crate::boot::apply_boot_configuration(
                &mut self.device.cpu_mut().bus_mut(),
                address,
            )?;
            if let Some(target) = calls.first() {
                return Err(format!(
                    "boot configuration entry calls the function at 0x{target:08x}; this path \
                     only applies read/modify/write entries"
                ));
            }
        }
        // The load description is owned by the machine and the write borrows the device, so
        // the segments are taken out for the duration and put back.
        let load = std::mem::replace(&mut self.load, Load::Segments(Vec::new()));
        match &load {
            Load::Segments(segments) => {
                for segment in segments {
                    self.device
                        .fabric()
                        .load_image_bytes(segment.address, &segment.data);
                }
            }
            Load::BootTable(table, address, entry) => {
                let address = *address;
                let entry = *entry;
                self.device.cpu_mut().write_bytes(address, table);
                crate::boot::load_boot_table(&mut self.device.cpu_mut().bus_mut(), address)?;
                self.device.cpu_mut().set_pc(entry);
            }
            // Nothing to preload: the host, bus or network supplies the image during boot.
            Load::Wire => {}
        }
        self.load = load;
        self.with_ethernet(Ethernet::preload);
        if let Some((path, json, ring, start)) = self.trace.clone() {
            self.open_trace(&path, json, ring, start)
                .map_err(|error| format!("{}: {error}", path.display()))?;
        }
        Ok(())
    }

    fn step(&mut self) -> Result<(), String> {
        self.with_ethernet(Ethernet::service);
        self.service_host_port();
        self.pump_cio()?;
        // Every core out of reset runs this cycle, and the shared peripherals advance once.
        self.device.step_cycle();
        if self.ethernet.history_enabled() {
            self.with_ethernet(Ethernet::collect);
        }
        if self.trace_exceptions {
            if let Some(cause) = self.device.cpu_mut().take_exception_cause() {
                eprintln!("exception at {:#010x}: {cause}", self.device.cpu_mut().pc());
            }
        }
        Ok(())
    }

    /// The clock a run budget and host polling measure against. A boot that holds core 0 spends
    /// device cycles waiting for the host, bus or wire while the core retires nothing. Keep
    /// that clock after release too, so a budget never switches between two counters.
    fn run_clock(&self) -> u64 {
        if self.device.boot_holds_core_zero() {
            self.device.cycle()
        } else {
            self.device.cpu().cycles()
        }
    }

    /// The Ethernet backend with the device's fabric and the run clock it measures by. The
    /// machine chooses the clock; the backend owns poll scheduling and frame lifetimes.
    fn with_ethernet<R>(
        &mut self,
        operation: impl FnOnce(&mut Ethernet, &mut Fabric, u64) -> R,
    ) -> R {
        let cycle = self.run_clock();
        operation(&mut self.ethernet, self.device.fabric(), cycle)
    }

    fn service_host_port(&mut self) {
        let cycle = self.run_clock();
        if self.host_link.is_none() || !self.hpi_poll.take_due(cycle) {
            return;
        }
        let Some(link) = self.host_link.as_mut() else {
            return;
        };
        let mut requests = Vec::new();
        while let Some((direction, port, value)) = link.receive() {
            requests.push((direction, port, value));
        }
        for (direction, port, value) in requests {
            let Some(port) = host_port(port) else {
                continue;
            };
            if direction == b'r' {
                let reply = self.device.cpu_mut().bus_mut().hpi_host_read(port);
                let link = self.host_link.as_mut().expect("host link");
                match reply {
                    Some(value) => link.reply(&value.to_le_bytes()),
                    None => link.reply(&[]),
                }
            } else {
                self.device.cpu_mut().bus_mut().hpi_host_write(port, value);
            }
        }
    }

    /// Answers the RTS library's CIO request on every core the program has parked on the
    /// handshake, so `printf` output appears while it runs under any front end.
    ///
    /// The core that answers is the one that reached the trap, which is not always the focused
    /// one: the handshake is per core while the buffer is a single address in the image.
    /// Servicing only the focused core left every other core parked on the trap for good: the
    /// next batch found it due again at its first cycle, so the run loop could never advance it.
    fn pump_cio(&mut self) -> Result<bool, String> {
        let Some((breakpoint, buffer)) = self.cio else {
            return Ok(false);
        };
        let mut handled = false;
        for core in 0..self.device.cores() {
            if !self.device.core(core).ready_to_fetch_at(breakpoint) {
                continue;
            }
            handled = true;
            // The core stays where the program left it and the fabric comes back, so a
            // request answered here is one the focused core never sees.
            let output = self
                .device
                .with_core(core, |cpu| cio::handle(cpu, buffer))?;
            if let Some(output) = output {
                self.cio_output.extend_from_slice(&output.bytes);
                let mut stream: Box<dyn Write> = if output.fd == 2 {
                    Box::new(io::stderr())
                } else {
                    Box::new(io::stdout())
                };
                stream
                    .write_all(&output.bytes)
                    .and_then(|()| stream.flush())
                    .map_err(|error| error.to_string())?;
            }
        }
        Ok(handled)
    }

    pub(crate) fn enable_ethernet_history(&mut self) {
        let cycle = self.run_clock();
        self.ethernet.enable_history(cycle);
    }

    pub(crate) fn ethernet_history_json(&mut self) -> Value {
        self.with_ethernet(Ethernet::history_json)
    }

    /// Flushes the trace and writes any captured frames. Every front end calls this on the way
    /// out, so a session that ends in the REPL, in gdb or in the browser leaves the same files
    /// behind as a batch run.
    pub fn finish(&mut self) -> Result<(), String> {
        self.device.cpu_mut().trace_mut().flush();
        self.with_ethernet(Ethernet::finish)
    }

    /// Bytes written through the TI RTS CIO channel during this run.
    pub(crate) fn cio_output(&self) -> &[u8] {
        &self.cio_output
    }

    pub fn stop_address(&self) -> Option<u32> {
        self.stop
    }

    /// True when the program has reached its exit point on any core. On a multicore device the
    /// run ends when whichever core got there first is about to fetch it, and the halt rule has
    /// stopped every other core at the same address too.
    pub fn at_stop(&self) -> bool {
        let Some(address) = self.stop else {
            return false;
        };
        // A core held in reset is not about to fetch anything, whatever its reset PC happens
        // to be: during a ROM boot that address is one the boot has not branched it to yet.
        (0..self.device.cores()).any(|core| {
            !self.device.held_in_reset(core) && self.device.core(core).ready_to_fetch_at(address)
        })
    }
    /// Replace the driver's breakpoint addresses. The machine retains them across resume
    /// and reload, and unions them with the program's exit point on every core.
    pub fn arm_halt(&mut self, breakpoints: &[u32]) {
        self.breakpoints.clear();
        self.breakpoints.extend_from_slice(breakpoints);
        self.install_halt(false);
    }

    /// Install the full halt rule, or its interactive resume form. Exclusion is per core:
    /// a core moving off its own address must not disable that address for another core.
    fn install_halt(&mut self, exclude_current: bool) {
        let stop = self.stop;
        let breakpoints = &self.breakpoints;
        for core in 0..self.device.cores() {
            self.device.with_core(core, |cpu| {
                let current = cpu.pc();
                let mut addresses: Vec<u32> = breakpoints
                    .iter()
                    .copied()
                    .filter(|address| !exclude_current || *address != current)
                    .collect();
                addresses.extend(stop.filter(|address| !exclude_current || *address != current));
                cpu.set_stop_on_fetch(addresses);
            });
        }
    }

    /// Run from the current state, including a breakpoint on the entry address. Batch
    /// execution uses this; interactive continue and step use `resume` instead.
    pub fn run(&mut self, conditions: &mut [StopCondition]) -> Result<StopReason, String> {
        self.install_halt(false);
        self.run_until_stop(conditions)
    }

    /// Continue or step with each core's current address excluded for this call. The full
    /// halt rule is restored before returning, including on host-I/O errors.
    pub fn resume(&mut self, conditions: &mut [StopCondition]) -> Result<StopReason, String> {
        self.install_halt(true);
        let result = self.run_until_stop(conditions);
        self.install_halt(false);
        result
    }

    /// Evaluate machine-native conditions before driver conditions, once per cycle.
    /// The budget-only path batches cycles without changing that halt precedence.
    fn run_until_stop(&mut self, conditions: &mut [StopCondition]) -> Result<StopReason, String> {
        let start_cycles = self.run_clock();
        let start_packets = self.device.cpu_mut().packets_executed();
        if !self.ethernet.history_enabled() && !self.trace_exceptions {
            if let [StopCondition::Budget(budget)] = conditions {
                return self.resume_budget(start_cycles, *budget);
            }
        }
        loop {
            if self.device.cpu_mut().fault().is_some() {
                return Ok(StopReason::Fault);
            }
            if self.at_stop() {
                return Ok(StopReason::Exit);
            }
            if let Some(hit) = self.halt_location() {
                return Ok(StopReason::Breakpoint(hit));
            }
            for condition in conditions.iter_mut() {
                match condition {
                    StopCondition::Budget(budget) => {
                        if *budget != 0 && self.run_clock() >= start_cycles + *budget {
                            return Ok(StopReason::Budget);
                        }
                    }
                    StopCondition::Packets(packets) => {
                        if *packets != 0
                            && self.device.cpu_mut().packets_executed() >= start_packets + *packets
                        {
                            return Ok(StopReason::Packets);
                        }
                    }
                    StopCondition::Check(check) => {
                        if let Some(reason) = check(self) {
                            return Ok(reason);
                        }
                    }
                }
            }
            self.step()?;
        }
    }

    /// Locate a fetch halt without changing the focused core; `Device::halted_core` owns the
    /// attribution rule.
    fn halt_location(&self) -> Option<BreakpointHit> {
        self.device.halted_core().map(|core| BreakpointHit {
            core,
            address: self.device.core(core).pc(),
        })
    }

    fn resume_budget(&mut self, start_cycles: u64, budget: u64) -> Result<StopReason, String> {
        let deadline = (budget != 0).then(|| start_cycles.saturating_add(budget));
        loop {
            if termination_requested() {
                return Ok(StopReason::Terminated);
            }
            if self.device.cpu_mut().fault().is_some() {
                return Ok(StopReason::Fault);
            }
            if self.at_stop() {
                return Ok(StopReason::Exit);
            }
            if let Some(hit) = self.halt_location() {
                return Ok(StopReason::Breakpoint(hit));
            }
            let current = self.run_clock();
            if deadline.is_some_and(|limit| current >= limit) {
                return Ok(StopReason::Budget);
            }

            self.with_ethernet(Ethernet::service);
            self.service_host_port();
            let handled_cio = self.pump_cio()?;

            let mut batch = 4096u64;
            if let Some(delay) = self.ethernet.poll_delay(current) {
                batch = batch.min(delay);
            }
            if self.host_link.is_some() {
                batch = batch.min(self.hpi_poll.delay(current));
            }
            if let Some(limit) = deadline {
                batch = batch.min(limit.saturating_sub(current).max(1));
            }
            let cio = self.cio.map(|(breakpoint, _)| breakpoint);
            self.device.run_cycles(batch, cio, handled_cio);
        }
    }

    /// The focused core's program counter, cycle count and fault, asked one at a time.
    ///
    /// These exist beside `snapshot` because a stop condition is evaluated on every cycle of a
    /// run and takes `&Machine`: it asks where the program counter is, and it cannot afford - or
    /// borrow mutably for - a whole projection to find out. A front end rendering state takes
    /// one snapshot instead of calling these a dozen times.
    pub fn pc(&self) -> u32 {
        self.device.cpu().pc()
    }

    pub fn cycles(&self) -> u64 {
        self.device.cpu().cycles()
    }

    pub fn fault(&self) -> Option<&str> {
        self.device.cpu().fault()
    }

    /// One register of the focused core, for a watchpoint that samples a single one.
    pub fn register(&self, bank: char, index: usize) -> u32 {
        self.device.cpu().register(bank, index)
    }

    /// The endianness the device is strapped to, which the GDB stub's word order follows.
    pub fn endian(&self) -> Endian {
        self.device.cpu().endian()
    }

    /// Everything a front end displays, read once.
    ///
    /// One projection, so the batch runner's JSON, the console debugger's dump, the browser
    /// dashboard and the GDB stub cannot disagree about what the machine's state is. They used
    /// to: this returned only the focused core's registers, the complete state had a second
    /// projection in `state_json`, the dashboard built a third by reading the CPU and the
    /// memory hierarchy itself, and the GDB stub mapped a fourth.
    ///
    /// What is *not* here is what costs more than the rest put together and only one caller
    /// wants: the cache, interrupt-selector, event-flag and cacheability views, the memory
    /// window, and the hotspot histogram. Those are their own operations.
    pub fn snapshot(&mut self) -> MachineSnapshot {
        let focus = self.focus();
        let cores = (0..self.cores())
            .map(|core| CoreSummary {
                core,
                pc: self.device.core(core).pc(),
                cycles: self.device.core(core).cycles(),
                halted: self.device.core(core).halted(),
                held_in_reset: self.device.fabric().held_in_reset(core),
                fault: self.device.core(core).fault().map(str::to_owned),
                focused: core == focus,
            })
            .collect();
        let at_stop = self.at_stop();
        let breakpoint = if at_stop { None } else { self.halt_location() };
        let cpu = self.device.cpu();
        MachineSnapshot {
            core: cpu.snapshot(),
            cycles: cpu.cycles(),
            packets_executed: cpu.packets_executed(),
            halted: cpu.halted(),
            breakpoint,
            at_stop,
            fault: cpu.fault().map(str::to_owned),
            loop_active: cpu.loop_active(),
            stats: cpu.stats().counters(),
            last_issue: cpu.last_issue().to_vec(),
            sploop: cpu.sploop_dump(),
            exit_code: cpu.register('A', 4) as u8,
            device: self.profile.name,
            cores,
        }
    }

    /// The busiest execute-packet addresses, from the histogram `--profile` keeps.
    ///
    /// One implementation: the batch state dump and the dashboard's hotspot table both used to
    /// sort the histogram themselves, which is two chances to disagree about ordering when two
    /// addresses have issued the same number of times.
    pub fn hotspots(&self, limit: usize) -> Vec<(u32, u64)> {
        let mut histogram: Vec<(u32, u64)> = self
            .device
            .cpu()
            .stats()
            .pc_histogram
            .iter()
            .map(|(pc, count)| (*pc, *count))
            .collect();
        histogram.sort_unstable_by_key(|(pc, count)| (std::cmp::Reverse(*count), *pc));
        histogram.truncate(limit);
        histogram
    }

    /// Read memory the way a front end must: through the host view, so inspecting a program
    /// does not change it.
    ///
    /// This is the seam's one memory policy, and it exists because there used to be two. The
    /// console debugger and the dashboard read through the host view, which snoops the caches
    /// without allocating in them; the GDB stub read through the CPU's own data path, which
    /// fills L1D lines, moves the replacement clock and counts in the cache statistics. A `m`
    /// packet therefore made the run it was inspecting differ from the run without a debugger
    /// attached. Which of the two a front end got was decided by which method it happened to
    /// call, so the rule now lives here and the perturbing pair is gone.
    pub fn read_memory(&mut self, address: u32, length: usize) -> Vec<u8> {
        self.device.cpu().host_read_bytes(address, length)
    }

    /// Write memory through the same host view. A front end editing memory is not the program
    /// storing to it, so it allocates nothing and counts as nothing.
    pub fn write_memory(&mut self, address: u32, bytes: &[u8]) {
        self.device.cpu_mut().host_write_bytes(address, bytes);
    }

    /// Render the instruction at `address` and answer where the next one starts. Read through
    /// the host view like every other inspection, so listing code fills no L1P line.
    pub fn disassemble(&self, address: u32) -> (String, u32) {
        self.device.cpu().disassemble(address)
    }

    /// The focused core's register banks, by the names a front end takes from a user.
    pub fn set_register(&mut self, name: &str, value: u32) -> Result<(), String> {
        self.device.cpu_mut().set_register(name, value)
    }

    pub fn write_control(&mut self, register: u8, value: u32) {
        self.device.cpu_mut().write_control(register, value);
    }

    pub fn set_pc(&mut self, pc: u32) {
        self.device.cpu_mut().set_pc(pc);
    }

    /// The focused megamodule's cache state, for a front end that draws it.
    pub fn cache_views(&mut self) -> [CacheView; 3] {
        self.device.cpu_mut().bus_mut().cache_views()
    }

    /// One resident line, or nothing if that level does not hold the address. Passive: reading
    /// it moves no replacement clock.
    pub fn cache_line_data(&mut self, level: usize, address: u32) -> Option<Vec<u8>> {
        self.device
            .cpu_mut()
            .bus_mut()
            .cache_line_data(level, address)
            .map(<[u8]>::to_vec)
    }

    /// The focused megamodule's interrupt selectors, event flags and cacheability registers.
    pub fn interrupt_selectors(&mut self) -> [u32; 3] {
        self.device.cpu_mut().bus_mut().interrupt_selectors()
    }

    pub fn event_flag_registers(&mut self) -> [u32; 4] {
        self.device.cpu_mut().bus_mut().event_flag_registers()
    }

    pub fn mar_registers(&mut self) -> [u32; 256] {
        self.device.cpu_mut().bus_mut().mar_registers()
    }

    /// Keep the per-address packet histogram the hotspot views read.
    pub fn set_profiling(&mut self, on: bool) {
        self.device.cpu_mut().set_profiling(on);
    }

    /// Keep the last execute packet's unit issue, which the debugger and dashboard display.
    pub fn set_issue_view(&mut self, on: bool) {
        self.device.cpu_mut().set_issue_view(on);
    }

    /// Start recording a trace to `path`, from the cycle the machine is on now.
    ///
    /// The machine opens the sink rather than being handed one, so the front end turning tracing
    /// on at a prompt and the command line arming it at startup go through the same place. Both
    /// end up in `open_trace`, which is the only thing that constructs a sink.
    pub fn start_trace(&mut self, path: &Path, json: bool, ring: Option<usize>) -> io::Result<()> {
        let start = self.device.cpu().cycles();
        self.open_trace(path, json, ring, start)
    }

    /// A trace's event timestamps are relative to the cycle recording begins at, so that cycle
    /// is an argument: `--trace-start` names one ahead of the run, a prompt means now.
    fn open_trace(
        &mut self,
        path: &Path,
        json: bool,
        ring: Option<usize>,
        start: u64,
    ) -> io::Result<()> {
        let file = fs::File::create(path)?;
        self.device
            .cpu_mut()
            .set_trace(Trace::open(BufWriter::new(file), json, ring, start));
        Ok(())
    }

    /// Flush and detach the trace sink.
    pub fn stop_trace(&mut self) {
        self.device.cpu_mut().trace_mut().flush();
        self.device.cpu_mut().set_trace(Trace::off());
    }

    /// The exit code the program left in A4, the EABI return register.
    pub fn exit_code(&self) -> u8 {
        self.device.cpu().register('A', 4) as u8
    }

    /// The breakpoints named on the command line, for a front end to seed its own set with.
    pub fn initial_breakpoints(&self) -> &[u32] {
        &self.initial_breakpoints
    }

    /// The image's symbols sorted by address, for naming a PC or listing them.
    pub fn symbols_by_address(&self) -> &[(u32, String)] {
        &self.by_address
    }

    /// An address, a symbol name, or `.` for the current PC.
    pub fn resolve(&self, token: &str) -> Result<u32, String> {
        if token == "." {
            return Ok(self.device.cpu().pc());
        }
        if let Some(address) = self.symbols.get(token) {
            return Ok(*address);
        }
        crate::parse_address(token)
    }

    /// The symbol covering an address, as `name+0x1c`, or nothing when none does.
    pub fn symbol_at(&self, address: u32) -> Option<String> {
        let (base, name) = self
            .by_address
            .binary_search_by_key(&address, |(symbol, _)| *symbol)
            .map_or_else(|index| index.checked_sub(1), Some)
            .and_then(|index| self.by_address.get(index))?;
        Some(if *base == address {
            name.clone()
        } else {
            format!("{name}+{:#x}", address - base)
        })
    }

    /// An address with its symbol, for a diagnostic line.
    pub fn describe(&self, address: u32) -> String {
        match self.symbol_at(address) {
            Some(name) => format!("0x{address:08x} {name}"),
            None => format!("0x{address:08x}"),
        }
    }

    /// The addresses the loaded image covers, for spotting a PC that has left it.
    pub fn loaded_ranges(&self) -> Vec<(u32, u32)> {
        match &self.load {
            Load::Segments(segments) => segments
                .iter()
                .map(|segment| (segment.address, segment.address + segment.data.len() as u32))
                .collect(),
            Load::BootTable(table, address, _) => {
                vec![(*address, address + table.len() as u32)]
            }
            // Nothing was loaded from the command line, so the boot image's ranges are not
            // known in advance.
            Load::Wire => Vec::new(),
        }
    }

    /// The machine state a batch run reports, and the same JSON the debugger's `state` command
    /// writes: one description of a finished program, whoever ran it.
    pub fn state_json(&mut self, profile: bool) -> Value {
        let tx_frames: Vec<String> = self.with_ethernet(|ethernet, fabric, cycle| {
            ethernet
                .transmitted(fabric, cycle)
                .iter()
                .map(|frame| hex(frame))
                .collect()
        });
        let snapshot = self.snapshot();
        let mut registers = serde_json::Map::with_capacity(64);
        for (bank, file) in [('A', snapshot.core.a), ('B', snapshot.core.b)] {
            for (index, value) in file.iter().enumerate() {
                registers.insert(format!("{bank}{index}"), json!(value));
            }
        }
        let mut state = json!({
            "halted": snapshot.at_stop,
            "breakpoint": snapshot.breakpoint,
            "fault": snapshot.fault,
            "pc": snapshot.core.pc,
            "cycles": snapshot.cycles,
            "loop_active": snapshot.loop_active,
            "exit_code": snapshot.exit_code,
            "registers": registers,
            "cio_output": String::from_utf8_lossy(&self.cio_output),
            "tx_frames": tx_frames,
            "net_dropped": self.ethernet.dropped(),
            "device": snapshot.device,
            "cores": snapshot.cores,
            "stats": snapshot.stats,
        });
        if let Some(boot) = self.device.ethernet_boot() {
            // What the ROM's Ethernet boot made of the wire, which is the only way to tell a
            // boot that completed from one still waiting for its next frame.
            state["ethernet_boot"] = json!({
                "frames_accepted": boot.accepted,
                "frames_discarded": boot.discarded,
                "table_bytes": boot.table_bytes,
                "entry": boot.entry,
            });
        }
        if let Some(boot) = self.device.i2c_boot() {
            // What the ROM's I2C boot made of the EEPROM, which is how a case tells a boot that
            // read its table from one that stopped on a bad block.
            state["i2c_boot"] = json!({
                "parameter_blocks": boot.parameter_blocks,
                "table_blocks": boot.table_blocks,
                "table_bytes": boot.table_bytes,
                "checksum_failures": boot.checksum_failures,
                "entry": boot.entry,
                "error": boot.error,
            });
        }
        if self.boot_mode == crate::hpi::BOOTMODE_HPI {
            state["hpi_boot"] = json!({
                "entry": self.device.hpi_boot_entry(0),
                "entries": (0..self.cores())
                    .map(|core| self.device.hpi_boot_entry(core))
                    .collect::<Vec<_>>(),
            });
        }
        if profile {
            state["profile"] = json!(self
                .hotspots(32)
                .into_iter()
                .map(|(pc, count)| json!({
                    "pc": format!("0x{pc:08x}"),
                    "where": self.symbol_at(pc),
                    "packets": count,
                }))
                .collect::<Vec<_>>());
            state["cache_stats"] =
                json!(self
                    .device
                    .cpu_mut()
                    .bus_mut()
                    .cache_views()
                    .map(|cache| json!({
                        "name": cache.name,
                        "enabled": cache.enabled,
                        "frozen": cache.frozen,
                        "capacity_bytes": cache.capacity_lines * cache.line_size as usize,
                        "resident_lines": cache.lines.len(),
                        "dirty_lines": cache.lines.iter().filter(|line| line.dirty).count(),
                        "line_size": cache.line_size,
                        "ways": cache.ways,
                        "sets": cache.capacity_lines / cache.ways,
                        "stats": cache.stats,
                    })));
        }
        state
    }

    /// A `perf stat`-style summary of where the emulated CPU spent its cycles: the
    /// instruction mix, the stall and idle breakdown, interrupt service, and the cache hit
    /// rates. Unlike host `perf`, every number is exactly what the modelled C64x+ pipeline
    /// retired, so the report is reproducible across machines.
    pub fn perf_report(&mut self) -> String {
        use std::fmt::Write as _;
        let stats = self.device.cpu().stats();
        let cycles = self.device.cpu().cycles();
        let executing = stats.packets + stats.replay_packets;
        let stalls = stats.stall_memory + stats.stall_nop + stats.stall_interrupt;
        let accounted = executing + stats.idle_cycles + stalls + stats.halt_cycles;
        let pct = |value: u64| {
            if cycles == 0 {
                0.0
            } else {
                value as f64 * 100.0 / cycles as f64
            }
        };
        let cpi = if stats.retired == 0 {
            0.0
        } else {
            cycles as f64 / stats.retired as f64
        };
        let mut out = String::new();
        let _ = writeln!(out, "Performance counter report for the emulated C64x+");
        let _ = writeln!(
            out,
            "  {cycles:>14} cycles           100.0% of emulated time"
        );
        let _ = writeln!(
            out,
            "  {executing:>14} executing        {:5.1}%  (packets issued, incl. SPLOOP replays)",
            pct(executing)
        );
        let _ = writeln!(
            out,
            "  {:>14} idle             {:5.1}%  (IDLE waiting for an interrupt)",
            stats.idle_cycles,
            pct(stats.idle_cycles)
        );
        let _ = writeln!(
            out,
            "  {stalls:>14} stall            {:5.1}%  (pipeline frozen)",
            pct(stalls)
        );
        let _ = writeln!(
            out,
            "  {:>14}   memory          (L1D/L1P miss, bank conflict, write buffer)",
            stats.stall_memory
        );
        let _ = writeln!(
            out,
            "  {:>14}   nop             (multicycle NOP)",
            stats.stall_nop
        );
        let _ = writeln!(
            out,
            "  {:>14}   interrupt       (pipe-down into a service routine)",
            stats.stall_interrupt
        );
        let _ = writeln!(
            out,
            "  {:>14} halted           {:5.1}%  (parked on a stop address)",
            stats.halt_cycles,
            pct(stats.halt_cycles)
        );
        let _ = writeln!(
            out,
            "  {:>14} unaccounted      {:5.1}%  (faulted cycles and rounding)",
            cycles.saturating_sub(accounted),
            pct(cycles.saturating_sub(accounted))
        );
        let _ = writeln!(out, "  {cycles:>14} total cycles (including stalls)");
        let _ = writeln!(out);
        let _ = writeln!(out, "  Instructions");
        let _ = writeln!(out, "  {:>14} instructions retired", stats.retired);
        let _ = writeln!(out, "  {:>14} packets issued", stats.packets);
        let _ = writeln!(out, "  {:>14} SPLOOP replays", stats.replay_packets);
        let _ = writeln!(out, "  {:>14.2} cycles per instruction (CPI)", cpi);
        let _ = writeln!(
            out,
            "  {:>14.2} instructions per cycle (IPC)",
            1.0 / cpi.max(1e-9)
        );
        let _ = writeln!(out);
        let _ = writeln!(out, "  Branches and interrupts");
        let _ = writeln!(out, "  {:>14} branches taken", stats.branches_taken);
        let _ = writeln!(out, "  {:>14} interrupts taken", stats.interrupts);
        let _ = writeln!(
            out,
            "  {:>14} interrupts with a wait",
            stats.interrupt_waited
        );
        let _ = writeln!(
            out,
            "  {:>14} cycles spent waiting to take a ready interrupt",
            stats.interrupt_wait_cycles
        );
        if stats.interrupt_waited != 0 {
            let _ = writeln!(
                out,
                "  {:>14.2} average wait cycles per deferred interrupt",
                stats.interrupt_wait_cycles as f64 / stats.interrupt_waited as f64
            );
        }
        let _ = writeln!(
            out,
            "  {:>14} cycles of 9-cycle pipe-down ({} total)",
            stats.interrupts * 9,
            stats.interrupts
        );
        let _ = writeln!(out);
        let _ = writeln!(out, "  Memory and data");
        let _ = writeln!(out, "  {:>14} stores committed", stats.stores);
        let _ = writeln!(
            out,
            "  {:>14} functional-unit saturations",
            stats.saturations
        );
        let _ = writeln!(out, "  {:>14} faults", stats.faults);
        for cache in self.device.cpu_mut().bus_mut().cache_views() {
            let reads = cache.stats.read_hits + cache.stats.read_misses;
            let writes = cache.stats.write_hits + cache.stats.write_misses;
            let read_rate = if reads == 0 {
                0.0
            } else {
                cache.stats.read_hits as f64 * 100.0 / reads as f64
            };
            let write_rate = if writes == 0 {
                0.0
            } else {
                cache.stats.write_hits as f64 * 100.0 / writes as f64
            };
            let _ = writeln!(out);
            let _ = writeln!(
                out,
                "  {}  {} lines, {} bytes, {} ways, enabled={}",
                cache.name,
                cache.capacity_lines,
                cache.capacity_lines * cache.line_size as usize,
                cache.ways,
                cache.enabled
            );
            let _ = writeln!(
                out,
                "    {:>12.1}% read hit  ({:>12} hits, {:>12} misses)",
                read_rate, cache.stats.read_hits, cache.stats.read_misses
            );
            let _ = writeln!(
                out,
                "    {:>12.1}% write hit ({:>12} hits, {:>12} misses)",
                write_rate, cache.stats.write_hits, cache.stats.write_misses
            );
            let _ = writeln!(
                out,
                "    {:>12} line fills, {:>12} evictions, {:>12} writebacks, {:>12} invalidations",
                cache.stats.line_fills,
                cache.stats.evictions,
                cache.stats.writebacks,
                cache.stats.invalidations
            );
        }
        out
    }
}

/// The two symbols the CIO handshake needs: the breakpoint the RTS library spins on, and the
/// buffer the request and its reply are exchanged through.
fn cio_handshake(image: &Image) -> Option<(u32, u32)> {
    let breakpoint = image
        .symbols
        .get("C$$IO$$")
        .or_else(|| image.symbols.get("C$$IO"))?;
    let buffer = image
        .symbols
        .get("__CIOBUF_")
        .or_else(|| image.symbols.get("_CIOBUF_"))?;
    Some((*breakpoint, *buffer))
}

/// The MAC port register base an Ethernet boot uses, when the strapped boot mode asks for one:
/// nothing when it does not, and the reason when the device has no such boot mode.
///
/// Only the C6472's ROM boots over Ethernet (SPRUEC6G section 4.3.5, boot modes 9 and 10). The
/// C6455's boot mode table has no Ethernet entry at all (section 2.3), so the same pins mean
/// something else there and the option is refused rather than quietly held in reset.
fn ethernet_boot_port(
    profile: &'static DeviceProfile,
    boot_mode: u8,
) -> Option<Result<u32, String>> {
    let port = crate::romboot::mac_boot_port(boot_mode)?;
    if !profile.has_boot_controller || profile.emac_bases.len() <= port {
        return Some(Err(format!(
            "boot mode {boot_mode} is the C6472's boot over MAC port {port}; the {} has no \
             Ethernet boot mode (SPRUEC6G sections 2.3 and 4.3.5)",
            profile.name
        )));
    }
    Some(Ok(profile.emac_bases[port]))
}

/// `FILE:ADDR`: the raw image a bootloader reads, and where it sits in memory.
fn parse_file_address(text: &str) -> Result<(PathBuf, u32), String> {
    let (path, address) = text
        .rsplit_once(':')
        .ok_or_else(|| "expected FILE:ADDR".to_string())?;
    Ok((PathBuf::from(path), crate::parse_address(address)?))
}

/// `AA:BB:CC:DD:EE:FF`: an Ethernet address, as every tool prints one.
fn parse_mac(text: &str) -> Result<[u8; 6], String> {
    let mut mac = [0u8; 6];
    let mut bytes = text.split([':', '-']);
    for byte in &mut mac {
        let field = bytes
            .next()
            .ok_or_else(|| format!("{text} is not six colon-separated bytes"))?;
        *byte = u8::from_str_radix(field, 16).map_err(|_| format!("invalid MAC byte {field}"))?;
    }
    if bytes.next().is_some() {
        return Err(format!("{text} is not six colon-separated bytes"));
    }
    Ok(mac)
}

fn non_zero_cycles(text: &str) -> Result<u64, String> {
    match text.parse() {
        Ok(0) => Err("poll interval must be at least one cycle".into()),
        Ok(cycles) => Ok(cycles),
        Err(_) => Err(format!("invalid cycle count {text}")),
    }
}

/// The HCNTL encoding a host process sends, as SPRU969B Table 4 numbers it.
fn host_port(code: u8) -> Option<crate::hpi::HostPort> {
    match code {
        0 => Some(crate::hpi::HostPort::Control),
        1 => Some(crate::hpi::HostPort::DataAutoIncrement),
        2 => Some(crate::hpi::HostPort::Address),
        3 => Some(crate::hpi::HostPort::Data),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::Cpu;

    /// The halt rule: the driver's breakpoints plus the machine's exit point, with the
    /// current address dropped when the target is about to move off it.
    #[test]
    fn arm_halt_unions_breakpoints_and_exit() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.stop = Some(12);
        machine.device.cpu_mut().write_words(0, &[0; 8]);
        machine.arm_halt(&[4, 8]);
        let mut conditions: [StopCondition; 0] = [];
        assert_eq!(
            machine.resume(&mut conditions).unwrap(),
            StopReason::Breakpoint(BreakpointHit {
                core: 0,
                address: 4
            })
        );
        assert_eq!(machine.device.cpu_mut().pc(), 4);
    }

    /// A fresh run observes an entry breakpoint; subsequent continue/step calls own the
    /// exclusion and restoration, including the halt that coincides with a packet budget.
    #[test]
    fn run_and_resume_own_the_halt_lifecycle() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.stop = Some(12);
        machine.device.cpu_mut().write_words(0, &[0; 8]);
        machine.arm_halt(&[0, 4, 8]);
        assert_eq!(
            machine.run(&mut [StopCondition::Budget(100)]).unwrap(),
            StopReason::Breakpoint(BreakpointHit {
                core: 0,
                address: 0
            })
        );
        assert_eq!(machine.snapshot().packets_executed, 0);
        assert_eq!(
            machine.resume(&mut [StopCondition::Packets(1)]).unwrap(),
            StopReason::Breakpoint(BreakpointHit {
                core: 0,
                address: 4
            })
        );
        assert_eq!(
            machine.snapshot().breakpoint,
            Some(BreakpointHit {
                core: 0,
                address: 4
            })
        );
        assert_eq!(
            machine.resume(&mut [StopCondition::Budget(100)]).unwrap(),
            StopReason::Breakpoint(BreakpointHit {
                core: 0,
                address: 8
            })
        );
        assert_eq!(machine.resume(&mut []).unwrap(), StopReason::Exit);
        assert_eq!(machine.snapshot().breakpoint, None);
        assert_eq!(machine.snapshot().packets_executed, 3);
    }

    /// A resume with no stop conditions runs to the exit point, which the halt rule added to
    /// the fetch-halt set even though the driver named no breakpoints.
    #[test]
    fn resume_runs_to_the_exit_point() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.stop = Some(4);
        machine.device.cpu_mut().write_words(0, &[0; 4]);
        machine.arm_halt(&[]);
        let mut conditions: [StopCondition; 0] = [];
        assert_eq!(machine.resume(&mut conditions).unwrap(), StopReason::Exit);
        assert_eq!(machine.device.cpu_mut().pc(), 4);
    }

    /// The exit point outranks a breakpoint at the same address: a run to the end of the
    /// program reports an exit, not a halt.
    #[test]
    fn exit_outranks_a_breakpoint_at_the_same_address() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.stop = Some(8);
        machine.device.cpu_mut().write_words(0, &[0; 8]);
        machine.arm_halt(&[8]);
        let mut conditions: [StopCondition; 0] = [];
        assert_eq!(machine.resume(&mut conditions).unwrap(), StopReason::Exit);
        assert_eq!(machine.device.cpu_mut().pc(), 8);
    }

    /// A halt at a breakpoint happens at the fetch boundary: the instruction at the address
    /// has not executed when the run returns.
    #[test]
    fn resume_halts_before_fetching_a_breakpoint() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.device.cpu_mut().write_words(0, &[0; 8]);
        machine.arm_halt(&[8]);
        let mut conditions: [StopCondition; 0] = [];
        assert_eq!(
            machine.resume(&mut conditions).unwrap(),
            StopReason::Breakpoint(BreakpointHit {
                core: 0,
                address: 8
            })
        );
        assert_eq!(machine.device.cpu_mut().pc(), 8);
        assert_eq!(machine.device.cpu_mut().packets_executed(), 2);
    }

    #[test]
    fn budget_condition_limits_the_run() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.device.cpu_mut().write_words(0, &[0; 200]);
        let mut conditions = [StopCondition::Budget(100)];
        assert_eq!(machine.resume(&mut conditions).unwrap(), StopReason::Budget);
        assert_eq!(machine.device.cpu_mut().cycles(), 100);
    }

    #[test]
    fn packets_condition_counts_executed_packets() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.device.cpu_mut().write_words(0, &[0; 200]);
        let mut conditions = [StopCondition::Packets(5)];
        assert_eq!(
            machine.resume(&mut conditions).unwrap(),
            StopReason::Packets
        );
        assert_eq!(machine.device.cpu_mut().packets_executed(), 5);
    }

    /// A driver check runs once per cycle before the cycle, so it can stop the run with its
    /// own reason and see the machine read-only.
    #[test]
    fn a_driver_check_stops_the_run_with_its_reason() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.device.cpu_mut().write_words(0, &[0; 200]);
        let mut calls = 0;
        let mut conditions = [StopCondition::Check(&mut |_machine: &Machine| {
            calls += 1;
            (calls >= 3).then_some(StopReason::Watchpoint)
        })];
        assert_eq!(
            machine.resume(&mut conditions).unwrap(),
            StopReason::Watchpoint
        );
        assert_eq!(calls, 3);
        assert_eq!(machine.device.cpu_mut().cycles(), 2);
    }

    #[test]
    fn resume_reports_a_fault() {
        let mut machine = Machine::bare(Cpu::new(0));
        // An instruction word the device does not have, with exceptions disabled at reset:
        // the model refuses to run rather than delivering it to firmware.
        machine.device.cpu_mut().write_words(0, &[0, 0x5645_4314]);
        let mut conditions: [StopCondition; 0] = [];
        assert_eq!(machine.resume(&mut conditions).unwrap(), StopReason::Fault);
        assert!(machine.device.cpu_mut().fault().is_some());
    }

    /// Inspecting memory must not change what the program will observe. Reading through the
    /// CPU's data path fills L1D lines, moves the replacement clock and counts in the cache
    /// statistics, so a front end that inspects that way makes the run it is inspecting
    /// different from the run without a debugger attached.
    #[test]
    fn inspecting_memory_does_not_fill_a_cache() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.enable_caches();
        // Make the DDR2 window cacheable, the way an image would.
        machine
            .device
            .cpu_mut()
            .bus_mut()
            .write32(crate::memory::MAR_BASE + 4 * 0xe0, 1);
        let fills = |machine: &mut Machine| machine.cache_views()[1].stats.line_fills;
        let before = fills(&mut machine);
        machine.read_memory(0xe000_0000, 64);
        assert_eq!(fills(&mut machine), before, "an inspection read filled L1D");
    }
    #[test]
    fn multicore_breakpoint_identity_does_not_change_focus() {
        // Exercise both execution paths: a bare budget uses batching, while an additional
        // condition uses per-cycle checks. Held cores must not win simultaneous halt ties.
        for batched in [false, true] {
            let mut machine = Machine::bare(Cpu::new(0));
            machine.device = Device::with_boot_mode(&crate::device::C6472, 0, Endian::Little, 0);
            for core in 0..6 {
                machine.device.with_core(core, |cpu| {
                    cpu.write_words(0, &[0; 32]);
                    cpu.set_pc(if core == 1 || core == 3 { 0 } else { 32 });
                });
            }
            machine.device.with_core(0, |cpu| cpu.set_pc(4));
            machine.device.fabric().set_held_in_reset(0, true);
            machine.set_focus(2);
            machine.arm_halt(&[4]);
            let mut conditions = vec![StopCondition::Budget(100)];
            if !batched {
                conditions.push(StopCondition::Packets(100));
            }
            assert_eq!(
                machine.run(&mut conditions).unwrap(),
                StopReason::Breakpoint(BreakpointHit {
                    core: 1,
                    address: 4
                })
            );
            assert_eq!(machine.focus(), 2);
            let snapshot = machine.snapshot();
            assert_eq!(snapshot.core.pc, 36);
            assert_eq!(
                snapshot.breakpoint,
                Some(BreakpointHit {
                    core: 1,
                    address: 4
                })
            );
            let state = machine.state_json(false);
            assert_eq!(state["breakpoint"], json!({"core": 1, "address": 4}));
            assert_eq!(state["pc"], 36);
            assert_eq!(
                machine.resume(&mut [StopCondition::Budget(1)]).unwrap(),
                StopReason::Budget
            );
            assert_eq!(machine.focus(), 2);
            assert_eq!(machine.device.core(1).pc(), 8);
            assert_eq!(machine.device.core(3).pc(), 8);
        }
    }

    #[test]
    fn resume_excludes_current_addresses_per_core() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.device = Device::with_boot_mode(&crate::device::C6472, 0, Endian::Little, 0);
        for core in 0..6 {
            machine
                .device
                .with_core(core, |cpu| cpu.write_words(0, &[0; 16]));
        }
        machine.device.with_core(0, |cpu| cpu.set_pc(4));
        machine.arm_halt(&[4]);
        // Core 0 leaves its breakpoint, but the same address stays armed for core 1.
        assert_eq!(
            machine.resume(&mut [StopCondition::Budget(100)]).unwrap(),
            StopReason::Breakpoint(BreakpointHit {
                core: 1,
                address: 4
            })
        );
        assert_eq!(machine.focus(), 0);
        assert_eq!(machine.pc(), 8);
    }

    #[test]
    fn resume_restores_halts_after_host_io_errors() {
        for batched in [false, true] {
            let mut machine = Machine::bare(Cpu::new(0));
            machine.device.cpu_mut().write_words(0, &[0; 16]);
            machine.cio = Some((0, 128));
            machine.device.cpu_mut().write_words(128, &[u32::MAX]);
            machine.arm_halt(&[0]);
            let mut conditions = vec![StopCondition::Budget(10)];
            if !batched {
                conditions.push(StopCondition::Packets(10));
            }
            assert!(machine.resume(&mut conditions).is_err());
            // A failed resume did not leave the current breakpoint excluded.
            assert_eq!(
                machine.snapshot().breakpoint,
                Some(BreakpointHit {
                    core: 0,
                    address: 0
                })
            );
            assert_eq!(
                machine.run(&mut []).unwrap(),
                StopReason::Breakpoint(BreakpointHit {
                    core: 0,
                    address: 0
                })
            );
        }
    }

    #[test]
    fn reload_retains_the_halt_rule_without_caller_rearming() {
        let mut machine = Machine::bare(Cpu::new(0));
        machine.stop = Some(8);
        machine.arm_halt(&[0, 4]);
        machine.reload().unwrap();
        assert_eq!(
            machine.run(&mut []).unwrap(),
            StopReason::Breakpoint(BreakpointHit {
                core: 0,
                address: 0
            })
        );
        assert_eq!(
            machine.resume(&mut []).unwrap(),
            StopReason::Breakpoint(BreakpointHit {
                core: 0,
                address: 4
            })
        );
        assert_eq!(machine.resume(&mut []).unwrap(), StopReason::Exit);
    }

    /// The core that reached the CIO handshake answers it, whichever one that is, and both the
    /// batched and the stepped run loop carry it on to the budget. Servicing only the focused
    /// core parked every other core on the trap for good - a secondary core's `printf` on a
    /// C6472 never came back, because the run loop could not advance it.
    #[test]
    fn running_through_nonfocused_cio_reaches_the_cycle_budget() {
        const TRAP: u32 = 0x0080_0200;
        const BUFFER: u32 = 0x0080_1000;
        for batched in [false, true] {
            let mut machine = Machine::bare(Cpu::new(0));
            machine.device = Device::new(&crate::device::C6472, 0, Endian::Little);
            for core in 0..machine.device.cores() {
                machine.device.with_core(core, |cpu| {
                    cpu.write_words(0, &[0; 32]);
                    cpu.set_pc(0x40);
                });
            }
            let mut request = Vec::new();
            request.extend_from_slice(&5u32.to_le_bytes());
            request.push(0xf3);
            request.extend_from_slice(&1i16.to_le_bytes());
            request.extend_from_slice(&5u16.to_le_bytes());
            request.extend_from_slice(&[0; 4]);
            request.extend_from_slice(b"hello");
            machine.device.with_core(1, |cpu| {
                cpu.write_words(TRAP - 4, &[0; 16]);
                cpu.host_write_bytes(BUFFER, &request);
                cpu.set_pc(TRAP - 4);
            });
            machine.cio = Some((TRAP, BUFFER));
            let mut conditions = vec![StopCondition::Budget(4)];
            if !batched {
                conditions.push(StopCondition::Packets(100));
            }
            assert_eq!(machine.run(&mut conditions).unwrap(), StopReason::Budget);
            assert_eq!(machine.cio_output(), b"hello");
            assert_eq!(machine.focus(), 0);
            assert_eq!(machine.device.cycle(), 4);
            assert_eq!(machine.cycles(), 4);
            assert_eq!(machine.pc(), 0x50);
            assert_eq!(machine.device.core(1).cycles(), 4);
            assert_eq!(machine.device.core(1).pc(), TRAP + 12);
            assert_eq!(
                machine
                    .device
                    .with_core(1, |cpu| cpu.host_read_bytes(BUFFER, 6)),
                [0, 0, 0, 0, 5, 0]
            );
        }
    }
}
