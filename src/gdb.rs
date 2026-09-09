//! GDB remote serial protocol stub over TCP: one of the three front ends `debug` offers.
//!
//! The register layout is GDB's own tic6x target description: 0-31 are A0-A15 and B0-B15,
//! 32 CSR, 33 PC, 34-65 the upper halves A16-A31 and B16-B31, then TSR, ILC and RILC.
//! Breakpoints halt in the CPU itself at a fetch boundary, so a branch that resolves straight
//! onto a breakpoint cannot fetch past it in the same cycle; `s` dispatches one execute
//! packet; the machine's own step services the CIO handshake and the Ethernet and HPI
//! backends, so a printf-style or networked program behaves under gdb as it does under `run`;
//! and reaching the program's exit point is reported as an exited process rather than as a
//! target that mysteriously stopped.

use crate::machine::{BreakpointHit, Machine, StopCondition, StopReason};
use crate::system::Endian;
use std::collections::BTreeSet;
use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};

const REGISTER_COUNT: usize = 69;
/// SIGTRAP: a breakpoint, a completed step, or an exhausted run budget.
const TRAP: &str = "S05";
/// SIGILL: the emulator refused to execute something.
const ILLEGAL_INSTRUCTION: &str = "S04";
/// SIGINT: the client interrupted a running target with ^C.
const INTERRUPT: &str = "S02";
/// Cycle budget for one `continue` when a session is driven in process rather than over a
/// socket, which is what the conformance harness does. Without a limit an unreachable
/// breakpoint would hang the caller rather than report a stop.
const IN_PROCESS_BUDGET: u64 = 100_000_000;

pub fn serve(machine: Machine, listen: &str, max_cycles: u64) -> Result<u8, String> {
    let listener = TcpListener::bind(listen).map_err(|error| format!("{listen}: {error}"))?;
    let bound = listener.local_addr().map_err(|error| error.to_string())?;
    println!("GDB listening on {bound}");
    io::stdout().flush().map_err(|error| error.to_string())?;
    let (stream, peer) = listener.accept().map_err(|error| error.to_string())?;
    println!("connection from {peer}");
    let mut session = Session::new(machine);
    session.max_cycles = max_cycles;
    session.breakpoints = session
        .machine
        .initial_breakpoints()
        .iter()
        .copied()
        .collect();
    session.arm_breakpoints();
    session.serve(stream)?;
    Ok(0)
}

pub struct Session {
    machine: Machine,
    breakpoints: BTreeSet<u32>,
    last_stop: String,
    detached: bool,
    /// Cycle budget for one continue; 0 runs without a limit.
    max_cycles: u64,
    /// A second handle on the client socket, polled while the target runs so ^C stops it.
    /// Absent for an in-process session, which cannot be interrupted.
    client: Option<TcpStream>,
}

impl Session {
    pub fn new(machine: Machine) -> Self {
        Self {
            machine,
            breakpoints: BTreeSet::new(),
            last_stop: TRAP.into(),
            detached: false,
            max_cycles: IN_PROCESS_BUDGET,
            client: None,
        }
    }

    pub fn handle(&mut self, packet: &str) -> Result<String, String> {
        if packet == "?" {
            return Ok(self.last_stop.clone());
        }
        if packet == "g" {
            let endian = self.machine.endian();
            return Ok(self
                .registers()
                .into_iter()
                .map(|value| encode_u32(value, endian))
                .collect::<String>());
        }
        if let Some(encoded) = packet.strip_prefix('G') {
            let bytes = decode_hex(encoded)?;
            if bytes.len() != REGISTER_COUNT * 4 {
                return Ok("E01".into());
            }
            let endian = self.machine.endian();
            for (index, chunk) in bytes.chunks_exact(4).enumerate() {
                self.write_register(index, decode_u32(chunk, endian))?;
            }
            return Ok("OK".into());
        }
        if let Some(number) = packet.strip_prefix('p') {
            let index = usize::from_str_radix(number, 16).map_err(|_| "invalid register")?;
            let endian = self.machine.endian();
            return self
                .read_register(index)
                .map(|value| encode_u32(value, endian))
                .ok_or_else(|| "unknown register".to_string());
        }
        if let Some(body) = packet.strip_prefix('P') {
            let (number, value) = body
                .split_once('=')
                .ok_or_else(|| "invalid register write".to_string())?;
            let index = usize::from_str_radix(number, 16).map_err(|_| "invalid register")?;
            let bytes = decode_hex(value)?;
            if bytes.len() != 4 {
                return Ok("E01".into());
            }
            let endian = self.machine.endian();
            self.write_register(index, decode_u32(&bytes, endian))?;
            return Ok("OK".into());
        }
        if let Some(body) = packet.strip_prefix('m') {
            let (address, length) = parse_address_length(body)?;
            return Ok(encode_hex(&self.machine.read_memory(address, length)));
        }
        if let Some(body) = packet.strip_prefix('M') {
            let (range, data) = body
                .split_once(':')
                .ok_or_else(|| "invalid memory write".to_string())?;
            let (address, length) = parse_address_length(range)?;
            let bytes = decode_hex(data)?;
            if bytes.len() != length {
                return Ok("E02".into());
            }
            self.machine.write_memory(address, &bytes);
            return Ok("OK".into());
        }
        if let Some(address) = packet.strip_prefix('c') {
            if !address.is_empty() {
                self.machine.set_pc(parse_hex_u32(address)?);
            }
            return Ok(self.resume());
        }
        if let Some(address) = packet.strip_prefix('s') {
            if !address.is_empty() {
                self.machine.set_pc(parse_hex_u32(address)?);
            }
            return Ok(self.step_packet());
        }
        // Software breakpoints (kind 0) and hardware ones (kind 1) are the same thing to an
        // emulator: the CPU is told to halt rather than having an opcode patched in.
        if let Some(body) = packet
            .strip_prefix("Z0,")
            .or_else(|| packet.strip_prefix("Z1,"))
        {
            let address = parse_hex_u32(body.split(',').next().unwrap_or_default())?;
            if self.breakpoints.insert(address) {
                self.arm_breakpoints();
            }
            return Ok("OK".into());
        }
        if let Some(body) = packet
            .strip_prefix("z0,")
            .or_else(|| packet.strip_prefix("z1,"))
        {
            let address = parse_hex_u32(body.split(',').next().unwrap_or_default())?;
            if self.breakpoints.remove(&address) {
                self.arm_breakpoints();
            }
            return Ok("OK".into());
        }
        if packet.starts_with("qSupported") {
            return Ok("PacketSize=4000;qXfer:features:read+".into());
        }
        if let Some(body) = packet.strip_prefix("qXfer:features:read:target.xml:") {
            let (offset, length) = parse_usize_pair(body)?;
            let xml = target_xml();
            if offset >= xml.len() {
                return Ok("l".into());
            }
            let end = offset.saturating_add(length).min(xml.len());
            let marker = if end == xml.len() { 'l' } else { 'm' };
            return Ok(format!("{marker}{}", &xml[offset..end]));
        }
        if packet == "qAttached" {
            return Ok("1".into());
        }
        if packet == "qC" {
            // Thread ids are one-based, cores zero-based: thread n+1 is core n.
            return Ok(format!("QC{:x}", self.machine.focus() + 1));
        }
        // One thread per core. Without these gdb reports "Remote replied unexpectedly" on
        // `info threads` and on every stop that carries a thread id; with them, `info threads`
        // lists a six-core device's cores and `thread n` selects one.
        if packet == "qfThreadInfo" {
            let threads: Vec<String> = (1..=self.machine.cores())
                .map(|thread| format!("{thread:x}"))
                .collect();
            return Ok(format!("m{}", threads.join(",")));
        }
        if packet == "qsThreadInfo" {
            return Ok("l".into());
        }
        if let Some(thread) = packet.strip_prefix('T').and_then(parse_thread) {
            // "Is this thread alive?" - a core that exists is alive, held in reset or not.
            return Ok(if thread < self.machine.cores() {
                "OK".into()
            } else {
                "E01".into()
            });
        }
        if let Some(name) = packet.strip_prefix("qThreadExtraInfo,") {
            let Some(core) = parse_thread(name) else {
                return Ok("E01".into());
            };
            if core >= self.machine.cores() {
                return Ok("E01".into());
            }
            let held = self.machine.core_held_in_reset(core);
            let text = if held {
                format!("core {core} (held in reset)")
            } else {
                format!("core {core}")
            };
            return Ok(encode_hex(text.as_bytes()));
        }
        if packet == "vCont?" {
            return Ok("vCont;c;C;s;S".into());
        }
        if let Some(actions) = packet.strip_prefix("vCont;") {
            // Actions may carry a thread id ("c:1"); with one thread the id adds nothing.
            return match actions.split(&[':', ';'][..]).next() {
                Some("c") | Some("C") => Ok(self.resume()),
                Some("s") | Some("S") => Ok(self.step_packet()),
                _ => Ok(String::new()),
            };
        }
        // "Hg n" selects the thread the register and memory packets act on, which on a
        // multicore device is which core the session is looking at.
        if let Some(thread) = packet.strip_prefix("Hg").and_then(parse_thread) {
            if thread >= self.machine.cores() {
                return Ok("E01".into());
            }
            self.machine.set_focus(thread);
            return Ok("OK".into());
        }
        if packet.starts_with("Hc") || packet.starts_with("Hg") {
            return Ok("OK".into());
        }
        if packet == "D" || packet == "k" {
            self.detached = true;
            return Ok("OK".into());
        }
        Ok(String::new())
    }

    /// Installs the driver-owned breakpoint list after startup or an actual edit.
    fn arm_breakpoints(&mut self) {
        let breakpoints: Vec<u32> = self.breakpoints.iter().copied().collect();
        self.machine.arm_halt(&breakpoints);
    }

    /// Runs until a breakpoint, the program's exit, a fault, the client's ^C, or the budget.
    fn resume(&mut self) -> String {
        let client = &mut self.client;
        let mut conditions = [
            StopCondition::Budget(self.max_cycles),
            StopCondition::Check(&mut |_machine: &Machine| {
                client
                    .as_mut()
                    .map(client_interrupted)
                    .unwrap_or(false)
                    .then_some(StopReason::Interrupted)
            }),
        ];
        let reply = match self.machine.resume(&mut conditions) {
            Ok(StopReason::Breakpoint(hit)) => breakpoint_stop(hit, self.machine.endian()),
            Ok(StopReason::Fault) => ILLEGAL_INSTRUCTION.to_string(),
            Ok(StopReason::Exit) => self.exited(),
            Ok(StopReason::Interrupted) => INTERRUPT.to_string(),
            Ok(_) => TRAP.to_string(),
            Err(error) => {
                eprintln!("c64xemu: gdb stub: {error}");
                TRAP.to_string()
            }
        };
        self.last_stop = reply.clone();
        reply
    }

    /// Dispatches one execute packet. Stalls mean a cycle need not issue anything, so this
    /// steps cycles until the packet count moves - which is what a `stepi` should do.
    fn step_packet(&mut self) -> String {
        let mut conditions = [StopCondition::Packets(1)];
        let reply = match self.machine.resume(&mut conditions) {
            Ok(StopReason::Breakpoint(hit)) => breakpoint_stop(hit, self.machine.endian()),
            Ok(StopReason::Fault) => ILLEGAL_INSTRUCTION.into(),
            Ok(StopReason::Exit) => self.exited(),
            Ok(_) => TRAP.into(),
            Err(error) => {
                eprintln!("c64xemu: gdb stub: {error}");
                TRAP.into()
            }
        };
        self.last_stop = reply.clone();
        reply
    }

    /// The program reached its exit point: report a process exit carrying A4, the EABI return
    /// register, which is the same code `run` exits with.
    fn exited(&mut self) -> String {
        let _ = self.machine.finish();
        format!("W{:02x}", self.machine.exit_code())
    }
    fn registers(&mut self) -> Vec<u32> {
        let snapshot = self.machine.snapshot();
        let mut values = Vec::with_capacity(REGISTER_COUNT);
        values.extend_from_slice(&snapshot.core.a[0..16]);
        values.extend_from_slice(&snapshot.core.b[0..16]);
        values.push(snapshot.core.control[0x01]);
        values.push(snapshot.core.pc);
        values.extend_from_slice(&snapshot.core.a[16..32]);
        values.extend_from_slice(&snapshot.core.b[16..32]);
        values.push(snapshot.core.control[0x1a]);
        values.push(snapshot.core.control[0x0d]);
        values.push(snapshot.core.control[0x0e]);
        values
    }

    fn read_register(&mut self, index: usize) -> Option<u32> {
        let snapshot = self.machine.snapshot();
        match index {
            0..=15 => Some(snapshot.core.a[index]),
            16..=31 => Some(snapshot.core.b[index - 16]),
            32 => Some(snapshot.core.control[0x01]),
            33 => Some(snapshot.core.pc),
            34..=49 => Some(snapshot.core.a[index - 18]),
            50..=65 => Some(snapshot.core.b[index - 34]),
            66 => Some(snapshot.core.control[0x1a]),
            67 => Some(snapshot.core.control[0x0d]),
            68 => Some(snapshot.core.control[0x0e]),
            _ => None,
        }
    }

    fn write_register(&mut self, index: usize, value: u32) -> Result<(), String> {
        match index {
            0..=15 => self.machine.set_register(&format!("A{index}"), value),
            16..=31 => self
                .machine
                .set_register(&format!("B{}", index - 16), value),
            32 => {
                self.machine.write_control(0x01, value);
                Ok(())
            }
            33 => {
                self.machine.set_pc(value);
                Ok(())
            }
            34..=49 => self
                .machine
                .set_register(&format!("A{}", index - 18), value),
            50..=65 => self
                .machine
                .set_register(&format!("B{}", index - 34), value),
            66 => {
                self.machine.write_control(0x1a, value);
                Ok(())
            }
            67 => {
                self.machine.write_control(0x0d, value);
                Ok(())
            }
            68 => {
                self.machine.write_control(0x0e, value);
                Ok(())
            }
            _ => Err("unknown register".into()),
        }
    }

    fn serve(mut self, mut stream: TcpStream) -> Result<(), String> {
        stream
            .set_nodelay(true)
            .map_err(|error| error.to_string())?;
        self.client = stream.try_clone().ok();
        while !self.detached {
            let Some(packet) = read_packet(&mut stream)? else {
                break;
            };
            stream.write_all(b"+").map_err(|error| error.to_string())?;
            let reply = self.handle(&packet).unwrap_or_else(|_| "E01".into());
            write_packet(&mut stream, &reply)?;
        }
        Ok(())
    }
}

fn breakpoint_stop(hit: BreakpointHit, endian: Endian) -> String {
    format!(
        "T05thread:{:x};21:{};",
        hit.core + 1,
        encode_u32(hit.address, endian)
    )
}

/// True when the client sent ^C. Any other byte arriving while the target runs is protocol
/// noise - gdb sends nothing else until it sees a stop reply - and is dropped.
fn client_interrupted(client: &mut TcpStream) -> bool {
    if client.set_nonblocking(true).is_err() {
        return false;
    }
    let mut byte = [0u8; 1];
    let interrupted = matches!(client.read(&mut byte), Ok(1) if byte[0] == 3);
    let _ = client.set_nonblocking(false);
    interrupted
}

fn read_packet(stream: &mut TcpStream) -> Result<Option<String>, String> {
    let mut byte = [0u8; 1];
    loop {
        match stream.read_exact(&mut byte) {
            Ok(()) if byte[0] == b'$' => break,
            Ok(()) if byte[0] == 3 => return Ok(Some("?".into())),
            Ok(()) => {}
            Err(error) if error.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(error) => return Err(error.to_string()),
        }
    }
    let mut payload = Vec::new();
    loop {
        stream
            .read_exact(&mut byte)
            .map_err(|error| error.to_string())?;
        if byte[0] == b'#' {
            break;
        }
        payload.push(byte[0]);
    }
    let mut checksum = [0u8; 2];
    stream
        .read_exact(&mut checksum)
        .map_err(|error| error.to_string())?;
    let expected = u8::from_str_radix(std::str::from_utf8(&checksum).map_err(|_| "checksum")?, 16)
        .map_err(|_| "invalid checksum")?;
    let actual = payload
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
    if actual != expected {
        stream.write_all(b"-").map_err(|error| error.to_string())?;
        return Err("remote packet checksum mismatch".into());
    }
    String::from_utf8(payload)
        .map(Some)
        .map_err(|error| error.to_string())
}

fn write_packet(stream: &mut TcpStream, payload: &str) -> Result<(), String> {
    let checksum = payload
        .as_bytes()
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
    write!(stream, "${payload}#{checksum:02x}").map_err(|error| error.to_string())
}

fn parse_address_length(body: &str) -> Result<(u32, usize), String> {
    let (address, length) = body
        .split_once(',')
        .ok_or_else(|| "invalid address range".to_string())?;
    Ok((parse_hex_u32(address)?, parse_hex_usize(length)?))
}

fn parse_usize_pair(body: &str) -> Result<(usize, usize), String> {
    let (first, second) = body
        .split_once(',')
        .ok_or_else(|| "invalid pair".to_string())?;
    Ok((parse_hex_usize(first)?, parse_hex_usize(second)?))
}

fn parse_hex_u32(text: &str) -> Result<u32, String> {
    u32::from_str_radix(text, 16).map_err(|_| "invalid hexadecimal value".into())
}

fn parse_hex_usize(text: &str) -> Result<usize, String> {
    usize::from_str_radix(text, 16).map_err(|_| "invalid hexadecimal value".into())
}

fn decode_hex(text: &str) -> Result<Vec<u8>, String> {
    if text.len() % 2 != 0 {
        return Err("odd hexadecimal byte string".into());
    }
    text.as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let pair = std::str::from_utf8(pair).map_err(|_| "invalid hexadecimal byte")?;
            u8::from_str_radix(pair, 16).map_err(|_| "invalid hexadecimal byte".into())
        })
        .collect()
}

/// A gdb thread id as a core number: thread ids are one-based, cores are zero-based, and "-1"
/// or "0" mean "any thread", which is the focused core.
fn parse_thread(text: &str) -> Option<usize> {
    let text = text.trim();
    if text.is_empty() || text == "-1" || text == "0" {
        return Some(0);
    }
    usize::from_str_radix(text.trim_start_matches('p'), 16)
        .ok()
        .and_then(|thread| thread.checked_sub(1))
}

fn encode_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn encode_u32(value: u32, endian: Endian) -> String {
    let bytes = match endian {
        Endian::Little => value.to_le_bytes(),
        Endian::Big => value.to_be_bytes(),
    };
    encode_hex(&bytes)
}

fn decode_u32(bytes: &[u8], endian: Endian) -> u32 {
    let bytes: [u8; 4] = bytes.try_into().unwrap();
    match endian {
        Endian::Little => u32::from_le_bytes(bytes),
        Endian::Big => u32::from_be_bytes(bytes),
    }
}

fn target_xml() -> String {
    let mut xml = String::from("<?xml version=\"1.0\"?><target><architecture>tic6x</architecture>");
    xml.push_str("<feature name=\"org.gnu.gdb.tic6x.core\">");
    for bank in ['A', 'B'] {
        for index in 0..16 {
            xml.push_str(&format!("<reg name=\"{bank}{index}\" bitsize=\"32\"/>"));
        }
    }
    xml.push_str("<reg name=\"CSR\" bitsize=\"32\"/><reg name=\"PC\" bitsize=\"32\" type=\"code_ptr\"/></feature>");
    xml.push_str("<feature name=\"org.gnu.gdb.tic6x.gp\">");
    for bank in ['A', 'B'] {
        for index in 16..32 {
            xml.push_str(&format!("<reg name=\"{bank}{index}\" bitsize=\"32\"/>"));
        }
    }
    xml.push_str("</feature><feature name=\"org.gnu.gdb.tic6x.c6xp\"><reg name=\"TSR\" bitsize=\"32\"/><reg name=\"ILC\" bitsize=\"32\"/><reg name=\"RILC\" bitsize=\"32\"/></feature></target>");
    xml
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::Cpu;
    use crate::device::{Device, C6472};

    #[test]
    fn breakpoint_reply_identifies_the_hit_without_selecting_its_core() {
        for endian in [Endian::Little, Endian::Big] {
            let mut machine = Machine::bare(Cpu::new(0));
            machine.device = Device::with_boot_mode(&C6472, 0, endian, 0);
            for core in 0..6 {
                machine
                    .device
                    .with_core(core, |cpu| cpu.write_words(0, &[0; 16]));
            }
            machine.device.with_core(0, |cpu| cpu.set_pc(32));
            let mut session = Session::new(machine);
            session.handle("Z0,4,4").unwrap();
            let stopped_pc = match endian {
                Endian::Little => "04000000",
                Endian::Big => "00000004",
            };
            let reply = format!("T05thread:2;21:{stopped_pc};");
            assert_eq!(session.handle("c").unwrap(), reply);
            assert_eq!(session.handle("?").unwrap(), reply);
            assert_eq!(session.handle("qC").unwrap(), "QC1");
            assert_eq!(
                session.handle("p21").unwrap(),
                match endian {
                    Endian::Little => "24000000",
                    Endian::Big => "00000024",
                }
            );
            session.handle("Hg2").unwrap();
            assert_eq!(session.handle("?").unwrap(), reply);
            assert_eq!(session.handle("p21").unwrap(), stopped_pc);
            assert_eq!(session.handle("s").unwrap(), "S05");
            assert_eq!(session.handle("?").unwrap(), "S05");
            assert_eq!(session.handle("qC").unwrap(), "QC2");
        }
    }
}
