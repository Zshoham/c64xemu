//! The HPI host side driven from a timed CSV file (`--hpi-replay`).
//!
//! It is the HPI host's second adapter beside the datagram socket (`host::HpiHost`): serviced
//! from the run loop at a cycle boundary, through the same `Fabric::hpi_host_read` and
//! `hpi_host_write` the socket host uses, so the FIFOs, bursts, DSPINT and the C6472 boot
//! controller behave exactly as they do for a live host. What the file adds is time: each row
//! names the run-clock tick it happens at, and `next_due` tells the run loop how far it may
//! batch, so every access lands on its tick however the loop is batching.
//!
//! The file is parsed once, when the machine is built, and is never a load phase: nothing runs
//! until the run loop reaches a row's tick, and a reload only rewinds it.
//!
//! ```text
//! TICK,PORT,write,VALUE
//! TICK,PORT,read,-                       report the value
//! TICK,PORT,read,VALUE                   exact 32-bit expectation
//! TICK,control,wait,VALUE,MASK,WITHIN    poll HPIC until (hpic & MASK) == VALUE
//! TICK := N | +D                         absolute tick, or D after the previous row completed
//! PORT := control | data-auto | address | data    HCNTL 00 / 01 / 10 / 11, SPRU969B Table 4
//! ```

use crate::hpi::HostPort;
use crate::system::Fabric;
use serde_json::{json, Value};
use std::path::{Path, PathBuf};

/// When a row runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum When {
    /// At this absolute tick of the run clock.
    At(u64),
    /// This many ticks after the previous row completed (a wait completes when it matches).
    After(u64),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Op {
    Write(u32),
    /// A read, checked against the value when there is one and reported when there is not.
    Read(Option<u32>),
    /// Sample HPIC every tick from the one the row is due at through `within` ticks after it.
    Wait {
        value: u32,
        mask: u32,
        within: u64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Line {
    /// The line of the file the row is on, counted from one, for every message about it.
    number: usize,
    when: When,
    port: HostPort,
    op: Op,
}

/// A read the file asked to have reported rather than checked.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Readback {
    pub line: usize,
    pub tick: u64,
    pub port: HostPort,
    pub value: u32,
}

/// The first thing that went wrong. It stops the file for good, until a rewind.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Failure {
    pub line: usize,
    pub tick: u64,
    /// The whole report, file and line included, which is what `Machine::fault` answers.
    pub message: String,
}

/// A timed file of HPI host accesses and how far a run has got through it.
pub(crate) struct HostFile {
    path: PathBuf,
    /// Parsed once at load, and kept across reloads.
    lines: Vec<Line>,
    // ----- run state, reset by `rewind` on every install -----
    next: usize,
    /// The tick the previous row completed, which a `+D` row counts from; zero before the first.
    previous: u64,
    /// The tick the current row's wait started, while it has not matched.
    waiting: Option<u64>,
    reads: Vec<Readback>,
    failure: Option<Failure>,
}

impl HostFile {
    /// Reads and validates the file. Nothing in it runs until the run loop reaches its ticks.
    pub(crate) fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|error| format!("{}: {error}", path.display()))?;
        Self::parse(path, &text)
    }

    pub(crate) fn parse(path: &Path, text: &str) -> Result<Self, String> {
        let error =
            |number: usize, message: String| format!("{} line {number}: {message}", path.display());
        let mut lines = Vec::new();
        let mut meaningful = 0usize;
        let mut last_absolute = 0u64;
        for (index, raw) in text.lines().enumerate() {
            let number = index + 1;
            // A `#` starts a comment anywhere on the line, so a commented header is a comment.
            let content = raw.split_once('#').map_or(raw, |(before, _)| before).trim();
            if content.is_empty() {
                continue;
            }
            meaningful += 1;
            let fields: Vec<&str> = content.split(',').map(str::trim).collect();
            if is_header(&fields) {
                if meaningful != 1 {
                    return Err(error(
                        number,
                        "a header is only allowed as the first row".into(),
                    ));
                }
                continue;
            }
            let line = parse_row(number, &fields).map_err(|message| error(number, message))?;
            if let When::At(tick) = line.when {
                if tick < last_absolute {
                    return Err(error(
                        number,
                        format!("tick {tick} is before the earlier tick {last_absolute}"),
                    ));
                }
                last_absolute = tick;
            }
            lines.push(line);
        }
        if lines.is_empty() {
            return Err(format!("{}: no accesses in the file", path.display()));
        }
        Ok(Self {
            path: path.to_owned(),
            lines,
            next: 0,
            previous: 0,
            waiting: None,
            reads: Vec::new(),
            failure: None,
        })
    }

    /// Back to the first row, with nothing performed: the device has just been (re)built.
    pub(crate) fn rewind(&mut self) {
        self.next = 0;
        self.previous = 0;
        self.waiting = None;
        self.reads.clear();
        self.failure = None;
    }

    /// Runs every row due at `now`, in file order, until a row is due later, the file ends,
    /// a row fails, a wait has not matched yet, or strict physical-memory mode has refused an
    /// access. The refusal is checked after every row, so no later row touches a target the
    /// diagnostic has already stopped the run on. Rows already performed stay performed.
    pub(crate) fn service(&mut self, fabric: &mut Fabric, now: u64) {
        // A refusal the program made: the run stops for it before anything here runs.
        if fabric.strict_stopped() {
            return;
        }
        while self.failure.is_none() && self.next < self.lines.len() {
            let line = self.lines[self.next];
            if let Op::Wait {
                value,
                mask,
                within,
            } = line.op
            {
                let start = match self.waiting {
                    Some(start) => start,
                    None => match self.due() {
                        Some(due) if due > now => return,
                        Some(due) if due < now => return self.missed(line, due, now),
                        Some(_) => *self.waiting.insert(now),
                        None => return self.overflow(line, now),
                    },
                };
                // A host read of HPIC has no side effects (`Hpi::host_read`), so sampling it
                // every tick changes nothing the firmware can see; a held-off sample is no match.
                // A window past the end of the clock is reported, not quietly shortened.
                let Some(deadline) = start.checked_add(within) else {
                    return self.fail(
                        line,
                        now,
                        format!(
                            "the wait's window ends past the end of the clock (started at {start})"
                        ),
                    );
                };
                let sample = fabric.hpi_host_read(HostPort::Control);
                match sample {
                    Some(hpic) if hpic & mask == value => {
                        eprintln!(
                            "hpi-replay line={} tick={now} port=control wait=0x{value:08x} \
                             mask=0x{mask:08x} matched=0x{hpic:08x}",
                            line.number
                        );
                        self.complete(now);
                    }
                    _ if now >= deadline => {
                        let actual = match sample {
                            Some(hpic) => format!("last read 0x{hpic:08x}"),
                            None => "held off by HPIRST".into(),
                        };
                        return self.fail(
                            line,
                            now,
                            format!(
                                "wait for HPIC & 0x{mask:08x} == 0x{value:08x} timed out at \
                                 tick {deadline} (started at {start}): {actual}"
                            ),
                        );
                    }
                    _ => return,
                }
            } else {
                match self.due() {
                    Some(due) if due > now => return,
                    Some(due) if due < now => return self.missed(line, due, now),
                    Some(_) => {}
                    None => return self.overflow(line, now),
                }
                if fabric.hpi_host_held_off() {
                    let access = if matches!(line.op, Op::Write(_)) {
                        "write"
                    } else {
                        "read"
                    };
                    return self.fail(
                        line,
                        now,
                        format!(
                            "{} {access} held off: HPIRST is set (SPRU969B section 8.3)",
                            port_name(line.port)
                        ),
                    );
                }
                match line.op {
                    Op::Write(value) => fabric.hpi_host_write(line.port, value),
                    Op::Read(expected) => {
                        let value = fabric
                            .hpi_host_read(line.port)
                            .expect("an HPI the host is not held off from answers");
                        match expected {
                            Some(expected) if expected != value => {
                                return self.fail(
                                    line,
                                    now,
                                    format!(
                                        "{} read 0x{value:08x}, expected 0x{expected:08x}",
                                        port_name(line.port)
                                    ),
                                );
                            }
                            Some(_) => {}
                            None => {
                                eprintln!(
                                    "hpi-replay line={} tick={now} port={} read=0x{value:08x}",
                                    line.number,
                                    port_name(line.port)
                                );
                                self.reads.push(Readback {
                                    line: line.number,
                                    tick: now,
                                    port: line.port,
                                    value,
                                });
                            }
                        }
                    }
                    Op::Wait { .. } => unreachable!(),
                }
                self.complete(now);
            }
            if fabric.strict_stopped() {
                let performed = self.lines[self.next - 1];
                return self.fail(
                    performed,
                    now,
                    "strict physical memory refused the access".into(),
                );
            }
        }
    }

    /// How many ticks the run loop may advance from `now` before this file is due again, never
    /// zero: one while a wait is sampling, `None` once the file is finished or has failed.
    pub(crate) fn next_due(&self, now: u64) -> Option<u64> {
        if self.failure.is_some() || self.next >= self.lines.len() {
            return None;
        }
        if self.waiting.is_some() {
            return Some(1);
        }
        // A due tick past the clock's range is reported by `service`, which has to run for it.
        Some(self.due().map_or(1, |due| due.saturating_sub(now).max(1)))
    }

    /// Whether the file has stopped on a failure; the run must not advance past it.
    pub(crate) fn failed(&self) -> bool {
        self.failure.is_some()
    }

    pub(crate) fn failure(&self) -> Option<&Failure> {
        self.failure.as_ref()
    }

    /// The line of the next row to run, while the file has neither finished nor failed.
    pub(crate) fn pending_line(&self) -> Option<usize> {
        if self.failure.is_some() {
            return None;
        }
        self.lines.get(self.next).map(|line| line.number)
    }

    #[cfg(test)]
    pub(crate) fn reads(&self) -> &[Readback] {
        &self.reads
    }

    /// What the state report says about the file at run-clock tick `now`.
    pub(crate) fn json(&self, now: u64) -> Value {
        let next = self.lines.get(self.next).filter(|_| self.failure.is_none());
        json!({
            "file": self.path.display().to_string(),
            "tick": now,
            "lines": self.lines.len(),
            "next_line": next.map(|line| line.number),
            "next_tick": next.and_then(|_| self.waiting.or_else(|| self.due())),
            "complete": self.failure.is_none() && self.next >= self.lines.len(),
            "reads": self.reads.iter().map(|read| json!({
                "line": read.line,
                "tick": read.tick,
                "port": port_name(read.port),
                "value": format!("0x{:08x}", read.value),
            })).collect::<Vec<_>>(),
            "failure": self.failure.as_ref().map(|failure| json!({
                "line": failure.line,
                "tick": failure.tick,
                "message": failure.message,
            })),
        })
    }

    /// The tick the next row is due at; `None` when counting it overflows the clock.
    fn due(&self) -> Option<u64> {
        match self.lines[self.next].when {
            When::At(tick) => Some(tick),
            When::After(delay) => self.previous.checked_add(delay),
        }
    }

    fn complete(&mut self, now: u64) {
        self.next += 1;
        self.previous = now;
        self.waiting = None;
    }

    fn missed(&mut self, line: Line, due: u64, now: u64) {
        self.fail(
            line,
            now,
            format!(
                "missed tick: due at tick {due}, but the previous row completed at tick {}; \
                 use +D for a row that follows a wait",
                self.previous
            ),
        );
    }

    fn overflow(&mut self, line: Line, now: u64) {
        self.fail(
            line,
            now,
            "the row's tick is past the end of the clock".into(),
        );
    }

    fn fail(&mut self, line: Line, tick: u64, message: String) {
        self.failure = Some(Failure {
            line: line.number,
            tick,
            message: format!(
                "hpi replay {} line {} tick {tick}: {message}",
                self.path.display(),
                line.number
            ),
        });
    }
}

/// The optional header, with or without the two wait columns.
fn is_header(fields: &[&str]) -> bool {
    matches!(
        fields,
        ["tick", "port", "direction", "value"]
            | ["tick", "port", "direction", "value", "mask", "within"]
    )
}

fn parse_row(number: usize, fields: &[&str]) -> Result<Line, String> {
    let [tick, port, direction, rest @ ..] = fields else {
        return Err(format!(
            "expected tick,port,direction,value; found {} fields",
            fields.len()
        ));
    };
    let when = match tick.strip_prefix('+') {
        Some(delay) => When::After(number_u64(delay, "tick")?),
        None => When::At(number_u64(tick, "tick")?),
    };
    let port = match *port {
        "control" => HostPort::Control,
        "data-auto" => HostPort::DataAutoIncrement,
        "address" => HostPort::Address,
        "data" => HostPort::Data,
        other => {
            return Err(format!(
                "unknown port {other:?}: expected control, data-auto, address or data"
            ))
        }
    };
    let op = match (*direction, rest) {
        ("write", [value]) => Op::Write(number_u32(value, "value")?),
        ("read", ["-"]) => Op::Read(None),
        ("read", [value]) => Op::Read(Some(number_u32(value, "value")?)),
        ("wait", [value, mask, within]) => {
            if port != HostPort::Control {
                return Err("wait polls HPIC, so its port must be control".into());
            }
            let value = number_u32(value, "value")?;
            let mask = number_u32(mask, "mask")?;
            let within = number_u64(within, "within")?;
            if mask == 0 {
                return Err("a wait's mask must select at least one bit".into());
            }
            if value & !mask != 0 {
                return Err(format!(
                    "wait value 0x{value:08x} has bits outside mask 0x{mask:08x}"
                ));
            }
            if within == 0 {
                return Err("a wait's within must be at least one tick".into());
            }
            Op::Wait {
                value,
                mask,
                within,
            }
        }
        ("write" | "read", _) => {
            return Err(format!(
                "a {direction} has 4 fields; found {}",
                fields.len()
            ))
        }
        ("wait", _) => return Err(format!("a wait has 6 fields; found {}", fields.len())),
        (other, _) => {
            return Err(format!(
                "unknown direction {other:?}: expected write, read or wait"
            ))
        }
    };
    Ok(Line {
        number,
        when,
        port,
        op,
    })
}

/// A decimal or `0x` hexadecimal number. Anything else, including a sign or an empty field, is
/// an error rather than a guess.
fn number_u64(text: &str, what: &str) -> Result<u64, String> {
    let (digits, radix) = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => (hex, 16),
        None => (text, 10),
    };
    if digits.is_empty() || !digits.chars().all(|digit| digit.is_digit(radix)) {
        return Err(format!("invalid {what} {text:?}"));
    }
    u64::from_str_radix(digits, radix).map_err(|_| format!("{what} {text} is out of range"))
}

fn number_u32(text: &str, what: &str) -> Result<u32, String> {
    let value = number_u64(text, what)?;
    u32::try_from(value).map_err(|_| format!("{what} {text} does not fit in 32 bits"))
}

/// The port's name in the file, which is also how every report names it.
pub(crate) fn port_name(port: HostPort) -> &'static str {
    match port {
        HostPort::Control => "control",
        HostPort::DataAutoIncrement => "data-auto",
        HostPort::Address => "address",
        HostPort::Data => "data",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const L2: u32 = 0x0080_0000;

    fn parse(text: &str) -> Result<HostFile, String> {
        HostFile::parse(Path::new("host.csv"), text)
    }

    fn file(text: &str) -> HostFile {
        parse(text).unwrap()
    }

    /// The CPU writing HPIC to zero, which releases the software reset that holds the host off
    /// (SPRU969B section 7.2).
    fn open_port(fabric: &mut Fabric) {
        fabric.write32(0, 0x0288_0030, 0);
    }

    /// Services the file at every tick from `from` through `to`, as the stepped loop would.
    fn run(file: &mut HostFile, fabric: &mut Fabric, from: u64, to: u64) {
        for tick in from..=to {
            file.service(fabric, tick);
        }
    }

    #[test]
    fn the_parser_takes_headers_comments_and_relative_ticks() {
        let parsed = file(
            "tick,port,direction,value,mask,within\n\
             \n\
             # a comment\n\
             0 , address , write , 0x00200000   # trailing\n\
             +5,data-auto,read,-\n\
             7,control,wait,0x8,0x8,10\n\
             7,data,read,42\n",
        );
        assert_eq!(
            parsed.lines,
            [
                Line {
                    number: 4,
                    when: When::At(0),
                    port: HostPort::Address,
                    op: Op::Write(0x0020_0000),
                },
                Line {
                    number: 5,
                    when: When::After(5),
                    port: HostPort::DataAutoIncrement,
                    op: Op::Read(None),
                },
                Line {
                    number: 6,
                    when: When::At(7),
                    port: HostPort::Control,
                    op: Op::Wait {
                        value: 8,
                        mask: 8,
                        within: 10,
                    },
                },
                Line {
                    number: 7,
                    when: When::At(7),
                    port: HostPort::Data,
                    op: Op::Read(Some(42)),
                },
            ]
        );
        assert_eq!(
            file("# tick,port,direction,value\n0,control,write,0\n")
                .lines
                .len(),
            1
        );
        assert_eq!(
            file("tick,port,direction,value\n0,control,write,0")
                .lines
                .len(),
            1
        );
    }

    #[test]
    fn the_parser_rejects_anything_it_would_have_to_guess_at() {
        for (text, message) in [
            ("", "host.csv: no accesses in the file"),
            ("# only\n\ntick,port,direction,value\n", "no accesses"),
            (
                "0,control,write,0\ntick,port,direction,value",
                "line 2: a header is only",
            ),
            (
                "5,control,write,0\n4,control,write,0",
                "line 2: tick 4 is before",
            ),
            ("0,control,write", "line 1: a write has 4 fields; found 3"),
            (
                "0,control",
                "line 1: expected tick,port,direction,value; found 2",
            ),
            (
                "0,control,write,0,1",
                "line 1: a write has 4 fields; found 5",
            ),
            (
                "0,control,read,-,1,2",
                "line 1: a read has 4 fields; found 6",
            ),
            ("0,control,wait,8,8", "line 1: a wait has 6 fields; found 5"),
            ("0,control,w,0", "unknown direction \"w\""),
            ("0,control,r,-", "unknown direction \"r\""),
            ("0,0,write,0", "unknown port \"0\""),
            ("0,data-auto-increment,write,0", "unknown port"),
            ("0,hpic,write,0", "unknown port"),
            (
                "0,control,write,0x100000000",
                "value 0x100000000 does not fit",
            ),
            ("0,control,write,-1", "invalid value \"-1\""),
            ("0,control,write,-", "invalid value \"-\""),
            ("x,control,write,0", "invalid tick \"x\""),
            ("++1,control,write,0", "invalid tick \"+1\""),
            (
                "18446744073709551616,control,write,0",
                "tick 18446744073709551616 is out",
            ),
            ("0,data,wait,8,8,1", "its port must be control"),
            ("0,control,wait,8,0,1", "mask must select"),
            ("0,control,wait,9,8,1", "outside mask"),
            ("0,control,wait,8,8,0", "within must be at least one"),
        ] {
            let error = parse(text)
                .err()
                .unwrap_or_else(|| panic!("{text:?} parsed"));
            assert!(error.contains(message), "{text:?}: {error}");
        }
    }

    /// Rows at one tick run back to back, in file order, before that tick's cycle; the next
    /// row waits for its own tick.
    #[test]
    fn rows_due_at_one_tick_run_in_file_order() {
        let mut fabric = Fabric::default();
        open_port(&mut fabric);
        let mut replay = file(
            "0,address,write,0x00200000\n\
             0,data,write,0x11111111\n\
             0,address,write,0x00200001\n\
             0,data,write,0x22222222\n\
             0,address,write,0x00200000\n\
             0,data,read,-\n\
             +3,data,read,-\n",
        );
        assert_eq!(replay.next_due(0), Some(1), "the first row is due at once");
        replay.service(&mut fabric, 0);
        assert_eq!(replay.reads().len(), 1);
        assert_eq!(replay.reads()[0].value, 0x1111_1111);
        assert_eq!(replay.next_due(0), Some(3));
        assert_eq!(replay.next_due(2), Some(1));
        replay.service(&mut fabric, 2);
        assert_eq!(replay.reads().len(), 1, "not yet due");
        replay.service(&mut fabric, 3);
        assert_eq!(replay.reads()[1].tick, 3);
        assert_eq!(replay.pending_line(), None);
        assert_eq!(replay.next_due(3), None, "a finished file is never due");
    }

    /// One to three autoincrement writes sit in the write FIFO until something flushes it
    /// (SPRU969B sections 6.2 and 6.3): the file has to fence, nothing does it for it. A host
    /// read would flush the FIFO itself, so the backing store is what is looked at.
    #[test]
    fn short_autoincrement_runs_reach_memory_only_at_a_fence() {
        for words in 1..=3u32 {
            let mut fabric = Fabric::default();
            open_port(&mut fabric);
            let mut text = String::from("0,address,write,0x00200000\n");
            for word in 0..words {
                text += &format!("0,data-auto,write,{}\n", 0x100 + word);
            }
            text += "1,address,write,0x00200000\n";
            let mut replay = file(&text);
            replay.service(&mut fabric, 0);
            for word in 0..words {
                assert_eq!(fabric.read32(0, L2 + 4 * word), 0, "{words} words posted");
            }
            replay.service(&mut fabric, 1);
            for word in 0..words {
                assert_eq!(
                    fabric.read32(0, L2 + 4 * word),
                    0x100 + word,
                    "after the fence"
                );
            }
        }
    }

    #[test]
    fn an_access_the_software_reset_holds_off_fails_the_file() {
        for (row, access) in [("0,address,write,1", "write"), ("0,control,read,-", "read")] {
            let mut fabric = Fabric::default();
            let mut replay = file(&format!("{row}\n1,control,read,-\n"));
            replay.service(&mut fabric, 0);
            let failure = replay.failure().unwrap();
            assert_eq!((failure.line, failure.tick), (1, 0));
            assert!(
                failure.message.contains(&format!(
                    "line 1 tick 0: {} {access} held off",
                    row.split(',').nth(1).unwrap()
                )),
                "{}",
                failure.message
            );
            assert_eq!(replay.next_due(0), None);
            assert_eq!(replay.pending_line(), None);
            open_port(&mut fabric);
            replay.service(&mut fabric, 1);
            assert!(replay.reads().is_empty(), "nothing runs after a failure");
        }
    }

    #[test]
    fn a_read_mismatch_fails_and_keeps_the_reads_before_it() {
        let mut fabric = Fabric::default();
        open_port(&mut fabric);
        let mut replay = file(
            "0,address,write,0x00200000\n\
             0,address,read,-\n\
             0,address,read,0x00200001\n\
             0,address,read,-\n",
        );
        replay.service(&mut fabric, 0);
        assert_eq!(
            replay.reads(),
            [Readback {
                line: 2,
                tick: 0,
                port: HostPort::Address,
                value: 0x0020_0000,
            }]
        );
        assert_eq!(
            replay.failure().unwrap().message,
            "hpi replay host.csv line 3 tick 0: address read 0x00200000, expected 0x00200001"
        );
        let state = replay.json(5);
        assert_eq!(state["complete"], false);
        assert_eq!(state["next_line"], Value::Null);
        assert_eq!(state["failure"]["line"], 3);
        assert_eq!(state["reads"][0]["port"], "address");
        assert_eq!(state["reads"][0]["value"], "0x00200000");
    }

    /// A wait samples HPIC at the tick it becomes due and at every tick through `WITHIN` after
    /// it; a sample at the last tick still matches, and the next row counts from the match.
    #[test]
    fn a_wait_samples_every_tick_through_its_window() {
        // HINT, which the CPU raises by writing it to HPIC (SPRU969B section 5.2).
        let raise_hint = |fabric: &mut Fabric| fabric.write32(0, 0x0288_0030, 4);
        let text = "10,control,wait,0x4,0x4,5\n+2,control,read,-\n";
        for (raised_at, matched) in [(10, Some(10)), (13, Some(13)), (15, Some(15)), (16, None)] {
            let mut fabric = Fabric::default();
            open_port(&mut fabric);
            let mut replay = file(text);
            assert_eq!(replay.next_due(0), Some(10));
            for tick in 0..=20 {
                if tick == raised_at {
                    raise_hint(&mut fabric);
                }
                replay.service(&mut fabric, tick);
                if (10..15).contains(&tick) && matched.is_none_or(|at| tick < at) {
                    assert_eq!(replay.next_due(tick), Some(1), "sampling at {tick}");
                }
            }
            match matched {
                Some(at) => {
                    assert!(replay.failure().is_none(), "raised at {raised_at}");
                    assert_eq!(replay.reads()[0].tick, at + 2);
                    assert_eq!(replay.reads()[0].value & 4, 4);
                }
                None => {
                    let failure = replay.failure().unwrap();
                    assert_eq!(failure.tick, 15);
                    assert!(
                        failure
                            .message
                            .contains("timed out at tick 15 (started at 10): last read 0x00000008"),
                        "{}",
                        failure.message
                    );
                }
            }
        }
    }

    #[test]
    fn a_wait_that_is_held_off_never_matches() {
        let mut fabric = Fabric::default();
        let mut replay = file("0,control,wait,0x0,0x8,3\n");
        run(&mut replay, &mut fabric, 0, 3);
        assert!(replay
            .failure()
            .unwrap()
            .message
            .ends_with("held off by HPIRST"));
    }

    /// An absolute row behind the tick its previous row completed at cannot run on time.
    #[test]
    fn an_absolute_row_after_a_late_wait_is_a_missed_tick() {
        let mut fabric = Fabric::default();
        let mut replay = file("0,control,wait,0x8,0x8,100\n5,control,read,-\n");
        // HRDY reads back while the software reset is released (SPRU969B Table 8).
        run(&mut replay, &mut fabric, 0, 7);
        assert!(replay.failure().is_none());
        open_port(&mut fabric);
        replay.service(&mut fabric, 8);
        let failure = replay.failure().unwrap();
        assert_eq!((failure.line, failure.tick), (2, 8));
        assert!(failure
            .message
            .contains("missed tick: due at tick 5, but the previous row completed at tick 8"));
    }

    /// Strict physical-memory mode refusing a row's access stops the file at that row, so no
    /// later row reaches the target the diagnostic stopped the run on.
    #[test]
    fn a_strict_refusal_stops_the_rows_after_it() {
        let mut fabric = Fabric::default();
        open_port(&mut fabric);
        fabric.set_strict_physical_memory(true);
        // 0x0100_0000 is reserved on the C6455 (SPRS276M Table 2-2).
        let mut replay = file(
            "0,address,write,0x00400000\n\
             0,data,write,1\n\
             0,address,write,0x00200000\n",
        );
        replay.service(&mut fabric, 0);
        assert!(fabric.strict_stopped());
        let failure = replay.failure().unwrap();
        assert_eq!(failure.line, 2);
        assert!(failure
            .message
            .ends_with("strict physical memory refused the access"));
        assert_eq!(replay.json(0)["next_line"], Value::Null);
        assert_eq!(
            fabric.hpi_host_read(HostPort::Address),
            Some(0x0040_0000),
            "the row after it did not run"
        );
    }

    /// Tick arithmetic is checked, never saturated: a relative row or a wait window that runs
    /// past the end of the clock fails at the row rather than running at a tick it did not name.
    #[test]
    fn a_tick_past_the_end_of_the_clock_fails_the_row() {
        let mut fabric = Fabric::default();
        open_port(&mut fabric);
        let mut relative = file("5,control,read,-\n+18446744073709551615,control,read,-\n");
        relative.service(&mut fabric, 5);
        let failure = relative.failure().unwrap();
        assert_eq!((failure.line, failure.tick), (2, 5));
        assert!(failure
            .message
            .ends_with("the row's tick is past the end of the clock"));

        let mut wait = file("1,control,wait,0x4,0x4,18446744073709551615\n");
        wait.service(&mut fabric, 1);
        let failure = wait.failure().unwrap();
        assert_eq!((failure.line, failure.tick), (1, 1));
        assert!(
            failure
                .message
                .ends_with("the wait's window ends past the end of the clock (started at 1)"),
            "{}",
            failure.message
        );
        assert_eq!(wait.next_due(1), None);
    }

    #[test]
    fn rewinding_starts_the_file_over() {
        let mut fabric = Fabric::default();
        open_port(&mut fabric);
        let mut replay = file("0,control,read,-\n0,control,read,0xdead\n");
        replay.service(&mut fabric, 0);
        assert!(replay.failure().is_some());
        replay.rewind();
        assert!(replay.failure().is_none());
        assert!(replay.reads().is_empty());
        assert_eq!(replay.pending_line(), Some(1));
        assert_eq!(replay.next_due(0), Some(1));
    }
}
