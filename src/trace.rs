//! Execution tracing: one event stream, two renderings.
//!
//! JSON Lines for machines (stable field names, hex-encoded words), aligned text
//! for humans. A ring-buffer mode keeps only the last N events in memory and
//! flushes them at exit, which answers "what happened right before the fault or
//! hang" without paying to record a ten-million-cycle run.
//!
//! Tracing is observationally passive: every event is rendered from state the
//! emulator already committed, and every emit site is guarded by `enabled()` so
//! a run without `--trace` allocates nothing.

use crate::view::UnitIssue;
use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufWriter, Write};

/// One thing the machine did during a cycle.
pub enum Event {
    /// An execute packet issued from the fetch path.
    Packet {
        pc: u32,
        text: Vec<String>,
        issues: Vec<UnitIssue>,
    },
    /// A SPLOOP replay cycle re-executing buffered packets.
    Replay {
        offset: u32,
        packets: usize,
        issues: Vec<UnitIssue>,
    },
    /// The whole cycle was consumed by a stall.
    Stall { reason: &'static str },
    /// Architectural register write-back.
    Write { name: String, old: u32, new: u32 },
    /// Memory store committed.
    Store {
        address: u32,
        size: u8,
        old: u32,
        new: u32,
    },
    /// Control register write committed.
    Control {
        name: &'static str,
        old: u32,
        new: u32,
    },
    /// A scheduled branch resolved.
    Branch { to: u32 },
    /// CPU took an interrupt.
    Interrupt { interrupt: u8, from: u32, to: u32 },
    /// SPLOOP buffer started filling.
    SploopStart { ii: u32 },
    /// SPLOOP finished and drained.
    SploopEnd,
    /// Fault raised; IERR carries the raised bits.
    Fault { message: String, ierr: u32 },
}

enum Sink {
    Off,
    Text(BufWriter<File>),
    Json(BufWriter<File>),
}

pub struct Trace {
    sink: Sink,
    ring: Option<VecDeque<String>>,
    ring_limit: usize,
    start_cycle: u64,
}

impl Trace {
    pub fn off() -> Self {
        Self {
            sink: Sink::Off,
            ring: None,
            ring_limit: 0,
            start_cycle: 0,
        }
    }

    pub fn open(
        writer: BufWriter<File>,
        json: bool,
        ring: Option<usize>,
        start_cycle: u64,
    ) -> Self {
        Self {
            sink: if json {
                Sink::Json(writer)
            } else {
                Sink::Text(writer)
            },
            ring: ring.map(VecDeque::with_capacity),
            ring_limit: ring.unwrap_or(0),
            start_cycle,
        }
    }

    /// False when nothing is being recorded. Emit sites test this before building
    /// an event so tracing costs nothing when it is off.
    pub fn enabled(&self) -> bool {
        !matches!(self.sink, Sink::Off)
    }

    fn render(event: &Event) -> (String, String) {
        let text = match event {
            Event::Packet { pc, text, issues } => {
                format!("packet 0x{pc:08x}: {}", tagged(text, issues))
            }
            Event::Replay {
                offset,
                packets,
                issues,
            } => format!(
                "replay offset={offset} packets={packets}: {}",
                tagged(
                    &issues
                        .iter()
                        .map(|issue| issue.text.clone())
                        .collect::<Vec<_>>(),
                    issues
                )
            ),
            Event::Stall { reason } => format!("stall {reason}"),
            Event::Write { name, old, new } => {
                format!("{name} = 0x{new:08x} (was 0x{old:08x})")
            }
            Event::Store {
                address,
                size,
                old,
                new,
            } => {
                let width = *size as usize * 2;
                format!("store [0x{address:08x}] = 0x{new:0width$x} (was 0x{old:0width$x})")
            }
            Event::Control { name, old, new } => {
                format!("{name} = 0x{new:08x} (was 0x{old:08x})")
            }
            Event::Branch { to } => format!("branch -> 0x{to:08x}"),
            Event::Interrupt {
                interrupt,
                from,
                to,
            } => format!("interrupt {interrupt}: 0x{from:08x} -> 0x{to:08x}"),
            Event::SploopStart { ii } => format!("sploop start ii={ii}"),
            Event::SploopEnd => "sploop end".into(),
            Event::Fault { message, ierr } => {
                format!("FAULT {message} (ierr=0x{ierr:04x})")
            }
        };
        let json = match event {
            Event::Packet { pc, issues, .. } => {
                let instructions = issues
                    .iter()
                    .map(|issue| json_string(&issue.text))
                    .collect::<Vec<_>>()
                    .join(",");
                format!(
                    "{{\"type\":\"packet\",\"pc\":\"0x{pc:08x}\",\"insn\":[{instructions}],\"units\":{}}}",
                    units_json(issues)
                )
            }
            Event::Replay {
                offset,
                packets,
                issues,
            } => format!(
                "{{\"type\":\"replay\",\"offset\":{offset},\"packets\":{packets},\"units\":{}}}",
                units_json(issues)
            ),
            Event::Stall { reason } => {
                format!("{{\"type\":\"stall\",\"reason\":\"{reason}\"}}")
            }
            Event::Write { name, old, new } => format!(
                "{{\"type\":\"write\",\"reg\":\"{name}\",\"old\":\"0x{old:08x}\",\"new\":\"0x{new:08x}\"}}"
            ),
            Event::Store {
                address,
                size,
                old,
                new,
            } => format!(
                "{{\"type\":\"store\",\"addr\":\"0x{address:08x}\",\"size\":{size},\"old\":\"0x{old:08x}\",\"new\":\"0x{new:08x}\"}}"
            ),
            Event::Control { name, old, new } => format!(
                "{{\"type\":\"control\",\"reg\":\"{name}\",\"old\":\"0x{old:08x}\",\"new\":\"0x{new:08x}\"}}"
            ),
            Event::Branch { to } => {
                format!("{{\"type\":\"branch\",\"to\":\"0x{to:08x}\"}}")
            }
            Event::Interrupt {
                interrupt,
                from,
                to,
            } => format!(
                "{{\"type\":\"interrupt\",\"int\":{interrupt},\"from\":\"0x{from:08x}\",\"to\":\"0x{to:08x}\"}}"
            ),
            Event::SploopStart { ii } => {
                format!("{{\"type\":\"sploop_start\",\"ii\":{ii}}}")
            }
            Event::SploopEnd => "{\"type\":\"sploop_end\"}".into(),
            Event::Fault { message, ierr } => format!(
                "{{\"type\":\"fault\",\"message\":{},\"ierr\":\"0x{ierr:04x}\"}}",
                json_string(message)
            ),
        };
        (text, json)
    }

    pub fn emit(&mut self, cycle: u64, event: Event) {
        if matches!(self.sink, Sink::Off) || cycle < self.start_cycle {
            return;
        }
        let (text, json) = Self::render(&event);
        let line = match &self.sink {
            Sink::Off => unreachable!(),
            Sink::Text(_) => format!("[{cycle:8}] {text}\n"),
            Sink::Json(_) => {
                let fields = json
                    .strip_prefix('{')
                    .and_then(|rest| rest.strip_suffix('}'))
                    .unwrap_or(&json);
                format!("{{\"cycle\":{cycle},{fields}}}\n")
            }
        };
        match (&mut self.ring, &mut self.sink) {
            (Some(ring), _) => {
                if ring.len() >= self.ring_limit {
                    ring.pop_front();
                }
                ring.push_back(line);
            }
            (None, Sink::Text(writer) | Sink::Json(writer)) => {
                let _ = writer.write_all(line.as_bytes());
            }
            (None, Sink::Off) => {}
        }
    }

    /// Writes any retained ring events and flushes the underlying stream.
    pub fn flush(&mut self) {
        match &mut self.sink {
            Sink::Off => {}
            Sink::Text(writer) | Sink::Json(writer) => {
                if let Some(ring) = &mut self.ring {
                    for line in ring.drain(..) {
                        let _ = writer.write_all(line.as_bytes());
                    }
                }
                let _ = writer.flush();
            }
        }
    }
}

/// Renders one instruction prefixed with the datapath resources it used:
/// `<D2X>` = .D2 with cross path, `<M1T2>` = .M1 writing data path T2.
fn unit_tag(issue: &UnitIssue) -> String {
    if issue.unit == '?' {
        return String::new();
    }
    let mut tag = format!(
        "<{}{}{}",
        issue.unit,
        if issue.side_a { 1 } else { 2 },
        if issue.cross { "X" } else { "" }
    );
    if let Some(first) = issue.mem_side_a {
        tag.push('T');
        tag.push(if first { '1' } else { '2' });
    }
    if issue.replayed {
        tag.push('R');
    }
    tag.push('>');
    tag
}

/// Joins instruction texts with their unit tags; untagged entries pass through.
fn tagged(text: &[String], issues: &[UnitIssue]) -> String {
    text.iter()
        .enumerate()
        .map(|(index, line)| match issues.get(index) {
            Some(issue) => {
                let tag = unit_tag(issue);
                if tag.is_empty() {
                    line.clone()
                } else {
                    format!("{tag} {line}")
                }
            }
            None => line.clone(),
        })
        .collect::<Vec<_>>()
        .join(" || ")
}

fn units_json(issues: &[UnitIssue]) -> String {
    let entries = issues
        .iter()
        .map(|issue| {
            format!(
                "{{\"u\":\"{}\",\"s\":{},\"x\":{},\"t\":{},\"r\":{},\"i\":{}}}",
                issue.unit,
                if issue.side_a { 1 } else { 2 },
                issue.cross,
                match issue.mem_side_a {
                    Some(true) => "1",
                    Some(false) => "2",
                    None => "null",
                },
                issue.replayed,
                json_string(&issue.text)
            )
        })
        .collect::<Vec<_>>()
        .join(",");
    format!("[{entries}]")
}

fn json_string(value: &str) -> String {
    let mut out = String::with_capacity(value.len() + 2);
    out.push('"');
    for character in value.chars() {
        match character {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            _ => out.push(character),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renders_text_and_json_consistently() {
        let (text, json) = Trace::render(&Event::Write {
            name: "b0".into(),
            old: 1,
            new: 0xdead_beef,
        });
        assert_eq!(text, "b0 = 0xdeadbeef (was 0x00000001)");
        assert_eq!(
            json,
            "{\"type\":\"write\",\"reg\":\"b0\",\"old\":\"0x00000001\",\"new\":\"0xdeadbeef\"}"
        );
    }

    #[test]
    fn stores_render_at_their_access_width() {
        let (text, _) = Trace::render(&Event::Store {
            address: 0x8000_0000,
            size: 2,
            old: 0,
            new: 0x1234,
        });
        assert_eq!(text, "store [0x80000000] = 0x1234 (was 0x0000)");
    }

    #[test]
    fn escapes_quotes_in_fault_messages() {
        let (_, json) = Trace::render(&Event::Fault {
            message: "bad \"opcode\" here".into(),
            ierr: 8,
        });
        assert!(json.contains("\\\"opcode\\\""));
    }

    #[test]
    fn a_ring_keeps_only_the_last_events() {
        let file = std::env::temp_dir().join("c64xemu-trace-ring-test.txt");
        let mut trace = Trace::open(
            BufWriter::new(File::create(&file).unwrap()),
            false,
            Some(2),
            0,
        );
        for value in 0..5u32 {
            trace.emit(value as u64, Event::Branch { to: value });
        }
        trace.flush();
        let written = std::fs::read_to_string(&file).unwrap();
        std::fs::remove_file(&file).ok();
        assert_eq!(
            written,
            "[       3] branch -> 0x00000003\n[       4] branch -> 0x00000004\n"
        );
    }
}
