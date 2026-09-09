//! Browser dashboard: one page that renders the whole machine - functional units, both
//! register files, control registers, the interrupt machinery, the SPLOOP buffer, the caches,
//! the MARs, memory and disassembly - and drives execution: step cycles or packets, run in
//! chunks, set breakpoints, reset.
//!
//! It is a single-threaded HTTP server over the same `Cpu` the console debugger drives, which
//! is what keeps the two views consistent: the dashboard has no model of its own.
//!
//!   GET  /                             the dashboard
//!   GET  /api/state?mem=HEX&len=N      full snapshot plus a memory window
//!   GET  /api/disasm?addr=&count=&mark= disassembly window
//!   GET  /api/cache-line?level=N&addr=HEX  resident cache-line bytes
//!   GET  /api/symbols                  name/address table for client annotation
//!   POST /api/step {"cycles":N}        advance N cycles, return a snapshot
//!   POST /api/run  {"cycles":N}        run until halt, breakpoint, fault or budget
//!   POST /api/breakpoints {"addresses":[...]}  replace the breakpoint set
//!   POST /api/reset                    reload the image, keep breakpoints

use crate::machine::{Machine, StopCondition};
use serde_json::{json, Value};
use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};

const INDEX_HTML: &str = include_str!("ui.html");

pub fn serve(mut machine: Machine, listen: &str) -> Result<u8, String> {
    let listener = TcpListener::bind(listen).map_err(|error| format!("{listen}: {error}"))?;
    let bound = listener.local_addr().map_err(|error| error.to_string())?;
    println!("c64xemu dashboard on http://{bound}");
    io::stdout().flush().map_err(|error| error.to_string())?;

    // The hotspot table is part of the page, so this session pays for the histogram whether
    // or not `--profile` was given.
    machine.set_profiling(true);
    machine.set_issue_view(true);
    machine.enable_ethernet_history();
    let breakpoints = machine.initial_breakpoints().to_vec();
    let mut session = Session {
        machine,
        breakpoints,
        previous: [[0; 32]; 2],
    };
    session.machine.arm_halt(&session.breakpoints);
    for stream in listener.incoming() {
        let mut stream = match stream {
            Ok(stream) => stream,
            Err(_) => continue,
        };
        if let Err(error) = session.serve_connection(&mut stream) {
            eprintln!("c64xemu: dashboard: {error}");
        }
    }
    session.machine.finish()?;
    Ok(0)
}

struct Session {
    machine: Machine,
    breakpoints: Vec<u32>,
    /// Register files as of the previous snapshot, so a refresh can flag what moved.
    previous: [[u32; 32]; 2],
}

impl Session {
    fn serve_connection(&mut self, stream: &mut TcpStream) -> Result<(), String> {
        let (method, target, body) = read_request(stream)?;
        let (path, query) = match target.split_once('?') {
            Some((path, query)) => (path.to_string(), query.to_string()),
            None => (target, String::new()),
        };
        let query = Query::new(&query);
        let response = match (method.as_str(), path.as_str()) {
            ("GET", "/") => Response::html(INDEX_HTML),
            ("GET", "/api/symbols") => Response::json(self.symbols_json()),
            ("GET", "/api/disasm") => {
                let mark = query.hex("mark").unwrap_or_else(|| self.machine.pc());
                let anchor = query.hex("addr").unwrap_or(mark);
                let count = query.number("count").unwrap_or(8).clamp(1, 64);
                // Centring puts the anchor packet in the middle of the window, which is what
                // "follow the PC" wants; otherwise the window starts there.
                let base = if query.number("center").unwrap_or(0) != 0 {
                    (anchor & !0x1f).wrapping_sub((count as u32 / 2) * 32)
                } else {
                    anchor & !0x1f
                };
                Response::json(json!({
                    "base": format!("0x{base:08x}"),
                    "rows": self.disassemble_packets(base, count, mark),
                }))
            }
            ("GET", "/api/cache-line") => {
                let level = query.number("level").unwrap_or(usize::MAX);
                let line = query.hex("addr").and_then(|address| {
                    self.machine.cache_line_data(level, address).map(|bytes| {
                        json!({
                            "found": true,
                            "address": format!("0x{address:08x}"),
                            "length": bytes.len(),
                            "hex": hex_string(&bytes),
                        })
                    })
                });
                Response::json(line.unwrap_or_else(|| json!({"found": false})))
            }
            ("GET", "/api/state") => {
                let window = self.window(&query);
                Response::json(self.snapshot(window))
            }
            // Which core the dashboard is looking at. Every other view - registers, memory,
            // caches, disassembly - is the focused core's, so this is the only multicore
            // control the page needs.
            ("POST", "/api/focus") => {
                let request: Value =
                    serde_json::from_str(&body).unwrap_or_else(|_| json!({"core": 0}));
                let core = request.get("core").and_then(Value::as_u64).unwrap_or(0) as usize;
                if core >= self.machine.cores() {
                    return Err(format!("no core {core}"));
                }
                self.machine.set_focus(core);
                self.previous = [[u32::MAX; 32]; 2];
                let window = self.window(&query);
                Response::json(self.snapshot(window))
            }
            ("POST", "/api/step") | ("POST", "/api/run") => {
                let request: Value =
                    serde_json::from_str(&body).unwrap_or_else(|_| json!({"cycles": 1}));
                let cycles = request
                    .get("cycles")
                    .and_then(Value::as_u64)
                    .unwrap_or(1)
                    .min(2_000_000);
                self.advance(cycles)?;
                let window = self.window(&query);
                Response::json(self.snapshot(window))
            }
            ("POST", "/api/breakpoints") => {
                let request: Value = serde_json::from_str(&body)
                    .map_err(|error| format!("bad breakpoints body: {error}"))?;
                self.breakpoints = request
                    .get("addresses")
                    .and_then(Value::as_array)
                    .map(|list| {
                        list.iter()
                            .filter_map(Value::as_u64)
                            .map(|address| address as u32)
                            .collect()
                    })
                    .unwrap_or_default();
                self.machine.arm_halt(&self.breakpoints);
                let window = self.window(&query);
                Response::json(self.snapshot(window))
            }
            ("POST", "/api/reset") => {
                self.machine.reload()?;
                self.machine.set_profiling(true);
                self.previous = [[0; 32]; 2];
                let window = self.window(&query);
                Response::json(self.snapshot(window))
            }
            _ => Response::not_found(),
        };
        response.write(stream)
    }

    fn window(&self, query: &Query) -> (u32, usize) {
        (
            query.hex("mem").unwrap_or_else(|| self.machine.pc() & !0xf),
            query.number("len").unwrap_or(128).min(4096),
        )
    }

    /// Runs a bounded chunk of cycles, stopping early on a halt, a breakpoint or a fault. The
    /// browser calls this repeatedly, which is what keeps a long run answerable.
    fn advance(&mut self, cycles: u64) -> Result<(), String> {
        // A program that has reached its exit point is over: running on would execute
        // whatever follows it. `reset` is how the page starts again.
        if self.machine.at_stop() {
            return Ok(());
        }
        let snapshot = self.machine.snapshot();
        self.previous = [snapshot.core.a, snapshot.core.b];
        let mut conditions = [StopCondition::Budget(cycles)];
        self.machine.resume(&mut conditions)?;
        Ok(())
    }

    fn symbols_json(&self) -> Value {
        json!(self
            .machine
            .symbols_by_address()
            .iter()
            .map(|(address, name)| json!({"address": format!("0x{address:08x}"), "name": name}))
            .collect::<Vec<_>>())
    }

    /// Full machine snapshot. Change flags are relative to the last `advance`, so stepping
    /// highlights exactly what that step wrote.
    fn snapshot(&mut self, window: (u32, usize)) -> Value {
        let (mem_address, mem_len) = window;
        let ethernet = self.machine.ethernet_history_json();
        let cpu = self.machine.snapshot();
        let cores: Vec<Value> = cpu
            .cores
            .iter()
            .map(|core| {
                json!({
                    "core": core.core,
                    "pc": format!("0x{:08x}", core.pc),
                    "cycles": core.cycles,
                    "halted": core.halted,
                    "held": core.held_in_reset,
                    "focused": core.focused,
                })
            })
            .collect();
        let bank = |file: &[u32; 32], previous: &[u32; 32]| -> Vec<Value> {
            (0..32)
                .map(|index| {
                    let value = file[index];
                    json!({"v": format!("0x{value:08x}"), "c": previous[index] != value})
                })
                .collect()
        };
        let reg_a = bank(&cpu.core.a, &self.previous[0]);
        let reg_b = bank(&cpu.core.b, &self.previous[1]);
        let control: Vec<Value> = (0..=0x1fu8)
            .filter_map(|register| {
                let name = crate::control::register_name(register)?;
                Some(json!({
                    "name": name.to_ascii_uppercase(),
                    "value": format!("0x{:08x}", cpu.core.control[register as usize]),
                }))
            })
            .collect();
        let hotspots: Vec<Value> = self
            .machine
            .hotspots(12)
            .into_iter()
            .map(|(pc, count)| {
                json!({
                    "pc": format!("0x{pc:08x}"),
                    "packets": count,
                    "where": self.machine.symbol_at(pc),
                })
            })
            .collect();
        let stats = json!({
            "packets": cpu.stats.packets,
            "retired": cpu.stats.retired,
            "replay_packets": cpu.stats.replay_packets,
            "branches_taken": cpu.stats.branches_taken,
            "interrupts": cpu.stats.interrupts,
            "stores": cpu.stats.stores,
            "faults": cpu.stats.faults,
        });
        let pc = cpu.core.pc;
        let halted = cpu.halted;
        let mem_hex = hex_string(&self.machine.read_memory(mem_address, mem_len));
        let units = json!(cpu.last_issue);
        let sploop = json!(cpu.sploop);
        let fault = cpu.fault.clone();
        let cycle = cpu.cycles;
        let ifr = cpu.core.control[0x02];
        let caches = json!(self.machine.cache_views());
        let intmux = self.machine.interrupt_selectors();
        let evtflags = self.machine.event_flag_registers();
        let mar = self.machine.mar_registers().to_vec();
        let cio_output_bytes = self.machine.cio_output().len();
        let cio_output = String::from_utf8_lossy(self.machine.cio_output()).into_owned();
        let breakpoint = cpu.breakpoint.map(|hit| {
            json!({
                "core": hit.core,
                "address": format!("0x{:08x}", hit.address),
            })
        });
        json!({
            "pc": format!("0x{pc:08x}"),
            "pc_where": self.machine.symbol_at(pc),
            "cycle": cycle,
            "fault": fault,
            "cio_output": cio_output,
            "cio_output_bytes": cio_output_bytes,
            "halted": halted,
            "at_breakpoint": breakpoint.is_some(),
            "breakpoint": breakpoint,
            "reg_a": reg_a,
            "reg_b": reg_b,
            "control": control,
            "intmux": intmux,
            "evtflags": evtflags,
            "ifr_pending": format!("0x{ifr:08x}"),
            "units": units,
            "sploop": sploop,
            "caches": caches,
            "mar": mar,
            "ethernet": ethernet,
            "cores": cores,
            "stats": stats,
            "hotspots": hotspots,
            "breakpoints": self
                .breakpoints
                .iter()
                .map(|address| format!("0x{address:08x}"))
                .collect::<Vec<_>>(),
            "mem_addr": format!("0x{mem_address:08x}"),
            "mem_hex": mem_hex,
        })
    }

    /// Disassembles `count` 32-byte fetch packets from `base`, flagging the instruction at
    /// `mark` - normally the PC.
    fn disassemble_packets(&self, base: u32, count: usize, mark: u32) -> Vec<Value> {
        let mut rows = Vec::with_capacity(count);
        let mut packet = base;
        for _ in 0..count.max(1) {
            let mut instructions = Vec::new();
            let mut address = packet;
            loop {
                let (text, next) = self.machine.disassemble(address);
                instructions.push(json!({
                    "addr": format!("0x{address:08x}"),
                    "text": text,
                    "current": address == mark,
                }));
                if next <= address || next >= packet.wrapping_add(32) || instructions.len() == 16 {
                    break;
                }
                address = next;
            }
            rows.push(json!({
                "base": format!("0x{packet:08x}"),
                "where": self.machine.symbol_at(packet),
                "instructions": instructions,
                "current": packet <= mark && mark < packet.wrapping_add(32),
            }));
            packet = packet.wrapping_add(32);
        }
        rows
    }
}

struct Response {
    status: &'static str,
    content_type: &'static str,
    body: String,
}

impl Response {
    fn html(body: &str) -> Self {
        Self {
            status: "200 OK",
            content_type: "text/html; charset=utf-8",
            body: body.to_string(),
        }
    }

    fn json(body: Value) -> Self {
        Self {
            status: "200 OK",
            content_type: "application/json",
            body: body.to_string(),
        }
    }

    fn not_found() -> Self {
        Self {
            status: "404 Not Found",
            content_type: "text/plain",
            body: "not found".to_string(),
        }
    }

    fn write(&self, stream: &mut TcpStream) -> Result<(), String> {
        let head = format!(
            "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
            self.status,
            self.content_type,
            self.body.len()
        );
        stream
            .write_all(head.as_bytes())
            .and_then(|()| stream.write_all(self.body.as_bytes()))
            .and_then(|()| stream.flush())
            .map_err(|error| error.to_string())
    }
}

struct Query<'a> {
    pairs: Vec<(&'a str, &'a str)>,
}

impl<'a> Query<'a> {
    fn new(query: &'a str) -> Self {
        Self {
            pairs: query
                .split('&')
                .filter_map(|pair| pair.split_once('='))
                .collect(),
        }
    }

    fn value(&self, key: &str) -> Option<&'a str> {
        self.pairs
            .iter()
            .find(|(name, _)| *name == key)
            .map(|(_, value)| *value)
    }

    fn hex(&self, key: &str) -> Option<u32> {
        self.value(key)
            .and_then(|value| u32::from_str_radix(value.trim_start_matches("0x"), 16).ok())
    }

    fn number(&self, key: &str) -> Option<usize> {
        self.value(key).and_then(|value| value.parse().ok())
    }
}

fn read_request(stream: &mut TcpStream) -> Result<(String, String, String), String> {
    let mut buffer = Vec::new();
    let mut byte = [0u8; 1];
    while !buffer.ends_with(b"\r\n\r\n") {
        match stream.read(&mut byte) {
            Ok(0) => return Err("client closed".into()),
            Ok(_) => buffer.push(byte[0]),
            // Background jobs get signals constantly; EINTR just means retry.
            Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error.to_string()),
        }
        if buffer.len() > 64 * 1024 {
            return Err("request head too large".into());
        }
    }
    let head = String::from_utf8_lossy(&buffer).into_owned();
    let mut fields = head.lines().next().unwrap_or_default().split_whitespace();
    let method = fields.next().unwrap_or_default().to_string();
    let target = fields.next().unwrap_or_default().to_string();
    let content_length = head
        .lines()
        .find(|line| line.to_ascii_lowercase().starts_with("content-length:"))
        .and_then(|line| line.split(':').nth(1))
        .and_then(|value| value.trim().parse::<usize>().ok())
        .unwrap_or(0);
    let mut body = vec![0u8; content_length];
    if content_length > 0 {
        stream
            .read_exact(&mut body)
            .map_err(|error| error.to_string())?;
    }
    Ok((method, target, String::from_utf8_lossy(&body).into_owned()))
}

fn hex_string(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}
