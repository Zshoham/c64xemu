//! Low-level conformance cases run in-process; full target programs use the public batch runner.

use crate::cpu::Cpu;
use crate::gdb::Session as GdbSession;
use crate::system::{Endian, Fabric};
use serde_json::{json, Value};
fn run_decode(request: &Value) -> Value {
    let word = request
        .get("word")
        .and_then(Value::as_u64)
        .and_then(|word| u32::try_from(word).ok())
        .unwrap_or(0);
    let header = request
        .get("header")
        .and_then(Value::as_u64)
        .and_then(|header| u32::try_from(header).ok());
    serde_json::to_value(crate::view::Instruction::new(0, word, header))
        .expect("serializing an instruction cannot fail")
}
fn run_gdb(request: &Value) -> Result<Value, String> {
    let pc = request
        .get("pc")
        .and_then(Value::as_u64)
        .and_then(|value| u32::try_from(value).ok())
        .ok_or_else(|| "gdb request missing pc".to_string())?;
    let mut cpu = Cpu::new_with_endian(pc, request_endian(request)?);
    if let Some(registers) = request.get("registers").and_then(Value::as_object) {
        for (name, value) in registers {
            let value = value
                .as_u64()
                .and_then(|value| u32::try_from(value).ok())
                .ok_or_else(|| format!("invalid value for register {name}"))?;
            cpu.set_register(name, value)?;
        }
    }
    let words = request
        .get("words")
        .and_then(Value::as_array)
        .ok_or_else(|| "gdb request missing words".to_string())?
        .iter()
        .map(|word| {
            word.as_u64()
                .and_then(|value| u32::try_from(value).ok())
                .ok_or_else(|| "invalid gdb program word".to_string())
        })
        .collect::<Result<Vec<_>, _>>()?;
    cpu.write_words(pc, &words);
    // The words were written straight into memory: this session has no image behind it, so
    // there is no CIO handshake, no exit point and no host backend to service.
    let mut session = GdbSession::new(crate::machine::Machine::bare(cpu));
    let packets = request
        .get("packets")
        .and_then(Value::as_array)
        .ok_or_else(|| "gdb request missing packets".to_string())?;
    let mut replies = Vec::with_capacity(packets.len());
    for packet in packets {
        let packet = packet
            .as_str()
            .ok_or_else(|| "invalid gdb packet".to_string())?;
        replies.push(session.handle(packet)?);
    }
    Ok(json!({"replies": replies}))
}

fn run_execute(request: &Value) -> Result<Value, String> {
    let pc = request
        .get("pc")
        .and_then(Value::as_u64)
        .and_then(|value| u32::try_from(value).ok())
        .ok_or_else(|| "execute request missing pc".to_string())?;
    let cycles = request
        .get("cycles")
        .and_then(Value::as_u64)
        .ok_or_else(|| "execute request missing cycles".to_string())?;
    let mut cpu = Cpu::new_with_endian(pc, request_endian(request)?);
    if let Some(registers) = request.get("registers").and_then(Value::as_object) {
        for (name, value) in registers {
            let value = value
                .as_u64()
                .and_then(|value| u32::try_from(value).ok())
                .ok_or_else(|| format!("invalid value for register {name}"))?;
            cpu.set_register(name, value)?;
        }
    }
    if let Some(memory) = request.get("memory").and_then(Value::as_array) {
        for region in memory {
            let address = region
                .get("address")
                .and_then(Value::as_u64)
                .and_then(|value| u32::try_from(value).ok())
                .ok_or_else(|| "memory region missing address".to_string())?;
            let bytes = region
                .get("hex")
                .and_then(Value::as_str)
                .ok_or_else(|| "memory region missing hex".to_string())
                .and_then(decode_hex)?;
            cpu.write_bytes(address, &bytes);
        }
    }
    if let Some(words) = request.get("words").and_then(Value::as_array) {
        let words = words
            .iter()
            .map(|word| {
                word.as_u64()
                    .and_then(|value| u32::try_from(value).ok())
                    .ok_or_else(|| "invalid program word".to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
        cpu.write_words(pc, &words);
    } else if let Some(program) = request.get("program_hex").and_then(Value::as_str) {
        cpu.write_bytes(pc, &decode_hex(program)?);
    } else {
        return Err("execute request missing program".into());
    }
    if let Some(controls) = request.get("controls").and_then(Value::as_object) {
        // Interrupt timing cases need the control registers that only MVC can reach: ISTP,
        // IER, CSR and the interrupt set register.
        for (number, value) in controls {
            let number = number
                .parse::<u8>()
                .map_err(|_| format!("invalid control register {number}"))?;
            let value = value
                .as_u64()
                .and_then(|value| u32::try_from(value).ok())
                .ok_or_else(|| format!("invalid value for control register {number}"))?;
            cpu.write_control(number, value);
        }
    }
    if let Some(setup) = request.get("setup").and_then(Value::as_array) {
        // Timing cases configure the caches through the memory-mapped registers before the
        // measured sequence starts.
        for operation in setup {
            cpu.bus_mut().apply(operation)?;
        }
    }
    if request.get("nmi").and_then(Value::as_bool) == Some(true) {
        cpu.set_nmi_line(true);
    }
    let mut control_events = Vec::new();
    if let Some(events) = request.get("control_events").and_then(Value::as_array) {
        for event in events {
            let cycle = event
                .get("cycle")
                .and_then(Value::as_u64)
                .ok_or_else(|| "control event missing cycle".to_string())?;
            let register = event
                .get("register")
                .and_then(Value::as_u64)
                .and_then(|value| u8::try_from(value).ok())
                .ok_or_else(|| "control event missing register".to_string())?;
            let value = event
                .get("value")
                .and_then(Value::as_u64)
                .and_then(|value| u32::try_from(value).ok())
                .ok_or_else(|| "control event missing value".to_string())?;
            if control_events
                .last()
                .is_some_and(|(previous, _, _)| *previous > cycle)
            {
                return Err("control events are not ordered by cycle".into());
            }
            control_events.push((cycle, register, value));
        }
    }
    let mut system_events = Vec::new();
    if let Some(events) = request.get("system_events").and_then(Value::as_array) {
        for event in events {
            let cycle = event
                .get("cycle")
                .and_then(Value::as_u64)
                .ok_or_else(|| "system event missing cycle".to_string())?;
            if system_events
                .last()
                .is_some_and(|(previous, _)| *previous > cycle)
            {
                return Err("system events are not ordered by cycle".into());
            }
            system_events.push((cycle, event.clone()));
        }
    }
    let stop_pc = request
        .get("stop_pc")
        .and_then(Value::as_u64)
        .and_then(|value| u32::try_from(value).ok());
    let mut next_event = 0;
    let mut next_system_event = 0;
    while cpu.cycles() < cycles
        && cpu.fault().is_none()
        && !stop_pc.is_some_and(|stop| cpu.ready_to_fetch_at(stop))
    {
        while next_event < control_events.len() && control_events[next_event].0 <= cpu.cycles() {
            let (_, register, value) = control_events[next_event];
            cpu.write_control(register, value);
            next_event += 1;
        }
        while next_system_event < system_events.len()
            && system_events[next_system_event].0 <= cpu.cycles()
        {
            cpu.bus_mut().apply(&system_events[next_system_event].1)?;
            next_system_event += 1;
        }
        cpu.step_cycle();
    }
    let mut controls = serde_json::Map::new();
    if let Some(numbers) = request.get("observe_controls").and_then(Value::as_array) {
        for number in numbers {
            let number = number
                .as_u64()
                .and_then(|value| u8::try_from(value).ok())
                .ok_or_else(|| "invalid observed control register".to_string())?;
            controls.insert(number.to_string(), json!(cpu.read_control(number)));
        }
    }
    let mut system = serde_json::Map::new();
    if let Some(addresses) = request.get("observe_system").and_then(Value::as_array) {
        for address in addresses {
            let address = address
                .as_u64()
                .and_then(|value| u32::try_from(value).ok())
                .ok_or_else(|| "invalid observed system register".to_string())?;
            system.insert(address.to_string(), json!(cpu.bus_mut().read32(address)));
        }
    }
    let mut registers = serde_json::Map::with_capacity(64);
    for bank in ['A', 'B'] {
        for index in 0..32 {
            registers.insert(format!("{bank}{index}"), json!(cpu.register(bank, index)));
        }
    }
    Ok(json!({
        "registers": registers,
        "cycles": cpu.cycles(),
        "reached_stop": stop_pc.is_some_and(|stop| cpu.ready_to_fetch_at(stop)),
        "fault": cpu.fault(),
        "controls": controls,
        "system": system,
        "pc": cpu.pc(),
    }))
}
fn decode_hex(text: &str) -> Result<Vec<u8>, String> {
    if text.len() % 2 != 0 {
        return Err("hex string has odd length".into());
    }
    text.as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let text = std::str::from_utf8(pair).map_err(|_| "invalid hex")?;
            u8::from_str_radix(text, 16).map_err(|_| "invalid hex".into())
        })
        .collect()
}

fn request_endian(request: &Value) -> Result<Endian, String> {
    match request
        .get("endian")
        .and_then(Value::as_str)
        .unwrap_or("little")
    {
        "little" => Ok(Endian::Little),
        "big" => Ok(Endian::Big),
        other => Err(format!("unsupported byte order {other}")),
    }
}

fn run_system(request: &Value) -> Result<Value, String> {
    let operations = request
        .get("operations")
        .and_then(Value::as_array)
        .ok_or_else(|| "system request missing operations".to_string())?;
    let mut system = Fabric::default();
    let mut observations = Vec::new();
    for operation in operations {
        if let Some(value) = system.apply(operation)? {
            observations.push(value);
        }
    }
    Ok(json!({"observations": observations}))
}

fn matches(expected: &Value, actual: &Value) -> bool {
    if let Some(values) = expected
        .as_object()
        .filter(|object| object.len() == 1)
        .and_then(|object| object.get("$one_of"))
        .and_then(Value::as_array)
    {
        return values.iter().any(|value| value == actual);
    }
    if let Some(bits) = expected
        .as_object()
        .filter(|object| object.len() == 1)
        .and_then(|object| object.get("$bits"))
        .and_then(Value::as_object)
    {
        let Some(actual) = actual.as_u64() else {
            return false;
        };
        let set = bits.get("set").and_then(Value::as_u64).unwrap_or(0);
        let clear = bits.get("clear").and_then(Value::as_u64).unwrap_or(0);
        return actual & set == set && actual & clear == 0;
    }
    match (expected, actual) {
        (Value::Object(expected), Value::Object(actual)) => expected
            .iter()
            .all(|(key, value)| actual.get(key).is_some_and(|got| matches(value, got))),
        (Value::Array(expected), Value::Array(actual)) => {
            expected.len() == actual.len()
                && expected
                    .iter()
                    .zip(actual)
                    .all(|(want, got)| matches(want, got))
        }
        _ => expected == actual,
    }
}

fn vector_root() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("vectors")
        .join("component")
}

fn load_case(root: &std::path::Path, path: &std::path::Path) -> Value {
    let definition = std::fs::read_to_string(path)
        .unwrap_or_else(|error| panic!("could not read {}: {error}", path.display()));
    let case: Value = serde_json::from_str(&definition)
        .unwrap_or_else(|error| panic!("{} is not valid JSON: {error}", path.display()));
    let mut parts = path
        .strip_prefix(root)
        .expect("component vector must be below its root")
        .iter()
        .map(|part| part.to_string_lossy().into_owned())
        .collect::<Vec<_>>();
    let filename = parts.last_mut().expect("component vector has no filename");
    *filename = filename
        .strip_suffix(".json")
        .expect("component vector filename does not end in .json")
        .to_owned();
    assert_eq!(
        case_id(&case),
        parts.join(":"),
        "{} does not match its case id",
        path.display()
    );
    case
}

fn case_id(case: &Value) -> &str {
    case.get("request")
        .and_then(|request| request.get("id"))
        .and_then(Value::as_str)
        .unwrap_or("<unnamed>")
}

fn check_case(case: Value) {
    let identifier = case_id(&case);
    let request = case
        .get("request")
        .unwrap_or_else(|| panic!("{identifier} has no request"));
    let action = request
        .get("action")
        .and_then(Value::as_str)
        .unwrap_or_else(|| panic!("{identifier} has no action"));
    let actual = match action {
        "decode" => Ok(run_decode(request)),
        "execute" => run_execute(request),
        "gdb" => run_gdb(request),
        "system" => run_system(request),
        other => Err(format!("unsupported component action {other}")),
    }
    .unwrap_or_else(|error| panic!("{identifier} failed: {error}"));
    let expected = case
        .get("expected")
        .unwrap_or_else(|| panic!("{identifier} has no expectation"));
    assert!(
        matches(expected, &actual),
        "{identifier} did not match\nexpected: {expected:#}\nactual:   {actual:#}"
    );
}

pub(crate) fn run_vector(relative: &str) {
    let root = vector_root();
    check_case(load_case(&root, &root.join(relative)));
}
