//! Replay scheduled host register transactions through the device's HPI host port.

use crate::hpi::HostPort;
use crate::machine::Machine;
use std::fs;
use std::path::Path;

#[derive(Clone, Copy)]
enum Direction {
    Read,
    Write(u32),
}

#[derive(Clone, Copy)]
struct Transaction {
    tick: u64,
    port: HostPort,
    code: u8,
    direction: Direction,
    line: usize,
}

/// Parse and deliver a simple CSV schedule. A read reports the value seen at its event tick;
/// writes use the same Fabric HPI path as socket driven host accesses.
pub(crate) fn replay(machine: &mut Machine, path: &Path) -> Result<Vec<String>, String> {
    let input = fs::read_to_string(path).map_err(|error| format!("{}: {error}", path.display()))?;
    let transactions = parse(&input).map_err(|error| format!("{}: {error}", path.display()))?;
    let ticks = transactions
        .iter()
        .map(|transaction| transaction.tick)
        .collect::<Vec<_>>();
    let mut readbacks = Vec::new();
    machine.replay_release_ticks(&ticks, |machine, index, tick| {
        let transaction = transactions[index];
        match transaction.direction {
            Direction::Read => match machine.device.fabric().hpi_host_read(transaction.port) {
                Some(value) => {
                    readbacks.push(format!(
                        "tick={tick} port={} read=0x{value:08x}",
                        transaction.code
                    ));
                    Ok(())
                }
                None => Err(format!(
                    "HPI read on line {} was blocked by HPIRST",
                    transaction.line
                )),
            },
            Direction::Write(value) => {
                machine
                    .device
                    .fabric()
                    .hpi_host_write(transaction.port, value);
                Ok(())
            }
        }
    })?;
    Ok(readbacks)
}

fn parse(input: &str) -> Result<Vec<Transaction>, String> {
    let mut transactions = Vec::new();
    for (index, raw) in input.lines().enumerate() {
        let line = index + 1;
        let raw = raw.split('#').next().unwrap_or_default().trim();
        if raw.is_empty() {
            continue;
        }
        let fields = raw.split(',').map(str::trim).collect::<Vec<_>>();
        if fields.len() != 4 {
            return Err(format!("line {line}: expected tick,port,read|write,value"));
        }
        let tick = parse_u64(fields[0]).map_err(|error| format!("line {line}: {error}"))?;
        let (code, port) =
            parse_port(fields[1]).map_err(|error| format!("line {line}: {error}"))?;
        let direction = match fields[2] {
            "read" | "r" if fields[3] == "-" => Direction::Read,
            "read" | "r" => {
                return Err(format!(
                    "line {line}: read transactions require '-' as value"
                ));
            }
            "write" | "w" => Direction::Write(
                parse_u32(fields[3]).map_err(|error| format!("line {line}: {error}"))?,
            ),
            _ => return Err(format!("line {line}: direction must be read or write")),
        };
        transactions.push(Transaction {
            tick,
            port,
            code,
            direction,
            line,
        });
    }
    if transactions.is_empty() {
        return Err("schedule has no transactions".into());
    }
    if transactions
        .windows(2)
        .any(|pair| pair[0].tick > pair[1].tick)
    {
        return Err("schedule ticks must be sorted".into());
    }
    Ok(transactions)
}

fn parse_port(text: &str) -> Result<(u8, HostPort), String> {
    let code = match text {
        "control" => 0,
        "data-auto" | "data-auto-increment" => 1,
        "address" => 2,
        "data" => 3,
        _ => parse_u32(text)?
            .try_into()
            .map_err(|_| format!("invalid HPI port {text}"))?,
    };
    let port = match code {
        0 => HostPort::Control,
        1 => HostPort::DataAutoIncrement,
        2 => HostPort::Address,
        3 => HostPort::Data,
        _ => return Err(format!("invalid HPI port code {code}")),
    };
    Ok((code, port))
}

fn parse_u64(text: &str) -> Result<u64, String> {
    if let Some(hex) = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).map_err(|_| format!("invalid number {text}"))
    } else {
        text.parse().map_err(|_| format!("invalid number {text}"))
    }
}

fn parse_u32(text: &str) -> Result<u32, String> {
    parse_u64(text)?
        .try_into()
        .map_err(|_| format!("number out of range {text}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::Cpu;
    use crate::machine::Machine;

    #[test]
    fn parser_accepts_documented_names_codes_and_comments() {
        let transactions = parse(
            "# tick,port,direction,value\n0,control,write,0\n0,2,w,0x200400\n4,data-auto,r,-\n",
        )
        .unwrap();
        assert_eq!(transactions.len(), 3);
        assert_eq!(transactions[0].code, 0);
        assert_eq!(transactions[1].code, 2);
        assert_eq!(transactions[2].tick, 4);
    }

    #[test]
    fn scheduled_hpi_writes_are_fenced_and_read_back_through_host_port() {
        let directory =
            std::env::temp_dir().join(format!("c64xemu-hpi-replay-{}", std::process::id()));
        fs::create_dir_all(&directory).unwrap();
        let schedule = directory.join("schedule.csv");
        fs::write(
            &schedule,
            "0,address,write,0x200400\n0,data-auto,write,0x12345678\n0,data-auto,write,0x90abcdef\n0,data-auto,write,0x01020304\n0,data-auto,write,0xaabbccdd\n1,address,write,0x200400\n1,address,write,0x200400\n1,data-auto,read,-\n1,data-auto,read,-\n",
        )
        .unwrap();
        let mut machine = Machine::bare(Cpu::new(0));
        machine.device.cpu_mut().write_words(0, &[0; 32]);
        machine.device.cpu_mut().bus_mut().write32(0x0288_0030, 0);

        let readbacks = replay(&mut machine, &schedule).unwrap();
        assert_eq!(
            readbacks,
            [
                "tick=1 port=1 read=0x12345678",
                "tick=1 port=1 read=0x90abcdef",
            ]
        );
        assert_eq!(machine.event_time(), 1);
        fs::remove_dir_all(directory).unwrap();
    }
}
