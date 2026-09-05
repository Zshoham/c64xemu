//! Command-line access to the instruction decoder.

use clap::Args;

#[derive(Clone)]
struct Request {
    word: u32,
    header: Option<u32>,
}

#[derive(Args)]
pub struct DecodeArgs {
    /// Instruction word, optionally followed by its compact fetch-packet header
    #[arg(required = true, value_name = "WORD[/HEADER]", value_parser = parse_instruction)]
    instructions: Vec<Request>,
}

fn parse_word(text: &str) -> Result<u32, String> {
    let digits = text
        .strip_prefix("0x")
        .or_else(|| text.strip_prefix("0X"))
        .unwrap_or(text);
    u32::from_str_radix(digits, 16).map_err(|_| format!("invalid instruction word {text}"))
}

fn parse_instruction(text: &str) -> Result<Request, String> {
    let (word, header) = match text.split_once('/') {
        Some((word, header)) => (parse_word(word)?, Some(parse_word(header)?)),
        None => (parse_word(text)?, None),
    };
    if header.is_some() && word > u16::MAX as u32 {
        return Err(format!(
            "compact instruction {word:#x} is wider than 16 bits"
        ));
    }
    Ok(Request { word, header })
}

pub fn run(args: DecodeArgs) -> Result<u8, String> {
    let stdout = std::io::stdout();
    let mut output = stdout.lock();
    for request in args.instructions {
        serde_json::to_writer(
            &mut output,
            &crate::view::Instruction::new(0, request.word, request.header),
        )
        .map_err(|error| error.to_string())?;
        std::io::Write::write_all(&mut output, b"\n").map_err(|error| error.to_string())?;
    }
    Ok(0)
}
