//! The instruction view: what an instruction is, as resources, as text and as JSON.
//!
//! One interface for every consumer that reads an instruction without executing it - the
//! differential in `tools/dis6x_diff.py`, the decode CLI, the trace, the debugger, the
//! dashboard and the GDB stub. Family identification lives in `decoder.rs`; this module is
//! the projection over it: the unit, data path and cross-path an instruction claims, its
//! predicate and operands as TI's tools print them, and the resource accounting the executor
//! validates packets against. The executor consults `Instruction::resources` the same way
//! the view does, so validation and display cannot disagree about what an encoding claims.
use crate::control::{register_name, written_register_name};
use crate::decoder::{
    compact_add_constant, compact_constant5, compact_constant8, compact_register,
    compact_stack_constant, decode, decode_compact, long_roles, mpy_roles, CompactOpcode, MvkForm,
    Opcode, SetClrOp,
};

/// IERR bit 3, the opcode exception: an instruction word that names no C64x+ instruction.
/// The executor raises the same bit; this module reports it from packet validation.
const IERR_OPX: u32 = 1 << 3;

/// A register file side, and the unit side that shares its letter.
#[derive(Clone, Copy, Eq, PartialEq)]
pub(crate) enum Bank {
    A,
    B,
}

pub(crate) fn bank_letter(bank: Bank) -> char {
    match bank {
        Bank::A => 'a',
        Bank::B => 'b',
    }
}

pub(crate) fn bank_index(bank: Bank) -> usize {
    match bank {
        Bank::A => 0,
        Bank::B => 1,
    }
}

pub(crate) fn bank_name(bank: Bank) -> &'static str {
    match bank {
        Bank::A => "1",
        Bank::B => "2",
    }
}

/// The register file's own letter, for messages that name a register rather than a unit side.
pub(crate) fn register_file_name(bank: Bank) -> &'static str {
    match bank {
        Bank::A => "A",
        Bank::B => "B",
    }
}

/// The register file on the other side, for cross-path operands.
pub(crate) fn opposite(bank: Bank) -> Bank {
    match bank {
        Bank::A => Bank::B,
        Bank::B => Bank::A,
    }
}

/// How an instruction is encoded: a 32-bit word, or a 16-bit compact word that cannot be
/// named without its fetch-packet header (CONTEXT.md: compact instruction).
#[derive(Clone, Copy)]
pub enum Encoding {
    Wide(u32),
    Compact(u16, u32),
}

#[derive(Clone, Copy)]
pub(crate) enum DecodedOpcode {
    Wide(Opcode),
    Compact(CompactOpcode),
}

/// One instruction as the view presents it: the encoding with its fetch-packet header
/// pairing resolved once at construction, so naming, resource accounting and operand
/// reading all dispatch on the same value. The executor's fetch path builds one per word,
/// and every consumer that reads an instruction without executing it - the differential,
/// the decode CLI, the trace, the debugger, the dashboard, the GDB stub - crosses the
/// view's seam through this value, so display and validation cannot disagree about what
/// an encoding claims.
#[derive(Clone, Copy)]
pub struct Instruction {
    pub address: u32,
    pub encoding: Encoding,
    decoded: Option<DecodedOpcode>,
    resources: Option<Resources>,
}

impl Instruction {
    /// The encoding from a raw word and its optional fetch-packet header: a header makes
    /// the instruction compact, and a compact instruction cannot be named without it.
    pub fn new(address: u32, word: u32, header: Option<u32>) -> Self {
        let encoding = match header {
            Some(header) => Encoding::Compact(word as u16, header),
            None => Encoding::Wide(word),
        };
        let decoded = match encoding {
            Encoding::Compact(word, header) => {
                decode_compact(word, header).map(DecodedOpcode::Compact)
            }
            Encoding::Wide(word) => {
                let opcode = decode(word);
                (opcode != Opcode::Unknown).then_some(DecodedOpcode::Wide(opcode))
            }
        };
        let resources = match (encoding, decoded) {
            (Encoding::Compact(word, header), Some(DecodedOpcode::Compact(family))) => {
                compact_resources(address, word, header, family).ok()
            }
            (Encoding::Wide(word), Some(DecodedOpcode::Wide(opcode))) => {
                wide_resources(address, word, opcode).ok()
            }
            _ => None,
        };
        Self {
            address,
            encoding,
            decoded,
            resources,
        }
    }

    pub(crate) fn decoded(&self) -> Option<DecodedOpcode> {
        self.decoded
    }

    fn mnemonic(&self) -> Option<&'static str> {
        match (self.encoding, self.decoded?) {
            (Encoding::Wide(word), DecodedOpcode::Wide(opcode)) => opcode.mnemonic(word),
            (Encoding::Compact(_, _), DecodedOpcode::Compact(family)) => Some(family.mnemonic()),
            _ => unreachable!("encoding and cached opcode disagree"),
        }
    }

    /// One instruction as text, assembled from the same field extractors
    /// `tools/dis6x_diff.py` compares against TI's `dis6x` over linked images: the mnemonic,
    /// the functional unit and data path it claims, its predicate, and the operands the
    /// model reads. There is deliberately no second decoder here - a rendering that
    /// disagreed with execution would be worse than a terse one - so an encoding the model
    /// does not implement renders as its raw word, and a family whose operands are not
    /// extracted renders without them.
    pub fn text(&self) -> String {
        let Some(mnemonic) = self.mnemonic() else {
            return match self.encoding {
                Encoding::Compact(word, _) => format!(".half 0x{word:04x}"),
                Encoding::Wide(word) => format!(".word 0x{word:08x}"),
            };
        };
        let mut text = String::with_capacity(32);
        if let Some(predicate) = self.predicate() {
            text.push('[');
            text.push_str(&predicate);
            text.push_str("] ");
        }
        text.push_str(mnemonic);
        if let Some(unit) = self.unit() {
            text.push_str(" .");
            text.push_str(&unit);
            if let Some(path) = self.data_path() {
                text.push_str(&path);
            }
        }
        let operands = self.operands();
        let address = self.address_operand();
        if let Some(operands) = operands.or(address) {
            text.push(' ');
            text.push_str(&operands);
        }
        text
    }

    /// How the instruction is predicated, as TI's tools print it: `a0`, `!b1` and so on, or
    /// `None` for an unconditional instruction. The five-bit field has seven valid encodings
    /// and the rest are unconditional (SPRUFE8B Table 3-1); in the compact space only two
    /// families are conditional - the branches, whose selector is A0 or B0 by the side bit
    /// with the sense in bit 4, and the write-zero-or-one form, whose two-bit condition field
    /// names both the register and the sense - plus the nested-loop SPLOOPD, whose selector
    /// is A0 or B0 by the side bit and is never inverted.
    pub fn predicate(&self) -> Option<String> {
        match (self.encoding, self.decoded) {
            (Encoding::Compact(word, _), Some(DecodedOpcode::Compact(CompactOpcode::Bnop))) => {
                if word & 0x002e != 0x002a {
                    return None;
                }
                let register = if word & 1 == 0 { "a0" } else { "b0" };
                // Bit 4 set tests for zero, which is the inverted sense.
                let sense = if word & (1 << 4) != 0 { "!" } else { "" };
                Some(format!("{sense}{register}"))
            }
            (Encoding::Compact(word, _), Some(DecodedOpcode::Compact(CompactOpcode::MvkWrite))) => {
                Some(
                    match (word >> 14) & 3 {
                        0 => "a0",
                        1 => "!a0",
                        2 => "b0",
                        _ => "!b0",
                    }
                    .to_string(),
                )
            }
            (
                Encoding::Compact(word, _),
                Some(DecodedOpcode::Compact(CompactOpcode::Sploop { nested: true, .. })),
            ) => Some(if word & 1 == 0 { "a0" } else { "b0" }.to_string()),
            (Encoding::Compact(_, _), _) => None,
            (Encoding::Wide(word), Some(DecodedOpcode::Wide(_))) => {
                let (bank, register, zero) = predicate_register(word)?;
                Some(format!(
                    "{}{}{}",
                    if zero { "!" } else { "" },
                    register_file_name(bank).to_ascii_lowercase(),
                    register
                ))
            }
            (Encoding::Wide(_), None) => None,
            _ => unreachable!("encoding and cached opcode disagree"),
        }
    }

    /// The operands TI's tools print, for every family whose fields the model reads through a
    /// shared accessor: the two-source ALU forms, the multiplies, the shifts, the bit-field
    /// operations, the constant moves, the control-register moves and the D unit's address
    /// arithmetic, plus the compact three-operand ALU, two-operand add/subtract and the
    /// compact move and constant forms. `None` for every other family, so the differential
    /// compares what the model reads rather than guessing. The register fields go through the
    /// same accessors the executor uses, so the header's RS bit remaps the compact window here
    /// exactly as it does there.
    pub fn operands(&self) -> Option<String> {
        match (self.encoding, self.decoded) {
            (Encoding::Compact(word, header), Some(DecodedOpcode::Compact(family))) => {
                compact_operands(word, header, family)
            }
            (Encoding::Wide(word), Some(DecodedOpcode::Wide(opcode))) => {
                wide_operands(word, opcode)
            }
            (_, None) => None,
            _ => unreachable!("encoding and cached opcode disagree"),
        }
    }

    /// How TI's tools write a memory access's address operand: `*+B15[134]`, `*A4++[1]`,
    /// `*--B6[2]` and so on. The fields come from the same readings of the encoding the
    /// executor uses - `address_shape` for the 32-bit forms, and the compact rules for the
    /// 16-bit ones, including the two a defect once hid behind: the updating forms step by
    /// access-sized units, and the non-aligned doubleword forms count their offsets in bytes
    /// rather than in doublewords.
    pub fn address_operand(&self) -> Option<String> {
        match (self.encoding, self.decoded) {
            (Encoding::Compact(word, header), Some(DecodedOpcode::Compact(family))) => {
                let side = if word & 1 == 0 { Bank::A } else { Bank::B };
                let name = |bank: Bank, register: usize| {
                    format!(
                        "{}{}",
                        register_file_name(bank).to_ascii_lowercase(),
                        register
                    )
                };
                match family {
                    CompactOpcode::LdstB15(_) => {
                        // The B15 window: a word-scaled unsigned offset that never updates the
                        // pointer. TI's tools write a compact offset form without the sign the
                        // 32-bit forms carry.
                        let constant = (((word >> 7) & 7) << 2) | ((word >> 13) & 3);
                        Some(format!("*{}[{constant}]", name(Bank::B, 15)))
                    }
                    CompactOpcode::LdstStack(_) => {
                        // The stack forms: a load pre-increments B15, a store post-decrements it.
                        let count = ((word >> 13) & 1) + 1;
                        let base = name(Bank::B, 15);
                        Some(if word & (1 << 14) != 0 {
                            format!("*++{base}[{count}]")
                        } else {
                            format!("*{base}--[{count}]")
                        })
                    }
                    CompactOpcode::LdstPointer(_) => {
                        let indexed = word & (1 << 10) != 0;
                        let updating = indexed && word & (1 << 11) != 0;
                        let pointer = name(side, 4 + ((word >> 7) & 3) as usize);
                        if updating {
                            let count = ((word >> 13) & 1) + 1;
                            return Some(if word & (1 << 14) != 0 {
                                format!("*--{pointer}[{count}]")
                            } else {
                                format!("*{pointer}++[{count}]")
                            });
                        }
                        if indexed {
                            let offset = compact_register((word >> 13) as u32, header);
                            return Some(format!("*{pointer}[{}]", name(side, offset)));
                        }
                        let constant = ((word >> 13) & 7) | (((word >> 11) & 1) << 3);
                        Some(format!("*{pointer}[{constant}]"))
                    }
                    _ => None,
                }
            }
            (Encoding::Compact(_, _), _) => None,
            (Encoding::Wide(word), Some(DecodedOpcode::Wide(opcode))) => {
                if matches!(opcode, Opcode::LoadLong | Opcode::StoreLong) {
                    let offset = (word >> 8) & 0x7fff;
                    let base = if word & (1 << 7) == 0 { "b14" } else { "b15" };
                    return Some(format!("*+{base}[{offset}]"));
                }
                if !matches!(
                    opcode,
                    Opcode::Ldb
                        | Opcode::Ldbu
                        | Opcode::Ldh
                        | Opcode::Ldhu
                        | Opcode::Ldw
                        | Opcode::Lddw
                        | Opcode::Ldndw
                        | Opcode::Ldnw
                        | Opcode::Stb
                        | Opcode::Sth
                        | Opcode::Stw
                        | Opcode::Stdw
                        | Opcode::Stndw
                        | Opcode::Stnw
                        | Opcode::LoadLinked
                        | Opcode::StoreLinked
                ) {
                    return None;
                }
                let shape = address_shape(word);
                let base = format!(
                    "{}{}",
                    register_file_name(shape.side).to_ascii_lowercase(),
                    shape.base
                );
                let offset = match shape.offset_register {
                    Some(register) => format!(
                        "{}{}",
                        register_file_name(shape.side).to_ascii_lowercase(),
                        register
                    ),
                    None => shape.offset.to_string(),
                };
                let sign = if shape.add { "+" } else { "-" };
                Some(match shape.update {
                    None => format!("*{sign}{base}[{offset}]"),
                    Some(false) => format!("*{sign}{sign}{base}[{offset}]"),
                    Some(true) => format!("*{base}{sign}{sign}[{offset}]"),
                })
            }
            (Encoding::Wide(_), None) => None,
            _ => unreachable!("encoding and cached opcode disagree"),
        }
    }

    /// The data path a memory access uses - T1 or T2 - which decides which of the two a
    /// packet's two memory operations may occupy.
    pub fn data_path(&self) -> Option<String> {
        let (_, _, path) = self.resources().ok()?;
        Some(format!("t{}", bank_name(path?)))
    }

    /// The functional unit the instruction claims, as `.L1`, `.S2`, `.M1`, `.D2` and so on,
    /// or `None` for the instructions that claim none - NOP, SPMASK, SPKERNEL and the compact
    /// branches. This is the assignment the packet validator enforces, exposed so
    /// `tools/dis6x_diff.py` can compare it with the unit TI's disassembler prints: unit
    /// assignment is where the long ADDA defect lived.
    pub fn unit(&self) -> Option<String> {
        let (unit, cross, _) = self.resources().ok()?;
        let (letter, side) = unit?;
        let side = match side {
            Bank::A => '1',
            Bank::B => '2',
        };
        let mut name = format!("{}{}", (letter as char).to_ascii_lowercase(), side);
        if cross.is_some() {
            name.push('x');
        }
        Some(name)
    }

    /// The unit, cross path and memory path the instruction claims, as a projection of its
    /// family. The packet validator enforces the same answer the dashboard displays, because
    /// both read this one method.
    pub fn resources(&self) -> Result<Resources, (String, u32)> {
        if let Some(resources) = self.resources {
            return Ok(resources);
        }
        // Only malformed encodings reach the slow path; rebuild its diagnostic on demand
        // from the stored family rather than identifying the raw encoding again.
        match (self.encoding, self.decoded) {
            (Encoding::Compact(word, header), Some(DecodedOpcode::Compact(family))) => {
                compact_resources(self.address, word, header, family)
            }
            (Encoding::Wide(word), Some(DecodedOpcode::Wide(opcode))) => {
                wide_resources(self.address, word, opcode)
            }
            (Encoding::Compact(word, _), None) => Err((
                format!(
                    "invalid compact opcode at {:#010x}: {word:#06x}",
                    self.address
                ),
                IERR_OPX,
            )),
            (Encoding::Wide(word), None) => Err((
                format!("invalid opcode at {:#010x}: {word:#010x}", self.address),
                IERR_OPX,
            )),
            _ => unreachable!("encoding and cached opcode disagree"),
        }
    }

    /// The datapath footprint of the instruction, for the trace's unit tags and the
    /// debugger's and dashboard's functional unit view.
    pub fn issue(&self, replayed: bool) -> UnitIssue {
        let text = self.text();
        // The side bit is the fallback for the encodings that claim no unit at all, so the
        // view still shows which register file the instruction belongs to.
        let fallback_side_a = match self.encoding {
            Encoding::Wide(word) => word & 2 == 0,
            Encoding::Compact(word, _) => word & 1 == 0,
        };
        // `resources` applies the cross-capable whitelist; testing bit 12 directly
        // would misread constant fields - MVK's 16-bit constant, for one - as a cross path.
        match self.resources() {
            Ok((unit, cross, memory_path)) => UnitIssue {
                unit: unit.map_or('?', |(letter, _)| letter as char),
                side_a: unit.map_or(fallback_side_a, |(_, bank)| bank == Bank::A),
                cross: cross.is_some(),
                mem_side_a: memory_path.map(|bank| bank == Bank::A),
                replayed,
                text,
            },
            Err(_) => UnitIssue {
                unit: '?',
                side_a: fallback_side_a,
                cross: false,
                mem_side_a: None,
                replayed,
                text,
            },
        }
    }
}

/// One instruction decoded against its fetch-packet header, as the JSON the differential and
/// the decode CLI consume: the mnemonic, and the unit, data path, address operand, operands
/// and predicate the model reads, each present only when the family carries it. The keys are
/// emitted in the sorted order `serde_json`'s own `Value` map renders them in, so the decode
/// CLI output is unchanged.
impl serde::Serialize for Instruction {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeMap;
        let mnemonic = self.mnemonic();
        let mut map = serializer.serialize_map(None)?;
        let Some(mnemonic) = mnemonic else {
            return map.end();
        };
        if let Some(operand) = self.address_operand() {
            map.serialize_entry("address", &operand)?;
        }
        if let Some(path) = self.data_path() {
            map.serialize_entry("data_path", &path)?;
        }
        map.serialize_entry("mnemonic", mnemonic)?;
        if let Some(operands) = self.operands() {
            map.serialize_entry("operands", &operands)?;
        }
        if let Some(predicate) = self.predicate() {
            map.serialize_entry("predicate", &predicate)?;
        }
        if let Some(unit) = self.unit() {
            map.serialize_entry("unit", &unit)?;
        }
        map.end()
    }
}

pub(crate) fn sign_extend(value: u32, width: u32) -> u32 {
    let shift = 32 - width;
    ((value << shift) as i32 >> shift) as u32
}

/// The register named by a 32-bit instruction's `creg`/`z` fields, with the flag saying
/// whether the instruction runs when that register is zero.
pub(crate) fn predicate_register(word: u32) -> Option<(Bank, usize, bool)> {
    let zero = word & (1 << 28) != 0;
    match (word >> 29) & 7 {
        1 => Some((Bank::B, 0, zero)),
        2 => Some((Bank::B, 1, zero)),
        3 => Some((Bank::B, 2, zero)),
        4 => Some((Bank::A, 1, zero)),
        5 => Some((Bank::A, 2, zero)),
        6 => Some((Bank::A, 0, zero)),
        _ => None,
    }
}

/// The data bank a memory access uses - T1 or T2 - by the `y` bit.
pub(crate) fn encoded_data_bank(word: u32) -> Bank {
    if word & 2 == 0 {
        Bank::A
    } else {
        Bank::B
    }
}

/// The functional unit that an ALU-family 32-bit word encodes. The binutils format masks
/// separate the L, S, D and M groups by bits 2-6 and 10-11 alone, so an instruction that
/// exists on more than one unit - `MAX2`, `PACKH2`, `SHLMB`, `UNPKHU4` and friends - can be
/// placed without consulting its mnemonic.
pub(crate) fn encoded_alu_unit(word: u32) -> u8 {
    if word & 0x1c == 0x18 {
        return b'L';
    }
    if word & 0x3c == 0x20 || word & 0xc3c == 0xc30 {
        return b'S';
    }
    if word & 0x7c == 0x40 || word & 0xc3c == 0x830 {
        return b'D';
    }
    if word & 0x83c == 0x30 {
        return b'M';
    }
    b'L'
}

/// The register file an address operation's base register belongs to, by the `s` bit.
pub(crate) fn encoded_address_bank(word: u32) -> Bank {
    if word & (1 << 7) == 0 {
        Bank::A
    } else {
        Bank::B
    }
}

/// The unit, cross path and memory path an instruction claims.
pub(crate) type Resources = (Option<(u8, Bank)>, Option<(Bank, u8)>, Option<Bank>);

fn compact_resources(
    address: u32,
    word: u16,
    header: u32,
    family: CompactOpcode,
) -> Result<Resources, (String, u32)> {
    let side = if word & 1 == 0 { Bank::A } else { Bank::B };
    // The unit, cross path and memory path are projections of the family already identified
    // by `Instruction::new`.
    let resources = match family {
        CompactOpcode::Nop
        | CompactOpcode::Spmask
        | CompactOpcode::Spmaskr
        | CompactOpcode::Bnop
        | CompactOpcode::Spkernel
        | CompactOpcode::Sploop { .. } => (None, None, None),
        CompactOpcode::Callp => (Some((b'S', side)), None, None),
        CompactOpcode::Mpy { .. } => {
            let cross = (word & (1 << 12) != 0).then_some((side, ((word >> 7) & 7) as u8));
            (Some((b'M', side)), cross, None)
        }
        CompactOpcode::MvkWrite | CompactOpcode::MvkCond(_) | CompactOpcode::Mv => {
            // The unit field names L, S or D; unit 3 never reaches here because the
            // multiply cascade claims those words first (checked by the decoder's own
            // ordering), but the fault stays as the defensive answer.
            let unit = match (word >> 3) & 3 {
                0 => b'L',
                1 => b'S',
                2 => b'D',
                _ => return Err((format!("invalid compact unit at {address:#010x}"), IERR_OPX)),
            };
            let cross = match family {
                CompactOpcode::Mv => {
                    let src = if word & 0x0066 == 0x0046 {
                        ((word >> 7) & 7) as u8
                    } else {
                        ((((word >> 10) & 3) << 3) | ((word >> 13) & 7)) as u8
                    };
                    (word & (1 << 12) != 0).then_some((side, src))
                }
                _ => None,
            };
            (Some((unit, side)), cross, None)
        }
        CompactOpcode::LdstB15(_) | CompactOpcode::LdstStack(_) | CompactOpcode::LdstPointer(_) => {
            let path = if word & (1 << 12) == 0 {
                Bank::A
            } else {
                Bank::B
            };
            (Some((b'D', side)), None, Some(path))
        }
        CompactOpcode::AddConst | CompactOpcode::Logic2op(_) => {
            let cross = (word & (1 << 12) != 0).then_some((side, ((word >> 7) & 7) as u8));
            (Some((b'L', side)), cross, None)
        }
        CompactOpcode::ShiftConst5(_)
        | CompactOpcode::ExtExtu { .. }
        | CompactOpcode::SetClrExtu(_)
        | CompactOpcode::Addk
        | CompactOpcode::ShiftReg(_)
        | CompactOpcode::MvkConst8
        | CompactOpcode::BnopReg => (Some((b'S', side)), None, None),
        CompactOpcode::AddawB15 => {
            // ADDAW from B15: on the .D1 unit the base comes over the 1X cross path,
            // the same rule the long ADDA form follows.
            let cross = matches!(side, Bank::A).then_some((side, 15));
            (Some((b'D', side)), cross, None)
        }
        CompactOpcode::AddawSubawB15 { .. } => (Some((b'D', side)), None, None),
        CompactOpcode::AddSub2op { .. } => {
            let unit = if word & 0x047e == 0x002e { b'S' } else { b'D' };
            let cross = (word & (1 << 12) != 0)
                .then_some((side, compact_register((word >> 7) as u32, header) as u8));
            (Some((unit, side)), cross, None)
        }
        CompactOpcode::CmpeqConst3 | CompactOpcode::CmplConst1(_) => {
            (Some((b'L', side)), None, None)
        }
        CompactOpcode::ShiftCoded3 { .. } => {
            let cross = (word & (1 << 12) != 0).then_some((side, ((word >> 7) & 7) as u8));
            (Some((b'S', side)), cross, None)
        }
        CompactOpcode::MvkConst5 => (Some((b'L', side)), None, None),
        CompactOpcode::Alu3op(_) => {
            // The L-unit forms have bits 1-3 clear; the S-unit forms have them as 5.
            let unit = if (word >> 1) & 7 == 0 { b'L' } else { b'S' };
            let cross = (word & (1 << 12) != 0).then_some((side, ((word >> 7) & 7) as u8));
            (Some((unit, side)), cross, None)
        }
    };
    Ok(resources)
}

fn wide_resources(address: u32, word: u32, opcode: Opcode) -> Result<Resources, (String, u32)> {
    let side = if word & 2 == 0 { Bank::A } else { Bank::B };
    let unit = match opcode {
        // The B14/B15-relative long-offset loads and stores are .D2 only: their `s`
        // bit is `data_fu` in the binutils table, picking the data path T1/T2 rather
        // than the unit. ADDA's long form encodes `s` as `fu`, so it runs on either
        // .D unit and reads its B14/B15 base across the cross path when it is on .D1.
        Opcode::LoadLong | Opcode::StoreLong => Some((b'D', Bank::B)),
        Opcode::AddaLong => Some((b'D', side)),
        Opcode::Mpy
        | Opcode::MpyCompound
        | Opcode::MpyUnary
        | Opcode::MpyExtended
        | Opcode::Xormpy
        | Opcode::Gmpy => Some((b'M', side)),
        Opcode::Addk | Opcode::BitField | Opcode::BitFieldRegister => Some((b'S', side)),
        Opcode::Maxu4
        | Opcode::Minu4
        | Opcode::Abs
        | Opcode::Abs2
        | Opcode::Swap4
        | Opcode::Subabs4
        | Opcode::LongArith
        | Opcode::Lmbd
        | Opcode::Ssub2
        | Opcode::Ssub
        | Opcode::AddSub
        | Opcode::DPack
        | Opcode::Shfl3
        | Opcode::Packh4
        | Opcode::Subc => Some((b'L', side)),
        // Families that exist on more than one unit are placed by their encoding.
        Opcode::Shlmb
        | Opcode::Shrmb
        | Opcode::Max2
        | Opcode::Min2
        | Opcode::Packh2
        | Opcode::Packlh2
        | Opcode::Packhl2
        | Opcode::Unpkhu4
        | Opcode::Unpklu4 => Some((encoded_alu_unit(word), side)),
        Opcode::Shr2
        | Opcode::Shru2
        | Opcode::Sadd2
        | Opcode::Spack2
        | Opcode::Spacku4
        | Opcode::Saddu4
        | Opcode::Saddus2
        | Opcode::Dmv
        | Opcode::Rpack2
        | Opcode::Sshl
        | Opcode::CmpEq2
        | Opcode::CmpGt2
        | Opcode::CmpEq4
        | Opcode::CmpGtu4 => Some((b'S', side)),
        // SADD exists on both units; SSUB is L only.
        Opcode::Sadd => Some((encoded_alu_unit(word), side)),
        Opcode::Nop
        | Opcode::Sploop
        | Opcode::Sploopd
        | Opcode::Sploopw
        | Opcode::Spkernel
        | Opcode::Spkernelr
        | Opcode::Spmask
        | Opcode::Spmaskr => None,
        Opcode::SubCross => Some((b'L', side)),
        Opcode::SubCrossS => Some((b'S', side)),
        Opcode::Ldb
        | Opcode::Ldbu
        | Opcode::Ldh
        | Opcode::Ldhu
        | Opcode::Ldndw
        | Opcode::Stndw
        | Opcode::Ldw
        | Opcode::Ldnw
        | Opcode::Stb
        | Opcode::Sth
        | Opcode::Stw
        | Opcode::Stnw
        | Opcode::Lddw
        | Opcode::Stdw => Some((b'D', encoded_address_bank(word))),
        Opcode::LoadLinked | Opcode::StoreLinked | Opcode::CommitLinked => Some((b'D', Bank::B)),
        Opcode::MvkShort | Opcode::MoveLong => Some((b'L', side)),
        Opcode::MvkLong
        | Opcode::Mvkh
        | Opcode::MoveShort
        | Opcode::MvcFrom
        | Opcode::MvcTo
        | Opcode::Branch
        | Opcode::BranchRegister
        | Opcode::BranchIrp
        | Opcode::BranchNrp
        | Opcode::BnopRegister
        | Opcode::Bdec
        | Opcode::Bpos
        | Opcode::Callp => Some((b'S', side)),
        // ADDKPC is .S2 only (SIDE_B_ONLY), like the long-offset D forms.
        Opcode::Addkpc => Some((b'S', Bank::B)),
        // A 32-bit immediate BNOP with the unit bit clear is the C64x+ macro form
        // that uses no functional unit, which is how the compiler pairs it with an
        // S-unit instruction in the same execute packet.
        Opcode::BnopImmediate => (word & 2 != 0).then_some((b'S', side)),
        Opcode::AddD
        | Opcode::SubD
        | Opcode::Addab
        | Opcode::Subab
        | Opcode::Addah
        | Opcode::Subah
        | Opcode::Addaw
        | Opcode::Subaw
        | Opcode::Addad
        | Opcode::MvkD => Some((b'D', side)),
        // `DINT`, `RINT` and `SWE`/`SWENR` use no functional unit.
        Opcode::Dint | Opcode::Rint | Opcode::SoftwareException | Opcode::Idle => None,
        _ if word & 0xc3c == 0xc30 => Some((b'S', side)),
        _ if word & 0xc3c == 0x830 => Some((b'D', side)),
        _ if word & 0x1c == 0x18 => Some((b'L', side)),
        _ if word & 0x3c == 0x20 => Some((b'S', side)),
        _ => {
            return Err((
                format!("unsupported functional unit at {address:#010x}"),
                IERR_OPX,
            ))
        }
    };
    let memory = matches!(
        opcode,
        Opcode::Ldb
            | Opcode::Ldbu
            | Opcode::Ldh
            | Opcode::Ldhu
            | Opcode::Ldw
            | Opcode::Lddw
            | Opcode::Ldndw
            | Opcode::Ldnw
            | Opcode::Stb
            | Opcode::Sth
            | Opcode::Stw
            | Opcode::Stdw
            | Opcode::Stndw
            | Opcode::Stnw
            | Opcode::LoadLinked
            | Opcode::StoreLinked
            | Opcode::CommitLinked
            | Opcode::LoadLong
            | Opcode::StoreLong
    );
    let cross_capable = matches!(
        opcode,
        Opcode::Add
            | Opcode::Sub
            | Opcode::And
            | Opcode::Or
            | Opcode::Xor
            | Opcode::CmpEq
            | Opcode::CmpGt
            | Opcode::CmpGtu
            | Opcode::Add2
            | Opcode::Sub2
            | Opcode::Add4
            | Opcode::Sub4
            | Opcode::Shl
            | Opcode::LongShift
            | Opcode::Shr
            | Opcode::Shru
            | Opcode::MoveLong
            | Opcode::MoveShort
            | Opcode::MvcTo
            | Opcode::BranchRegister
            | Opcode::BnopRegister
            | Opcode::Sadd
            | Opcode::Ssub
            | Opcode::Sadd2
            | Opcode::Ssub2
            | Opcode::Saddu4
            | Opcode::Saddus2
            | Opcode::Max2
            | Opcode::Min2
            | Opcode::Maxu4
            | Opcode::Minu4
            | Opcode::Sshl
            | Opcode::Shr2
            | Opcode::Shru2
            | Opcode::Shlmb
            | Opcode::Shrmb
            | Opcode::Pack2
            | Opcode::Packh2
            | Opcode::Packl4
            | Opcode::Packh4
            | Opcode::Packlh2
            | Opcode::Packhl2
            | Opcode::Spack2
            | Opcode::Spacku4
            | Opcode::Subabs4
            | Opcode::CmpEq2
            | Opcode::CmpGt2
            | Opcode::CmpEq4
            | Opcode::CmpGtu4
            | Opcode::Dmv
            | Opcode::Rpack2
            | Opcode::Xormpy
            | Opcode::Lmbd
            | Opcode::Subc
            | Opcode::AddSub
            | Opcode::DPack
            | Opcode::Shfl3
            // Families whose cross-path read TI's own disassembler marks with an X and
            // which this list used to miss: the remaining L-unit compares and ANDN, the
            // 40-bit long arithmetic, the L-unit unary ABS and the whole M unit. The
            // D-unit address arithmetic is deliberately absent: bit 12 is not its
            // cross-path selector, and `dis6x` prints those forms without an X.
                                | Opcode::CmpLt
                    | Opcode::CmpLtu
                    | Opcode::Andn
                    | Opcode::LongArith
                    | Opcode::Abs
                    | Opcode::Abs2
                    | Opcode::Swap4
                    | Opcode::Mpy
                    | Opcode::MpyCompound
                    | Opcode::MpyUnary
                    | Opcode::MpyExtended
                    | Opcode::Gmpy
                    // The S-unit bit-field register form: src2 crosses when bit 12 is set,
                    // which is what the executor reads and what TI's disassembler marks with an
                    // X. The constant form's bit 12 is part of the encoding, so it stays out.
                    | Opcode::BitFieldRegister
                    // The reversed-operand subtract forms, which are their own opcodes.
                    | Opcode::SubCross
                    | Opcode::SubCrossS
    );
    // The crossed operand is src2 for almost everything; the 40-bit long forms cross
    // src1, which is the operand their execution reads across.
    let crossed_field = if opcode == Opcode::LongArith {
        ((word >> 13) & 0x1f) as u8
    } else {
        ((word >> 18) & 0x1f) as u8
    };
    let mut cross = (cross_capable && word & (1 << 12) != 0).then_some((side, crossed_field));
    if opcode == Opcode::AddaLong && matches!(side, Bank::A) {
        // The base is B14 or B15 by bit 7, so a .D1 ADDA long reads it over 1X.
        cross = Some((side, if word & (1 << 7) == 0 { 14 } else { 15 }));
    }
    let memory_path = memory.then_some(encoded_data_bank(word));
    Ok((unit, cross, memory_path))
}

/// One instruction's datapath footprint, for the debugger's and the dashboard's functional
/// unit view and for the trace's unit tags.
#[derive(Clone, serde::Serialize)]
pub struct UnitIssue {
    /// Functional unit letter - L, S, M or D - or `?` for an instruction that claims none.
    pub unit: char,
    /// Side selector: true = .1 (register file A), false = .2 (register file B).
    pub side_a: bool,
    /// The instruction reads a register across the cross path.
    pub cross: bool,
    /// The memory path side, when the instruction accesses memory.
    pub mem_side_a: Option<bool>,
    /// The instruction came from the loop buffer rather than from a fetch.
    pub replayed: bool,
    pub text: String,
}

/// One buffered execute packet inside an active SPLOOP.
#[derive(serde::Serialize)]
pub struct SploopEntry {
    pub offset: u32,
    pub instructions: Vec<String>,
}

#[derive(serde::Serialize)]
pub struct SploopDump {
    pub ii: u32,
    pub filling: bool,
    pub started: u32,
    pub total: Option<u32>,
    pub entries: Vec<SploopEntry>,
}

/// Prefixes an instruction with the parallel-execution marker the assembler uses.
pub(crate) fn marked(text: String, parallel: bool) -> String {
    if parallel {
        format!("|| {text}")
    } else {
        text
    }
}

/// The `1_or_2_src` field positions every three-operand family shares (SPRUFE8B chapter 3):
/// src1 at bits 13-17, src2 at bits 18-22, the destination at bits 23-27 and the cross-path
/// bit at 12. Which of them is a constant, which is a register pair and which TI's tools
/// print first is each family's own business; the positions are not, so the executor and the
/// operand rendering read them here.
pub(crate) struct SourceFields {
    pub src1: usize,
    pub src2: usize,
    pub dst: usize,
    pub cross: bool,
}

pub(crate) fn source_fields(word: u32) -> SourceFields {
    SourceFields {
        src1: ((word >> 13) & 0x1f) as usize,
        src2: ((word >> 18) & 0x1f) as usize,
        dst: ((word >> 23) & 0x1f) as usize,
        cross: word & (1 << 12) != 0,
    }
}

/// The operand fields the ALU families share: src1, src2, the destination, whether src1 is a
/// constant rather than a register, whether that constant is unsigned, and whether src2 comes
/// across the cross path (SPRUFE8B chapter 3's `1_or_2_src` formats). `binary_operands` reads the
/// encoding through this, and so does the rendering `tools/dis6x_diff.py` compares with `dis6x`,
/// so there is one reading of the fields rather than two.
pub(crate) struct OperandShape {
    pub src1: usize,
    pub src2: usize,
    pub dst: usize,
    pub immediate: bool,
    pub unsigned_immediate: bool,
    pub cross: bool,
}

pub(crate) fn binary_operand_shape(word: u32) -> OperandShape {
    let l_format = word & 0x1c == 0x18;
    let d_ext_format = word & 0xc3c == 0x830;
    let op = if l_format {
        (word >> 5) & 0x7f
    } else if d_ext_format {
        (word >> 6) & 0x0f
    } else {
        (word >> 6) & 0x3f
    };
    let immediate = if l_format {
        matches!(
            op,
            0x02 | 0x06
                | 0x0e
                | 0x12
                | 0x46
                | 0x4e
                | 0x52
                | 0x56
                | 0x5e
                | 0x6a
                | 0x6e
                | 0x7a
                | 0x7e
        )
    } else if d_ext_format {
        matches!(op, 0x03 | 0x07 | 0x0b | 0x0d | 0x0f)
    } else {
        matches!(op, 0x06 | 0x0a | 0x16 | 0x18 | 0x19 | 0x1a | 0x1e | 0x22)
    };
    let fields = source_fields(word);
    OperandShape {
        src1: fields.src1,
        src2: fields.src2,
        dst: fields.dst,
        immediate,
        unsigned_immediate: l_format && matches!(op, 0x4e | 0x5e),
        cross: fields.cross,
    }
}

/// The address generator's fields for a 32-bit D-unit access (SPRUFE8B Table 3-11): which
/// register file, the base register, whether the offset is a register or a constant, whether it is
/// added or subtracted, and whether the base is updated before or after the access. Both the
/// executor and the operand rendering used by `tools/dis6x_diff.py` read the encoding through
/// this, so there is one reading of it rather than two.
pub(crate) struct AddressShape {
    pub side: Bank,
    pub base: usize,
    pub offset_register: Option<usize>,
    pub offset: u32,
    pub add: bool,
    /// `None` for the offset-only forms, `Some(false)` for pre-modify and `Some(true)` for post.
    pub update: Option<bool>,
}

pub(crate) fn address_shape(word: u32) -> AddressShape {
    let mode = (word >> 9) & 0xf;
    let field = ((word >> 13) & 0x1f) as usize;
    AddressShape {
        side: encoded_address_bank(word),
        base: ((word >> 18) & 0x1f) as usize,
        offset_register: (mode & 4 != 0).then_some(field),
        offset: field as u32,
        add: mode & 1 != 0,
        update: (mode >= 8).then_some(mode & 2 != 0),
    }
}

/// The eight unit bits an SPMASK carries, in the order SPRUFE8B numbers them. The loop buffer
/// masks instructions out by these bits and the SPMASK rendering names them, both from this
/// one table, so a mask the buffer applies and a mask TI's tools print cannot disagree.
pub(crate) const SPMASK_UNITS: [(u8, Bank); 8] = [
    (b'L', Bank::A),
    (b'L', Bank::B),
    (b'S', Bank::A),
    (b'S', Bank::B),
    (b'D', Bank::A),
    (b'D', Bank::B),
    (b'M', Bank::A),
    (b'M', Bank::B),
];

/// The bit an instruction's functional unit occupies in an SPMASK mask.
pub(crate) fn unit_mask_bit(unit: u8, bank: Bank) -> Option<u8> {
    let index = SPMASK_UNITS
        .iter()
        .position(|entry| *entry == (unit, bank))?;
    Some(1 << index)
}

/// The mask an SPMASK or SPMASKR encoding carries: bits 18-25 of the 32-bit form, and the
/// three fields the compact form assembles it from - the side bit, bits 7-9 and bits 14-15.
pub(crate) fn spmask_mask(encoding: Encoding) -> u8 {
    match encoding {
        Encoding::Wide(word) => ((word >> 18) & 0xff) as u8,
        Encoding::Compact(word, _) => {
            (word & 1) as u8 | (((word >> 7) & 7) as u8) << 1 | (((word >> 14) & 3) as u8) << 4
        }
    }
}

/// The units an SPMASK names, as TI's tools list them: `l1`, `l2,d2` and so on.
fn spmask_text(encoding: Encoding) -> String {
    let mask = spmask_mask(encoding);
    SPMASK_UNITS
        .iter()
        .enumerate()
        .filter(|(index, _)| mask & (1 << index) != 0)
        .map(|(_, (unit, bank))| {
            format!(
                "{}{}",
                (*unit as char).to_ascii_lowercase(),
                bank_name(*bank)
            )
        })
        .collect::<Vec<_>>()
        .join(",")
}

/// A register as TI's tools write it: `a4`, `b15`.
fn register_text(bank: Bank, register: usize) -> String {
    format!("{}{register}", bank_letter(bank))
}

/// A register pair as TI's tools write it - the odd register, then the even one: `a5:a4`.
fn pair_text(bank: Bank, register: usize) -> String {
    format!(
        "{}{}:{}{}",
        bank_letter(bank),
        register + 1,
        bank_letter(bank),
        register
    )
}

/// The first source of a two-source ALU form: a register, a signed five-bit constant, or the
/// unsigned one the C64x compares take.
fn alu_src1_text(shape: &OperandShape, side: Bank) -> String {
    if !shape.immediate {
        return register_text(side, shape.src1);
    }
    if shape.unsigned_immediate {
        shape.src1.to_string()
    } else {
        (sign_extend(shape.src1 as u32, 5) as i32).to_string()
    }
}

/// A shift count: the op field's low bit - bit 6 - selects a register over the five-bit
/// constant in src1's field, which is how `execute_wide` reads it too (SPRUFE8B SHL/SHR/SHRU
/// and the 40-bit shift forms).
fn shift_count_text(word: u32, side: Bank, field: usize) -> String {
    if word & (1 << 6) == 0 {
        field.to_string()
    } else {
        register_text(side, field)
    }
}

/// The operands a 32-bit instruction prints, for the families whose fields the model reads
/// through `source_fields`, `binary_operand_shape` and the op-field tables in `decoder.rs`.
/// Everything else answers `None`, which is what stops `tools/dis6x_diff.py` comparing an
/// operand list the model does not claim to read.
fn wide_operands(word: u32, opcode: Opcode) -> Option<String> {
    let side = if word & 2 == 0 { Bank::A } else { Bank::B };
    let fields = source_fields(word);
    let src2_side = if fields.cross { opposite(side) } else { side };
    match opcode {
        Opcode::Add
        | Opcode::Sub
        | Opcode::And
        | Opcode::Or
        | Opcode::Xor
        | Opcode::Andn
        | Opcode::CmpEq
        | Opcode::CmpGt
        | Opcode::CmpGtu
        | Opcode::CmpLt
        | Opcode::CmpLtu
        | Opcode::Add2
        | Opcode::Sub2
        | Opcode::Pack2
        | Opcode::Packl4
        | Opcode::Packh4
        | Opcode::Add4
        | Opcode::Lmbd
        | Opcode::Subc
        | Opcode::Shlmb
        | Opcode::Shrmb
        | Opcode::Sub4 => {
            let shape = binary_operand_shape(word);
            Some(format!(
                "{},{},{}",
                alu_src1_text(&shape, side),
                register_text(if shape.cross { opposite(side) } else { side }, shape.src2),
                register_text(side, shape.dst)
            ))
        }
        // The packed and saturating shifts share the ALU format but are written
        // `src2, src1, dst`: the value is shifted by the count (SPRUFE8B SHR2/SHRU2/SSHL).
        Opcode::Shr2 | Opcode::Shru2 | Opcode::Sshl => {
            let shape = binary_operand_shape(word);
            Some(format!(
                "{},{},{}",
                register_text(if shape.cross { opposite(side) } else { side }, shape.src2),
                alu_src1_text(&shape, side),
                register_text(side, shape.dst)
            ))
        }
        // The M unit. Which operand comes first, which is a constant and which are register
        // pairs comes from `mpy_roles` - the same op-field tables that name the instruction.
        Opcode::Mpy
        | Opcode::MpyCompound
        | Opcode::MpyExtended
        | Opcode::MpyUnary
        | Opcode::Gmpy
        | Opcode::Xormpy => {
            let roles = mpy_roles(opcode, word);
            let dst = if roles.dst_pair {
                pair_text(side, fields.dst)
            } else {
                register_text(side, fields.dst)
            };
            let src2 = register_text(src2_side, fields.src2);
            if roles.unary {
                return Some(format!("{src2},{dst}"));
            }
            let src1 = if roles.src1_constant {
                if roles.src1_constant_unsigned {
                    fields.src1.to_string()
                } else {
                    (sign_extend(fields.src1 as u32, 5) as i32).to_string()
                }
            } else if roles.src1_pair {
                pair_text(side, fields.src1)
            } else {
                register_text(side, fields.src1)
            };
            Some(if roles.src2_first {
                format!("{src2},{src1},{dst}")
            } else {
                format!("{src1},{src2},{dst}")
            })
        }
        // `SHL (.unit) src2, src1, dst`: the value is src2 and it is the operand that crosses.
        Opcode::Shl | Opcode::Shr | Opcode::Shru => Some(format!(
            "{},{},{}",
            register_text(src2_side, fields.src2),
            shift_count_text(word, side, fields.src1),
            register_text(side, fields.dst)
        )),
        // The 40-bit shifts write a register pair and read one, except the widening left
        // shift, whose source is a 32-bit register (SPRUFE8B SHL long forms).
        Opcode::LongShift => {
            let widening = matches!((word >> 6) & 0x3f, 0x12 | 0x13);
            let src2 = if widening {
                register_text(src2_side, fields.src2)
            } else {
                pair_text(side, fields.src2)
            };
            Some(format!(
                "{src2},{},{}",
                shift_count_text(word, side, fields.src1),
                pair_text(side, fields.dst)
            ))
        }
        // `EXT (.unit) src2, csta, cstb, dst`: csta sits in src1's field and cstb at bits
        // 8-12, which is where the executor reads them (SPRUFE8B EXT/EXTU/SET/CLR).
        Opcode::BitField => Some(format!(
            "{},{},{},{}",
            register_text(side, fields.src2),
            fields.src1,
            (word >> 8) & 0x1f,
            register_text(side, fields.dst)
        )),
        // The register form takes the two field positions from one control register, and it
        // is the value - src2 - that crosses.
        Opcode::BitFieldRegister => Some(format!(
            "{},{},{}",
            register_text(src2_side, fields.src2),
            register_text(side, fields.src1),
            register_text(side, fields.dst)
        )),
        // The constant moves. The five-bit forms print a signed decimal constant; the 16-bit
        // MVK prints its constant sign-extended to 32 bits, and MVKH the halfword it places
        // in the upper half, which is what `dis6x` shows.
        Opcode::MvkShort => Some(format!(
            "{},{}",
            sign_extend(fields.src2 as u32, 5) as i32,
            register_text(side, fields.dst)
        )),
        Opcode::MvkD => Some(format!(
            "{},{}",
            sign_extend(fields.src1 as u32, 5) as i32,
            register_text(side, fields.dst)
        )),
        Opcode::MvkLong => Some(format!(
            "{:#x},{}",
            sign_extend((word >> 7) & 0xffff, 16),
            register_text(side, fields.dst)
        )),
        Opcode::Mvkh => Some(format!(
            "{:#x},{}",
            ((word >> 7) & 0xffff) << 16,
            register_text(side, fields.dst)
        )),
        Opcode::Addk => Some(format!(
            "{},{}",
            sign_extend((word >> 7) & 0xffff, 16) as i32,
            register_text(side, fields.dst)
        )),
        // The control-register moves name the register the control file names it, so the
        // rendering cannot disagree with the register the executor reads or writes. An
        // unnamed encoding renders nothing rather than a number.
        Opcode::MvcFrom => Some(format!(
            "{},{}",
            register_name(fields.src2 as u8)?,
            register_text(Bank::B, fields.dst)
        )),
        Opcode::MvcTo => {
            let bank = if word & (1 << 12) == 0 {
                Bank::B
            } else {
                Bank::A
            };
            Some(format!(
                "{},{}",
                register_text(bank, fields.src2),
                written_register_name(fields.dst as u8)?
            ))
        }
        // The D unit's address arithmetic is written `src2, src1, dst`: the base first, then
        // the offset - a register, or an unsigned five-bit constant selected by the op
        // field's low bit, which is bit 7 for ADDAD and bit 8 for the rest.
        Opcode::AddD
        | Opcode::SubD
        | Opcode::Addab
        | Opcode::Subab
        | Opcode::Addah
        | Opcode::Subah
        | Opcode::Addaw
        | Opcode::Subaw
        | Opcode::Addad => {
            let immediate = if matches!(opcode, Opcode::Addad) {
                word & (1 << 7) != 0
            } else {
                word & (1 << 8) != 0
            };
            let offset = if immediate {
                fields.src1.to_string()
            } else {
                register_text(side, fields.src1)
            };
            Some(format!(
                "{},{offset},{}",
                register_text(side, fields.src2),
                register_text(side, fields.dst)
            ))
        }
        // `ADDAW .D2 B14/B15, ucst15, dst`: the B14/B15 window's own form, whose base bit is
        // the one the long loads and stores use.
        Opcode::AddaLong => Some(format!(
            "{},{},{}",
            register_text(Bank::B, if word & (1 << 7) == 0 { 14 } else { 15 }),
            (word >> 8) & 0x7fff,
            register_text(side, fields.dst)
        )),
        // The .L unit's 40-bit family, whose roles say which operands are register pairs and
        // which side of the cross path each source is read on.
        Opcode::LongArith => {
            let roles = long_roles(word);
            let src2 = if roles.src2_pair {
                pair_text(side, fields.src2)
            } else if roles.src1_cross {
                register_text(side, fields.src2)
            } else {
                register_text(src2_side, fields.src2)
            };
            let dst = if roles.dst_pair {
                pair_text(side, fields.dst)
            } else {
                register_text(side, fields.dst)
            };
            if roles.unary {
                return Some(format!("{src2},{dst}"));
            }
            let src1 = if roles.src1_constant {
                if roles.src1_constant_unsigned {
                    fields.src1.to_string()
                } else {
                    (sign_extend(fields.src1 as u32, 5) as i32).to_string()
                }
            } else if roles.src1_cross {
                register_text(src2_side, fields.src1)
            } else {
                register_text(side, fields.src1)
            };
            Some(format!("{src1},{src2},{dst}"))
        }
        // The register-to-register moves: TI's tools print the real MV encodings as a move
        // rather than as the ADD they resemble.
        Opcode::MoveLong | Opcode::MoveShort => Some(format!(
            "{},{}",
            register_text(src2_side, fields.src2),
            register_text(side, fields.dst)
        )),
        // `SUB .L1X src1, src2, dst`: here the cross path feeds src1 while src2 stays on the
        // unit's own side.
        Opcode::SubCross | Opcode::SubCrossS => Some(format!(
            "{},{},{}",
            register_text(src2_side, fields.src1),
            register_text(side, fields.src2),
            register_text(side, fields.dst)
        )),
        // The register branches: the target register, plus the NOP count BNOP folds in. The
        // label forms render nothing, because an address the decoder cannot resolve to a
        // symbol is not something to compare.
        Opcode::BranchRegister | Opcode::BnopRegister => {
            let bank = if word & (1 << 12) == 0 {
                Bank::B
            } else {
                Bank::A
            };
            let register = register_text(bank, fields.src2);
            Some(match opcode {
                Opcode::BnopRegister => format!("{register},{}", (word >> 13) & 7),
                _ => register,
            })
        }
        // A multicycle NOP prints its count; the one-cycle NOP prints nothing, which is how
        // TI's tools write it.
        Opcode::Nop => (((word >> 13) & 0xf) != 0)
            .then(|| ((word >> 13) & 0xf) + 1)
            .map(|count| count.to_string()),
        // The loop buffer's own operands: the iteration interval SPLOOP loads, and the units
        // SPMASK excludes from the replay.
        Opcode::Sploop | Opcode::Sploopd | Opcode::Sploopw => {
            Some((((word >> 23) & 0x1f) + 1).to_string())
        }
        Opcode::Spmask | Opcode::Spmaskr => Some(spmask_text(Encoding::Wide(word))),
        _ => None,
    }
}

/// The operands a compact instruction prints. Every register field goes through
/// `compact_register` and every constant through the `compact_*` accessors in `decoder.rs`,
/// so the header's RS bit remaps the window here exactly as it does in `execute_compact`.
/// The families whose operand list TI's tools write in a form the model does not read - the
/// conditional arithmetic forms, the branches and the SPLOOP family - answer `None`.
fn compact_operands(word: u16, header: u32, family: CompactOpcode) -> Option<String> {
    let side = if word & 1 == 0 { Bank::A } else { Bank::B };
    let cross = word & (1 << 12) != 0;
    let cross_side = if cross { opposite(side) } else { side };
    let bits = u32::from(word);
    let register = |field: u32, bank: Bank| register_text(bank, compact_register(field, header));
    match family {
        // Compact three-operand ALU: `src1, src2, dst`.
        CompactOpcode::Alu3op(_) => Some(format!(
            "{},{},{}",
            register(bits >> 13, side),
            register(bits >> 7, cross_side),
            register(bits >> 4, side)
        )),
        // The destination doubles as the first source, and TI's tools write it out both
        // times rather than printing a two-operand form.
        CompactOpcode::AddSub2op { .. } => {
            let destination = register(bits >> 13, side);
            Some(format!(
                "{destination},{},{destination}",
                register(bits >> 7, cross_side)
            ))
        }
        // `scst_l3i`: source, constant, destination, with the source on the cross path when
        // bit 12 is set.
        CompactOpcode::AddConst => Some(format!(
            "{},{},{}",
            register(bits >> 7, cross_side),
            compact_add_constant(word) as i32,
            register(bits >> 4, side)
        )),
        // The compact shifts read and write one register, which TI's tools write out both
        // times, with the count between them.
        CompactOpcode::ShiftConst5(_) => {
            let destination = register(bits >> 7, side);
            Some(format!(
                "{destination},{},{destination}",
                compact_constant5(word)
            ))
        }
        CompactOpcode::ShiftReg(_) => {
            let destination = register(bits >> 7, side);
            Some(format!(
                "{destination},{},{destination}",
                register(bits >> 13, side)
            ))
        }
        // `s_s3i`: the coded three-bit count, whose zero encodes sixteen and seven eight.
        CompactOpcode::ShiftCoded3 { .. } => {
            let count = match (bits >> 13) & 7 {
                0 => 16,
                7 => 8,
                value => value,
            };
            Some(format!(
                "{},{count},{}",
                register(bits >> 7, cross_side),
                register(bits >> 4, side)
            ))
        }
        // `s_sc5`: SET and CLR name one bit, so TI's tools print it as both field bounds.
        // The single-bit EXTU extracts bit `constant` upwards, which is the four-operand
        // EXTU with cstb 31 and the predicate destination the unit fixes.
        CompactOpcode::SetClrExtu(op) => {
            let source = register(bits >> 7, side);
            let constant = compact_constant5(word);
            Some(match op {
                SetClrOp::Extu => {
                    format!("{source},{constant},31,{}", register_text(side, 0))
                }
                _ => format!("{source},{constant},{constant},{source}"),
            })
        }
        // `s_s2ext`: EXT and EXTU with the fixed 16/16 and 24/24 field positions.
        CompactOpcode::ExtExtu { .. } => {
            let shift = if (bits >> 11) & 1 == 0 { 16 } else { 24 };
            Some(format!(
                "{},{shift},{shift},{}",
                register(bits >> 7, side),
                register(bits >> 13, side)
            ))
        }
        // `s_sx5`: ADDK adds its constant to the register it names.
        CompactOpcode::Addk => Some(format!(
            "{},{}",
            compact_constant5(word),
            register(bits >> 7, side)
        )),
        // `d_dx5` and `d_dx5p`: the B15 address arithmetic, whose constant TI's tools print
        // as the word count rather than the byte offset the executor scales it into.
        CompactOpcode::AddawB15 => Some(format!(
            "{},{},{}",
            register_text(Bank::B, 15),
            compact_constant5(word),
            register(bits >> 7, side)
        )),
        CompactOpcode::AddawSubawB15 { .. } => {
            let stack = register_text(Bank::B, 15);
            Some(format!("{stack},{},{stack}", compact_stack_constant(word)))
        }
        CompactOpcode::MvkConst5 => Some(format!(
            "{},{}",
            sign_extend(compact_constant5(word), 5) as i32,
            register(bits >> 7, side)
        )),
        CompactOpcode::MvkConst8 => Some(format!(
            "{},{}",
            compact_constant8(word),
            register(bits >> 7, side)
        )),
        // The predicated write of zero or one: TI's tools print it as a move of the constant.
        CompactOpcode::MvkWrite => Some(format!(
            "{},{}",
            (bits >> 13) & 1,
            register(bits >> 7, side)
        )),
        // The conditional single-register forms. The arithmetic ones take an implicit one, and
        // NEG is written as the two-operand negation of the register onto itself, which is
        // what `execute_compact` computes.
        CompactOpcode::MvkCond(form) => {
            let register = register(bits >> 7, side);
            Some(match form {
                MvkForm::Mvk0 => format!("0,{register}"),
                MvkForm::Mvk1 => format!("1,{register}"),
                MvkForm::Add | MvkForm::Xor | MvkForm::Sub => {
                    format!("{register},1,{register}")
                }
                MvkForm::Neg => format!("{register},{register}"),
                MvkForm::Mvc => format!("{register},{}", register_name(0x0d)?),
            })
        }
        // `lsdmvfr` and `lsdmvto`: which side of the move carries the five-bit register is
        // what distinguishes them, so both go through the same reading the executor uses.
        CompactOpcode::Mv => {
            let high = (((bits >> 10) & 3) << 3) as usize;
            let (source, destination) = if bits & 0x0066 == 0x0046 {
                (
                    register_text(cross_side, compact_register(bits >> 7, header)),
                    register_text(side, high | ((bits >> 13) & 7) as usize),
                )
            } else {
                (
                    register_text(cross_side, high | ((bits >> 7) & 7) as usize),
                    register_text(side, compact_register(bits >> 13, header)),
                )
            };
            Some(format!("{source},{destination}"))
        }
        // `m_m3`: the compact multiplies, whose two-bit destination field names an even
        // register in the window.
        CompactOpcode::Mpy { .. } => Some(format!(
            "{},{},{}",
            register(bits >> 13, side),
            register(bits >> 7, cross_side),
            register(((bits >> 10) & 3) << 1, side)
        )),
        // `l_l2c`, `l_lx3c` and `l_lx1c`: the compare and logical forms whose one-bit
        // destination names a predicate register the header's RS bit never remaps.
        CompactOpcode::Logic2op(_) => Some(format!(
            "{},{},{}",
            register(bits >> 13, side),
            register(bits >> 7, cross_side),
            register_text(side, ((bits >> 4) & 1) as usize)
        )),
        CompactOpcode::CmpeqConst3 => Some(format!(
            "{},{},{}",
            (bits >> 13) & 7,
            register(bits >> 7, side),
            register_text(side, ((bits >> 11) & 1) as usize)
        )),
        CompactOpcode::CmplConst1(_) => Some(format!(
            "{},{},{}",
            (bits >> 13) & 1,
            register(bits >> 7, side),
            register_text(side, ((bits >> 11) & 1) as usize)
        )),
        // The compact loop-buffer and branch forms: the iteration interval, the excluded
        // units, and the register branch with the NOP count it folds in.
        CompactOpcode::Sploop { .. } => {
            Some(((((bits >> 7) & 7) | (((bits >> 14) & 1) << 3)) + 1).to_string())
        }
        CompactOpcode::Spmask | CompactOpcode::Spmaskr => {
            Some(spmask_text(Encoding::Compact(word, header)))
        }
        CompactOpcode::BnopReg => Some(format!(
            "{},{}",
            register_text(Bank::B, ((bits >> 7) & 0x0f) as usize),
            (bits >> 13) & 7
        )),
        CompactOpcode::Nop => Some((((bits >> 13) & 7) + 1).to_string()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A header marking every one of the seven data words as holding two compact instructions,
    /// which is what a compact encoding has to be constructed against.
    const COMPACT_HEADER: u32 = 0xe020_0000;

    fn wide(word: u32) -> Instruction {
        Instruction::new(0, word, None)
    }

    fn compact(word: u16) -> Instruction {
        Instruction::new(0, u32::from(word), Some(COMPACT_HEADER))
    }

    /// `text` is the one projection of the instruction view with no oracle behind it.
    ///
    /// Every field it prints - the mnemonic, the unit, the data path, the predicate, the
    /// operands - is compared against TI's `dis6x` over 436813 instructions on every suite pass,
    /// and `resources` is swept against the decoder over the whole compact word space. What none
    /// of that sees is the assembly itself: where the brackets, the dot and the spaces go, and
    /// which of the operand forms wins. That is what these pin, so the debugger's listing and the
    /// trace cannot lose a field without a test noticing.
    #[test]
    fn text_prints_the_mnemonic_unit_and_operands() {
        // ADD .S1 A0,A0,A1 - the ordinary shape: mnemonic, dotted unit, operands.
        assert_eq!(wide(0x0080_01e0).text(), "add .s1 a0,a0,a1");
    }

    #[test]
    fn text_appends_the_cross_path_to_the_unit() {
        // The cross path is part of the unit's own name, so it needs no separator.
        assert_eq!(wide(0x0084_11e2).text(), "add .s2x b0,a1,b1");
    }

    #[test]
    fn text_prints_the_data_path_against_the_unit_and_the_address_as_the_operand() {
        // A load claims a unit, a data path and an address rather than an operand list, so the
        // data path joins the unit without a separator and the address stands in for operands.
        assert_eq!(wide(0x0080_3264).text(), "ldw .d1t1 *++a0[1]");
        assert_eq!(wide(0x0080_000c).text(), "ldhu .d2t1 *+b14[0]");
    }

    #[test]
    fn text_brackets_the_predicate_before_the_mnemonic() {
        assert_eq!(wide(0x2080_01e0).text(), "[b0] add .s1 a0,a0,a1");
        // The inverted sense keeps the bang inside the brackets, the way TI's tools print it.
        assert_eq!(wide(0x3080_01e0).text(), "[!b0] add .s1 a0,a0,a1");
    }

    #[test]
    fn text_omits_a_unit_an_instruction_does_not_claim() {
        // NOP claims no functional unit, so there is no dot and no bogus side.
        assert_eq!(wide(0).text(), "nop");
        // The compact NOP carries its cycle count as an operand but still claims no unit.
        assert_eq!(compact(0x0c6e).text(), "nop 1");
    }

    /// An encoding the model does not implement renders as its raw word rather than as a guess.
    /// The widths differ because a compact instruction is a halfword.
    #[test]
    fn an_unimplemented_encoding_renders_as_its_raw_word() {
        assert_eq!(wide(0x5645_4314).text(), ".word 0x56454314");
        assert_eq!(compact(0x0008).text(), ".half 0x0008");
    }

    /// `issue` is the other unasserted projection: the debugger's and the dashboard's unit view.
    /// It reads the same `resources` the packet validator enforces, so the facts are pinned
    /// elsewhere; what is not is the mapping onto the letter, the side and the two path flags.
    #[test]
    fn issue_reports_the_unit_side_and_paths_resources_claims() {
        let issue = wide(0x0080_01e0).issue(false);
        assert_eq!(issue.unit, 'S');
        assert!(issue.side_a);
        assert!(!issue.cross);
        assert_eq!(issue.mem_side_a, None);
        assert!(!issue.replayed);
        assert_eq!(issue.text, "add .s1 a0,a0,a1");

        // The cross path and the B side.
        let crossed = wide(0x0084_11e2).issue(false);
        assert_eq!(crossed.unit, 'S');
        assert!(!crossed.side_a);
        assert!(crossed.cross);

        // A load also names the side its memory path uses.
        let load = wide(0x0080_3264).issue(false);
        assert_eq!(load.unit, 'D');
        assert_eq!(load.mem_side_a, Some(true));

        // Replay is the caller's fact, not the instruction's, and is carried through.
        assert!(wide(0).issue(true).replayed);
    }

    /// An instruction that claims no unit still shows which register file it belongs to, from
    /// the side bit, so the view never prints a bogus `.?1`.
    #[test]
    fn issue_falls_back_to_the_side_bit_when_no_unit_is_claimed() {
        let unimplemented = wide(0x5645_4314).issue(false);
        assert_eq!(unimplemented.unit, '?');
        assert!(!unimplemented.cross);
        assert_eq!(unimplemented.mem_side_a, None);
        // Bit 1 of a wide encoding is the side, and 0x...14 leaves it clear: the A side.
        assert!(unimplemented.side_a);
        assert!(!wide(0x5645_4316).issue(false).side_a);

        // A compact encoding takes its side from bit 0 instead.
        assert_eq!(compact(0x0008).issue(false).unit, '?');
        assert!(compact(0x0008).issue(false).side_a);
        assert!(!compact(0x0009).issue(false).side_a);
    }
}
