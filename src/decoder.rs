#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Opcode {
    Nop,
    Idle,
    Ldb,
    Ldbu,
    Ldh,
    Ldhu,
    Ldw,
    LoadLinked,
    StoreLinked,
    CommitLinked,
    Ldnw,
    Lddw,
    Ldndw,
    Stw,
    Stnw,
    Stb,
    Sth,
    Stdw,
    MvkLong,
    Stndw,
    Mvkh,
    MvkShort,
    Add,
    Sub,
    And,
    Or,
    Xor,
    Andn,
    AddD,
    SubD,
    Addab,
    Subab,
    Addah,
    Subah,
    Addaw,
    Subaw,
    Addad,
    MvkD,
    LoadLong,
    StoreLong,
    AddaLong,
    Mpy,
    MpyCompound,
    MpyUnary,
    MpyExtended,
    Addk,
    SubCross,
    SubCrossS,
    BitField,
    BitFieldRegister,
    Max2,
    Packh2,
    LongArith,
    LongShift,
    Bdec,
    Bpos,
    Dint,
    Rint,
    SoftwareException,
    Lmbd,
    Subc,
    Shlmb,
    Shrmb,
    Packlh2,
    Packhl2,
    Shr2,
    Shru2,
    Sadd2,
    Ssub2,
    Spack2,
    Spacku4,
    Sshl,
    Xormpy,
    Gmpy,
    AddSub,
    DPack,
    Shfl3,
    Sadd,
    Ssub,
    CmpEq2,
    CmpGt2,
    CmpEq4,
    CmpGtu4,
    Saddu4,
    Saddus2,
    Dmv,
    Rpack2,
    Abs2,
    Swap4,
    Unpkhu4,
    Unpklu4,
    Min2,
    Maxu4,
    Minu4,
    Abs,
    Subabs4,
    CmpEq,
    CmpGt,
    CmpGtu,
    CmpLt,
    CmpLtu,
    Add2,
    Sub2,
    Add4,
    Sub4,
    Pack2,
    Packl4,
    Packh4,
    Shl,
    Shr,
    Shru,
    MoveLong,
    MoveShort,
    MvcFrom,
    MvcTo,
    Sploop,
    Sploopd,
    Sploopw,
    Spkernel,
    Spkernelr,
    Spmask,
    Spmaskr,
    Branch,
    BranchRegister,
    BranchIrp,
    BranchNrp,
    BnopRegister,
    BnopImmediate,
    Addkpc,
    Callp,
    Unknown,
}

pub fn decode(word: u32) -> Opcode {
    let instruction = word & !1;
    if instruction & 0xf03f_fffc == 0x0003_4000 {
        return Opcode::Spkernel;
    }
    if instruction & 0xffff_fffc == 0x0003_6000 {
        return Opcode::Spkernelr;
    }
    if instruction & 0x0002_1ffc == 0x0002_0000 {
        match (instruction >> 13) & 0xf {
            0x8 => return Opcode::Spmask,
            0x9 => return Opcode::Spmaskr,
            0xc => return Opcode::Sploop,
            0xd => return Opcode::Sploopd,
            0xf => return Opcode::Sploopw,
            _ => {}
        }
    }
    // D-unit long-offset forms: a 15-bit unsigned offset from B14 or B15.
    if instruction & 0xf000_000c == 0x1000_000c {
        return Opcode::AddaLong;
    }
    if instruction & 0xc == 0xc {
        return match (instruction >> 4) & 7 {
            0 | 1 | 2 | 4 | 6 => Opcode::LoadLong,
            _ => Opcode::StoreLong,
        };
    }
    // `nfu_nop_idle`: the count lives in a four-bit op field at bits 13-16 and runs from 1 to
    // 9 (op 0 to 8); op 0xf in the same slot is IDLE, which parks the CPU until an interrupt.
    if instruction & 0xfffe_1ffe == 0 {
        let op = (instruction >> 13) & 0xf;
        if op <= 8 {
            return Opcode::Nop;
        }
        if op == 0xf {
            return Opcode::Idle;
        }
    }
    // The unit-less control instructions have fixed encodings that would otherwise fall
    // into the M-unit multiply space.
    match instruction & 0xffff_fffc {
        0x1000_4000 => return Opcode::Dint,
        0x1000_6000 => return Opcode::Rint,
        0x1000_0000 | 0x1000_2000 => return Opcode::SoftwareException,
        _ => {}
    }

    match instruction & 0x0003_fffe {
        0x0642 => return Opcode::LoadLinked,
        0x06c2 => return Opcode::StoreLinked,
        0x0742 => return Opcode::CommitLinked,
        _ => {}
    }
    match instruction & 0x17c {
        0x004 => return Opcode::Ldhu,
        0x014 => return Opcode::Ldbu,
        0x024 => return Opcode::Ldb,
        0x034 => return Opcode::Stb,
        0x044 => return Opcode::Ldh,
        0x054 => return Opcode::Sth,
        _ => {}
    }
    if instruction & 0x17c == 0x0124 {
        return Opcode::Ldndw;
    }
    if instruction & 0x17c == 0x0174 {
        return Opcode::Stndw;
    }
    if instruction & 0x17c == 0x0164 {
        return Opcode::Lddw;
    }
    if instruction & 0x17c == 0x0144 {
        return Opcode::Stdw;
    }
    if instruction & 0x17c == 0x0134 {
        return Opcode::Ldnw;
    }
    if instruction & 0x17c == 0x0154 {
        return Opcode::Stnw;
    }
    if instruction & 0x17c == 0x064 {
        return Opcode::Ldw;
    }
    if instruction & 0x17c == 0x074 {
        return Opcode::Stw;
    }
    if instruction & 0x1ffc == 0x0120 {
        return Opcode::BnopImmediate;
    }
    if instruction & 0x1ffc == 0x0160 {
        return Opcode::Addkpc;
    }
    if instruction & 0xf000_007c == 0x1000_0010 {
        return Opcode::Callp;
    }
    if instruction & 0x007f_effc == 0x0018_00e0 {
        return Opcode::BranchIrp;
    }
    if instruction & 0x007f_effc == 0x001c_00e0 {
        return Opcode::BranchNrp;
    }
    if instruction & 0x0f83_0ffc == 0x0080_0360 {
        return Opcode::BnopRegister;
    }
    if instruction & 0x0f83_effc == 0x0000_0360 {
        return Opcode::BranchRegister;
    }
    if instruction & 0x7c == 0x10 {
        return Opcode::Branch;
    }
    if instruction & 0xffc == 0x358 && ((instruction >> 13) & 0x1f) == 5 {
        return Opcode::MvkShort;
    }
    if instruction & 0x3c == 0x28 {
        return if instruction & (1 << 6) == 0 {
            Opcode::MvkLong
        } else {
            Opcode::Mvkh
        };
    }
    if instruction & 0x7c == 0x50 {
        return Opcode::Addk;
    }
    // S-unit bit-field operations with constant positions (`s_field`).
    if instruction & 0x3c == 0x08 {
        return Opcode::BitField;
    }
    // M unit: the multiply and packed-multiply families.
    if instruction & 0xf000_083c == 0x1000_0030 {
        return match (instruction >> 6) & 0x1f {
            0x1b => Opcode::Xormpy,
            0x1f => Opcode::Gmpy,
            _ => Opcode::MpyExtended,
        };
    }
    if instruction & 0xffc == 0x00f0 {
        return Opcode::MpyUnary;
    }
    if instruction & 0x83c == 0x0030 {
        // MPYSPDP and MPYSP2DP are C67x floating-point multiplies; TI's own tools call their
        // encodings invalid for a C6400+, so this device has no instruction there.
        return match ((instruction & !1) >> 6) & 0x1f {
            0x16 | 0x17 => Opcode::Unknown,
            _ => Opcode::MpyCompound,
        };
    }
    if instruction & 0x7c == 0x0000 {
        // The same for the .M unit's floating-point slots: MPYI and MPYID (the 32-bit integer
        // multiplies the C67x adds), MPYDP and MPYSP. `dis6x` disassembles a C6400+ object
        // holding any of these words as data, and `cl6x -mv6400+` refuses their mnemonics.
        return match ((instruction & !1) >> 7) & 0x1f {
            0x04 | 0x06 | 0x08 | 0x0c | 0x0e | 0x1c => Opcode::Unknown,
            _ => Opcode::Mpy,
        };
    }
    if instruction & 0xc3c == 0xc30 {
        match (instruction >> 6) & 0xf {
            0x0 => return Opcode::Sadd2,
            0x1 => return Opcode::Saddus2,
            0x2 => return Opcode::Spack2,
            0x3 => return Opcode::Saddu4,
            0x4 => return Opcode::Spacku4,
            0xc => return Opcode::Min2,
            0xd => return Opcode::Max2,
            // `RPACK2` takes the one encoding in this slot that no predicate can produce:
            // creg zero with the z bit set. Everything else at op 0xb is `DMV`.
            0xb => {
                return if (instruction >> 28) & 0xf == 1 {
                    Opcode::Rpack2
                } else {
                    Opcode::Dmv
                };
            }
            0x6 => return Opcode::Andn,
            0x5 => return Opcode::SubCrossS,
            0x9 => return Opcode::Shlmb,
            0xa => return Opcode::Shrmb,
            0x7 => return Opcode::Shr2,
            0x8 => return Opcode::Shru2,
            0xf => return Opcode::Pack2,
            _ => {}
        }
    }
    if instruction & 0xc3c == 0x830 {
        match (instruction >> 6) & 0xf {
            0x0 => return Opcode::Andn,
            0x2 | 0x3 => return Opcode::Or,
            0x4 => return Opcode::Add2,
            0x5 => return Opcode::Sub2,
            0x6 | 0x7 => return Opcode::And,
            0xa | 0xb => return Opcode::Add,
            0xc | 0xd => return Opcode::Sub,
            0xe | 0xf => return Opcode::Xor,
            _ => {}
        }
    }
    if instruction & 0x7c == 0x40 {
        match (instruction >> 7) & 0x3f {
            0x00 => return Opcode::MvkD,
            0x10 | 0x12 => return Opcode::AddD,
            0x11 | 0x13 => return Opcode::SubD,
            0x30 | 0x32 => return Opcode::Addab,
            0x31 | 0x33 => return Opcode::Subab,
            0x34 | 0x36 => return Opcode::Addah,
            0x35 | 0x37 => return Opcode::Subah,
            0x38 | 0x3a => return Opcode::Addaw,
            0x39 | 0x3b => return Opcode::Subaw,
            0x3c | 0x3d => return Opcode::Addad,
            _ => {}
        }
    }
    // L-unit and S-unit unary families (`l_unary`, `s_unary`).
    if instruction & 0xffc == 0x358 || instruction & 0xffc == 0xf20 {
        return match (instruction >> 13) & 0x1f {
            // The .L unit's ABS lives here; the same slot on the .S unit is the C67x ABSSP,
            // which this device does not have - `dis6x` leaves that word as data.
            0x0 if instruction & 0xffc == 0xf20 => Opcode::Unknown,
            0x0 => Opcode::Abs,
            0x1 => Opcode::Swap4,
            0x2 => Opcode::Unpklu4,
            0x3 => Opcode::Unpkhu4,
            0x4 => Opcode::Abs2,
            0x05 => Opcode::MvkLong,
            _ => Opcode::Unknown,
        };
    }
    // `BDEC` and `BPOS` occupy fixed slots in the S-unit branch space, so they must be
    // recognised before the general S-unit group dispatch.
    if instruction & 0x1ffc == 0x1020 {
        return Opcode::Bdec;
    }
    if instruction & 0x1ffc == 0x0020 {
        return Opcode::Bpos;
    }
    // `l_1_or_2_src_noncond`: the L-unit pair-writing family, which takes the creg/z pattern
    // no predicate can produce (creg zero with z set).
    if instruction & 0xf000_001c == 0x1000_0018 {
        match (instruction >> 5) & 0x7f {
            0x0c..=0x0f => return Opcode::AddSub,
            0x33 | 0x34 => return Opcode::DPack,
            0x36 => return Opcode::Shfl3,
            _ => {}
        }
    }
    if instruction & 0x1c == 0x18 {
        let op = (instruction >> 5) & 0x7f;
        let src1 = (instruction >> 13) & 0x1f;
        match op {
            0x02 | 0x03 => return Opcode::Add,
            0x06 | 0x07 => return Opcode::Sub,
            // The 40-bit "long" family: results (and often one source) live in a register
            // pair whose odd register holds bits 39-32.
            0x20 | 0x21 | 0x23 | 0x24 | 0x27 | 0x29 | 0x2b | 0x2c | 0x2f | 0x30 | 0x31 | 0x37
            | 0x38 | 0x3f | 0x40 | 0x60 | 0x63 => return Opcode::LongArith,
            // Comparisons against a 40-bit long operand share the long family's encodings
            // but write a single-register result.
            0x44 | 0x45 | 0x4c | 0x4d | 0x50 | 0x51 | 0x54 | 0x55 | 0x5c | 0x5d => {
                return Opcode::LongArith
            }
            0x12 | 0x13 => return Opcode::Sadd,
            // SSUB has three int forms: 0x0f crosses src2, 0x1f crosses src1 and 0x0e takes
            // a 5-bit signed constant on src1 (SPRUFE8B SSUB opcode map).
            0x0e | 0x0f | 0x1f => return Opcode::Ssub,
            0x42 => return Opcode::Max2,
            0x1e => return Opcode::Packh2,
            0x1c => return Opcode::Packhl2,
            0x1b => return Opcode::Packlh2,
            0x64 => return Opcode::Ssub2,
            0x6a | 0x6b => return Opcode::Lmbd,
            0x4b => return Opcode::Subc,
            0x61 => return Opcode::Shlmb,
            0x62 => return Opcode::Shrmb,
            0x41 => return Opcode::Min2,
            0x43 => return Opcode::Maxu4,
            0x48 => return Opcode::Minu4,
            0x5a => return Opcode::Subabs4,
            0x16 | 0x17 => return Opcode::SubCross,
            0x56 | 0x57 => return Opcode::CmpLt,
            0x5e | 0x5f => return Opcode::CmpLtu,
            0x7a | 0x7b => return Opcode::And,
            0x7f => return Opcode::Or,
            0x7e if src1 != 0 => return Opcode::Or,
            0x6e | 0x6f => return Opcode::Xor,
            0x7c | 0x7d => return Opcode::Andn,
            0x52 | 0x53 => return Opcode::CmpEq,
            0x46 | 0x47 => return Opcode::CmpGt,
            0x4e | 0x4f => return Opcode::CmpGtu,
            0x05 => return Opcode::Add2,
            0x00 => return Opcode::Pack2,
            0x68 => return Opcode::Packl4,
            0x69 => return Opcode::Packh4,
            0x04 => return Opcode::Sub2,
            0x65 => return Opcode::Add4,
            0x66 => return Opcode::Sub4,
            _ => {}
        }
        if op == 0x7e && src1 == 0 {
            return Opcode::MoveLong;
        }
    }
    if instruction & 0x3c == 0x20 {
        let op = (instruction >> 6) & 0x3f;
        let src1 = (instruction >> 13) & 0x1f;
        if instruction & 2 != 0 && op == 0x0f && instruction & (1 << 12) == 0 {
            return Opcode::MvcFrom;
        }
        if instruction & 2 != 0 && op == 0x0e {
            return Opcode::MvcTo;
        }
        match op {
            // The long (40-bit) shift forms: 0x12/0x13 widen an int into a long, and
            // 0x24/0x25, 0x30/0x31, 0x34/0x35 shift a long operand in place.
            0x12 | 0x13 | 0x24 | 0x25 | 0x30 | 0x31 | 0x34 | 0x35 => return Opcode::LongShift,
            0x32 | 0x33 => return Opcode::Shl,
            0x36 | 0x37 => return Opcode::Shr,
            0x26 | 0x27 => return Opcode::Shru,
            _ => {}
        }
        match op {
            0x06 | 0x07 => return Opcode::Add,
            0x16 | 0x17 => return Opcode::Sub,
            0x1e | 0x1f => return Opcode::And,
            0x1b => return Opcode::Or,
            0x1a if src1 != 0 => return Opcode::Or,
            0x0a | 0x0b => return Opcode::Xor,
            0x01 => return Opcode::Add2,
            0x11 => return Opcode::Sub2,
            0x2b | 0x2f | 0x3b | 0x3f => return Opcode::BitFieldRegister,
            0x14 => return Opcode::CmpGt2,
            0x15 => return Opcode::CmpGtu4,
            0x1c => return Opcode::CmpEq4,
            0x1d => return Opcode::CmpEq2,
            0x18 => return Opcode::Shr2,
            0x19 => return Opcode::Shru2,
            0x20 => return Opcode::Sadd,
            0x22 | 0x23 => return Opcode::Sshl,
            0x09 => return Opcode::Packh2,
            0x08 => return Opcode::Packhl2,
            0x10 => return Opcode::Packlh2,
            _ => {}
        }
        if op == 0x1a && src1 == 0 {
            return Opcode::MoveShort;
        }
    }
    Opcode::Unknown
}

impl Opcode {
    pub fn mnemonic(self, word: u32) -> Option<&'static str> {
        match self {
            Opcode::Ldndw => Some("ldndw"),
            Opcode::Nop => Some("nop"),
            Opcode::Idle => Some("idle"),
            Opcode::Ldb => Some("ldb"),
            Opcode::Lddw => Some("lddw"),
            Opcode::Ldbu => Some("ldbu"),
            Opcode::Stndw => Some("stndw"),
            Opcode::Ldh => Some("ldh"),
            Opcode::Ldhu => Some("ldhu"),
            Opcode::Stdw => Some("stdw"),
            Opcode::Ldw => Some("ldw"),
            Opcode::LoadLinked => Some("ll"),
            Opcode::StoreLinked => Some("sl"),
            Opcode::CommitLinked => Some("cmtl"),
            Opcode::Ldnw => Some("ldnw"),
            Opcode::Stw => Some("stw"),
            Opcode::Stb => Some("stb"),
            Opcode::Sth => Some("sth"),
            Opcode::Stnw => Some("stnw"),
            Opcode::MvkLong | Opcode::MvkShort => Some("mvk"),
            Opcode::Mvkh => Some("mvkh"),
            Opcode::Add => Some("add"),
            Opcode::Sub => Some("sub"),
            Opcode::And => Some("and"),
            Opcode::Or => Some("or"),
            Opcode::Xor => Some("xor"),
            Opcode::Andn => Some("andn"),
            Opcode::AddD => Some("add"),
            Opcode::SubD => Some("sub"),
            Opcode::Addab => Some("addab"),
            Opcode::Subab => Some("subab"),
            Opcode::Addah => Some("addah"),
            Opcode::Subah => Some("subah"),
            Opcode::SubCross | Opcode::SubCrossS => Some("sub"),
            Opcode::BitField => Some(match ((word & !1) >> 6) & 3 {
                0 => "extu",
                1 => "ext",
                2 => "set",
                _ => "clr",
            }),
            Opcode::Max2 => Some("max2"),
            Opcode::Packh2 => Some("packh2"),
            Opcode::LongArith => Some(long_arith_mnemonic((word >> 5) & 0x7f)),
            Opcode::LongShift => Some(long_shift_mnemonic((word >> 6) & 0x3f)),
            Opcode::Bdec => Some("bdec"),
            Opcode::Bpos => Some("bpos"),
            Opcode::Dint => Some("dint"),
            Opcode::Rint => Some("rint"),
            Opcode::SoftwareException => Some(if word & 0x2000 == 0 { "swe" } else { "swenr" }),
            Opcode::Lmbd => Some("lmbd"),
            Opcode::Subc => Some("subc"),
            Opcode::Shlmb => Some("shlmb"),
            Opcode::Shrmb => Some("shrmb"),
            Opcode::Packlh2 => Some("packlh2"),
            Opcode::Packhl2 => Some("packhl2"),
            Opcode::Shr2 => Some("shr2"),
            Opcode::Shru2 => Some("shru2"),
            Opcode::Sadd2 => Some("sadd2"),
            Opcode::Ssub2 => Some("ssub2"),
            Opcode::Spack2 => Some("spack2"),
            Opcode::Spacku4 => Some("spacku4"),
            Opcode::Sshl => Some("sshl"),
            Opcode::Xormpy => Some("xormpy"),
            Opcode::Gmpy => Some("gmpy"),
            Opcode::AddSub => Some(match (word >> 5) & 0x7f {
                0x0c => "addsub",
                0x0d => "addsub2",
                0x0e => "saddsub",
                _ => "saddsub2",
            }),
            // DPACK2 pairs PACK2 with PACKH2; DPACKX2 pairs the two PACKLH2 halves.
            Opcode::DPack => Some(if (word >> 5) & 0x7f == 0x34 {
                "dpack2"
            } else {
                "dpackx2"
            }),
            Opcode::Shfl3 => Some("shfl3"),
            Opcode::Sadd => Some("sadd"),
            Opcode::Ssub => Some("ssub"),
            Opcode::CmpEq2 => Some("cmpeq2"),
            Opcode::CmpGt2 => Some("cmpgt2"),
            Opcode::CmpEq4 => Some("cmpeq4"),
            Opcode::CmpGtu4 => Some("cmpgtu4"),
            Opcode::Saddu4 => Some("saddu4"),
            Opcode::Saddus2 => Some("saddus2"),
            Opcode::Dmv => Some("dmv"),
            Opcode::Rpack2 => Some("rpack2"),
            Opcode::Min2 => Some("min2"),
            Opcode::Maxu4 => Some("maxu4"),
            Opcode::Minu4 => Some("minu4"),
            Opcode::Abs => Some("abs"),
            Opcode::Abs2 => Some("abs2"),
            Opcode::Swap4 => Some("swap4"),
            Opcode::Unpkhu4 => Some("unpkhu4"),
            Opcode::Unpklu4 => Some("unpklu4"),
            Opcode::Subabs4 => Some("subabs4"),
            Opcode::BitFieldRegister => Some(match ((word & !1) >> 6) & 0x3f {
                0x2b => "extu",
                0x2f => "ext",
                0x3b => "set",
                _ => "clr",
            }),
            Opcode::Addk => Some("addk"),
            Opcode::Mpy => Some(mpy_mnemonic(((word & !1) >> 7) & 0x1f)),
            Opcode::MpyCompound => Some(mpy_compound_mnemonic(((word & !1) >> 6) & 0x1f)),
            Opcode::MpyUnary => Some(mpy_unary_mnemonic(((word & !1) >> 13) & 0x1f)),
            Opcode::MpyExtended => Some(mpy_extended_mnemonic(((word & !1) >> 6) & 0x1f)),
            Opcode::LoadLong | Opcode::StoreLong => Some(match ((word & !1) >> 4) & 7 {
                0 => "ldhu",
                1 => "ldbu",
                2 => "ldb",
                3 => "stb",
                4 => "ldh",
                5 => "sth",
                6 => "ldw",
                _ => "stw",
            }),
            Opcode::AddaLong => Some(match ((word & !1) >> 4) & 7 {
                3 => "addab",
                5 => "addah",
                _ => "addaw",
            }),
            Opcode::Addaw => Some("addaw"),
            Opcode::Subaw => Some("subaw"),
            Opcode::Addad => Some("addad"),
            Opcode::MvkD => Some("mvk"),
            Opcode::CmpEq => Some("cmpeq"),
            Opcode::CmpGt => Some("cmpgt"),
            Opcode::CmpGtu => Some("cmpgtu"),
            Opcode::Add2 => Some("add2"),
            Opcode::Sub2 => Some("sub2"),
            Opcode::CmpLt => Some("cmplt"),
            Opcode::CmpLtu => Some("cmpltu"),
            Opcode::Add4 => Some("add4"),
            Opcode::Sub4 => Some("sub4"),
            Opcode::MoveLong | Opcode::MoveShort => Some("mv"),
            Opcode::MvcFrom | Opcode::MvcTo => Some("mvc"),
            Opcode::Sploop => Some("sploop"),
            Opcode::Sploopd => Some("sploopd"),
            Opcode::Sploopw => Some("sploopw"),
            Opcode::Pack2 => Some("pack2"),
            Opcode::Packl4 => Some("packl4"),
            Opcode::Packh4 => Some("packh4"),
            Opcode::Spkernel => Some("spkernel"),
            Opcode::Spkernelr => Some("spkernelr"),
            Opcode::Shl => Some("shl"),
            Opcode::Shr => Some("shr"),
            Opcode::Shru => Some("shru"),
            Opcode::Spmask => Some("spmask"),
            Opcode::Spmaskr => Some("spmaskr"),
            Opcode::Branch => Some("b"),
            Opcode::BranchRegister | Opcode::BranchIrp | Opcode::BranchNrp => Some("b"),
            Opcode::BnopRegister | Opcode::BnopImmediate => Some("bnop"),
            Opcode::Addkpc => Some("addkpc"),
            Opcode::Callp => Some("callp"),
            Opcode::Unknown => None,
        }
    }
}

/// M-unit `m_mpy` format: op field at bits 7-11.
/// The .L unit's 40-bit family: each operation field names one instruction, which is what the
/// execution arms in `cpu.rs` implement (SPRUFE8B's opcode maps for the long forms).
pub fn long_arith_mnemonic(op: u32) -> &'static str {
    match op {
        0x20 | 0x21 | 0x23 => "add",
        0x24 | 0x27 | 0x37 => "sub",
        0x29 | 0x2b => "addu",
        0x2c => "ssub",
        0x2f | 0x3f => "subu",
        0x30 | 0x31 => "sadd",
        0x38 => "abs",
        0x40 => "sat",
        0x44 | 0x45 => "cmpgt",
        0x4c | 0x4d => "cmpgtu",
        0x50 | 0x51 => "cmpeq",
        0x54 | 0x55 => "cmplt",
        0x5c | 0x5d => "cmpltu",
        0x60 | 0x63 => "norm",
        _ => "long",
    }
}

/// What a .L unit 40-bit form's operands are, from the same op-field table that names it:
/// which of src1 and src2 the cross path feeds, whether src1 is a constant, and which
/// operands are register pairs. The rendering and `execute_wide`'s long arm read the roles
/// here, and the arm asserts them against the registers it actually reads and writes.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LongRoles {
    /// ABS, SAT and NORM take no src1: TI writes them `src2, dst`.
    pub unary: bool,
    pub src1_constant: bool,
    /// The unsigned compares take an unsigned five-bit constant.
    pub src1_constant_unsigned: bool,
    /// The cross path feeds src1 rather than src2, which is how the long forms use it: a
    /// 40-bit src2 is always read on the unit's own side.
    pub src1_cross: bool,
    pub src2_pair: bool,
    pub dst_pair: bool,
}

pub fn long_roles(word: u32) -> LongRoles {
    let op = (word >> 5) & 0x7f;
    LongRoles {
        unary: matches!(op, 0x38 | 0x40 | 0x60 | 0x63),
        src1_constant: matches!(
            op,
            0x20 | 0x24 | 0x2c | 0x30 | 0x44 | 0x4c | 0x50 | 0x54 | 0x5c
        ),
        src1_constant_unsigned: matches!(op, 0x4c | 0x5c),
        src1_cross: matches!(
            op,
            0x21 | 0x29 | 0x31 | 0x37 | 0x3f | 0x45 | 0x4d | 0x51 | 0x55 | 0x5d
        ),
        src2_pair: matches!(
            op,
            0x20 | 0x21
                | 0x24
                | 0x29
                | 0x2c
                | 0x30
                | 0x31
                | 0x38
                | 0x40
                | 0x44
                | 0x45
                | 0x4c
                | 0x4d
                | 0x50
                | 0x51
                | 0x54
                | 0x55
                | 0x5c
                | 0x5d
                | 0x60
        ),
        dst_pair: matches!(
            op,
            0x20 | 0x21
                | 0x23
                | 0x24
                | 0x27
                | 0x29
                | 0x2b
                | 0x2c
                | 0x2f
                | 0x30
                | 0x31
                | 0x37
                | 0x38
                | 0x3f
        ),
    }
}

/// The .S unit's 40-bit shift family: a widening left shift of a 32-bit source, and the left,
/// signed-right and unsigned-right shifts of a long.
pub fn long_shift_mnemonic(op: u32) -> &'static str {
    match op {
        0x12 | 0x13 | 0x30 | 0x31 => "shl",
        0x24 | 0x25 => "shru",
        0x34 | 0x35 => "shr",
        _ => "longshift",
    }
}

pub fn mpy_mnemonic(op: u32) -> &'static str {
    match op {
        0x00 => "mpy",
        0x01 => "mpyh",
        0x02 => "smpyh",
        0x03 => "mpyhsu",
        0x04 => "mpyi",
        0x05 => "mpyhus",
        0x06 => "mpyi",
        0x07 => "mpyhu",
        0x08 => "mpyid",
        0x09 => "mpyhl",
        0x0a => "smpyhl",
        0x0b => "mpyhslu",
        0x0c => "mpyid",
        0x0d => "mpyhuls",
        0x0e => "mpydp",
        0x0f => "mpyhlu",
        0x10 => "mpy32",
        0x11 => "mpylh",
        0x12 => "smpylh",
        0x13 => "mpylshu",
        0x14 => "mpy32",
        0x15 => "mpyluhs",
        0x16 => "mpy32su",
        0x17 => "mpylhu",
        0x18 => "mpy",
        0x19 => "mpy",
        0x1a => "smpy",
        0x1b => "mpysu",
        0x1c => "mpysp",
        0x1d => "mpyus",
        0x1e => "mpysu",
        _ => "mpyu",
    }
}

/// M-unit `m_compound` format: op field at bits 6-10.
pub fn mpy_compound_mnemonic(op: u32) -> &'static str {
    match op {
        0x00 => "mpy2",
        0x01 => "smpy2",
        0x02 => "dotpsu4",
        0x04 => "mpyu4",
        0x05 => "mpysu4",
        0x06 => "dotpu4",
        0x07 => "dotpnrsu2",
        0x09 => "dotpn2",
        0x0b => "dotp2",
        0x0c => "dotp2",
        0x0d => "dotprsu2",
        0x0e => "mpylir",
        0x10 => "mpyhir",
        0x11 => "gmpy4",
        0x12 => "avgu4",
        0x13 => "avg2",
        0x14 => "mpyhi",
        0x15 => "mpyli",
        0x16 => "mpyspdp",
        0x17 => "mpysp2dp",
        0x18 => "mpy32u",
        0x19 => "mpy32us",
        0x1a => "sshvr",
        0x1c => "sshvl",
        0x1d | 0x1e => "rotl",
        _ => "mpy2",
    }
}

/// M-unit `m_unary` format: op field at bits 13-17.
pub fn mpy_unary_mnemonic(op: u32) -> &'static str {
    match op {
        0x18 => "xpnd4",
        0x19 => "xpnd2",
        0x1a => "mvd",
        0x1c => "shfl",
        0x1d => "deal",
        0x1e => "bitc4",
        _ => "bitr",
    }
}

/// M-unit `m_1_or_2_src` format: op field at bits 6-10.
pub fn mpy_extended_mnemonic(op: u32) -> &'static str {
    match op {
        0x0a => "cmpy",
        0x0b => "cmpyr",
        0x0c => "cmpyr1",
        0x0f => "mpy2ir",
        0x14 => "ddotpl2r",
        0x15 => "ddotph2r",
        0x16 => "ddotpl2",
        0x17 => "ddotph2",
        0x18 => "ddotp4",
        0x19 => "smpy32",
        0x1b => "xormpy",
        _ => "gmpy",
    }
}

/// What an M-unit family's operands are, from the same op-field tables that name it: whether
/// TI writes src2 before src1, whether src1 is the five-bit constant rather than a register,
/// which operands are register pairs, and whether the src1 field is the operation selector
/// rather than an operand at all. One table decides both the name and the roles, so the
/// rendering `tools/dis6x_diff.py` compares cannot disagree with the mnemonic beside it, and
/// `execute_m_unit` asserts the pair roles against what it schedules.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MpyRoles {
    /// The shift and rotate forms on the M unit are written `src2, src1, dst`; every
    /// multiply is written `src1, src2, dst` (SPRUFE8B SSHVL, ROTL, MPY).
    pub src2_first: bool,
    pub src1_constant: bool,
    /// ROTL's count is an unsigned five-bit constant; the multiplies' constants are signed.
    pub src1_constant_unsigned: bool,
    pub src1_pair: bool,
    pub dst_pair: bool,
    /// The `m_unary` forms, whose src1 field selects the operation: `src2, dst`.
    pub unary: bool,
}

pub fn mpy_roles(opcode: Opcode, word: u32) -> MpyRoles {
    match opcode {
        // `m_1_or_2_src`, op field at bits 7-11: the two constant multiplies take a signed
        // five-bit src1, and the 64-bit MPY32 forms write a register pair. The C67x slots -
        // MPYI, MPYID, MPYDP and MPYSP - are not this device's instructions and never reach
        // here, so they claim no roles.
        Opcode::Mpy => MpyRoles {
            src1_constant: matches!((word >> 7) & 0x1f, 0x18 | 0x1e),
            dst_pair: matches!((word >> 7) & 0x1f, 0x14 | 0x16),
            ..MpyRoles::default()
        },
        // `m_compound`, op field at bits 6-10. SSHVL, SSHVR and ROTL are the shift forms,
        // and ROTL's second encoding takes its count as a five-bit constant.
        Opcode::MpyCompound => MpyRoles {
            src2_first: matches!((word >> 6) & 0x1f, 0x1a | 0x1c | 0x1d | 0x1e),
            src1_constant: (word >> 6) & 0x1f == 0x1e,
            src1_constant_unsigned: (word >> 6) & 0x1f == 0x1e,
            dst_pair: matches!(
                (word >> 6) & 0x1f,
                0x00 | 0x01 | 0x04 | 0x05 | 0x0b | 0x14 | 0x15 | 0x18 | 0x19
            ),
            ..MpyRoles::default()
        },
        // The C64x+ complex-multiply space, op field at bits 6-10: the DDOTP forms read a
        // register pair on src1, and the unrounded ones write one.
        Opcode::MpyExtended => MpyRoles {
            src1_pair: matches!((word >> 6) & 0x1f, 0x14..=0x17),
            dst_pair: matches!((word >> 6) & 0x1f, 0x0a | 0x0f | 0x16 | 0x17 | 0x18),
            ..MpyRoles::default()
        },
        Opcode::MpyUnary => MpyRoles {
            unary: true,
            ..MpyRoles::default()
        },
        _ => MpyRoles::default(),
    }
}

// ---------------------------------------------------------------------------
// Compact (16-bit) instructions
// ---------------------------------------------------------------------------

/// Compact instructions address a restricted register window. When the fetch packet header
/// sets RS (bit 19), the 3-bit register fields select A16-A23 / B16-B23 instead of
/// A0-A7 / B0-B7. Both the executor and the operand rendering read the window through this,
/// so there is one reading of it rather than two.
pub(crate) fn compact_register(field: u32, header: u32) -> usize {
    let base = if header & (1 << 19) != 0 { 16 } else { 0 };
    (base + (field & 7)) as usize
}

/// The five-bit constant the compact shift, bit-set, ADDK, B15 address and MVK families
/// assemble from bits 13-15 and 11-12 (`s_ssh5`, `s_sc5`, `s_sx5`, `d_dx5` and `l_lx5c`).
/// Both the executor and the operand rendering read it here, so there is one reading of it.
pub(crate) fn compact_constant5(word: u16) -> u32 {
    u32::from(((word >> 13) & 7) | (((word >> 11) & 3) << 3))
}

/// `scst_l3i`: the compact three-operand ADD's four-bit signed constant, in which zero
/// encodes eight because adding zero needs no instruction.
pub(crate) fn compact_add_constant(word: u16) -> u32 {
    let field = u32::from(((word >> 13) & 7) | (((word >> 11) & 1) << 3));
    if field == 0 {
        8
    } else {
        ((field << 28) as i32 >> 28) as u32
    }
}

/// The eight-bit MVK constant, assembled from bits 13-15, 11-12, 5-6 and 10.
pub(crate) fn compact_constant8(word: u16) -> u32 {
    u32::from((word >> 13) & 7)
        | (u32::from((word >> 11) & 3) << 3)
        | (u32::from((word >> 5) & 3) << 5)
        | (u32::from((word >> 10) & 1) << 7)
}

/// `d_dx5p`: the word count ADDAW and SUBAW adjust B15 by, from bits 13-15 and 8-9. TI's
/// tools print the count rather than the byte offset the executor scales it into.
pub(crate) fn compact_stack_constant(word: u16) -> u32 {
    u32::from(((word >> 13) & 7) | (((word >> 8) & 3) << 3))
}

/// Size, signedness, data register and alignment for a compact load/store, all of which
/// depend on the fetch packet header's DSZ field plus the `sz` and `na` bits in the word.
pub(crate) fn compact_memory_shape(word: u16, header: u32) -> (u32, bool, usize, bool) {
    let dsz = (header >> 16) & 7;
    let secondary = word & (1 << 9) != 0;
    let (size, signed) = match (dsz, secondary) {
        (0, false) | (1, false) | (2, false) | (3, false) => (4u32, false),
        (0, true) => (1, false),
        (1, true) => (1, true),
        (2, true) => (2, false),
        (3, true) => (2, true),
        (4, false) | (5, false) | (6, false) | (7, false) => (8, false),
        (4, true) | (6, true) => (4, false),
        (5, true) => (1, true),
        (7, true) => (2, true),
        _ => unreachable!(),
    };
    if size == 8 {
        (
            size,
            signed,
            compact_register(((word >> 5) as u32 & 3) << 1, header),
            word & (1 << 4) == 0,
        )
    } else {
        (
            size,
            signed,
            compact_register((word >> 4) as u32, header),
            !(dsz == 6 && secondary),
        )
    }
}

/// The family a compact 16-bit instruction belongs to, and the identity bits that tell its
/// exact form: the op fields, the saturation gate and the data shape. Execution, naming and
/// resource accounting all dispatch on this one reading of the encoding; operand fields
/// (registers, constants, cross-path selectors) are read by the consumers themselves.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CompactOpcode {
    Nop,
    Spmask,
    Spmaskr,
    /// The predicated branch; the constant width is an operand field.
    Bnop,
    Callp,
    /// `m_m3`: the compact M-unit multiplies; the header SAT bit selects the saturating
    /// SMPY forms.
    Mpy {
        op: MpyOp,
        saturating: bool,
    },
    /// The predicated write-zero-or-one form, whose condition field names both the register
    /// and the sense; TI's tools print it as the plain MVK.
    MvkWrite,
    /// `d_dstk`: B15-relative with a scaled unsigned offset, never updating the pointer.
    LdstB15(LdstShape),
    /// `d_pp`: the stack-pointer forms; loads pre-increment B15, stores post-decrement it.
    LdstStack(LdstShape),
    /// `d_dinc`, `d_ddec`, `d_dind` and `d_doff4`: the pointer-register forms.
    LdstPointer(LdstShape),
    /// The three-operand ADD whose constant is `scst_l3i` (zero encodes eight).
    AddConst,
    /// `s_ssh5`: shift by a five-bit constant. Op field 3 is not a shift here.
    ShiftConst5(ShiftKind),
    Spkernel,
    Sploop {
        delayed: bool,
        nested: bool,
    },
    /// `s_s2ext`: EXT / EXTU with the fixed 16/16 and 24/24 field positions.
    ExtExtu {
        extu: bool,
    },
    /// `s_sc5`: single-bit SET and CLR, plus the one-bit EXTU form.
    SetClrExtu(SetClrOp),
    Addk,
    /// `d_dx5`: ADDAW from B15 with a word-scaled unsigned constant.
    AddawB15,
    /// `d_dx5p`: ADDAW / SUBAW adjusting B15 in place.
    AddawSubawB15 {
        subaw: bool,
    },
    /// `s_sx2op` / `d_dx2op`: two-operand ADD / SUB, destination doubling as first source.
    AddSub2op {
        subtract: bool,
    },
    /// `s_s2sh`: shift by a register amount; `Sshl` is the saturating shift left.
    ShiftReg(ShiftKind),
    /// `l_lx3c`: CMPEQ against an unsigned three-bit constant.
    CmpeqConst3,
    /// `s_s3i`: shift with a coded three-bit amount; only valid without the header BR flag.
    ShiftCoded3 {
        left: bool,
    },
    /// `l_lx1c`: compare against a 0/1 constant, writing a predicate register.
    CmplConst1(CmpOp),
    /// The compact register branch.
    BnopReg,
    /// The eight-bit-constant MVK.
    MvkConst8,
    /// The conditional write forms: MVK 0, MVK 1, ADD, XOR, NEG, SUB and MVC by op field.
    MvkCond(MvkForm),
    /// `lsdmvfr` / `lsdmvto`: the register-to-register moves.
    Mv,
    /// `l_l2c`: the L-unit two-operand logical and compare family, one-bit destination.
    Logic2op(LogicOp),
    /// The five-bit-constant MVK.
    MvkConst5,
    /// The three-operand ALU on the L and S units, with the header SAT bit selecting the
    /// saturating SADD / SSUB forms.
    Alu3op(AluOp),
}

/// The size, signedness, alignment and direction a compact load/store carries: the part of
/// the fetch-packet DSZ interpretation that names the instruction. The data register is an
/// operand field, read by the consumers through `compact_memory_shape`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LdstShape {
    pub size: u32,
    pub signed: bool,
    pub aligned: bool,
    pub load: bool,
}

/// The M-unit multiply a compact word encodes, before the header SAT bit decides saturation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MpyOp {
    Mpy,
    MpyH,
    MpyLh,
    MpyHl,
}

/// The shift a compact shift instruction encodes. `Sshl` is the saturating shift left; only
/// the register-amount form can encode it, which is why the constant forms never produce it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ShiftKind {
    Shl,
    Shr,
    Shru,
    Sshl,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AluOp {
    Add,
    Sub,
    Sadd,
    Ssub,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LogicOp {
    And,
    Or,
    Xor,
    Cmpeq,
    Cmplt,
    Cmpgt,
    Cmpltu,
    Cmpgtu,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CmpOp {
    Cmplt,
    Cmpgt,
    Cmpltu,
    Cmpgtu,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SetClrOp {
    Extu,
    Set,
    Clr,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MvkForm {
    Mvk0,
    Mvk1,
    Add,
    Xor,
    Neg,
    Sub,
    Mvc,
}

/// The family a compact 16-bit word encodes, decoded against its fetch-packet header. The
/// header's SAT (bit 14), BR (bit 15) and DSZ (bits 16-18) fields take part in the reading,
/// exactly as the executor and the differential use them. `None` names an encoding the
/// device does not have: the executor faults it, naming renders its raw word, and resource
/// accounting reports an opcode exception - the same fallback the 32-bit side gives
/// `Opcode::Unknown`.
pub fn decode_compact(word: u16, header: u32) -> Option<CompactOpcode> {
    let sat = header & (1 << 14) != 0;
    if word & 0x1fff == 0x0c6e {
        return Some(CompactOpcode::Nop);
    }
    if word & 0x3c7e == 0x3c66 {
        return Some(CompactOpcode::Spmaskr);
    }
    if word & 0x3c7e == 0x2c66 {
        return Some(CompactOpcode::Spmask);
    }
    if header & (1 << 15) != 0 {
        let conditional = word & 0x002e == 0x002a;
        let signed = word & 0x003e == 0x000a || conditional;
        let unsigned = word & 0xc03e == 0xc00a || word & 0xc02e == 0xc02a;
        if signed || unsigned {
            return Some(CompactOpcode::Bnop);
        }
        if word & 0x003e == 0x001a {
            return Some(CompactOpcode::Callp);
        }
    }
    if word & 0x001e == 0x001e {
        let op = match (word >> 5) & 3 {
            0 => MpyOp::Mpy,
            1 => MpyOp::MpyH,
            2 => MpyOp::MpyLh,
            _ => MpyOp::MpyHl,
        };
        return Some(CompactOpcode::Mpy {
            op,
            saturating: sat,
        });
    }
    if word & 0x1c66 == 0x0866 {
        return Some(CompactOpcode::MvkWrite);
    }
    if word & 0x8c06 == 0x8c04 {
        return Some(CompactOpcode::LdstB15(LdstShape {
            size: 4,
            signed: false,
            aligned: true,
            load: word & (1 << 3) != 0,
        }));
    }
    if word & 0x087f == 0x0077 {
        let doubleword = word & (1 << 15) != 0;
        return Some(CompactOpcode::LdstStack(LdstShape {
            size: if doubleword { 8 } else { 4 },
            signed: false,
            aligned: true,
            load: word & (1 << 14) != 0,
        }));
    }
    if word & 0x0006 == 0x0004 {
        let (size, signed, _, aligned) = compact_memory_shape(word, header);
        return Some(CompactOpcode::LdstPointer(LdstShape {
            size,
            signed,
            aligned,
            load: word & (1 << 3) != 0,
        }));
    }
    if word & 0x040e == 0x0400 {
        return Some(CompactOpcode::AddConst);
    }
    if word & 0x041e == 0x0402 && (word >> 5) & 3 != 3 {
        let kind = match (word >> 5) & 3 {
            0 => ShiftKind::Shl,
            1 => ShiftKind::Shr,
            2 if sat => ShiftKind::Sshl,
            2 => ShiftKind::Shru,
            _ => unreachable!("op field 3 is the compact register-count shift"),
        };
        return Some(CompactOpcode::ShiftConst5(kind));
    }
    if word & 0x3c7e == 0x1c66 {
        return Some(CompactOpcode::Spkernel);
    }
    if word & 0xbc7e == 0x8c66 {
        return Some(CompactOpcode::Sploop {
            delayed: word & 1 != 0,
            nested: true,
        });
    }
    if word & 0xbc7e == 0x0c66 {
        return Some(CompactOpcode::Sploop {
            delayed: word & 1 != 0,
            nested: false,
        });
    }
    if word & 0x047e == 0x0062 {
        return Some(CompactOpcode::ExtExtu {
            extu: word & (1 << 12) != 0,
        });
    }
    if word & 0x041e == 0x0002 {
        return match (word >> 5) & 3 {
            0 => Some(CompactOpcode::SetClrExtu(SetClrOp::Extu)),
            1 => Some(CompactOpcode::SetClrExtu(SetClrOp::Set)),
            2 => Some(CompactOpcode::SetClrExtu(SetClrOp::Clr)),
            // Op field 3 never reaches here: the `s_s2ext` arm above claims the encoding.
            _ => None,
        };
    }
    if word & 0x047e == 0x042e {
        return Some(CompactOpcode::Addk);
    }
    if word & 0x047e == 0x0436 {
        return Some(CompactOpcode::AddawB15);
    }
    if word & 0x1c7e == 0x0c76 {
        return Some(CompactOpcode::AddawSubawB15 {
            subaw: word & (1 << 7) != 0,
        });
    }
    if matches!(word & 0x047e, 0x002e | 0x0036) {
        return Some(CompactOpcode::AddSub2op {
            subtract: word & (1 << 11) != 0,
        });
    }
    if word & 0x047e == 0x0462 {
        let kind = match (word >> 11) & 3 {
            0 => ShiftKind::Shl,
            1 => ShiftKind::Shr,
            2 => ShiftKind::Shru,
            _ => ShiftKind::Sshl,
        };
        return Some(CompactOpcode::ShiftReg(kind));
    }
    if word & 0x147e == 0x0026 {
        return Some(CompactOpcode::CmpeqConst3);
    }
    if word & 0x040e == 0x040a && header & (1 << 15) == 0 {
        return Some(CompactOpcode::ShiftCoded3 {
            left: word & (1 << 11) == 0,
        });
    }
    if word & 0x147e == 0x1026 {
        let op = match (word >> 14) & 3 {
            0 => CmpOp::Cmplt,
            1 => CmpOp::Cmpgt,
            2 => CmpOp::Cmpltu,
            _ => CmpOp::Cmpgtu,
        };
        return Some(CompactOpcode::CmplConst1(op));
    }
    if word & 0x187e == 0x006e {
        return Some(CompactOpcode::BnopReg);
    }
    if word & 0x001e == 0x0012 {
        return Some(CompactOpcode::MvkConst8);
    }
    if word & 0x1c66 == 0x1866 {
        let form = match ((word >> 13) & 7, (word >> 3) & 3) {
            (0, _) => Some(MvkForm::Mvk0),
            (1, _) => Some(MvkForm::Mvk1),
            (5, _) => Some(MvkForm::Add),
            (7, _) => Some(MvkForm::Xor),
            (2, 0) | (2, 1) => Some(MvkForm::Neg),
            (3, _) => Some(MvkForm::Sub),
            (6, 1) => Some(MvkForm::Mvc),
            // The remaining op/unit combinations name no instruction: the executor faults
            // them like any other encoding the device does not have.
            _ => None,
        };
        return form.map(CompactOpcode::MvkCond);
    }
    if matches!(word & 0x0066, 0x0006 | 0x0046) {
        return Some(CompactOpcode::Mv);
    }
    if word & 0x040e == 0x0408 {
        let op = match ((word >> 5) & 3) | (((word >> 11) & 1) << 2) {
            0 => LogicOp::And,
            1 => LogicOp::Or,
            2 => LogicOp::Xor,
            3 => LogicOp::Cmpeq,
            4 => LogicOp::Cmplt,
            5 => LogicOp::Cmpgt,
            6 => LogicOp::Cmpltu,
            _ => LogicOp::Cmpgtu,
        };
        return Some(CompactOpcode::Logic2op(op));
    }
    if word & 0x047e == 0x0426 {
        return Some(CompactOpcode::MvkConst5);
    }
    if !matches!(word & 0x040e, 0x0000 | 0x000a) {
        return None;
    }
    let subtract = word & (1 << 11) != 0;
    // SSUB has no .S unit form, so only the .L unit's subtract saturates (SPRU732J).
    let saturating = sat && ((word >> 1) & 7 == 0 || !subtract);
    let op = match (subtract, saturating) {
        (false, false) => AluOp::Add,
        (true, false) => AluOp::Sub,
        (false, true) => AluOp::Sadd,
        (true, true) => AluOp::Ssub,
    };
    Some(CompactOpcode::Alu3op(op))
}

/// The name TI's tools print for an identified compact family. Every compact family has a
/// name; invalid encodings are excluded by `decode_compact` before this projection runs.
impl CompactOpcode {
    pub fn mnemonic(self) -> &'static str {
        match self {
            CompactOpcode::Nop => "nop",
            CompactOpcode::Spmask => "spmask",
            CompactOpcode::Spmaskr => "spmaskr",
            CompactOpcode::Bnop => "bnop",
            CompactOpcode::Callp => "callp",
            CompactOpcode::Mpy { op, saturating } => match (op, saturating) {
                (MpyOp::Mpy, false) => "mpy",
                (MpyOp::MpyH, false) => "mpyh",
                (MpyOp::MpyLh, false) => "mpylh",
                (MpyOp::MpyHl, false) => "mpyhl",
                (MpyOp::Mpy, true) => "smpy",
                (MpyOp::MpyH, true) => "smpyh",
                (MpyOp::MpyLh, true) => "smpylh",
                (MpyOp::MpyHl, true) => "smpyhl",
            },
            CompactOpcode::MvkWrite => "mvk",
            CompactOpcode::LdstB15(shape)
            | CompactOpcode::LdstStack(shape)
            | CompactOpcode::LdstPointer(shape) => ldst_name(shape),
            CompactOpcode::AddConst => "add",
            CompactOpcode::ShiftConst5(kind) | CompactOpcode::ShiftReg(kind) => shift_name(kind),
            CompactOpcode::Spkernel => "spkernel",
            CompactOpcode::Sploop { delayed, nested } => {
                if nested || delayed {
                    "sploopd"
                } else {
                    "sploop"
                }
            }
            CompactOpcode::ExtExtu { extu } => {
                if extu {
                    "extu"
                } else {
                    "ext"
                }
            }
            CompactOpcode::SetClrExtu(op) => match op {
                SetClrOp::Extu => "extu",
                SetClrOp::Set => "set",
                SetClrOp::Clr => "clr",
            },
            CompactOpcode::Addk => "addk",
            CompactOpcode::AddawB15 => "addaw",
            CompactOpcode::AddawSubawB15 { subaw } => {
                if subaw {
                    "subaw"
                } else {
                    "addaw"
                }
            }
            CompactOpcode::AddSub2op { subtract } => {
                if subtract {
                    "sub"
                } else {
                    "add"
                }
            }
            CompactOpcode::CmpeqConst3 => "cmpeq",
            CompactOpcode::ShiftCoded3 { left } => {
                if left {
                    "shl"
                } else {
                    "shr"
                }
            }
            CompactOpcode::CmplConst1(op) => match op {
                CmpOp::Cmplt => "cmplt",
                CmpOp::Cmpgt => "cmpgt",
                CmpOp::Cmpltu => "cmpltu",
                CmpOp::Cmpgtu => "cmpgtu",
            },
            CompactOpcode::BnopReg => "bnop",
            CompactOpcode::MvkConst8 => "mvk",
            CompactOpcode::MvkCond(form) => match form {
                MvkForm::Mvk0 | MvkForm::Mvk1 => "mvk",
                MvkForm::Add => "add",
                MvkForm::Xor => "xor",
                MvkForm::Neg => "neg",
                MvkForm::Sub => "sub",
                MvkForm::Mvc => "mvc",
            },
            CompactOpcode::Mv => "mv",
            CompactOpcode::Logic2op(op) => match op {
                LogicOp::And => "and",
                LogicOp::Or => "or",
                LogicOp::Xor => "xor",
                LogicOp::Cmpeq => "cmpeq",
                LogicOp::Cmplt => "cmplt",
                LogicOp::Cmpgt => "cmpgt",
                LogicOp::Cmpltu => "cmpltu",
                LogicOp::Cmpgtu => "cmpgtu",
            },
            CompactOpcode::MvkConst5 => "mvk",
            CompactOpcode::Alu3op(op) => match op {
                AluOp::Add => "add",
                AluOp::Sub => "sub",
                AluOp::Sadd => "sadd",
                AluOp::Ssub => "ssub",
            },
        }
    }
}

fn ldst_name(shape: LdstShape) -> &'static str {
    match (shape.size, shape.signed, shape.aligned, shape.load) {
        (1, false, _, true) => "ldbu",
        (1, true, _, true) => "ldb",
        (2, false, _, true) => "ldhu",
        (2, true, _, true) => "ldh",
        (4, _, true, true) => "ldw",
        (4, _, false, true) => "ldnw",
        (8, _, true, true) => "lddw",
        (8, _, false, true) => "ldndw",
        (1, _, _, false) => "stb",
        (2, _, _, false) => "sth",
        (4, _, true, false) => "stw",
        (4, _, false, false) => "stnw",
        (8, _, true, false) => "stdw",
        _ => "stndw",
    }
}

fn shift_name(kind: ShiftKind) -> &'static str {
    match kind {
        ShiftKind::Shl => "shl",
        ShiftKind::Shr => "shr",
        ShiftKind::Shru => "shru",
        ShiftKind::Sshl => "sshl",
    }
}
