use crate::system::Endian;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

const ELF_HEADER_SIZE: usize = 52;
const EM_TI_C6000: u16 = 140;
const PT_LOAD: u32 = 1;
const SHT_SYMTAB: u32 = 2;
const SHT_DYNSYM: u32 = 11;

pub struct Segment {
    pub address: u32,
    pub data: Vec<u8>,
}

pub struct Image {
    pub entry: u32,
    pub endian: Endian,
    pub segments: Vec<Segment>,
    pub symbols: BTreeMap<String, u32>,
}

impl Image {
    pub fn load(path: &Path) -> Result<Self, String> {
        let bytes = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
        parse(&bytes)
    }
}

fn parse(bytes: &[u8]) -> Result<Image, String> {
    if bytes.len() < ELF_HEADER_SIZE || &bytes[..4] != b"\x7fELF" {
        return Err("not an ELF file".into());
    }
    if bytes[4] != 1 {
        return Err("only ELF32 images are supported".into());
    }
    let endian = match bytes[5] {
        1 => Endian::Little,
        2 => Endian::Big,
        _ => return Err("unsupported ELF byte order".into()),
    };
    if u16_at(bytes, 18, endian)? != EM_TI_C6000 {
        return Err("ELF image is not for TI C6000".into());
    }
    let entry = u32_at(bytes, 24, endian)?;
    let phoff = u32_at(bytes, 28, endian)? as usize;
    let shoff = u32_at(bytes, 32, endian)? as usize;
    let phentsize = u16_at(bytes, 42, endian)? as usize;
    let phnum = u16_at(bytes, 44, endian)? as usize;
    let shentsize = u16_at(bytes, 46, endian)? as usize;
    let shnum = u16_at(bytes, 48, endian)? as usize;

    if phnum != 0 && phentsize < 32 {
        return Err("invalid ELF program-header size".into());
    }
    let mut segments = Vec::new();
    for index in 0..phnum {
        let offset = checked_table_offset(phoff, phentsize, index, bytes.len())?;
        if u32_at(bytes, offset, endian)? != PT_LOAD {
            continue;
        }
        let file_offset = u32_at(bytes, offset + 4, endian)? as usize;
        let virtual_address = u32_at(bytes, offset + 8, endian)?;
        let physical_address = u32_at(bytes, offset + 12, endian)?;
        let file_size = u32_at(bytes, offset + 16, endian)? as usize;
        let memory_size = u32_at(bytes, offset + 20, endian)? as usize;
        if file_size > memory_size {
            return Err("ELF segment file size exceeds memory size".into());
        }
        let source = bytes
            .get(
                file_offset
                    ..file_offset
                        .checked_add(file_size)
                        .ok_or("ELF segment overflow")?,
            )
            .ok_or("ELF segment lies outside file")?;
        let mut data = vec![0; memory_size];
        data[..file_size].copy_from_slice(source);
        segments.push(Segment {
            address: if physical_address == 0 {
                virtual_address
            } else {
                physical_address
            },
            data,
        });
    }

    let symbols = parse_symbols(bytes, shoff, shentsize, shnum, endian)?;
    Ok(Image {
        endian,
        entry,
        segments,
        symbols,
    })
}

fn parse_symbols(
    bytes: &[u8],
    shoff: usize,
    shentsize: usize,
    shnum: usize,
    endian: Endian,
) -> Result<BTreeMap<String, u32>, String> {
    let mut symbols = BTreeMap::new();
    if shnum == 0 {
        return Ok(symbols);
    }
    if shentsize < 40 {
        return Err("invalid ELF section-header size".into());
    }
    for index in 0..shnum {
        let section = checked_table_offset(shoff, shentsize, index, bytes.len())?;
        let kind = u32_at(bytes, section + 4, endian)?;
        if kind != SHT_SYMTAB && kind != SHT_DYNSYM {
            continue;
        }
        let symbol_offset = u32_at(bytes, section + 16, endian)? as usize;
        let symbol_size = u32_at(bytes, section + 20, endian)? as usize;
        let string_index = u32_at(bytes, section + 24, endian)? as usize;
        let entry_size = u32_at(bytes, section + 36, endian)? as usize;
        if entry_size < 16 || string_index >= shnum {
            return Err("invalid ELF symbol table".into());
        }
        let string_section = checked_table_offset(shoff, shentsize, string_index, bytes.len())?;
        let strings_offset = u32_at(bytes, string_section + 16, endian)? as usize;
        let strings_size = u32_at(bytes, string_section + 20, endian)? as usize;
        let strings = bytes
            .get(
                strings_offset
                    ..strings_offset
                        .checked_add(strings_size)
                        .ok_or("string table overflow")?,
            )
            .ok_or("ELF string table lies outside file")?;
        let end = symbol_offset
            .checked_add(symbol_size)
            .ok_or("symbol table overflow")?;
        if end > bytes.len() {
            return Err("ELF symbol table lies outside file".into());
        }
        let mut cursor = symbol_offset;
        while cursor + 16 <= end {
            let name_offset = u32_at(bytes, cursor, endian)? as usize;
            let value = u32_at(bytes, cursor + 4, endian)?;
            if name_offset != 0 {
                if let Some(name) = c_string(strings, name_offset) {
                    symbols.insert(name.to_owned(), value);
                }
            }
            cursor += entry_size;
        }
    }
    Ok(symbols)
}

fn checked_table_offset(
    base: usize,
    stride: usize,
    index: usize,
    length: usize,
) -> Result<usize, String> {
    let offset = base
        .checked_add(stride.checked_mul(index).ok_or("ELF table overflow")?)
        .ok_or("ELF table overflow")?;
    if offset.checked_add(stride).map_or(true, |end| end > length) {
        return Err("ELF table lies outside file".into());
    }
    Ok(offset)
}

fn c_string(strings: &[u8], offset: usize) -> Option<&str> {
    let tail = strings.get(offset..)?;
    let end = tail.iter().position(|byte| *byte == 0)?;
    std::str::from_utf8(&tail[..end]).ok()
}

fn u16_at(bytes: &[u8], offset: usize, endian: Endian) -> Result<u16, String> {
    let raw: [u8; 2] = bytes
        .get(offset..offset + 2)
        .ok_or("truncated ELF")?
        .try_into()
        .unwrap();
    Ok(match endian {
        Endian::Little => u16::from_le_bytes(raw),
        Endian::Big => u16::from_be_bytes(raw),
    })
}

fn u32_at(bytes: &[u8], offset: usize, endian: Endian) -> Result<u32, String> {
    let raw: [u8; 4] = bytes
        .get(offset..offset + 4)
        .ok_or("truncated ELF")?
        .try_into()
        .unwrap();
    Ok(match endian {
        Endian::Little => u32::from_le_bytes(raw),
        Endian::Big => u32::from_be_bytes(raw),
    })
}
