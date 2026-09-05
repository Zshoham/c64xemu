use std::fs;
use std::path::Path;

const PCAP_MAGIC_LE: u32 = 0xa1b2_c3d4;
const LINKTYPE_ETHERNET: u32 = 1;

pub fn read(path: &Path) -> Result<Vec<Vec<u8>>, String> {
    let bytes = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
    if bytes.len() < 24 || u32_at(&bytes, 0)? != PCAP_MAGIC_LE {
        return Err("unsupported pcap header".into());
    }
    if u32_at(&bytes, 20)? != LINKTYPE_ETHERNET {
        return Err("pcap does not contain Ethernet frames".into());
    }
    let mut frames = Vec::new();
    let mut cursor = 24usize;
    while cursor < bytes.len() {
        if cursor + 16 > bytes.len() {
            return Err("truncated pcap record".into());
        }
        let captured = u32_at(&bytes, cursor + 8)? as usize;
        cursor += 16;
        let end = cursor.checked_add(captured).ok_or("pcap record overflow")?;
        let frame = bytes.get(cursor..end).ok_or("truncated pcap frame")?;
        frames.push(frame.to_vec());
        cursor = end;
    }
    Ok(frames)
}

pub fn write(path: &Path, frames: &[Vec<u8>]) -> Result<(), String> {
    let capacity = 24 + frames.iter().map(|frame| frame.len() + 16).sum::<usize>();
    let mut bytes = Vec::with_capacity(capacity);
    bytes.extend_from_slice(&PCAP_MAGIC_LE.to_le_bytes());
    bytes.extend_from_slice(&2u16.to_le_bytes());
    bytes.extend_from_slice(&4u16.to_le_bytes());
    bytes.extend_from_slice(&0i32.to_le_bytes());
    bytes.extend_from_slice(&0u32.to_le_bytes());
    bytes.extend_from_slice(&65_535u32.to_le_bytes());
    bytes.extend_from_slice(&LINKTYPE_ETHERNET.to_le_bytes());
    for frame in frames {
        let length = u32::try_from(frame.len()).map_err(|_| "Ethernet frame too large")?;
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.extend_from_slice(&length.to_le_bytes());
        bytes.extend_from_slice(&length.to_le_bytes());
        bytes.extend_from_slice(frame);
    }
    fs::write(path, bytes).map_err(|error| format!("{}: {error}", path.display()))
}

fn u32_at(bytes: &[u8], offset: usize) -> Result<u32, String> {
    let raw: [u8; 4] = bytes
        .get(offset..offset + 4)
        .ok_or("truncated pcap")?
        .try_into()
        .unwrap();
    Ok(u32::from_le_bytes(raw))
}
