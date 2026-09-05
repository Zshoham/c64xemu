use crate::cpu::Cpu;
use crate::system::Endian;

const DT_OPEN: u8 = 0xf0;
const DT_CLOSE: u8 = 0xf1;
const DT_READ: u8 = 0xf2;
const DT_WRITE: u8 = 0xf3;
const DT_LSEEK: u8 = 0xf4;
const DT_UNLINK: u8 = 0xf5;
const DT_GETENV: u8 = 0xf6;
const DT_RENAME: u8 = 0xf7;
const DT_GETTIME: u8 = 0xf8;
const DT_GETCLK: u8 = 0xf9;
const DT_GETTIME64: u8 = 0xfa;
const DT_SYNC: u8 = 0xff;
const MAX_CIO_DATA: usize = 1 << 20;

pub struct CioOutput {
    pub fd: i16,
    pub bytes: Vec<u8>,
}

pub fn handle(cpu: &mut Cpu, buffer: u32) -> Result<Option<CioOutput>, String> {
    let header = cpu.host_read_bytes(buffer, 13);
    let length_bytes: [u8; 4] = header[0..4].try_into().unwrap();
    let length = match cpu.endian() {
        Endian::Little => u32::from_le_bytes(length_bytes),
        Endian::Big => u32::from_be_bytes(length_bytes),
    } as usize;
    if length > MAX_CIO_DATA {
        return Err(format!("CIO message length {length} exceeds limit"));
    }
    let command = header[4];
    let parameters: [u8; 8] = header[5..13].try_into().unwrap();
    let data = cpu.host_read_bytes(buffer.wrapping_add(13), length);
    let mut response_parameters = [0u8; 8];
    let mut response_data = Vec::new();
    let mut output = None;

    match command {
        DT_OPEN => {
            let llv_fd = i16_from(&parameters, 0);
            put_i16(&mut response_parameters, 0, llv_fd);
        }
        DT_CLOSE | DT_SYNC => put_i16(&mut response_parameters, 0, 0),
        DT_READ => put_i16(&mut response_parameters, 0, 0),
        DT_WRITE => {
            let fd = i16_from(&parameters, 0);
            let count = u16_from(&parameters, 2) as usize;
            let written = count.min(data.len()).min(i16::MAX as usize);
            put_i16(&mut response_parameters, 0, written as i16);
            output = Some(CioOutput {
                fd,
                bytes: data[..written].to_vec(),
            });
        }
        DT_LSEEK => response_parameters[..4].copy_from_slice(&(-1i32).to_le_bytes()),
        DT_UNLINK | DT_RENAME => put_i16(&mut response_parameters, 0, -1),
        DT_GETENV => response_data.push(0),
        DT_GETTIME | DT_GETCLK => response_parameters[..4].copy_from_slice(&0u32.to_le_bytes()),
        DT_GETTIME64 => response_parameters.copy_from_slice(&0u64.to_le_bytes()),
        other => return Err(format!("unsupported CIO command {other:#04x}")),
    }

    let mut response = Vec::with_capacity(12 + response_data.len());
    let response_length = response_data.len() as u32;
    response.extend_from_slice(&match cpu.endian() {
        Endian::Little => response_length.to_le_bytes(),
        Endian::Big => response_length.to_be_bytes(),
    });
    response.extend_from_slice(&response_parameters);
    response.extend_from_slice(&response_data);
    cpu.host_write_bytes(buffer, &response);
    Ok(output)
}

fn u16_from(bytes: &[u8; 8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn i16_from(bytes: &[u8; 8], offset: usize) -> i16 {
    i16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn put_i16(bytes: &mut [u8; 8], offset: usize, value: i16) {
    bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::{handle, DT_WRITE};
    use crate::cpu::Cpu;

    #[test]
    fn write_returns_count_and_output() {
        let mut cpu = Cpu::new(0);
        let buffer = 0x0080_1000;
        let mut request = Vec::new();
        request.extend_from_slice(&5u32.to_le_bytes());
        request.push(DT_WRITE);
        request.extend_from_slice(&1i16.to_le_bytes());
        request.extend_from_slice(&5u16.to_le_bytes());
        request.extend_from_slice(&[0; 4]);
        request.extend_from_slice(b"hello");
        cpu.host_write_bytes(buffer, &request);

        let output = handle(&mut cpu, buffer).unwrap().unwrap();
        assert_eq!(output.fd, 1);
        assert_eq!(output.bytes, b"hello");
        let response = cpu.host_read_bytes(buffer, 12);
        assert_eq!(&response[0..4], &0u32.to_le_bytes());
        assert_eq!(&response[4..6], &5i16.to_le_bytes());
    }
}
