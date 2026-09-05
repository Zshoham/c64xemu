//! The ROM bootloader's I2C EEPROM boot mode: the boot a C6455 performs with BOOTMODE[3:0] set
//! to 0101b, which SPRUEC6G Table 3 names "Master I2C boot" and section 2.3.4 describes.
//!
//! Like the Ethernet boot in `romboot.rs`, the ROM's *instructions* are not modelled - TI ships
//! them in on-chip ROM this project has no copy of - and what is modelled is everything the ROM
//! does that software or a device on the bus can observe. Here that is the whole point of the
//! item: the read sequence goes through the modelled I2C master's own registers, so ICSAR,
//! ICCNT, ICMDR, ICDXR and ICDRR are driven exactly as firmware drives them and the EEPROM on
//! the bus answers exactly as a board's would.
//!
//! What section 2.3.4 requires of the EEPROM, and what `i2c.rs` therefore models:
//!
//! * "The memory device uses two bytes for internal addressing."
//! * "The memory device has the capability to auto-increment its internal address counter such
//!   that the contents of the memory device can be read sequentially."
//! * "The bootloader requires the I2C EEPROM slave address to be 0x50."
//!
//! Everything on the EEPROM is blocked (section 2.3.4.1): "Each block has a maximum length of
//! 128 bytes, including the 4 byte block header", whose Table 5 is a 16-bit block size covering
//! the header and a 16-bit ones-complement checksum "including the block size and checksum
//! fields. Valid checksum values are 0 and -0", with "A value of 0 in the checksum field
//! disables the checksum check."
//!
//! The boot itself is two stages (section 2.3.4.2). First a block of boot parameters: "The boot
//! parameters begin at EEPROM address 0, and a boot parameter block consists of 128 bytes. The
//! DSP calculates the address of boot parameter block to load based on the value of the
//! CFGGP[2:0] bits of the Device Status Register (DEVSTAT) as follows: address = 0x80 *
//! CFGGP[2:0]." Its options field says what to do next: "A value of 0 indicates another boot
//! parameter table. A value of 1 indicates a boot table ... A value of 2 indicates a boot
//! configuration table". Then, for a boot table, "the bootloader reads from the I2C EEPROM
//! address specified in the boot table until the end of the table is reached (i.e., until all
//! code and data sections are loaded), and immediately begin execution of the loaded code by
//! branching to the entry point specified at the beginning of the boot table".
//!
//! Three places where the manual stops short and this model has to choose. Each is a choice
//! about the shape of a transaction, not about an answer:
//!
//! * Section 2.3.4 never says "repeated START", and gives no wire address bytes. Writing an
//!   internal address and then reading from it is one transaction on an I2C bus, and SPRU974A
//!   section 2.5 gives its only form - "When BB is set to 1 and STT is set to 1, a repeated
//!   START condition is generated" - so that is what the boot does: a master-transmit of the two
//!   address bytes with STP clear, leaving the bus busy, then a master-receive with STT set
//!   again and STP set, which SPRU974A Table 15 makes "START condition, slave address, n data
//!   words, STOP condition".
//! * The manual gives a block's maximum length and its size field but not how many bytes a read
//!   transaction asks for. This model reads the maximum, 128 bytes, and takes the block's real
//!   length from the size field in what came back - so one transaction serves one block, and the
//!   next block's address is the current one plus that size.
//! * Section 2.3.4 says the bus starts at "CLKIN1/6600" and is reprogrammed from the parameter
//!   table's CPU and I2C frequency fields, but names no prescaler or divider values. Nothing in
//!   this model is clocked from the I2C bus, so the boot programs the dividers it is given and
//!   the transfer takes the cycles `i2c.rs` already charges it.

use crate::boot::BootStream;
use crate::system::{CoreBus, Endian, Fabric};

/// BOOTMODE[3:0] 0101b is the C6455's master I2C boot (SPRUEC6G Table 3).
pub(crate) const BOOTMODE_I2C_EEPROM: u8 = 5;

/// The slave address section 2.3.4 requires the first EEPROM to answer at.
pub(crate) const EEPROM_ADDRESS: u32 = 0x50;

/// I2C register addresses (SPRU974A Table 2).
const BASE: u32 = 0x02b0_4000;
const ICCLKL: u32 = BASE + 0x0c;
const ICCLKH: u32 = BASE + 0x10;
const ICCNT: u32 = BASE + 0x14;
const ICDRR: u32 = BASE + 0x18;
const ICSAR: u32 = BASE + 0x1c;
const ICDXR: u32 = BASE + 0x20;
const ICMDR: u32 = BASE + 0x24;
const ICSTR: u32 = BASE + 0x08;
const ICPSC: u32 = BASE + 0x30;

/// ICMDR bits (SPRU974A Table 14).
const MDR_STT: u32 = 1 << 13;
const MDR_STP: u32 = 1 << 11;
const MDR_MST: u32 = 1 << 10;
const MDR_TRX: u32 = 1 << 9;
const MDR_IRS: u32 = 1 << 5;

/// ICSTR flags (SPRU974A Table 7).
const ICRRDY: u32 = 1 << 3;
const ICXRDY: u32 = 1 << 4;

/// One block, and the header inside it (SPRUEC6G section 2.3.4.1 and Table 5).
const BLOCK_BYTES: usize = 128;
const BLOCK_HEADER_BYTES: usize = 4;

/// Boot parameter fields, as offsets into the block (SPRUEC6G Table 6). The two words before
/// them are the block header the format shares with every other block.
const PARAM_OPTIONS: usize = 6;
const PARAM_DEVICE_ADDRESS_LOW: usize = 8;
const PARAM_DEVICE_ADDRESS_HIGH: usize = 10;
const PARAM_CPU_FREQUENCY: usize = 16;
const PARAM_I2C_FREQUENCY: usize = 18;

/// The options field's two least significant bits (SPRUEC6G section 2.3.4.2).
const OPTION_TABLE_TYPE: u16 = 0x3;
const TABLE_TYPE_PARAMETERS: u16 = 0;
const TABLE_TYPE_BOOT_TABLE: u16 = 1;

/// Where core 0 starts when the boot table names no entry point (SPRUEC6G section 2.3.4.2 for
/// the table's own entry point; this is the fall-back the Ethernet boot uses too).
const DEFAULT_ENTRY: u32 = 0x0080_0000;

/// How many boot parameter blocks the boot will chase before giving up. The format lets one
/// parameter table name another, so a malformed EEPROM could point at itself.
const MAX_PARAMETER_BLOCKS: usize = 8;

/// What the boot has done so far, for a front end to report.
#[derive(Clone, Copy, Default)]
pub(crate) struct Progress {
    /// Boot parameter blocks read and accepted.
    pub parameter_blocks: usize,
    /// Blocks of the boot table read.
    pub table_blocks: usize,
    /// Bytes of boot-table payload handed to the table processor.
    pub table_bytes: usize,
    /// Blocks whose checksum did not verify, and which were therefore re-read.
    pub checksum_failures: usize,
    pub entry: Option<u32>,
    /// Why the boot stopped, when it stopped without an entry point.
    pub error: Option<&'static str>,
}

/// Which stage of the read the boot is in. Every stage is one I2C transaction, and a transaction
/// is a phase of its own because the bus takes cycles the module charges.
enum Phase {
    /// Program the master, then start the first transaction.
    Configure,
    /// Writing the two internal address bytes, with `sent` of them gone.
    Address {
        sent: usize,
    },
    /// Reading a block, with what has arrived so far.
    Read {
        block: Vec<u8>,
    },
    /// The block is in hand: check it and decide what the next transaction is.
    Consume {
        block: Vec<u8>,
    },
    Done,
}

/// What the current block is being read for.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Target {
    Parameters,
    BootTable,
}

pub(crate) struct I2cBoot {
    /// The EEPROM's slave address, and the address inside it the next block starts at.
    slave: u32,
    at: u32,
    target: Target,
    phase: Phase,
    stream: BootStream,
    parameter_blocks: usize,
    progress: Progress,
}

impl I2cBoot {
    /// The boot as the pins strap it: the first EEPROM at 0x50, and the boot parameter block at
    /// `0x80 * index`, where the index comes from the configuration pins (SPRUEC6G section
    /// 2.3.4.2 for the C6455, Table 22 for the C6472's CFG[3:0]).
    pub(crate) fn new(index: u32, endian: Endian) -> Self {
        Self {
            slave: EEPROM_ADDRESS,
            at: index * BLOCK_BYTES as u32,
            target: Target::Parameters,
            phase: Phase::Configure,
            stream: BootStream::new(endian),
            parameter_blocks: 0,
            progress: Progress::default(),
        }
    }

    pub(crate) fn progress(&self) -> Progress {
        self.progress
    }

    /// One cycle of the ROM's boot loop. Returns the entry point once the boot table has ended,
    /// which is the device's signal to start core 0 there.
    pub(crate) fn tick(&mut self, fabric: &mut Fabric) -> Option<u32> {
        // The boot code runs on core 0, so its accesses are core 0's accesses.
        let mut bus = CoreBus::new(0, fabric);
        match std::mem::replace(&mut self.phase, Phase::Done) {
            Phase::Configure => {
                self.configure(&mut bus);
                self.begin_address(&mut bus);
                None
            }
            Phase::Address { sent } => {
                self.advance_address(&mut bus, sent);
                None
            }
            Phase::Read { block } => {
                self.advance_read(&mut bus, block);
                None
            }
            Phase::Consume { block } => self.consume(&mut bus, block),
            Phase::Done => None,
        }
    }

    /// Take the module out of reset and give it a bus clock. The dividers are the ones the boot
    /// parameters will replace; nothing this model counts is clocked from them.
    fn configure(&mut self, bus: &mut CoreBus<'_>) {
        bus.write32(ICMDR, 0);
        bus.write32(ICPSC, 8);
        bus.write32(ICCLKL, 10);
        bus.write32(ICCLKH, 10);
        bus.write32(ICMDR, MDR_IRS);
        bus.write32(ICSAR, self.slave);
    }

    /// Start the addressing phase: a master transmit of the two internal address bytes with STP
    /// clear, so the bus stays busy for the repeated START that follows.
    fn begin_address(&mut self, bus: &mut CoreBus<'_>) {
        bus.write32(ICSAR, self.slave);
        bus.write32(ICCNT, 2);
        bus.write32(ICMDR, MDR_IRS | MDR_MST | MDR_TRX | MDR_STT);
        bus.write32(ICDXR, (self.at >> 8) & 0xff);
        self.phase = Phase::Address { sent: 1 };
    }

    fn advance_address(&mut self, bus: &mut CoreBus<'_>, sent: usize) {
        if bus.read32(ICSTR) & ICXRDY == 0 {
            self.phase = Phase::Address { sent };
            return;
        }
        if sent < EEPROM_ADDRESS_BYTES {
            bus.write32(ICDXR, self.at & 0xff);
            self.phase = Phase::Address { sent: sent + 1 };
            return;
        }
        // Both address bytes have gone. The repeated START turns the master round without
        // releasing the bus, and STP ends the transaction after the count.
        bus.write32(ICCNT, BLOCK_BYTES as u32);
        bus.write32(ICMDR, MDR_IRS | MDR_MST | MDR_STT | MDR_STP);
        self.phase = Phase::Read {
            block: Vec::with_capacity(BLOCK_BYTES),
        };
    }

    fn advance_read(&mut self, bus: &mut CoreBus<'_>, mut block: Vec<u8>) {
        if bus.read32(ICSTR) & ICRRDY == 0 {
            self.phase = Phase::Read { block };
            return;
        }
        block.push(bus.read32(ICDRR) as u8);
        self.phase = if block.len() < BLOCK_BYTES {
            Phase::Read { block }
        } else {
            Phase::Consume { block }
        };
    }

    /// A block is in hand. Verify it, then either take its parameters or feed its data to the
    /// boot table processor and go round again.
    fn consume(&mut self, bus: &mut CoreBus<'_>, block: Vec<u8>) -> Option<u32> {
        let size = usize::from(be16(&block, 0));
        if !(BLOCK_HEADER_BYTES..=BLOCK_BYTES).contains(&size) {
            return self.fail("boot block size is outside the 4 to 128 byte range");
        }
        if !checksum_valid(&block[..size]) {
            // "If the checksum shows a failure, the block is re-read until the checksum is
            // valid" (SPRUEC6G section 2.3.4.1). Nothing on this bus can change its answer, so
            // re-reading it forever would hang the device; one retry proves the path and the
            // second failure is reported.
            self.progress.checksum_failures += 1;
            if self.progress.checksum_failures > 1 {
                return self.fail("boot block checksum did not verify on re-read");
            }
            self.begin_address(bus);
            return None;
        }
        self.progress.checksum_failures = 0;
        match self.target {
            Target::Parameters => {
                self.take_parameters(bus, &block, size);
                None
            }
            Target::BootTable => {
                self.progress.table_blocks += 1;
                let data = &block[BLOCK_HEADER_BYTES..size];
                self.progress.table_bytes += data.len();
                match self.stream.feed(data, bus) {
                    Err(_) => self.fail("boot table has no termination record"),
                    Ok(false) => {
                        self.at = self.at.wrapping_add(size as u32);
                        self.begin_address(bus);
                        None
                    }
                    Ok(true) => {
                        let entry = self.stream.entry().unwrap_or(0);
                        let entry = if entry == 0 { DEFAULT_ENTRY } else { entry };
                        // The ROM leaves the module in reset on the way out, as it found it.
                        bus.write32(ICMDR, 0);
                        self.progress.entry = Some(entry);
                        self.phase = Phase::Done;
                        Some(entry)
                    }
                }
            }
        }
    }

    /// Read the parameter block and act on its options field: another parameter table, or the
    /// boot table this boot exists to load.
    fn take_parameters(&mut self, bus: &mut CoreBus<'_>, block: &[u8], size: usize) {
        self.progress.parameter_blocks += 1;
        self.parameter_blocks += 1;
        if size < PARAM_I2C_FREQUENCY + 2 {
            self.fail("boot parameter block is shorter than the parameter structure");
            return;
        }
        // "This parameter configures the I2C clock" (Table 6): the two frequencies are what the
        // ROM reprograms the bus dividers from, so the registers end up holding what the EEPROM
        // asked for. Only the dividers: the manual names no prescaler value, and the prescaler
        // is the one field of the three this model's transfer time depends on, so reprogramming
        // it from a number the manual does not give would change how long the boot takes for a
        // reason no document backs.
        let cpu_mhz = u32::from(be16(block, PARAM_CPU_FREQUENCY));
        let bus_khz = u32::from(be16(block, PARAM_I2C_FREQUENCY));
        if cpu_mhz != 0 && bus_khz != 0 {
            let half = (cpu_mhz * 1000 / bus_khz / 2).clamp(1, 0xffff);
            bus.write32(ICCLKL, half);
            bus.write32(ICCLKH, half);
        }
        let options = be16(block, PARAM_OPTIONS) & OPTION_TABLE_TYPE;
        let low = u32::from(be16(block, PARAM_DEVICE_ADDRESS_LOW));
        let high = u32::from(be16(block, PARAM_DEVICE_ADDRESS_HIGH));
        // "This is an abstraction; this value is used as the I2C EEPROM slave address"
        // (Table 6). A zero there means the EEPROM the boot is already talking to.
        if high != 0 {
            self.slave = high & 0x3ff;
        }
        self.at = low;
        match options {
            TABLE_TYPE_BOOT_TABLE => self.target = Target::BootTable,
            TABLE_TYPE_PARAMETERS => {
                if self.parameter_blocks >= MAX_PARAMETER_BLOCKS {
                    self.fail("boot parameter tables chain without naming a boot table");
                    return;
                }
                self.target = Target::Parameters;
            }
            _ => {
                // A boot configuration table, or the reserved encoding. The table layer for a
                // configuration table exists in `boot.rs`, but it is a memory-resident format
                // rather than a streamed one, so this boot does not carry it.
                self.fail("boot parameter options name a table type this boot does not load");
                return;
            }
        }
        self.begin_address(bus);
    }

    fn fail(&mut self, reason: &'static str) -> Option<u32> {
        self.progress.error = Some(reason);
        self.phase = Phase::Done;
        None
    }
}

/// How many bytes of internal address the EEPROM consumes (SPRUEC6G section 2.3.4).
const EEPROM_ADDRESS_BYTES: usize = 2;

fn be16(bytes: &[u8], offset: usize) -> u16 {
    match (bytes.get(offset), bytes.get(offset + 1)) {
        (Some(high), Some(low)) => (u16::from(*high) << 8) | u16::from(*low),
        _ => 0,
    }
}

/// The block's ones-complement checksum "including the block size and checksum fields", where
/// "Valid checksum values are 0 and -0", and a zero in the field disables the check
/// (SPRUEC6G Table 5 and section 2.3.4.1).
fn checksum_valid(block: &[u8]) -> bool {
    if be16(block, 2) == 0 {
        return true;
    }
    let mut sum: u32 = 0;
    for word in block.chunks(2) {
        let value = if word.len() == 2 {
            (u32::from(word[0]) << 8) | u32::from(word[1])
        } else {
            u32::from(word[0]) << 8
        };
        sum += value;
        while sum > 0xffff {
            sum = (sum & 0xffff) + (sum >> 16);
        }
    }
    sum == 0 || sum == 0xffff
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::{Device, C6455};

    /// One block, laid out as SPRUEC6G Table 5 says: a 16-bit size that counts the header, a
    /// 16-bit checksum over the whole block, then the payload.
    fn block(payload: &[u8]) -> Vec<u8> {
        let size = (BLOCK_HEADER_BYTES + payload.len()) as u16;
        let mut body = Vec::new();
        body.extend_from_slice(&size.to_be_bytes());
        body.extend_from_slice(&0u16.to_be_bytes());
        body.extend_from_slice(payload);
        // The value that makes the ones-complement sum come out to -0.
        let mut sum: u32 = 0;
        for word in body.chunks(2) {
            sum += (u32::from(word[0]) << 8) | u32::from(*word.get(1).unwrap_or(&0));
            while sum > 0xffff {
                sum = (sum & 0xffff) + (sum >> 16);
            }
        }
        body[2..4].copy_from_slice(&(!sum as u16).to_be_bytes());
        body
    }

    /// The Table 6 boot parameter block: 2-byte big-endian fields at their offsets, padded out
    /// to the 128 bytes section 2.3.4.2 says the block consists of.
    fn parameters(options: u16, device_low: u16, device_high: u16) -> Vec<u8> {
        let mut payload = vec![0u8; BLOCK_BYTES - BLOCK_HEADER_BYTES];
        for (offset, value) in [
            (4usize, BOOTMODE_I2C_EEPROM as u16),
            (PARAM_OPTIONS, options),
            (PARAM_DEVICE_ADDRESS_LOW, device_low),
            (PARAM_DEVICE_ADDRESS_HIGH, device_high),
            (PARAM_CPU_FREQUENCY, 720),
            (PARAM_I2C_FREQUENCY, 100),
        ] {
            let at = offset - BLOCK_HEADER_BYTES;
            payload[at..at + 2].copy_from_slice(&value.to_be_bytes());
        }
        let block = block(&payload);
        assert_eq!(block.len(), BLOCK_BYTES);
        block
    }

    /// A boot table, then an EEPROM holding the parameter block and that table behind it.
    fn eeprom(table: &[u8]) -> Vec<u8> {
        let mut image = parameters(
            TABLE_TYPE_BOOT_TABLE,
            BLOCK_BYTES as u16,
            EEPROM_ADDRESS as u16,
        );
        for chunk in table.chunks(BLOCK_BYTES - BLOCK_HEADER_BYTES) {
            image.extend_from_slice(&block(chunk));
        }
        image
    }

    fn run(device: &mut Device, cycles: usize) {
        for _ in 0..cycles {
            device.step_cycle();
            if device
                .i2c_boot()
                .is_some_and(|boot| boot.entry.is_some() || boot.error.is_some())
            {
                return;
            }
        }
    }

    /// The whole boot, on the device and through the I2C master's own registers: the parameter
    /// block is read from address 0, the table from where that block points, and its sections
    /// land in memory with core 0 released at the entry point.
    #[test]
    fn an_i2c_boot_reads_its_table_off_the_bus_and_starts_the_core() {
        const ENTRY: u32 = 0x0080_0400;
        const DESTINATION: u32 = 0x0080_1000;
        // A table long enough to need two blocks, so a section's data straddles a block
        // boundary and the read has to come back for the rest.
        let payload: Vec<u8> = (0..200u32).map(|index| (index * 7 + 1) as u8).collect();
        let mut table: Vec<u8> = Vec::new();
        table.extend_from_slice(&ENTRY.to_le_bytes());
        table.extend_from_slice(&(payload.len() as u32).to_le_bytes());
        table.extend_from_slice(&DESTINATION.to_le_bytes());
        table.extend_from_slice(&payload);
        table.extend_from_slice(&0u32.to_le_bytes());

        let mut device = Device::with_boot_mode(&C6455, 0, Endian::Little, BOOTMODE_I2C_EEPROM);
        device.begin_i2c_boot(0, eeprom(&table));
        assert!(
            device.held_in_reset(0),
            "core 0 runs the ROM, so it is held"
        );
        run(&mut device, 200_000);

        let boot = device.i2c_boot().expect("the device is booting over I2C");
        assert_eq!(boot.error, None);
        assert_eq!(boot.entry, Some(ENTRY));
        assert_eq!(boot.parameter_blocks, 1);
        assert_eq!(boot.checksum_failures, 0);
        // Every byte of the table came off the bus, in blocks of at most 124 payload bytes.
        assert_eq!(
            boot.table_blocks,
            table.len().div_ceil(BLOCK_BYTES - BLOCK_HEADER_BYTES)
        );
        assert!(boot.table_bytes >= table.len());
        // The section landed where the table said, and core 0 is at the entry point with
        // nothing executed yet.
        assert_eq!(
            device.fabric().read_bytes(0, DESTINATION, payload.len()),
            payload
        );
        assert!(!device.held_in_reset(0));
        assert_eq!(device.core(0).pc(), ENTRY);
        assert_eq!(device.core(0).cycles(), 0);
    }

    /// "If the checksum shows a failure, the block is re-read until the checksum is valid"
    /// (SPRUEC6G section 2.3.4.1). Nothing on this bus can answer differently the second time,
    /// so the re-read happens once and then the boot reports it rather than spinning forever.
    #[test]
    fn a_block_whose_checksum_fails_is_reread_and_then_reported() {
        let mut image = parameters(
            TABLE_TYPE_BOOT_TABLE,
            BLOCK_BYTES as u16,
            EEPROM_ADDRESS as u16,
        );
        // Corrupt one payload byte, leaving the checksum field as it was.
        image[8] ^= 0xff;
        let mut device = Device::with_boot_mode(&C6455, 0, Endian::Little, BOOTMODE_I2C_EEPROM);
        device.begin_i2c_boot(0, image);
        run(&mut device, 200_000);

        let boot = device.i2c_boot().expect("the device is booting over I2C");
        assert_eq!(boot.entry, None);
        assert_eq!(boot.parameter_blocks, 0, "a bad block is never acted on");
        assert_eq!(boot.checksum_failures, 2, "read once, re-read once");
        assert_eq!(
            boot.error,
            Some("boot block checksum did not verify on re-read")
        );
        assert!(
            device.held_in_reset(0),
            "core 0 stays held: there is no image"
        );
    }

    /// A zero in the checksum field "disables the checksum check" (SPRUEC6G Table 5), which is
    /// what lets a board write a block without computing one.
    #[test]
    fn a_zero_checksum_field_disables_the_check() {
        let mut image = parameters(
            TABLE_TYPE_BOOT_TABLE,
            BLOCK_BYTES as u16,
            EEPROM_ADDRESS as u16,
        );
        image[2..4].fill(0);
        // A table that is nothing but its entry point and the terminator.
        let mut table = Vec::new();
        table.extend_from_slice(&0x0080_0800u32.to_le_bytes());
        table.extend_from_slice(&0u32.to_le_bytes());
        image.extend_from_slice(&block(&table));
        let mut device = Device::with_boot_mode(&C6455, 0, Endian::Little, BOOTMODE_I2C_EEPROM);
        device.begin_i2c_boot(0, image);
        run(&mut device, 200_000);

        let boot = device.i2c_boot().expect("the device is booting over I2C");
        assert_eq!(boot.error, None);
        assert_eq!(boot.entry, Some(0x0080_0800));
    }

    /// The parameter block the configuration pins select: "address = 0x80 * CFGGP[2:0]"
    /// (SPRUEC6G section 2.3.4.2). Index 1 reads the second block, not the first.
    #[test]
    fn the_configuration_pins_select_which_parameter_block_is_read() {
        // Block 0 names a table type this boot refuses; block 1 names a real boot table. A boot
        // that read block 0 would fail, so reaching the entry point is what proves the index.
        let mut image = parameters(2, 0, 0);
        image.extend_from_slice(&parameters(
            TABLE_TYPE_BOOT_TABLE,
            2 * BLOCK_BYTES as u16,
            EEPROM_ADDRESS as u16,
        ));
        let mut table = Vec::new();
        table.extend_from_slice(&0x0080_0c00u32.to_le_bytes());
        table.extend_from_slice(&0u32.to_le_bytes());
        image.extend_from_slice(&block(&table));
        let mut device = Device::with_boot_mode(&C6455, 0, Endian::Little, BOOTMODE_I2C_EEPROM);
        device.begin_i2c_boot(1, image);
        run(&mut device, 200_000);

        let boot = device.i2c_boot().expect("the device is booting over I2C");
        assert_eq!(boot.error, None);
        assert_eq!(boot.entry, Some(0x0080_0c00));
    }
}
