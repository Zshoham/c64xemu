//! The ROM bootloader's I2C EEPROM boot mode: the boot a C6455 performs with BOOTMODE[3:0] set
//! to 0101b, which SPRUEC6G Table 3 names "Master I2C boot" and section 2.3.4 describes, and the
//! C6472's with BOOTMODE 2 or 3, which section 4.3.3 says "is similar to the C6454/55 device's
//! process" with the differences that section lists.
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
//! * "The bootloader requires the I2C EEPROM slave address to be 0x50." The C6472 "supports both
//!   I2C address 0x50 and 0x51": BOOTMODE 2 is the first and 3 the second (Table 22).
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
//! branching to the entry point specified at the beginning of the boot table". For a boot
//! configuration table it applies the table, then "The next device address previously read from
//! the boot parameter block is copied into device address, the boot options are cleared, and the
//! bootloader performs a boot re-entry", which reads another parameter block from there.
//!
//! The C6472's differences (section 4.3.3 and Tables 22 to 25):
//!
//! * CFG[3:0] is the parameter block index and CFG[4] selects PLL1 "x9(0), x19(1)"; the boot
//!   programs that multiplier and Table 21's caches before it reads anything. Both Table 22 and
//!   the text say 9 and 19 for I2C where the MAC and SRIO rows say 10 and 20, and the model takes
//!   them as written.
//! * A parameter block starts with Table 23's ten-byte common header: size, checksum, an
//!   extended boot mode (Table 24), a port and a software PLL factor; the I2C fields of Table 25
//!   follow from offset 10. "The extended boot modes applies to all boot modes", so the block's
//!   boot mode says which boot its fields configure: 0x101 is this I2C master boot and acts on
//!   its options as above, and 0x104 hands the device to the UTOPIA boot with the block's Table
//!   26 fields (`utopiaboot.rs`). No other boot is modelled as a successor, so other values stop
//!   the boot. **[INFERENCE]** The software PLL factor is not applied: the manual gives it no
//!   effect for the C6472, and PLL1 stays as CFG[4] set it.
//! * Boot-table and configuration-table blocks keep Table 5's four-byte header and up to 124
//!   bytes of data. Table 23's header belongs to the parameter tables: TI's C6472 ROM header
//!   (`tiboot_c6472.h` in the IBL, git.ti.com/keystone-rtos/ibl) declares it as the first five
//!   fields of each BOOT_PARAMS_*_T, and the IBL's C6472 EVM EEPROM, which this ROM boots, wraps
//!   its boot table with `b2i2c` - "2 bytes of length, 2 bytes checksum, 124 bytes of data" -
//!   behind `romparse` parameter tables naming boot mode 257 (0x101). `b2i2c`'s output for a
//!   compiled boot table is byte-for-byte what the tests' block builders produce.
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

use crate::boot::{BootStream, ConfigStream};
use crate::system::{CoreBus, Endian, Fabric};
use crate::utopiaboot::{
    initialise_c6472_rom_boot, Parameters as UtopiaParameters, Progress as UtopiaProgress,
    UtopiaBoot,
};

/// BOOTMODE[3:0] 0101b is the C6455's master I2C boot (SPRUEC6G Table 3).
pub(crate) const BOOTMODE_I2C_EEPROM: u8 = 5;

/// The slave address section 2.3.4 requires the first EEPROM to answer at, and the C6472's
/// second one (Table 22).
pub(crate) const EEPROM_ADDRESS: u32 = 0x50;
const SECOND_EEPROM_ADDRESS: u32 = 0x51;

/// The EEPROM address a device's BOOTMODE pins name when they select the master I2C boot: the
/// C6455's mode 5 (Table 3), the C6472's modes 2 and 3 (Table 22). `None` for every other mode.
pub(crate) fn eeprom_address(c6472: bool, mode: u8) -> Option<u32> {
    match (c6472, mode) {
        (false, BOOTMODE_I2C_EEPROM) | (true, 2) => Some(EEPROM_ADDRESS),
        (true, 3) => Some(SECOND_EEPROM_ADDRESS),
        _ => None,
    }
}

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

/// One block, and the header inside it (SPRUEC6G section 2.3.4.1 and Table 5). Boot-table and
/// configuration-table blocks carry this header on the C6472 too.
const BLOCK_BYTES: usize = 128;
const BLOCK_HEADER_BYTES: usize = 4;

/// Where a boot parameter block keeps each field, as offsets into the block: the C6455's Table 6
/// behind the four-byte block header, or the C6472's Table 25 behind Table 23's ten-byte common
/// header with its extended boot mode at offset 4.
struct Layout {
    boot_mode: Option<usize>,
    options: usize,
    device_address_low: usize,
    device_address_high: usize,
    cpu_frequency: usize,
    i2c_frequency: usize,
    next_address_low: usize,
    next_address_high: usize,
}

const C6455_LAYOUT: Layout = Layout {
    boot_mode: None,
    options: 6,
    device_address_low: 8,
    device_address_high: 10,
    cpu_frequency: 16,
    i2c_frequency: 18,
    next_address_low: 20,
    next_address_high: 22,
};

const C6472_LAYOUT: Layout = Layout {
    boot_mode: Some(4),
    options: 10,
    device_address_low: 12,
    device_address_high: 14,
    cpu_frequency: 20,
    i2c_frequency: 22,
    next_address_low: 24,
    next_address_high: 26,
};

/// The options field's table types (SPRUEC6G section 2.3.4.2, Table 25), in bits 1:0 on both
/// devices. TI's C6472 ROM header (`tiboot_c6472.h` in the IBL, BOOT_PARAMS_I2C_OPTIONS_MASK)
/// masks two bits and puts the EEPROM type in bits 4:2, where Table 25 draws a three-bit field;
/// the ROM's own header is followed, so an EEPROM type never reads as a table type.
const OPTION_TABLE_TYPE: u16 = 0x3;
const TABLE_TYPE_PARAMETERS: u16 = 0;
const TABLE_TYPE_BOOT_TABLE: u16 = 1;
const TABLE_TYPE_CONFIGURATION: u16 = 2;

/// The C6472's extended boot modes this model runs (Table 24).
const EXTENDED_I2C_MASTER: u16 = 0x101;
const EXTENDED_UTOPIA: u16 = 0x104;

/// How many boot parameter blocks the boot will chase before giving up. The format lets one
/// parameter table name another, so a malformed EEPROM could point at itself.
const MAX_PARAMETER_BLOCKS: usize = 8;

/// Which ROM's I2C boot this is.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Rom {
    C6455,
    /// The C6472's, with the PLL1 multiplier CFG[4] selects.
    C6472 {
        pll_multiplier: u32,
    },
}

impl Rom {
    fn layout(self) -> &'static Layout {
        match self {
            Self::C6455 => &C6455_LAYOUT,
            Self::C6472 { .. } => &C6472_LAYOUT,
        }
    }
}

/// What the boot has done so far, for a front end to report.
#[derive(Clone, Default)]
pub(crate) struct Progress {
    /// Boot parameter blocks read and accepted.
    pub parameter_blocks: usize,
    /// Blocks of boot configuration table read and applied.
    pub config_blocks: usize,
    /// Blocks of the boot table read.
    pub table_blocks: usize,
    /// Bytes of boot-table payload handed to the table processor.
    pub table_bytes: usize,
    /// Blocks whose checksum did not verify, and which were therefore re-read.
    pub checksum_failures: usize,
    pub entry: Option<u32>,
    /// Why the boot stopped, when it stopped without an entry point.
    pub error: Option<String>,
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
    /// A parameter block named another boot, which now runs in this one's place.
    HandedOver(Box<UtopiaBoot>),
    Done,
}

/// What the current block is being read for.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Target {
    Parameters,
    BootTable,
    Configuration,
}

pub(crate) struct I2cBoot {
    rom: Rom,
    endian: Endian,
    /// The EEPROM's slave address, and the address inside it the next block starts at.
    slave: u32,
    at: u32,
    /// Where the parameter block after a boot configuration table is: the next device address
    /// the block that named the table gave, low word and high word.
    next: (u32, u32),
    target: Target,
    phase: Phase,
    stream: BootStream,
    config: ConfigStream,
    /// Table bytes on their way to the device's byte order, with a word that a block ended
    /// part-way through.
    words: WordOrder,
    parameter_blocks: usize,
    progress: Progress,
}

impl I2cBoot {
    /// The boot as the pins strap it: the first EEPROM at `slave`, and the boot parameter block
    /// at `0x80 * index`, where the index comes from the configuration pins (SPRUEC6G section
    /// 2.3.4.2 for the C6455, Table 22 for the C6472's CFG[3:0]).
    pub(crate) fn new(rom: Rom, slave: u32, index: u32, endian: Endian) -> Self {
        Self {
            rom,
            endian,
            slave,
            at: index * BLOCK_BYTES as u32,
            next: (0, 0),
            target: Target::Parameters,
            phase: Phase::Configure,
            stream: BootStream::new(endian),
            config: ConfigStream::new(endian),
            words: WordOrder::new(endian),
            parameter_blocks: 0,
            progress: Progress::default(),
        }
    }

    pub(crate) fn progress(&self) -> Progress {
        self.progress.clone()
    }

    /// What the UTOPIA boot a parameter block handed over to has done, once one has.
    pub(crate) fn utopia(&self) -> Option<UtopiaProgress> {
        match &self.phase {
            Phase::HandedOver(boot) => Some(boot.progress()),
            _ => None,
        }
    }

    /// One cycle of the ROM's boot loop. Returns the entry point once the boot table has ended,
    /// which is the device's signal to start core 0 there.
    pub(crate) fn tick(&mut self, fabric: &mut Fabric) -> Option<u32> {
        if let Phase::HandedOver(boot) = &mut self.phase {
            return boot.tick(fabric);
        }
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
            Phase::HandedOver(_) => unreachable!("handled above"),
            Phase::Done => None,
        }
    }

    /// Take the module out of reset and give it a bus clock. The dividers are the ones the boot
    /// parameters will replace; nothing this model counts is clocked from them. The C6472 sets
    /// its caches and PLL1 first, as every one of its ROM boots does (Tables 20 and 21).
    fn configure(&mut self, bus: &mut CoreBus<'_>) {
        if let Rom::C6472 { pll_multiplier } = self.rom {
            initialise_c6472_rom_boot(bus, pll_multiplier);
        }
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
    /// table processor it belongs to and go round again.
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
                self.take_parameters(bus, &block[..size]);
                None
            }
            Target::Configuration => {
                self.progress.config_blocks += 1;
                let data = self.words.convert(&block[BLOCK_HEADER_BYTES..size]);
                match self.config.feed(&data, bus) {
                    Err(message) => self.fail(message),
                    Ok(false) => {
                        self.at = self.at.wrapping_add(size as u32);
                        self.begin_address(bus);
                        None
                    }
                    Ok(true) => {
                        // The boot re-entry: the next device address becomes the device address,
                        // and what is there is another parameter block.
                        if self.parameter_blocks >= MAX_PARAMETER_BLOCKS {
                            return self
                                .fail("boot parameter tables chain without naming a boot table");
                        }
                        let (low, high) = self.next;
                        self.address(low, high);
                        self.config = ConfigStream::new(self.endian);
                        self.target = Target::Parameters;
                        self.begin_address(bus);
                        None
                    }
                }
            }
            Target::BootTable => {
                self.progress.table_blocks += 1;
                self.progress.table_bytes += size - BLOCK_HEADER_BYTES;
                let data = self.words.convert(&block[BLOCK_HEADER_BYTES..size]);
                match self.stream.feed(&data, bus) {
                    Err(_) => self.fail("boot table has no termination record"),
                    Ok(false) => {
                        self.at = self.at.wrapping_add(size as u32);
                        self.begin_address(bus);
                        None
                    }
                    Ok(true) => {
                        let entry = self.stream.entry().expect("an ended table has an exit");
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

    /// Read the parameter block - `block` is its `size` bytes - and act on it: on the C6472 its
    /// extended boot mode first, then its options field: another parameter table, a
    /// configuration table, or the boot table this boot exists to load.
    fn take_parameters(&mut self, bus: &mut CoreBus<'_>, block: &[u8]) {
        self.progress.parameter_blocks += 1;
        self.parameter_blocks += 1;
        let layout = self.rom.layout();
        if let Some(offset) = layout.boot_mode {
            if block.len() < offset + 2 {
                self.fail("boot parameter block is shorter than its common header");
                return;
            }
            match be16(block, offset) {
                EXTENDED_I2C_MASTER => {}
                EXTENDED_UTOPIA => {
                    self.hand_over(bus, block);
                    return;
                }
                _ => {
                    self.fail(
                        "boot parameter block names an extended boot mode this boot does not run",
                    );
                    return;
                }
            }
        }
        if block.len() < layout.i2c_frequency + 2 {
            self.fail("boot parameter block is shorter than the parameter structure");
            return;
        }
        // "This parameter configures the I2C clock" (Table 6): the two frequencies are what the
        // ROM reprograms the bus dividers from, so the registers end up holding what the EEPROM
        // asked for. Only the dividers: the manual names no prescaler value, and the prescaler
        // is the one field of the three this model's transfer time depends on, so reprogramming
        // it from a number the manual does not give would change how long the boot takes for a
        // reason no document backs.
        let cpu_mhz = u32::from(be16(block, layout.cpu_frequency));
        let bus_khz = u32::from(be16(block, layout.i2c_frequency));
        if cpu_mhz != 0 && bus_khz != 0 {
            let half = (cpu_mhz * 1000 / bus_khz / 2).clamp(1, 0xffff);
            bus.write32(ICCLKL, half);
            bus.write32(ICCLKH, half);
        }
        let options = be16(block, layout.options) & OPTION_TABLE_TYPE;
        let low = u32::from(be16(block, layout.device_address_low));
        let high = u32::from(be16(block, layout.device_address_high));
        match options {
            TABLE_TYPE_BOOT_TABLE => self.target = Target::BootTable,
            TABLE_TYPE_PARAMETERS => {
                if self.parameter_blocks >= MAX_PARAMETER_BLOCKS {
                    self.fail("boot parameter tables chain without naming a boot table");
                    return;
                }
                self.target = Target::Parameters;
            }
            TABLE_TYPE_CONFIGURATION => {
                if block.len() < layout.next_address_high + 2 {
                    self.fail("boot parameter block is shorter than the next device address");
                    return;
                }
                self.next = (
                    u32::from(be16(block, layout.next_address_low)),
                    u32::from(be16(block, layout.next_address_high)),
                );
                self.target = Target::Configuration;
            }
            _ => {
                // Slave receive options, the one encoding left: it reads no table off this
                // EEPROM for this device.
                self.fail("boot parameter options name a table type this boot does not load");
                return;
            }
        }
        self.address(low, high);
        self.begin_address(bus);
    }

    /// Point the next read at a device address: the low word is the address inside the EEPROM,
    /// and the high word "is used as the I2C EEPROM slave address" (Table 6), where zero means
    /// the EEPROM the boot is already talking to.
    /// A new table starts there, so no part of a word is carried over from the last.
    fn address(&mut self, low: u32, high: u32) {
        self.words = WordOrder::new(self.endian);
        if high != 0 {
            self.slave = high & 0x3ff;
        }
        self.at = low;
    }

    /// A C6472 parameter block named UTOPIA (Table 24's 0x104): its Table 26 fields configure
    /// the UTOPIA boot, which runs from here on in this one's place. The I2C module goes back
    /// into reset first, as it does when this boot ends.
    fn hand_over(&mut self, bus: &mut CoreBus<'_>, block: &[u8]) {
        match UtopiaParameters::from_block(block) {
            Err(reason) => {
                self.fail(reason);
            }
            Ok(parameters) => {
                bus.write32(ICMDR, 0);
                self.phase =
                    Phase::HandedOver(Box::new(UtopiaBoot::chained(parameters, self.endian)));
            }
        }
    }

    fn fail(&mut self, reason: impl Into<String>) -> Option<u32> {
        self.progress.error = Some(reason.into());
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

/// Table data as the device's memory holds it. The EEPROM carries every 32-bit word of a boot or
/// configuration table most significant byte first, whatever the device's byte order: SPRUEC6G
/// section 6.2.2.3.1 builds the table with `hex6x -a`, whose ASCII-hex output writes each 32-bit
/// word that way for `-order L` as for `-order M`, and TI's IBL tools say so outright -
/// `bconvert64x`: "The C6x boot loader expects all 32 bit values to be in big endian ... Data
/// bytes are also arranged to be 32 bit big endian represented" - before `b2i2c` blocks the
/// table. A little-endian device therefore stores each word with its bytes turned round.
///
/// TI's `b2i2c` blocks carry 124 data bytes, but section 2.3.4.1 sets only a 128-byte maximum,
/// so a block may end part-way through a word. The word's first bytes are held until the next
/// block of the same table brings the rest, and only whole words go on to the table processor.
struct WordOrder {
    endian: Endian,
    carry: Vec<u8>,
}

impl WordOrder {
    fn new(endian: Endian) -> Self {
        Self {
            endian,
            carry: Vec::new(),
        }
    }

    /// The next block's data, as whole words in the device's order.
    fn convert<'a>(&mut self, data: &'a [u8]) -> std::borrow::Cow<'a, [u8]> {
        if self.endian == Endian::Big {
            return std::borrow::Cow::Borrowed(data);
        }
        let mut bytes = std::mem::take(&mut self.carry);
        bytes.extend_from_slice(data);
        self.carry = bytes.split_off(bytes.len() & !3);
        for word in bytes.chunks_exact_mut(4) {
            word.reverse();
        }
        std::borrow::Cow::Owned(bytes)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::board::Board;
    use crate::device::{Device, C6455, C6472};

    /// One block, laid out as SPRUEC6G Table 5 says: a 16-bit size that counts the header, a
    /// 16-bit checksum over the whole block, then the payload.
    pub(crate) fn block(payload: &[u8]) -> Vec<u8> {
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

    /// A boot parameter block padded out to the 128 bytes section 2.3.4.2 says the block
    /// consists of, with 2-byte big-endian fields at their offsets into the block.
    fn parameter_block(fields: &[(usize, u16)]) -> Vec<u8> {
        let mut payload = vec![0u8; BLOCK_BYTES - BLOCK_HEADER_BYTES];
        for &(offset, value) in fields {
            let at = offset - BLOCK_HEADER_BYTES;
            payload[at..at + 2].copy_from_slice(&value.to_be_bytes());
        }
        let block = block(&payload);
        assert_eq!(block.len(), BLOCK_BYTES);
        block
    }

    /// The C6455's Table 6 block.
    fn parameters(options: u16, device_low: u16, device_high: u16) -> Vec<u8> {
        parameter_block(&[
            (4, BOOTMODE_I2C_EEPROM as u16),
            (C6455_LAYOUT.options, options),
            (C6455_LAYOUT.device_address_low, device_low),
            (C6455_LAYOUT.device_address_high, device_high),
            (C6455_LAYOUT.cpu_frequency, 720),
            (C6455_LAYOUT.i2c_frequency, 100),
        ])
    }

    /// A C6472 block: Table 23's common header naming `boot_mode` (Table 24), then `fields` at
    /// their offsets - Table 25's for an I2C block, Table 26's for a UTOPIA one.
    pub(crate) fn c6472_parameters(boot_mode: u16, fields: &[(usize, u16)]) -> Vec<u8> {
        let mut all = vec![(4, boot_mode)];
        all.extend_from_slice(fields);
        parameter_block(&all)
    }

    /// A C6472 Table 25 block that sends the boot to `options`' table at `device_low` on the
    /// EEPROM at `slave`, with Table 25's default frequencies.
    pub(crate) fn c6472_i2c_parameters(
        options: u16,
        device_low: u16,
        slave: u16,
        next: (u16, u16),
    ) -> Vec<u8> {
        c6472_parameters(
            EXTENDED_I2C_MASTER,
            &[
                (C6472_LAYOUT.options, options),
                (C6472_LAYOUT.device_address_low, device_low),
                (C6472_LAYOUT.device_address_high, slave),
                (C6472_LAYOUT.cpu_frequency, 50),
                (C6472_LAYOUT.i2c_frequency, 10),
                (C6472_LAYOUT.next_address_low, next.0),
                (C6472_LAYOUT.next_address_high, next.1),
            ],
        )
    }

    /// `bytes` cut into as many blocks as they need.
    pub(crate) fn blocks(bytes: &[u8]) -> Vec<u8> {
        bytes
            .chunks(BLOCK_BYTES - BLOCK_HEADER_BYTES)
            .flat_map(block)
            .collect()
    }

    /// A boot table, then an EEPROM holding the parameter block and that table behind it.
    fn eeprom(table: &[u8]) -> Vec<u8> {
        let mut image = parameters(
            TABLE_TYPE_BOOT_TABLE,
            BLOCK_BYTES as u16,
            EEPROM_ADDRESS as u16,
        );
        image.extend(blocks(table));
        image
    }

    /// Table words as the EEPROM carries them, most significant byte first on either byte order
    /// (`WordOrder`).
    pub(crate) fn words(words: &[u32]) -> Vec<u8> {
        words.iter().flat_map(|word| word.to_be_bytes()).collect()
    }

    fn run(device: &mut Device, cycles: usize) {
        for _ in 0..cycles {
            device.step_cycle();
            if device
                .boot()
                .i2c()
                .is_some_and(|boot| boot.entry.is_some() || boot.error.is_some())
            {
                return;
            }
        }
    }

    fn read(device: &mut Device, address: u32) -> u32 {
        device.with_core(0, |cpu| cpu.bus_mut().read32(address))
    }

    /// The whole boot, on the device and through the I2C master's own registers: the parameter
    /// block is read from address 0, the table from where that block points, and its sections
    /// land in memory with core 0 released at the entry point.
    #[test]
    fn an_i2c_boot_reads_its_table_off_the_bus_and_starts_the_core() {
        const ENTRY: u32 = 0x0080_0400;
        const DESTINATION: u32 = 0x0080_1000;
        // A table long enough to need two blocks, so a section's data straddles a block
        // boundary and the read has to come back for the rest. The data travels as words most
        // significant byte first, so on this little-endian device each word's bytes land
        // turned round: the wire carries the payload that way so memory holds it as written.
        let payload: Vec<u8> = (0..200u32).map(|index| (index * 7 + 1) as u8).collect();
        let mut table = words(&[ENTRY, payload.len() as u32, DESTINATION]);
        table.extend(
            payload
                .chunks(4)
                .flat_map(|word| word.iter().rev().copied()),
        );
        table.extend(words(&[0]));

        let mut device = Device::on_board(
            &Board::pins(&C6455, BOOTMODE_I2C_EEPROM).with_i2c_eeprom(0, eeprom(&table)),
            0,
            Endian::Little,
        );
        assert!(
            device.held_in_reset(0),
            "core 0 runs the ROM, so it is held"
        );
        run(&mut device, 200_000);

        let boot = device.boot().i2c().expect("the device is booting over I2C");
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
        let mut device = Device::on_board(
            &Board::pins(&C6455, BOOTMODE_I2C_EEPROM).with_i2c_eeprom(0, image),
            0,
            Endian::Little,
        );
        run(&mut device, 200_000);

        let boot = device.boot().i2c().expect("the device is booting over I2C");
        assert_eq!(boot.entry, None);
        assert_eq!(boot.parameter_blocks, 0, "a bad block is never acted on");
        assert_eq!(boot.checksum_failures, 2, "read once, re-read once");
        assert_eq!(
            boot.error.as_deref(),
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
        let table = words(&[0x0080_0800, 0]);
        image.extend_from_slice(&block(&table));
        let mut device = Device::on_board(
            &Board::pins(&C6455, BOOTMODE_I2C_EEPROM).with_i2c_eeprom(0, image),
            0,
            Endian::Little,
        );
        run(&mut device, 200_000);

        let boot = device.boot().i2c().expect("the device is booting over I2C");
        assert_eq!(boot.error, None);
        assert_eq!(boot.entry, Some(0x0080_0800));
    }

    /// The parameter block the configuration pins select: "address = 0x80 * CFGGP[2:0]"
    /// (SPRUEC6G section 2.3.4.2). Index 1 reads the second block, not the first.
    #[test]
    fn the_configuration_pins_select_which_parameter_block_is_read() {
        // Block 0 names the reserved table type, which this boot refuses; block 1 names a real
        // boot table. A boot that read block 0 would fail, so reaching the entry point is what
        // proves the index.
        let mut image = parameters(3, 0, 0);
        image.extend_from_slice(&parameters(
            TABLE_TYPE_BOOT_TABLE,
            2 * BLOCK_BYTES as u16,
            EEPROM_ADDRESS as u16,
        ));
        let table = words(&[0x0080_0c00, 0]);
        image.extend_from_slice(&block(&table));
        let mut device = Device::on_board(
            &Board::pins(&C6455, BOOTMODE_I2C_EEPROM).with_i2c_eeprom(1, image),
            0,
            Endian::Little,
        );
        run(&mut device, 200_000);

        let boot = device.boot().i2c().expect("the device is booting over I2C");
        assert_eq!(boot.error, None);
        assert_eq!(boot.entry, Some(0x0080_0c00));
    }

    /// MVK .L1 k,A4 with k in bits 22-18, and ADD .L1 1,A4,A4: with k = 5 and two adds, a
    /// program that leaves 7 in A4.
    const MVK_A4: u32 = 0x0200_a358;
    const ADD_A4: u32 = 0x0210_2058;
    const CODE: u32 = 0x0080_1000;
    const DATA: u32 = 0x0081_0000;

    /// SPRUEC6G section 4.3.3 and Tables 22 to 25 on the six-core device: BOOTMODE 2 reads the
    /// EEPROM at 0x50 and 3 the one at 0x51, CFG[3:0] picks the parameter block and CFG[4] PLL1's
    /// x9 or x19, every block of parameters carries Table 23's common header, and the boot table
    /// it names puts a program in core 0's L2 that core 0 then runs. Cores 1-5 stay held: nothing
    /// in the table releases them.
    #[test]
    fn a_c6472_boots_a_program_off_the_eeprom_its_pins_name() {
        let data: Vec<u32> = (0..60)
            .map(|index| 0x0102_0304u32.wrapping_mul(index + 1))
            .collect();
        for endian in [Endian::Little, Endian::Big] {
            for (mode, slave, cfggp, multiplier) in
                [(2u8, 0x50u16, 0x00u8, 9u32), (3, 0x51, 0x13, 19)]
            {
                let label = format!(
                    "big-endian {} mode {mode} CFG {cfggp:#x}",
                    endian == Endian::Big
                );
                let index = usize::from(cfggp & 0xf);
                // Every block before the selected one names a boot this model does not run, so
                // reading any of them stops the boot.
                let mut image = Vec::new();
                for _ in 0..index {
                    image.extend(c6472_parameters(0x105, &[]));
                }
                let table_at = ((index + 1) * BLOCK_BYTES) as u16;
                // Mode 3's block also names an EEPROM type in options bits 4:2 (1, the 24C32
                // family, in tiboot_c6472.h), which must not change the table type.
                let eeprom_type = if mode == 3 { 1 << 2 } else { 0 };
                image.extend(c6472_i2c_parameters(
                    TABLE_TYPE_BOOT_TABLE | eeprom_type,
                    table_at,
                    slave,
                    (0, 0),
                ));
                let mut table = vec![CODE, 12, CODE, MVK_A4 | (5 << 18), ADD_A4, ADD_A4];
                table.extend([4 * data.len() as u32, DATA]);
                table.extend(&data);
                table.push(0);
                image.extend(blocks(&words(&table)));

                let mut device = Device::on_board(
                    &Board::pins(&C6472, mode).with_i2c_eeprom(cfggp, image),
                    0,
                    endian,
                );
                for core in 0..6 {
                    assert!(device.held_in_reset(core), "{label}: core {core}");
                }
                run(&mut device, 400_000);

                let boot = device.boot().i2c().expect("the device is booting over I2C");
                assert_eq!(boot.error, None, "{label}");
                assert_eq!(boot.entry, Some(CODE), "{label}");
                assert_eq!(boot.parameter_blocks, 1, "{label}");
                assert_eq!(
                    boot.table_blocks, 3,
                    "{label}: 276 bytes in 124-byte blocks"
                );
                // The pins as software reads them, and what the ROM set from them first.
                assert_eq!(
                    (read(&mut device, 0x02a8_0000) >> 17) & 0x1f,
                    u32::from(cfggp)
                );
                assert_eq!(
                    read(&mut device, 0x029a_0110) & 0x3f,
                    multiplier - 1,
                    "{label}"
                );
                assert_eq!(read(&mut device, crate::memory::L1PCFG) & 7, 4, "{label}");
                assert_eq!(read(&mut device, crate::memory::L1DCFG) & 7, 3, "{label}");
                assert_eq!(read(&mut device, crate::memory::L2CFG) & 7, 0, "{label}");
                for (index, &word) in data.iter().enumerate() {
                    assert_eq!(read(&mut device, DATA + 4 * index as u32), word, "{label}");
                }

                assert!(!device.held_in_reset(0), "{label}");
                assert_eq!(device.core(0).pc(), CODE, "{label}");
                for _ in 0..400 {
                    device.step_cycle();
                }
                assert_eq!(device.core(0).register('A', 4), 7, "{label}");
                for core in 1..6 {
                    assert!(device.held_in_reset(core), "{label}: core {core}");
                }
            }
        }
    }

    /// Options 2: "the bootloader reads from the specified I2C EEPROM address until the end of
    /// the boot configuration table", then "The next device address previously read from the
    /// boot parameter block is copied into device address ... and the bootloader performs a boot
    /// re-entry" (SPRUEC6G section 2.3.4.2). The table's entries set, clear and toggle bits
    /// (section 6.2.3), and the parameter block the re-entry reads names the boot table.
    #[test]
    fn a_configuration_table_runs_and_the_boot_reenters_at_the_next_device_address() {
        const TARGET: u32 = 0x0080_2000;
        for (c6472, endian) in [
            (false, Endian::Little),
            (true, Endian::Little),
            (true, Endian::Big),
        ] {
            // A configuration table longer than a block, so an entry straddles the boundary.
            let mut config = Vec::new();
            for index in 0..12u32 {
                config.extend([TARGET + 4 * index, 0x00ff_00f0, 0x0f00_0000]);
            }
            // Both masks naming the same bits toggles them.
            config.extend([TARGET + 0x40, 0xf0f0_f0f0, 0xf0f0_f0f0, 0, 0, 0]);
            let config = blocks(&words(&config));
            let next = (BLOCK_BYTES + config.len()) as u16;
            let table_at = next + BLOCK_BYTES as u16;
            let mut image = if c6472 {
                c6472_i2c_parameters(TABLE_TYPE_CONFIGURATION, 0x80, 0x50, (next, 0x50))
            } else {
                parameter_block(&[
                    (C6455_LAYOUT.options, TABLE_TYPE_CONFIGURATION),
                    (C6455_LAYOUT.device_address_low, 0x80),
                    (C6455_LAYOUT.next_address_low, next),
                    (C6455_LAYOUT.next_address_high, 0x50),
                ])
            };
            image.extend(config);
            image.extend(if c6472 {
                c6472_i2c_parameters(TABLE_TYPE_BOOT_TABLE, table_at, 0, (0, 0))
            } else {
                parameters(TABLE_TYPE_BOOT_TABLE, table_at, 0)
            });
            image.extend(blocks(&words(&[0x0080_0c00, 0])));
            let (profile, mode) = if c6472 { (&C6472, 2) } else { (&C6455, 5) };
            let mut device = Device::on_board(
                &Board::pins(profile, mode).with_i2c_eeprom(0, image),
                0,
                endian,
            );
            device.with_core(0, |cpu| {
                let mut bus = cpu.bus_mut();
                for index in 0..12 {
                    bus.write32(TARGET + 4 * index, 0x0f00_0003);
                }
                bus.write32(TARGET + 0x40, 0x0ff0_0ff0);
            });
            run(&mut device, 400_000);

            let label = format!("C6472 {c6472} big-endian {}", endian == Endian::Big);
            let boot = device.boot().i2c().unwrap();
            assert_eq!(boot.error, None, "{label}");
            assert_eq!(boot.parameter_blocks, 2, "{label}");
            assert_eq!(boot.config_blocks, 2, "{label}");
            assert_eq!(boot.entry, Some(0x0080_0c00), "{label}");
            for index in 0..12 {
                assert_eq!(
                    read(&mut device, TARGET + 4 * index),
                    0x00ff_00f3,
                    "{label}"
                );
            }
            assert_eq!(read(&mut device, TARGET + 0x40), 0xff00_ff00, "{label}");
        }
    }

    /// What a C6472 parameter block can name that stops the boot with core 0 held: an entry
    /// asking the ROM to call a function, a boot this model does not chain to, and UTOPIA
    /// parameters the port cannot be set up from.
    #[test]
    fn a_c6472_block_this_boot_cannot_follow_stops_it() {
        let call = {
            let mut image =
                c6472_i2c_parameters(TABLE_TYPE_CONFIGURATION, 0x80, 0x50, (0x100, 0x50));
            image.extend(blocks(&words(&[0x0080_3000, 0, 0, 0, 0, 0])));
            image
        };
        let mac = c6472_parameters(0x105, &[]);
        // Options say a 16-bit bus, the bus width field 8 bits.
        let utopia = c6472_parameters(EXTENDED_UTOPIA, &[(10, 2), (12, 54), (14, 8), (18, 500)]);
        for (image, error) in [
            (call, "calls the function at 0x00803000"),
            (mac, "extended boot mode"),
            (utopia, "bus width"),
        ] {
            let mut device = Device::on_board(
                &Board::pins(&C6472, 2).with_i2c_eeprom(0, image),
                0,
                Endian::Little,
            );
            run(&mut device, 400_000);
            let boot = device.boot().i2c().unwrap();
            let reported = boot.error.expect(error);
            assert!(reported.contains(error), "{reported}");
            assert!(device.held_in_reset(0), "{error}");
            assert_eq!(device.boot().utopia(), None, "{error}");
        }
    }

    /// Section 2.3.4.1 bounds a block at 128 bytes and nothing more, so a board may block its
    /// tables at any length and a word can end in the next block. On a little-endian device the
    /// ROM still has to turn that word round whole: a configuration table cut every 10 bytes
    /// and a boot table cut every 122 put a word across nearly every boundary.
    #[test]
    fn a_word_split_across_blocks_reaches_memory_whole() {
        const TARGET: u32 = 0x0080_2000;
        let cut = |bytes: &[u8], payload: usize| -> Vec<u8> {
            bytes.chunks(payload).flat_map(block).collect()
        };
        let data: Vec<u32> = (0..60)
            .map(|index| 0x0102_0304u32.wrapping_mul(index + 1))
            .collect();
        let mut table = vec![CODE, 12, CODE, MVK_A4 | (5 << 18), ADD_A4, ADD_A4];
        table.extend([4 * data.len() as u32, DATA]);
        table.extend(&data);
        table.push(0);
        let config = cut(
            &words(&[TARGET, 0x1234_5678, 0, TARGET + 4, 0xa5a5_0000, 0, 0, 0, 0]),
            10,
        );
        let next = (BLOCK_BYTES + config.len()) as u16;
        let mut image = c6472_i2c_parameters(
            TABLE_TYPE_CONFIGURATION,
            BLOCK_BYTES as u16,
            0x50,
            (next, 0x50),
        );
        image.extend(config);
        image.extend(c6472_i2c_parameters(
            TABLE_TYPE_BOOT_TABLE,
            next + BLOCK_BYTES as u16,
            0x50,
            (0, 0),
        ));
        image.extend(cut(&words(&table), 122));

        let mut device = Device::on_board(
            &Board::pins(&C6472, 2).with_i2c_eeprom(0, image),
            0,
            Endian::Little,
        );
        run(&mut device, 600_000);
        let boot = device.boot().i2c().unwrap();
        assert_eq!(boot.error, None);
        assert_eq!(boot.checksum_failures, 0);
        assert_eq!(boot.entry, Some(CODE));
        assert_eq!(read(&mut device, TARGET), 0x1234_5678);
        assert_eq!(read(&mut device, TARGET + 4), 0xa5a5_0000);
        for (index, &word) in data.iter().enumerate() {
            assert_eq!(
                read(&mut device, DATA + 4 * index as u32),
                word,
                "data word {index}"
            );
        }
        for _ in 0..400 {
            device.step_cycle();
        }
        assert_eq!(device.core(0).register('A', 4), 7);
    }
}
