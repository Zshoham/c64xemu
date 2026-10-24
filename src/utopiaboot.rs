//! The ROM bootloader's UTOPIA boot mode: the boot the C6472 performs with BOOTMODE[3:0] set to
//! 5 through 8 (SPRUEC6G section 4.3.4 and Tables 20, 21, 22 and 26, SPRS612G Table 2-4).
//!
//! Core 0 runs the on-chip ROM while cores 1-5 stay internally held in reset. The ROM brings the
//! UTOPIA port up as a slave with big-endian cell assembly, routes every cell the ATM master
//! sends to PDMA channel 8 - the first receive channel, RXC0 - and has that channel fill 16 KB of
//! core 0's L1D as a circular buffer of cell packets. It sets local timer 0 to interrupt every
//! 100 us for an 8-bit port and every 50 us for a 16-bit one, both counted from the boot
//! parameter table's core frequency, and then sleeps until an interrupt. The PDMA interrupts at
//! the half and the full buffer mark, the timer at its period; whichever arrives, the ROM looks
//! for cells the PDMA has delivered since it last looked. A cell is accepted when it carries the
//! 32-bit magic 0x54492164 (the ASCII "TI!d", the first byte on the wire being the 0x54), and the
//! 32 bytes in front of the magic - searched from the end of the cell's payload towards its
//! start - are the next piece of an ordinary boot table, handed to boot-table processing. A cell
//! without the magic is discarded and counted. When the table ends the ROM disables every receive
//! PDMA channel and the timer and branches core 0 to the entry point the table named.
//!
//! The ROM's *instructions* are not modelled - TI ships them in on-chip ROM this project has no
//! copy of - and what is modelled is everything the ROM does that software or an ATM master can
//! observe, performed by this state machine on core 0's behalf while core 0 is held: it reads
//! the power/sleep controller, writes PLL1's multiplier, the cache configuration, the UTOPIA
//! control register, the PDMA registers and the timer exactly as firmware would, and takes its
//! wake-ups from the same latched interrupt flags (EVTFLAG0) a handler would find set. Sleeping
//! is simply not looking until one of them is.
//!
//! Where the manual leaves a choice, the choice is the one the other documents force:
//!
//! * The cell packet in memory is the word-aligned form of SPRUEG2E section 2 and Tables 6, 7, 12
//!   and 14: dummy bytes in front of the header pad the wire cell to a word multiple, so a
//!   standard cell is a 56-byte, 14-word packet whose 48 payload bytes are always its last twelve
//!   words. With UCR.BEND set each word holds four consecutive wire bytes with the first in its
//!   most significant byte. Section 4.3.4 passes "the payload" - the bytes in L1D - to the boot
//!   table processing function, which reads them as the device does, so a table word is the
//!   four wire bytes most significant first on either byte order: the table travels as big-endian
//!   words, the hex conversion utility's `-order M` [INFERENCE: no manual states the wire order
//!   for a little-endian device; this is what the documented BEND setting gives a ROM that does
//!   not swap bytes back].
//! * Section 4.3.4 gives the buffer as "256 to 292 cells" in 16 KB, which is 16384 divided by
//!   the packet size: 292 cells of 56 bytes for the 53- and 54-byte cells a cold boot uses, down
//!   to 256 cells of 64 bytes for the longest cell the port takes. The buffer size register
//!   therefore holds the whole packets that fit - 292 * 56 = 16352 bytes for a standard cell -
//!   and the PDMA wraps there.
//! * The channel's peripheral-side buffer size has no stated value for a constant-address
//!   peripheral; the ROM writes one cell packet, the unit the peripheral hands over per event.
//! * Table 26 gives the PHY ID the boot reads from CFGGP[4:0] and the PLL multiplier the mode
//!   selects; PLL1's multiplier register takes the multiplier minus one (SPRS612G Table 7-23:
//!   09h is x10), which is all of the PLL programming this model has a register for.
//! * The PDMA interrupt is system event 30 in the interrupt controller of the core the channel's
//!   SINT field names, and SINT 0 is core 0. Local timer 0 belongs to megamodule 0: its clock is
//!   gated with that megamodule's, so the ROM makes sure LPSC1 is enabled along with the UTOPIA.
//! * The table's entry point is resolved as every ROM boot resolves it: `BootStream::entry`
//!   makes zero the default boot exit address.
//!
//! Table 26's boot parameter table is what an I2C boot hands over when a parameter block names
//! UTOPIA (extended boot mode 0x104, Table 24); a cold boot builds the same table from the pins
//! with Table 26's defaults. How each field reaches the hardware:
//!
//! * The physical cell size is the standard cell (53 bytes, 54 on the 16-bit bus) plus UCR's
//!   RUDC user-defined header bytes, which SPRUEG2E section 2 puts in front of the header; a
//!   size the port cannot receive (below the standard cell, above 64 bytes) stops the boot.
//!   Its packet sets the PDMA's burst, the buffer's cell count and where each payload sits.
//! * Bit 0 of the options clears UCR.MPHY for a single-PHY port, which then answers every
//!   address; the slave ID is SLID either way.
//! * Bit 1 of the options and the bus width field say the same thing twice. **[INFERENCE]**
//!   The manual does not say which wins, so a table in which they disagree, or whose width is
//!   neither 8 nor 16, stops the boot rather than guessing.
//! * Bit 2 skips "port initialisation": **[INFERENCE]** the UCR write and nothing else. The
//!   ROM's own receive path - the PSC, PDMA channel 8 into L1D and timer 0 - is how it finds
//!   cells at all, so those are programmed regardless; the port is left as an earlier boot
//!   configuration table, or the reset state, left it.
//! * The core frequency is what the timer's period is counted from, "after PLL
//!   configuration". A chained boot leaves PLL1 as the I2C boot set it; only the cold boot
//!   programs the multiplier its mode names.

use crate::boot::BootStream;
use crate::memory::{L1DCFG, L1PCFG, L2CFG};
use crate::pdma::RECEIVE_QUEUE_PORT;
use crate::system::{CoreBus, Endian, Fabric};

/// BOOTMODE[3:0] 5-8: UTOPIA boot with an 8-bit port and PLL x10, 8-bit and x20, 16-bit and x10,
/// 16-bit and x20 (SPRUEC6G Table 22).
const BOOTMODE_FIRST: u8 = 5;
const BOOTMODE_LAST: u8 = 8;

pub(crate) fn is_boot_mode(mode: u8) -> bool {
    (BOOTMODE_FIRST..=BOOTMODE_LAST).contains(&mode)
}

/// The power/sleep controller: MDSTATn and MDCTLn belong to LPSC n+1, so module 0 is megamodule
/// 0 (whose clock gates local timer 0, SPRUEG3B section 4.1) and module 13 is the UTOPIA. PTCMD
/// and PTSTAT start and report a transition of domain 0, which holds every LPSC (SPRUEG3B Table
/// 3, section 4.1.2; SPRS612G Table 7-3).
const PSC: u32 = 0x02ae_0000;
const PSC_PTCMD: u32 = PSC + 0x120;
const PSC_PTSTAT: u32 = PSC + 0x128;
const PSC_MODULES: [u32; 2] = [0, 13];
const PSC_STATE_MASK: u32 = 0x3f;
const PSC_STATE_ENABLE: u32 = 0x03;
const PSC_LRSTZ: u32 = 1 << 8;
const PSC_GO_DOMAIN0: u32 = 1;

fn psc_status(module: u32) -> u32 {
    PSC + 0x800 + 4 * module
}

fn psc_control(module: u32) -> u32 {
    PSC + 0xa00 + 4 * module
}

/// PLL1's multiplier register (SPRS612G Table 7-20, Table 7-23).
const PLL1_PLLM: u32 = 0x029a_0110;

/// L1PCFG, L1DCFG and L2CFG: SPRUEC6G Table 21 leaves 32 KB of L1P and 16 KB of L1D as cache for
/// the boots from 2 to 14, and "L2 cache is always disabled". The mode field doubles the L1
/// cache from 4 KB at mode 1, so 3 is 16 KB and 4 is 32 KB.
const L1P_CACHE_32K: u32 = 4;
const L1D_CACHE_16K: u32 = 3;
const L2_CACHE_OFF: u32 = 0;

/// What every C6472 ROM boot from mode 2 to 14 programs first (SPRUEC6G Tables 20 and 21): the
/// cache settings, and PLL1's multiplier, whose register takes the multiplier minus one.
pub(crate) fn initialise_c6472_rom_boot(bus: &mut CoreBus<'_>, pll_multiplier: u32) {
    bus.write32(L1PCFG, L1P_CACHE_32K);
    bus.write32(L1DCFG, L1D_CACHE_16K);
    bus.write32(L2CFG, L2_CACHE_OFF);
    bus.write32(PLL1_PLLM, pll_multiplier - 1);
}

/// UCR (SPRUEG2E Table 19). The ROM configures the receive side only: the master drives the
/// transmit interface's address and clock, and nothing in this boot writes it.
const UCR: u32 = 0x02b4_0000;
const UCR_BEND: u32 = 1 << 31;
const UCR_SLID_SHIFT: u32 = 24;
const UCR_U16M: u32 = 1 << 13;
const UCR_MPHY: u32 = 1 << 14;
const UCR_UREN: u32 = 1 << 0;

/// The PDMA's global registers and channel proxies (SPRS612G Tables 7-143 and 7-144, SPRUEG2E
/// Tables 27 and 28). A channel's proxy block is 16 bytes from 0x100, and the receive channels
/// are channels 8 up: RXC0 at 0x180 is channel 8, which is where the ROM routes the cells.
const PDMA: u32 = 0x02bc_0000;
const PDMA_GCSR: u32 = PDMA + 0x04;
const PDMA_PCR1: u32 = PDMA + 0x64;
const PDMA_RXC0: u32 = PDMA + 0x180;
const PDMA_RX_CHANNELS: u32 = 6;
const PDMA_PROXY_STRIDE: u32 = 0x10;
const PROXY_SAR: u32 = 0x0;
const PROXY_PAR: u32 = 0x4;
const PROXY_BSR: u32 = 0x8;
const PROXY_TCR: u32 = 0xc;

/// GCSR.STRT enables the PDMA (SPRUEG2E Table 30).
const GCSR_STRT: u32 = 1 << 31;
/// PCR fields (SPRUEG2E Table 32): PE = 10b lets the event ID select the channel, SYNC asks for
/// synchronised transfers, PRI 11b, ABU auto-buffers, DIR moves data from the peripheral to the
/// switch, the burst lengths are a cell packet's words minus one, the peripheral address is
/// constant and the switch address increments.
const PCR_PE_EVENT_ID: u32 = 2 << 22;
const PCR_SYNC: u32 = 1 << 20;
const PCR_PRI_HIGHEST: u32 = 3 << 18;
const PCR_ABU: u32 = 1 << 17;
const PCR_DIR_PERIPHERAL_TO_SWITCH: u32 = 1 << 16;
const PCR_PBLEN_SHIFT: u32 = 12;
const PCR_PMOD_CONSTANT: u32 = 1 << 11;
const PCR_SBLEN_SHIFT: u32 = 4;
/// Transfer control (SPRUEG2E section 4.5 and Example 2): STRT enables the channel, IE with IMOD
/// asks for an interrupt at the half and the end of the buffer, SINT 0 is the line to core 0's
/// interrupt controller, and both elements are 32 bits (PSIZ = SSIZ = 2).
const TCR_STRT: u32 = 1 << 31;
const TCR_IMOD_HALF_AND_END: u32 = 1 << 28;
const TCR_IE: u32 = 1 << 27;
const TCR_SINT_CORE0: u32 = 0 << 24;
const TCR_PSIZ_32: u32 = 2 << 22;
const TCR_SSIZ_32: u32 = 2 << 20;

/// The cells the port receives: the standard cell, 53 bytes or 54 on the 16-bit bus, and up to
/// 64 with RUDC's user-defined header bytes in front (SPRUEG2E sections 2 and 10, SPRUE48 Table
/// 7). Each lands as a packet padded in front to whole words, so 14 to 16 of them.
const NARROW_CELL_BYTES: u32 = 53;
const WIDE_CELL_BYTES: u32 = 54;
const LONGEST_CELL_BYTES: u32 = 64;
const UCR_RUDC_SHIFT: u32 = 2;
const MAX_PACKET_WORDS: usize = 16;
/// The 16 KB of core 0's L1D the PDMA fills (section 4.3.4).
const BUFFER_SPACE: u32 = 16 * 1024;

/// Core 0's L1D, as the PDMA (a switch master) addresses it and as core 0 does (SPRS612G
/// Table 2-2).
const L1D_GLOBAL: u32 = 0x10f0_0000;
const L1D_LOCAL: u32 = 0x00f0_0000;

/// Local timer 0 (SPRS612G section 7.16.2): the registers it is programmed through (SPRU968)
/// and its low half's system event, TINTL, which is event 16 in core 0 (Table 7-10).
const TIMER0: u32 = 0x025e_0000;
const TIMER_CNTLO: u32 = TIMER0 + 0x10;
const TIMER_PRDLO: u32 = TIMER0 + 0x18;
const TIMER_TCR: u32 = TIMER0 + 0x20;
const TIMER_TGCR: u32 = TIMER0 + 0x24;
/// TGCR: TIMLO out of reset in the unchained 32-bit mode (TIMMODE 01b). TCR: ENAMODE_LO 10b, so
/// the half counts continuously.
const TGCR_LO_UNCHAINED: u32 = (1 << 2) | 1;
const TCR_ENAMODE_LO_CONTINUOUS: u32 = 2 << 6;
/// The timer's input clock is the CPU clock divided by six (SPRS612G section 7.16).
const TIMER_CLOCK_DIVIDER: u32 = 6;
/// Table 26's default core frequency, which a cold boot counts the timer from.
const DEFAULT_CORE_MHZ: u32 = 500;

/// EVTFLAG0 and EVTCLR0, and the two events the ROM sleeps on: TINTL, and the UTOPIA PDMA's
/// interrupt, system event 30 in core 0's interrupt controller (SINT 0).
const EVTFLAG0: u32 = 0x0180_0000;
const EVTCLR0: u32 = 0x0180_0040;
const EVENT_TIMER: u32 = 1 << 16;
const EVENT_PDMA: u32 = 1 << 30;

/// The boot-table cell: 32 bytes of table in front of the 32-bit magic, searched backwards from
/// the end of the payload. A standard payload is 48 bytes, the last twelve words of the packet
/// (SPRUEG2E section 2).
const TABLE_BYTES: usize = 32;
const TABLE_WORDS: usize = TABLE_BYTES / 4;
const PAYLOAD_WORDS: usize = 48 / 4;
const MAGIC: u32 = 0x5449_2164;

/// Table 26's fields, as offsets into a boot parameter block whose first ten bytes are Table
/// 23's common header. Every field is 16 bits, most significant byte first.
const PARAM_OPTIONS: usize = 10;
const PARAM_CELL_SIZE: usize = 12;
const PARAM_BUS_WIDTH: usize = 14;
const PARAM_SLAVE_ID: usize = 16;
const PARAM_CORE_MHZ: usize = 18;
const PARAM_BYTES: usize = 20;
const OPTION_SINGLE_PHY: u16 = 1 << 0;
const OPTION_WIDE: u16 = 1 << 1;
const OPTION_SKIP_PORT_INIT: u16 = 1 << 2;

/// SPRUEC6G Table 26, the UTOPIA boot parameter table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Parameters {
    /// Options bit 0: a single-PHY port rather than the default multi-PHY one.
    pub single_phy: bool,
    /// A 16-bit bus rather than an 8-bit one.
    pub wide: bool,
    /// Options bit 2: leave the port as it is instead of initialising it.
    pub skip_port_init: bool,
    /// The physical cell size in bytes.
    pub cell_bytes: u32,
    /// The PHY address the port answers in multi-PHY mode.
    pub slave_id: u8,
    /// The core frequency after PLL configuration, which the timer's period is counted from.
    pub core_mhz: u32,
}

impl Parameters {
    /// Table 26's defaults, as a cold boot takes them from the pins: BOOTMODE[3:0] names the bus
    /// width and CFGGP[4:0] the slave ID (Table 22).
    pub(crate) fn cold(mode: u8, phy: u8) -> Self {
        debug_assert!(is_boot_mode(mode), "not a UTOPIA boot mode");
        let wide = matches!(mode, 7 | 8);
        Self {
            single_phy: false,
            wide,
            skip_port_init: false,
            cell_bytes: standard_cell_bytes(wide),
            slave_id: phy & 0x1f,
            core_mhz: DEFAULT_CORE_MHZ,
        }
    }

    /// The table as a boot parameter block carries it. A field the port cannot be set up from
    /// stops the boot with the reason.
    pub(crate) fn from_block(block: &[u8]) -> Result<Self, &'static str> {
        if block.len() < PARAM_BYTES {
            return Err("UTOPIA boot parameter block is shorter than Table 26");
        }
        let field = |offset: usize| u16::from_be_bytes([block[offset], block[offset + 1]]);
        let options = field(PARAM_OPTIONS);
        let wide = options & OPTION_WIDE != 0;
        match (field(PARAM_BUS_WIDTH), wide) {
            (8, false) | (16, true) => {}
            (8 | 16, _) => {
                return Err(
                    "UTOPIA boot parameters name one bus width in the options and another \
                            in the bus width field",
                )
            }
            _ => return Err("UTOPIA boot parameters name a bus width other than 8 or 16 bits"),
        }
        let cell_bytes = u32::from(field(PARAM_CELL_SIZE));
        if !(standard_cell_bytes(wide)..=LONGEST_CELL_BYTES).contains(&cell_bytes) {
            return Err("UTOPIA boot cell size is outside what the port receives on this bus");
        }
        let slave_id = field(PARAM_SLAVE_ID);
        if slave_id > 0x1f {
            return Err("UTOPIA boot slave ID does not fit the five-bit PHY address");
        }
        let core_mhz = u32::from(field(PARAM_CORE_MHZ));
        if core_mhz == 0 {
            return Err("UTOPIA boot core frequency is zero, so the timer has no period");
        }
        Ok(Self {
            single_phy: options & OPTION_SINGLE_PHY != 0,
            wide,
            skip_port_init: options & OPTION_SKIP_PORT_INIT != 0,
            cell_bytes,
            slave_id: slave_id as u8,
            core_mhz,
        })
    }

    /// The words a cell occupies in the buffer: the wire cell padded in front to whole words.
    fn packet_words(&self) -> usize {
        self.cell_bytes.div_ceil(4) as usize
    }

    fn packet_bytes(&self) -> u32 {
        4 * self.packet_words() as u32
    }

    /// How many packets fit in the 16 KB buffer: "256 to 292 cells" (section 4.3.4).
    pub(crate) fn buffer_cells(&self) -> u32 {
        BUFFER_SPACE / self.packet_bytes()
    }

    /// The user-defined header bytes in front of the standard cell, UCR.RUDC.
    fn user_defined_bytes(&self) -> u32 {
        self.cell_bytes - standard_cell_bytes(self.wide)
    }

    /// Timer 0's period register: the timer clocks in 100 us (50 us on the 16-bit bus) less
    /// one, because the counter restarts from zero the clock after it matches (SPRU968 Table 12).
    fn timer_period(&self) -> u32 {
        let microseconds = if self.wide { 50 } else { 100 };
        self.core_mhz * microseconds / TIMER_CLOCK_DIVIDER - 1
    }
}

fn standard_cell_bytes(wide: bool) -> u32 {
    if wide {
        WIDE_CELL_BYTES
    } else {
        NARROW_CELL_BYTES
    }
}

/// What the boot has done so far, for a front end to report.
#[derive(Clone, Default, Debug, PartialEq, Eq)]
pub(crate) struct Progress {
    /// Cells that carried the magic and whose 32 table bytes were processed.
    pub accepted: u64,
    /// Cells without the magic, which are dropped and counted.
    pub discarded: u64,
    /// Table bytes handed to boot-table processing.
    pub table_bytes: u64,
    /// Where core 0 was sent, once the table has ended.
    pub entry: Option<u32>,
    /// Why the boot stopped, when it stopped without an entry point.
    pub error: Option<String>,
}

enum Phase {
    /// Megamodule 0 and the UTOPIA must be powered before they are used; `requested` is whether
    /// the ROM has asked for the transition and is waiting for it.
    PowerUp { requested: bool },
    /// Configured and asleep: cells are looked for when an interrupt flag says the PDMA or the
    /// timer woke the ROM.
    Wait,
    /// The table ended and core 0 has been given its entry point.
    Done,
    /// The table could not be processed. The core stays held, as it does when a ROM boot gives up.
    Failed,
}

pub(crate) struct UtopiaBoot {
    parameters: Parameters,
    /// The byte order the ROM reads its buffer in.
    endian: Endian,
    /// The PLL1 multiplier a cold boot's BOOTMODE selects: x10 for modes 5 and 7, x20 for 6 and
    /// 8. A boot an I2C boot handed over to leaves PLL1 as that boot set it.
    pll_multiplier: Option<u32>,
    phase: Phase,
    stream: BootStream,
    /// The next cell packet the ROM reads out of the circular buffer.
    next_cell: u32,
    progress: Progress,
}

impl UtopiaBoot {
    /// The boot as the pins strap it: BOOTMODE[3:0] picks the port width and the PLL
    /// multiplier, and CFGGP[4:0] is the PHY ID the port answers to (SPRUEC6G Table 22).
    pub(crate) fn cold(mode: u8, phy: u8, endian: Endian) -> Self {
        let multiplier = if matches!(mode, 5 | 7) { 10 } else { 20 };
        Self::new(Parameters::cold(mode, phy), Some(multiplier), endian)
    }

    /// The boot an I2C boot hands over to, with the parameter table it read (Table 24's
    /// extended boot mode 0x104).
    pub(crate) fn chained(parameters: Parameters, endian: Endian) -> Self {
        Self::new(parameters, None, endian)
    }

    fn new(parameters: Parameters, pll_multiplier: Option<u32>, endian: Endian) -> Self {
        Self {
            parameters,
            pll_multiplier,
            phase: Phase::PowerUp { requested: false },
            stream: BootStream::new(endian),
            endian,
            next_cell: 0,
            progress: Progress::default(),
        }
    }

    pub(crate) fn progress(&self) -> Progress {
        self.progress.clone()
    }

    /// Whether the boot runs a 16-bit port.
    pub(crate) fn is_wide(&self) -> bool {
        self.parameters.wide
    }

    /// One cycle of the ROM's boot loop. Returns the entry point once the boot table has ended,
    /// which is the device's signal to start core 0 there.
    pub(crate) fn tick(&mut self, fabric: &mut Fabric) -> Option<u32> {
        // The boot code runs on core 0, so its accesses are core 0's accesses.
        let mut bus = CoreBus::new(0, fabric);
        match self.phase {
            Phase::PowerUp { requested } => {
                if self.powered(&mut bus, requested) {
                    self.configure(&mut bus);
                    self.phase = Phase::Wait;
                }
                None
            }
            Phase::Wait => self.wake(&mut bus),
            Phase::Done | Phase::Failed => None,
        }
    }

    /// Whether megamodule 0 and the UTOPIA are enabled, asking the power/sleep controller to
    /// enable whichever is not (SPRUEG3B section 4.2: stage NEXT in MDCTL, then GO in PTCMD).
    fn powered(&mut self, bus: &mut CoreBus<'_>, requested: bool) -> bool {
        let mut waiting = bus.read32(PSC_PTSTAT) & PSC_GO_DOMAIN0 != 0;
        let mut staged = false;
        for module in PSC_MODULES {
            if bus.read32(psc_status(module)) & PSC_STATE_MASK != PSC_STATE_ENABLE {
                waiting = true;
                if !requested {
                    bus.write32(psc_control(module), PSC_LRSTZ | PSC_STATE_ENABLE);
                    staged = true;
                }
            }
        }
        if staged {
            bus.write32(PSC_PTCMD, PSC_GO_DOMAIN0);
            self.phase = Phase::PowerUp { requested: true };
        }
        !waiting
    }

    /// Everything the ROM programs before it sleeps (Table 20 and section 4.3.4), in the order
    /// SPRUEG2E section 13 gives for a UTOPIA slave: the PDMA channel first, then the port's
    /// registers, and the receiver is taken out of reset last.
    fn configure(&mut self, bus: &mut CoreBus<'_>) {
        let parameters = self.parameters;
        if let Some(multiplier) = self.pll_multiplier {
            // The multiplier the mode names. A chained boot's I2C boot has already done this.
            initialise_c6472_rom_boot(bus, multiplier);
        }

        // PDMA channel 8: every cell goes to a circular buffer in L1D, a cell packet per event.
        let burst = parameters.packet_words() as u32 - 1;
        let packet = parameters.packet_bytes();
        bus.write32(
            PDMA_PCR1,
            PCR_PE_EVENT_ID
                | PCR_SYNC
                | PCR_PRI_HIGHEST
                | PCR_ABU
                | PCR_DIR_PERIPHERAL_TO_SWITCH
                | (burst << PCR_PBLEN_SHIFT)
                | PCR_PMOD_CONSTANT
                | (burst << PCR_SBLEN_SHIFT),
        );
        bus.write32(PDMA_RXC0 + PROXY_SAR, L1D_GLOBAL);
        bus.write32(PDMA_RXC0 + PROXY_PAR, RECEIVE_QUEUE_PORT);
        bus.write32(
            PDMA_RXC0 + PROXY_BSR,
            (packet << 16) | (parameters.buffer_cells() * packet),
        );
        bus.write32(
            PDMA_RXC0 + PROXY_TCR,
            TCR_STRT | TCR_IMOD_HALF_AND_END | TCR_IE | TCR_SINT_CORE0 | TCR_PSIZ_32 | TCR_SSIZ_32,
        );
        bus.write32(PDMA_GCSR, GCSR_STRT);

        // The port: slave, big-endian cell assembly at the slave ID, multi-PHY unless the
        // options ask for single-PHY, the bus width, and the user-defined header bytes that make
        // the physical cell size.
        if !parameters.skip_port_init {
            let mut control = UCR_BEND
                | (u32::from(parameters.slave_id) << UCR_SLID_SHIFT)
                | (parameters.user_defined_bytes() << UCR_RUDC_SHIFT);
            if !parameters.single_phy {
                control |= UCR_MPHY;
            }
            if parameters.wide {
                control |= UCR_U16M;
            }
            bus.write32(UCR, control);
            bus.write32(UCR, control | UCR_UREN);
        }

        // Local timer 0: a 32-bit periodic timer that interrupts every 100 us (50 us in 16-bit
        // mode), so a cell that arrived without crossing a buffer mark is still found.
        bus.write32(TIMER_TGCR, 0);
        bus.write32(TIMER_CNTLO, 0);
        bus.write32(TIMER_PRDLO, parameters.timer_period());
        bus.write32(TIMER_TCR, TCR_ENAMODE_LO_CONTINUOUS);
        bus.write32(TIMER_TGCR, TGCR_LO_UNCHAINED);
        // Interrupts the port or the timer raised on the way up are not wake-ups.
        bus.write32(EVTCLR0, EVENT_PDMA | EVENT_TIMER);
    }

    /// The ROM is asleep until an interrupt: look at what woke it, acknowledge it, and then go
    /// through every cell the PDMA has delivered.
    fn wake(&mut self, bus: &mut CoreBus<'_>) -> Option<u32> {
        let woken = bus.read32(EVTFLAG0) & (EVENT_PDMA | EVENT_TIMER);
        if woken == 0 {
            return None;
        }
        // Acknowledge before looking, so a cell that lands while the buffer is being read raises
        // the flag again rather than being forgotten.
        bus.write32(EVTCLR0, woken);
        self.drain(bus)
    }

    /// Process every complete cell between the ROM's read position and the PDMA's write
    /// position, which the channel's switch address register reads back as (SPRUEG2E section 5:
    /// reading a channel's proxy block loads it from the live context).
    fn drain(&mut self, bus: &mut CoreBus<'_>) -> Option<u32> {
        let packet = self.parameters.packet_bytes();
        let cells = self.parameters.buffer_cells();
        let written = bus.read32(PDMA_RXC0 + PROXY_SAR).wrapping_sub(L1D_GLOBAL);
        let write_cell = (written / packet) % cells;
        let mut words = [0u32; MAX_PACKET_WORDS];
        let words = &mut words[..self.parameters.packet_words()];
        while self.next_cell != write_cell {
            let base = L1D_LOCAL + self.next_cell * packet;
            for (index, word) in words.iter_mut().enumerate() {
                *word = bus.read32(base + 4 * index as u32);
            }
            self.next_cell = (self.next_cell + 1) % cells;
            if let Some(entry) = self.process(bus, words) {
                return Some(self.finish(bus, entry));
            }
            if matches!(self.phase, Phase::Failed) {
                break;
            }
        }
        None
    }

    /// Validate one cell and hand its table bytes to boot-table processing.
    fn process(&mut self, bus: &mut CoreBus<'_>, words: &[u32]) -> Option<u32> {
        let Some(table) = table_bytes(words, self.endian) else {
            self.progress.discarded += 1;
            return None;
        };
        self.progress.accepted += 1;
        self.progress.table_bytes += TABLE_BYTES as u64;
        match self.stream.feed(&table, bus) {
            Ok(true) => self.stream.entry(),
            Ok(false) => None,
            Err(message) => {
                // A table this malformed cannot name an entry point: the ROM has nothing to
                // branch to, and the boot stops with the core still held.
                self.progress.error = Some(message);
                self.phase = Phase::Failed;
                None
            }
        }
    }

    /// The table ended: disable every receive PDMA channel and the timer, and send core 0 to the
    /// entry point the table named (section 4.3.4).
    fn finish(&mut self, bus: &mut CoreBus<'_>, entry: u32) -> u32 {
        for channel in 0..PDMA_RX_CHANNELS {
            // A transfer control write with STRT clear disables the channel whatever the other
            // proxies hold (SPRUEG2E section 5).
            bus.write32(PDMA_RXC0 + channel * PDMA_PROXY_STRIDE + PROXY_TCR, 0);
        }
        bus.write32(TIMER_TCR, 0);
        bus.write32(TIMER_TGCR, 0);
        bus.write32(EVTCLR0, EVENT_PDMA | EVENT_TIMER);
        self.progress.entry = Some(entry);
        self.phase = Phase::Done;
        entry
    }
}

/// The 32 table bytes of a cell packet as L1D holds them, or `None` for a cell without the
/// magic.
///
/// The payload is the last twelve words of the packet. It is searched from its end towards its
/// start for the magic word. The eight preceding words, still inside the payload, are the table
/// bytes, in the device's byte order as they sit in memory.
fn table_bytes(words: &[u32], endian: Endian) -> Option<[u8; TABLE_BYTES]> {
    let payload = words.len() - PAYLOAD_WORDS;
    let magic = (payload + TABLE_WORDS..words.len())
        .rev()
        .find(|&index| words[index] == MAGIC)?;
    let mut table = [0u8; TABLE_BYTES];
    for (chunk, word) in table
        .chunks_exact_mut(4)
        .zip(&words[magic - TABLE_WORDS..magic])
    {
        chunk.copy_from_slice(&match endian {
            Endian::Little => word.to_le_bytes(),
            Endian::Big => word.to_be_bytes(),
        });
    }
    Some(table)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A standard cell's packet, 14 words.
    fn packet(words: &[(usize, u32)]) -> [u32; 14] {
        let mut packet = [0u32; 14];
        for &(index, value) in words {
            packet[index] = value;
        }
        packet
    }

    #[test]
    fn the_four_modes_pick_the_port_width_and_the_pll_multiplier() {
        for (mode, wide, multiplier) in
            [(5, false, 10), (6, false, 20), (7, true, 10), (8, true, 20)]
        {
            assert!(is_boot_mode(mode));
            let boot = UtopiaBoot::cold(mode, 3, Endian::Little);
            assert_eq!(boot.is_wide(), wide, "mode {mode}");
            assert_eq!(boot.pll_multiplier, Some(multiplier), "mode {mode}");
        }
        for mode in [0, 1, 2, 4, 9, 10, 15] {
            assert!(!is_boot_mode(mode), "mode {mode}");
        }
    }

    #[test]
    fn the_table_is_the_thirty_two_bytes_in_front_of_the_magic_as_memory_holds_them() {
        let table: Vec<u32> = (0..8).map(|index| 0x0102_0304 * (index + 1)).collect();
        let mut words = vec![0x1111_1111, 0x2222_2222];
        words.extend(&table);
        words.extend([MAGIC, 0xffff_ffff, 0xeeee_eeee, 0xdddd_dddd]);
        let words: [u32; 14] = words.try_into().unwrap();
        for (endian, bytes) in [
            (Endian::Little, u32::to_le_bytes as fn(u32) -> [u8; 4]),
            (Endian::Big, u32::to_be_bytes),
        ] {
            let found = table_bytes(&words, endian).expect("a cell with the magic");
            let expected: Vec<u8> = table.iter().flat_map(|&word| bytes(word)).collect();
            assert_eq!(
                found.to_vec(),
                expected,
                "big-endian: {}",
                endian == Endian::Big
            );
        }
    }

    #[test]
    fn the_search_runs_from_the_end_of_the_payload() {
        // Two magic words: the later one is found first, so the table is the eight words in
        // front of it, and the earlier magic is table data.
        let mut words = packet(&[(9, MAGIC), (10, 0xb000_0000)]);
        words[11] = MAGIC;
        let bytes = table_bytes(&words, Endian::Big).unwrap();
        assert_eq!(&bytes[..4], &words[3].to_be_bytes());
        assert_eq!(&bytes[24..28], &MAGIC.to_be_bytes());
        assert_eq!(&bytes[28..], &0xb000_0000u32.to_be_bytes());
    }

    #[test]
    fn a_cell_without_the_magic_or_without_room_for_the_table_is_refused() {
        let find = |words: &[(usize, u32)]| table_bytes(&packet(words), Endian::Little);
        assert_eq!(find(&[]), None);
        // Wrong magic, in place.
        assert_eq!(find(&[(10, 0x5449_2165)]), None);
        // The magic in the header words has no 32 payload bytes in front of it.
        assert_eq!(find(&[(1, MAGIC)]), None);
        assert_eq!(find(&[(9, MAGIC)]), None);
        // The earliest position that leaves a whole table inside the payload.
        assert!(find(&[(10, MAGIC)]).is_some());
    }

    /// A Table 26 block: the ten-byte common header, then the fields big-endian.
    fn block(options: u16, cell: u16, width: u16, slave: u16, mhz: u16) -> Vec<u8> {
        let mut block = vec![0u8; PARAM_BYTES];
        for (offset, value) in [
            (PARAM_OPTIONS, options),
            (PARAM_CELL_SIZE, cell),
            (PARAM_BUS_WIDTH, width),
            (PARAM_SLAVE_ID, slave),
            (PARAM_CORE_MHZ, mhz),
        ] {
            block[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
        }
        block
    }

    /// "The actual number of cells which can be buffered varies by the cell size parameter in
    /// the boot parameter table, and ranges from 256 to 292 cells" (section 4.3.4).
    #[test]
    fn the_cell_size_sets_how_many_cells_the_buffer_holds() {
        for mode in 5..=8 {
            let cold = Parameters::cold(mode, 3);
            assert_eq!(cold.buffer_cells(), 292, "mode {mode}");
            assert_eq!(cold.user_defined_bytes(), 0, "mode {mode}");
            assert_eq!(cold.core_mhz, 500);
        }
        for (options, width, cell, cells, extra) in [
            (0, 8, 53, 292, 0),
            (0, 8, 57, 273, 4),
            (0, 8, 64, 256, 11),
            (OPTION_WIDE, 16, 54, 292, 0),
            (OPTION_WIDE, 16, 60, 273, 6),
            (OPTION_WIDE, 16, 64, 256, 10),
        ] {
            let parameters = Parameters::from_block(&block(options, cell, width, 7, 500)).unwrap();
            assert_eq!(
                parameters.buffer_cells(),
                cells,
                "{width}-bit {cell}-byte cell"
            );
            assert_eq!(
                parameters.user_defined_bytes(),
                extra,
                "{width}-bit {cell}-byte cell"
            );
        }
    }

    #[test]
    fn the_options_and_core_frequency_reach_the_parameters() {
        let options = OPTION_SINGLE_PHY | OPTION_WIDE | OPTION_SKIP_PORT_INIT;
        let parameters = Parameters::from_block(&block(options, 54, 16, 0x1e, 250)).unwrap();
        assert!(parameters.single_phy && parameters.wide && parameters.skip_port_init);
        assert_eq!(parameters.slave_id, 0x1e);
        // 50 us of a 250 MHz core's CPU/6 timer clocks, less one.
        assert_eq!(parameters.timer_period(), 2082);
        assert_eq!(Parameters::cold(5, 0).timer_period(), 8332);
    }

    #[test]
    fn a_table_the_port_cannot_be_set_up_from_stops_the_boot() {
        for (block, why) in [
            (block(0, 53, 16, 0, 500), "options say 8 bits, the field 16"),
            (
                block(OPTION_WIDE, 54, 8, 0, 500),
                "options say 16 bits, the field 8",
            ),
            (block(0, 53, 32, 0, 500), "no 32-bit UTOPIA"),
            (
                block(OPTION_WIDE, 53, 16, 0, 500),
                "shorter than the wide standard cell",
            ),
            (block(0, 52, 8, 0, 500), "shorter than the standard cell"),
            (block(0, 65, 8, 0, 500), "longer than the port takes"),
            (block(0, 53, 8, 0x20, 500), "slave ID beyond five bits"),
            (block(0, 53, 8, 0, 0), "no core frequency"),
            (vec![0; PARAM_BYTES - 1], "a block that ends early"),
        ] {
            assert!(Parameters::from_block(&block).is_err(), "{why}");
        }
    }
}
