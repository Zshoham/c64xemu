//! The C6455's UTOPIA 2 ATM slave interface.
//!
//! SPRUE48: the port is configured through UCR, watches for the master's clocks through CDR, and
//! reports errors through the EIER/EIPR pair behind CPU interrupt UINT. The transmit and receive
//! queues sit in their own data-port windows and drive EDMA3 synchronisation events UXEVT and
//! UREVT.
//!
//! The two clock-detect bits still say what they said: no master drives UXCLK or URCLK on this
//! model, so RCPP/XCPP stay clear and an enabled section with a non-zero clock count reaches the
//! clock-failure condition after that many peripheral clocks (SPRUE48 Table 8). The peripheral
//! clock is SYSCLK3, the CPU clock divided by six (SPRS276M section 7.6), the same clock the
//! timers, GPIO and I2C use.
//!
//! The cell path is modelled against an external ATM master that pulls cells out of the transmit
//! queue and pushes cells into the receive queue. Both queues hold two cells (SPRS276M section
//! 7.19.1), a standard cell is 53 bytes of which the queue packet is 14 words, and XUDC/RUDC add
//! one to eleven header bytes, taking the packet to 15 or 16 words (SPRUE48 sections 3.3, 3.5 and
//! Table 7). The events follow SPRUE48 sections 3.3 and 3.5 exactly: UXEVT is asserted while the
//! transmit queue has room for a cell and drops on the first write of a cell, UREVT is asserted
//! while a complete cell is waiting and drops on the first read of one. A write to a full
//! transmit queue is held rather than dropped - the manual says the write stalls until space
//! appears and that data is not overwritten - and XQSP reports it meanwhile; a read from an empty
//! receive queue raises RQSP.
//!
//! The master reaches the queues through the UTOPIA Level 2 address phase (SPRUE48 sections 3.1,
//! 3.2, 3.4 and 3.7). In multi-PHY mode (UCR.MPHY, the reset state) the slave answers only the
//! address UCR.SLID names: polled at that address it drives UXCLAV/URCLAV with whether it has a
//! cell to send or room for one, and selected there it transfers; at any other address its
//! outputs stay high-impedance and the master's transfer goes to some other slave. 1Fh is the
//! null PHY address, which the address pins are pulled to at reset (Table 4) and no slave
//! answers. In single-PHY mode the address bus is not decoded at all.
//!
//! The C6472 has the same port with the additions SPRUEG2E describes, which its device profile
//! names through `UtopiaProfile::pdma`. UCR gains U16M, which makes the standard cell 54 bytes
//! (a six-byte header) on the 16-bit bus. The receive routing unit comes with it: RRSR picks a
//! 16-bit field of the cell and the six mask-and-match registers RMMR0-5 decide which of six
//! PDMA events a cell raises, the lowest match winning and a cell that matches none belonging
//! to some other device and being dropped. And its queues are serviced by the UTOPIA PDMA
//! (`pdma.rs`) rather than EDMA3: the PDMA reads receive cell packets out and writes transmit
//! ones in over its own peripheral bus, on the same UREVT and UXEVT. SPRUEG2E Table 18 gives the
//! data ports as offsets (URQ 0, UXQ 4) and the C6472 data manual maps no window for them -
//! 0x2000_0000-0x77FF_FFFF, where the C6455's 0x3C00_0000 ports sit, is reserved (SPRS612G
//! Table 2-2) - so its profile names no CPU windows and the queues are reachable only through
//! the PDMA.
//!
//! On both devices a cell packet is laid out as SPRUE48 Figures 9-14 and SPRUEG2E Tables 6-15
//! draw it: dummy bytes *before* the header pad the wire cell to whole words, so both standard
//! cells put the 48-byte payload at packet byte 8.
use crate::port::{PortContext, PortWrite, SlavePort};
use crate::sysclk3::Sysclk3Block;
use std::collections::VecDeque;
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const BASE: u32 = 0x02b4_0000;
const UCR: u32 = BASE;
const CDR: u32 = BASE + 0x14;
const EIER: u32 = BASE + 0x18;
const EIPR: u32 = BASE + 0x1c;
/// The C6472's routing unit (SPRUEG2E Table 17; SPRS612G Table 7-142 lists RRMR0-5 only) and its
/// two trailing registers, which the data manual puts at 0x100 and 0x104.
const RRMR0: u32 = BASE + 0x20;
const RRMR_COUNT: usize = 6;
const RRSR: u32 = BASE + 0x40;
const UPIDR: u32 = BASE + 0x100;
const UPWREMU: u32 = BASE + 0x104;
/// UPIDR: type 3 in 23-16, class 3 in 15-8 and revision 1 in 7-0 (Table 25).
const UPIDR_VALUE: u32 = (3 << 16) | (3 << 8) | 1;
/// UPWREMU's SOFT and FREE bits. They are stored; the emulation halt they steer never happens.
const UPWREMU_WRITABLE: u32 = 0b11;
/// RRSR: RBYT in bits 21-16 and RBIT in 2-0 (Table 24).
const RRSR_WRITABLE: u32 = (0x3f << 16) | 0x7;
const RRSR_BYTE_SHIFT: u32 = 16;

/// The receive and transmit data ports on the C6455, a 1 KB window each (SPRS276M Table 2-2).
const C6455_RECEIVE_QUEUE: u32 = 0x3c00_0000;
const QUEUE_WINDOW: u32 = 0x400;

/// UCR's fields (SPRUE48 Figure 15 and Table 7): BEND in bit 31 selects big-endian cell assembly,
/// bits 30-29 read back the reserved one, SLID in 28-24 is the multi-PHY slave address, XUDC in
/// 21-18 and RUDC in 5-2 are the user-defined-cell header bytes, UXEN in 16 and UREN in 0 take
/// each section out of reset, and MPHY in 14 comes up set. Everything else resets clear.
const UCR_BEND: u32 = 1 << 31;
const UCR_RESERVED_ONES: u32 = 1 << 29;
const UCR_SLID_SHIFT: u32 = 24;
const UCR_XUDC_SHIFT: u32 = 18;
const UCR_UXEN: u32 = 1 << 16;
const UCR_MPHY: u32 = 1 << 14;
const UCR_RUDC_SHIFT: u32 = 2;
const UCR_UREN: u32 = 1 << 0;
/// The C6472's U16M in bit 13 selects the 16-bit bus; SPRUEG2E Figure 9 draws it reset to one.
const UCR_U16M: u32 = 1 << 13;
/// SLID is five bits, XUDC and RUDC four.
const UCR_SLID_BITS: u32 = 0x1f;
const UCR_UDC_BITS: u32 = 0xf;
const UCR_WRITABLE: u32 = UCR_BEND
    | (UCR_SLID_BITS << UCR_SLID_SHIFT)
    | (UCR_UDC_BITS << UCR_XUDC_SHIFT)
    | UCR_UXEN
    | UCR_MPHY
    | (UCR_UDC_BITS << UCR_RUDC_SHIFT)
    | UCR_UREN;
const UCR_RESET: u32 = UCR_RESERVED_ONES | UCR_MPHY;
/// The address no PHY answers (SPRUE48 Table 7), and where the pulled-up pins sit (Table 4).
pub(crate) const NULL_PHY: u8 = 0x1f;

/// Both clock counts reset to the maximum wait of 255 peripheral clocks (SPRUE48 Figure 16).
const CDR_RESET: u32 = (0xff << 16) | 0xff;
const CDR_WRITABLE: u32 = (0xff << 16) | 0xff;

/// EIPR/EIER share a layout: transmit clock present, clock failed and queue stall in bits 18-16,
/// the receive trio in bits 2-0 (SPRUE48 Figures 17 and 18).
const XCPP: u32 = 1 << 18;
const XCFP: u32 = 1 << 17;
const XQSP: u32 = 1 << 16;
const RCPP: u32 = 1 << 2;
const RCFP: u32 = 1 << 1;
const RQSP: u32 = 1 << 0;
const ERROR_MASK: u32 = XCPP | XCFP | XQSP | RCPP | RCFP | RQSP;
/// Only the four clock bits are write-one-to-clear; the two stall bits are read-only and the
/// interface clears them itself (SPRUE48 section 12.4).
const EIPR_CLEARABLE: u32 = XCPP | XCFP | RCPP | RCFP;

#[cfg(test)]
const CLOCK_DIVIDER: u32 = 6;
/// A standard ATM cell is a five-byte header and a 48-byte payload (SPRUE48 section 2). On the
/// C6472's 16-bit bus it is 54 bytes: the header grows a second UDF byte (SPRUEG2E Table 12).
pub(crate) const CELL_BYTES: usize = 53;
pub(crate) const WIDE_CELL_BYTES: usize = 54;
/// The longest cell packet: 53 wire bytes and eleven user-defined ones, or 54 and ten on the
/// wide bus, is 64 bytes in 16 words (SPRUE48 Table 7, SPRUEG2E section 10).
pub(crate) const MAX_CELL_BYTES: usize = 64;
const MAX_PACKET_WORDS: usize = MAX_CELL_BYTES / 4;
/// Both queues hold two cells (SPRS276M section 7.19.1).
const QUEUE_CELLS: usize = 2;

/// The words a cell packet occupies: the wire cell rounded up to a word (SPRUE48 section 4.1).
fn packet_words(wire_bytes: usize) -> usize {
    wire_bytes.div_ceil(4)
}

/// What one device's UTOPIA port has: a `DeviceProfile` field.
#[derive(Debug, PartialEq, Eq)]
pub(crate) struct UtopiaProfile {
    /// The base of the CPU's 1 KB window onto the receive queue, with the transmit queue's
    /// window following it; `None` where only a DMA controller reaches the queues.
    pub(crate) queue_windows: Option<u32>,
    /// The SPRUEG2E port: U16M, the receive routing unit with UPIDR and UPWREMU, and the
    /// six-channel PDMA at 02BC 0000h that services the queues in place of EDMA3 events. One
    /// field because the silicon gives one fact: SPRUEG2E documents them as one peripheral.
    pub(crate) pdma: bool,
}

pub(crate) const C6455_UTOPIA: UtopiaProfile = UtopiaProfile {
    queue_windows: Some(C6455_RECEIVE_QUEUE),
    pdma: false,
};

pub(crate) const C6472_UTOPIA: UtopiaProfile = UtopiaProfile {
    queue_windows: None,
    pdma: true,
};

/// What one peripheral clock of the interface produced.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct UtopiaEvents {
    /// UXEVT: the transmit queue has room for a cell packet (SPRUE48 Table 2).
    pub transmit: bool,
    /// UREVT: a complete cell is waiting in the receive queue.
    pub receive: bool,
}

/// One cell packet in a queue, held in place: no queue holds more than two of them.
#[derive(Clone, Copy, Default)]
#[cfg_attr(test, derive(Debug, PartialEq))]
struct Packet {
    words: [u32; MAX_PACKET_WORDS],
    len: usize,
}

impl Packet {
    fn words(&self) -> &[u32] {
        &self.words[..self.len]
    }

    fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// A cell as the master sees it on the wire: the packet without its dummy bytes.
#[derive(Clone, Copy)]
pub(crate) struct WireCell {
    bytes: [u8; MAX_CELL_BYTES],
    len: usize,
}

impl WireCell {
    pub(crate) fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// A cell in the receive queue: its packet, and which of the routing unit's six events it
/// raises.
#[cfg_attr(test, derive(Clone, Debug, PartialEq))]
struct ReceiveCell {
    route: usize,
    packet: Packet,
}

#[cfg_attr(test, derive(Clone, Debug, PartialEq))]
pub struct Utopia {
    profile: &'static UtopiaProfile,
    control: u32,
    clock_detect: u32,
    enable: u32,
    pending: u32,
    /// The receive routing unit's mask-and-match registers and select register (C6472).
    routing_matches: [u32; RRMR_COUNT],
    routing_select: u32,
    power_emulation: u32,
    /// Peripheral clocks since each section last saw a clock edge - which never happens here.
    transmit_idle: u32,
    receive_idle: u32,
    /// Complete cell packets waiting for the master, and the packet being written.
    #[cfg(test)]
    divide: u32,
    transmit_cells: VecDeque<Packet>,
    transmit_partial: Packet,
    /// A write that arrived with the queue full: the bus stalls, so exactly one word can be in
    /// flight, and it lands as soon as the master drains a cell.
    stalled_write: Option<u32>,
    /// Cells the master has sent, and how far the reader has got through the head one.
    receive_cells: VecDeque<ReceiveCell>,
    receive_read: usize,
    /// Whether each event is currently asserted: a cell's first access clears it, and it comes
    /// back when the queue is ready again (SPRUE48 sections 3.3 and 3.5).
    transmit_event: bool,
    receive_event: bool,
    /// How many times UREVT and UXEVT have been asserted, so the PDMA can tell a new event from
    /// the same one still waiting for a channel.
    receive_generation: u64,
    transmit_generation: u64,
}

impl Default for Utopia {
    fn default() -> Self {
        Self::new(&C6455_UTOPIA)
    }
}

impl SlavePort for Utopia {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        if self.receive_window().contains(&address) {
            return Some(self.read_receive_queue());
        }
        if self.transmit_window().contains(&address) {
            // The transmit queue is write-only to the CPU and the EDMA; reads see nothing.
            return Some(0);
        }
        let value = match address {
            UCR => self.control,
            CDR => self.clock_detect,
            EIER => self.enable,
            EIPR => self.pending,
            _ if self.profile.pdma => match address {
                RRSR => self.routing_select,
                UPIDR => UPIDR_VALUE,
                UPWREMU => self.power_emulation,
                _ => self
                    .routing_match_index(address)
                    .map(|i| self.routing_matches[i])?,
            },
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if self.receive_window().contains(&address) {
            return PortWrite::Done;
        }
        if self.transmit_window().contains(&address) {
            self.write_transmit_queue(value);
            return PortWrite::Done;
        }
        match address {
            UCR => {
                let previous = self.control;
                self.control = UCR_RESERVED_ONES | (value & self.control_writable());
                // Enabling a section takes it out of reset, which restarts its clock watchdog.
                if self.control & UCR_UXEN != 0 && previous & UCR_UXEN == 0 {
                    self.transmit_idle = 0;
                }
                if self.control & UCR_UREN != 0 && previous & UCR_UREN == 0 {
                    self.receive_idle = 0;
                }
            }
            CDR => self.clock_detect = value & CDR_WRITABLE,
            EIER => self.enable = value & ERROR_MASK,
            EIPR => self.pending &= !(value & EIPR_CLEARABLE),
            _ if self.profile.pdma => match address {
                RRSR => self.routing_select = value & RRSR_WRITABLE,
                // The ID register is constant, whatever Figure 15 labels its fields.
                UPIDR => {}
                UPWREMU => self.power_emulation = value & UPWREMU_WRITABLE,
                _ => match self.routing_match_index(address) {
                    Some(index) => self.routing_matches[index] = value,
                    None => return PortWrite::Unhandled,
                },
            },
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl Utopia {
    pub(crate) fn new(profile: &'static UtopiaProfile) -> Self {
        Self {
            profile,
            control: if profile.pdma {
                UCR_RESET | UCR_U16M
            } else {
                UCR_RESET
            },
            clock_detect: CDR_RESET,
            enable: 0,
            pending: 0,
            routing_matches: [0; RRMR_COUNT],
            routing_select: 0,
            power_emulation: 0,
            transmit_idle: 0,
            receive_idle: 0,
            transmit_cells: VecDeque::new(),
            #[cfg(test)]
            divide: 0,
            transmit_partial: Packet::default(),
            stalled_write: None,
            receive_cells: VecDeque::new(),
            receive_read: 0,
            transmit_event: true,
            receive_event: false,
            receive_generation: 0,
            transmit_generation: 0,
        }
    }

    fn control_writable(&self) -> u32 {
        if self.profile.pdma {
            UCR_WRITABLE | UCR_U16M
        } else {
            UCR_WRITABLE
        }
    }

    fn receive_window(&self) -> std::ops::Range<u32> {
        self.profile
            .queue_windows
            .map_or(0..0, |base| base..base + QUEUE_WINDOW)
    }

    fn transmit_window(&self) -> std::ops::Range<u32> {
        self.profile
            .queue_windows
            .map_or(0..0, |base| base + QUEUE_WINDOW..base + 2 * QUEUE_WINDOW)
    }

    /// Which of RMMR0-5 a word-aligned address names.
    fn routing_match_index(&self, address: u32) -> Option<usize> {
        let index = (address.wrapping_sub(RRMR0) / 4) as usize;
        (address & 3 == 0 && address >= RRMR0 && index < RRMR_COUNT).then_some(index)
    }

    /// The wire length of a cell with `udc` user-defined header bytes in front: the standard
    /// cell - 53 bytes, or 54 on the C6472's 16-bit bus - plus one to eleven (ten on the wide
    /// bus) bytes (SPRUE48 Table 7, SPRUEG2E sections 2 and 10). U16M is only writable on a
    /// port that has it.
    fn wire_cell_bytes(&self, udc: u32) -> usize {
        let wide = self.control & UCR_U16M != 0;
        let (standard, longest) = if wide {
            (WIDE_CELL_BYTES, 10)
        } else {
            (CELL_BYTES, 11)
        };
        standard + (udc as usize).min(longest)
    }

    /// The wire length of a transmitted cell, which XUDC extends.
    fn transmit_cell_bytes(&self) -> usize {
        self.wire_cell_bytes((self.control >> UCR_XUDC_SHIFT) & UCR_UDC_BITS)
    }

    /// The wire length of a received cell, which RUDC extends.
    fn receive_cell_bytes(&self) -> usize {
        self.wire_cell_bytes((self.control >> UCR_RUDC_SHIFT) & UCR_UDC_BITS)
    }

    /// How many of the two cell slots are taken: complete cells, plus the one being written.
    fn transmit_slots(&self) -> usize {
        self.transmit_cells.len() + usize::from(!self.transmit_partial.is_empty())
    }

    /// One 32-bit write into the transmit queue. A word that completes a packet moves it to the
    /// queue; a word that would need a slot the queue does not have is held, which is what the
    /// stall in SPRUE48 section 12.4 does, and XQSP says so until room appears.
    pub(crate) fn write_transmit_queue(&mut self, value: u32) {
        if self.transmit_partial.is_empty() && self.transmit_slots() >= QUEUE_CELLS {
            self.stalled_write = Some(value);
            self.pending |= XQSP;
            self.transmit_event = false;
            return;
        }
        // The first write of a cell clears the not-full event, which comes back below if the
        // queue still has room (SPRUE48 section 3.3).
        if self.transmit_partial.is_empty() {
            self.transmit_event = false;
        }
        // A UCR change in the middle of a cell can shorten the packet under the words already
        // written; the cell ends there rather than overrunning the slot.
        let packet_len = packet_words(self.transmit_cell_bytes());
        let partial = &mut self.transmit_partial;
        partial.words[partial.len] = value;
        partial.len += 1;
        if partial.len >= packet_len {
            let packet = std::mem::take(&mut self.transmit_partial);
            self.transmit_cells.push_back(packet);
        }
        self.update_transmit_event();
    }

    /// One 32-bit read out of the receive queue. The first read of a cell clears the event, an
    /// exhausted cell leaves the queue, and a read with no cell waiting stalls: RQSP records it
    /// (SPRUE48 sections 3.5 and 12.4).
    pub(crate) fn read_receive_queue(&mut self) -> u32 {
        let Some(cell) = self.receive_cells.front() else {
            self.pending |= RQSP;
            return 0;
        };
        if self.receive_read == 0 {
            self.receive_event = false;
        }
        let words = cell.packet.words();
        let value = words[self.receive_read];
        self.receive_read += 1;
        if self.receive_read == words.len() {
            self.receive_cells.pop_front();
            self.receive_read = 0;
        }
        self.update_receive_event();
        value
    }

    /// UXEVT is asserted while the queue has room for a whole cell, which the packet being
    /// written does not leave: that is what lets the next cell's transfer start before the
    /// current one is fully written (SPRUE48 section 3.3, step 3).
    fn update_transmit_event(&mut self) {
        if self.transmit_slots() < QUEUE_CELLS {
            if !self.transmit_event {
                self.transmit_generation += 1;
            }
            self.transmit_event = true;
        }
    }

    fn update_receive_event(&mut self) {
        // A complete cell is available whenever one is queued, and the reader has not started it.
        if self.receive_read == 0 && !self.receive_cells.is_empty() {
            if !self.receive_event {
                self.receive_generation += 1;
            }
            self.receive_event = true;
        }
    }

    /// The routing unit's channel for a cell (SPRUEG2E section 3.8): the 16-bit field whose
    /// least significant bit is bit RBIT of byte RBYT (counted from one at the first wire byte).
    /// It is ANDed with each mask and compared with the masked match, and the first register to
    /// agree names the PDMA event. RBYT = 0 means no routing is wanted, which sends every cell
    /// to RMMR0's event. `None` is a cell no register claims.
    fn route(&self, wire: &[u8]) -> Option<usize> {
        if !self.profile.pdma {
            return Some(0);
        }
        let byte = ((self.routing_select >> RRSR_BYTE_SHIFT) & 0x3f) as usize;
        if byte == 0 {
            return Some(0);
        }
        let bit = self.routing_select & 0x7;
        let at = |back: usize| {
            (byte - 1)
                .checked_sub(back)
                .and_then(|index| wire.get(index))
                .map_or(0, |&b| u32::from(b))
        };
        let window = (at(2) << 16) | (at(1) << 8) | at(0);
        let field = (window >> bit) & 0xffff;
        self.routing_matches.iter().position(|&register| {
            let mask = register & 0xffff;
            field & mask == (register >> 16) & mask
        })
    }

    /// Which assertion of UREVT is current: the count only moves when the event goes from
    /// clear to asserted.
    pub(crate) fn receive_generation(&self) -> u64 {
        self.receive_generation
    }

    /// UXEVT as the PDMA sees it: which assertion is current, and how many words the queue can
    /// take before a write would stall - the free slots' packets, less what the packet being
    /// written already holds. `None` while the event is not asserted or the transmitter is in
    /// reset.
    pub(crate) fn transmit_event(&self) -> Option<crate::pdma::TransmitEvent> {
        if !self.transmit_event || self.control & UCR_UXEN == 0 || self.stalled_write.is_some() {
            return None;
        }
        let packet = packet_words(self.transmit_cell_bytes());
        let room_words = ((QUEUE_CELLS - self.transmit_cells.len()) * packet)
            .saturating_sub(self.transmit_partial.len);
        Some(crate::pdma::TransmitEvent {
            generation: self.transmit_generation,
            room_words,
        })
    }

    /// The routing unit's channel for the cell UREVT is announcing, which is the PDMA event ID
    /// that goes with the event.
    pub(crate) fn receive_event_route(&self) -> Option<usize> {
        if !self.receive_event || self.control & UCR_UREN == 0 {
            return None;
        }
        self.receive_cells.front().map(|cell| cell.route)
    }

    /// The address phase: whether a master driving `address` onto UXADDR/URADDR is talking to
    /// this slave. Multi-PHY decodes the address against SLID, and SLID 1Fh is the null PHY no
    /// address reaches; single-PHY ignores the bus (SPRUE48 sections 3.7 and 12.1).
    fn addressed(&self, address: u8) -> bool {
        if self.control & UCR_MPHY == 0 {
            return true;
        }
        let slave = ((self.control >> UCR_SLID_SHIFT) & UCR_SLID_BITS) as u8;
        slave != NULL_PHY && u32::from(address) & UCR_SLID_BITS == u32::from(slave)
    }

    /// UXCLAV as the master samples it after polling `address`: `None` while the pin is
    /// high-impedance - another slave's address, or a transmitter held in reset, whose outputs
    /// are tri-stated (SPRUE48 section 9) - and otherwise whether a complete cell is queued
    /// (sections 3.2 and 3.3).
    pub fn master_poll_transmit(&self, address: u8) -> Option<bool> {
        (self.control & UCR_UXEN != 0 && self.addressed(address))
            .then_some(!self.transmit_cells.is_empty())
    }

    /// URCLAV the same way: driven only at this slave's address, and set while the receive
    /// queue has room for a whole cell (sections 3.4 and 3.5).
    pub fn master_poll_receive(&self, address: u8) -> Option<bool> {
        (self.control & UCR_UREN != 0 && self.addressed(address))
            .then_some(self.receive_cells.len() < QUEUE_CELLS)
    }

    /// The master selects `address` with UXENB and takes a cell if this slave is the one
    /// addressed and UXCLAV said it had one (SPRUE48 section 3.2). The cell comes off the wire in
    /// the byte order UCR.BEND selects, without the dummy bytes in front of its header
    /// (Figures 9-14), and a write the queue had stalled lands in the space that frees up.
    pub fn master_take_cell(&mut self, address: u8) -> Option<WireCell> {
        if self.master_poll_transmit(address) != Some(true) {
            return None;
        }
        let packet = self.transmit_cells.pop_front()?;
        let mut packed = [0; MAX_CELL_BYTES];
        for (chunk, &word) in packed.chunks_exact_mut(4).zip(packet.words()) {
            chunk.copy_from_slice(&self.word_bytes(word));
        }
        let packed_len = 4 * packet.len;
        let len = self.transmit_cell_bytes().min(packed_len);
        let mut cell = WireCell {
            bytes: [0; MAX_CELL_BYTES],
            len,
        };
        cell.bytes[..len].copy_from_slice(&packed[packed_len - len..packed_len]);
        if let Some(value) = self.stalled_write.take() {
            // The stalled write completes now that there is space, so the stall is over: XQSP
            // reports a write waiting, not a full queue (SPRUE48 section 12.4).
            self.pending &= !XQSP;
            self.write_transmit_queue(value);
        }
        self.update_transmit_event();
        Some(cell)
    }

    /// The master selects `address` with URENB and sends a cell, which this slave takes only if
    /// it is the one addressed and URCLAV said it had room (SPRUE48 section 3.4). The wire bytes
    /// are what RUDC says the interface expects - a short cell is padded at its end, a long one
    /// truncated - and the receive queue holds them behind the dummy bytes that make the packet
    /// a whole number of words. A cell the routing unit gives to no channel is taken off the
    /// wire and goes nowhere.
    pub fn master_send_cell(&mut self, address: u8, bytes: &[u8]) -> bool {
        if self.master_poll_receive(address) != Some(true) {
            return false;
        }
        let length = self.receive_cell_bytes();
        let mut wire = [0; MAX_CELL_BYTES];
        let copied = bytes.len().min(length);
        wire[..copied].copy_from_slice(&bytes[..copied]);
        if let Some(route) = self.route(&wire[..length]) {
            let packet = self.pack_packet(&wire[..length]);
            self.receive_cells.push_back(ReceiveCell { route, packet });
            self.update_receive_event();
        }
        true
    }

    /// The packet for a wire cell: dummy bytes in front of the header up to a whole number of
    /// words (SPRUE48 Figures 9-14, SPRUEG2E Tables 6-15).
    fn pack_packet(&self, wire: &[u8]) -> Packet {
        let len = packet_words(wire.len());
        let offset = 4 * len - wire.len();
        let mut bytes = [0; MAX_CELL_BYTES];
        bytes[offset..offset + wire.len()].copy_from_slice(wire);
        let mut packet = Packet {
            words: [0; MAX_PACKET_WORDS],
            len,
        };
        for (word, chunk) in packet
            .words
            .iter_mut()
            .zip(bytes[..4 * len].chunks_exact(4))
        {
            *word = self.bytes_word([chunk[0], chunk[1], chunk[2], chunk[3]]);
        }
        packet
    }

    /// UCR.BEND decides which end of a word the first byte on the wire sits at (SPRUE48 Table 7).
    fn word_bytes(&self, word: u32) -> [u8; 4] {
        if self.control & UCR_BEND != 0 {
            word.to_be_bytes()
        } else {
            word.to_le_bytes()
        }
    }

    fn bytes_word(&self, bytes: [u8; 4]) -> u32 {
        if self.control & UCR_BEND != 0 {
            u32::from_be_bytes(bytes)
        } else {
            u32::from_le_bytes(bytes)
        }
    }

    /// UINT is asserted while a pending error has its enable set (SPRUE48 section 12.3).
    pub fn interrupt_pending(&self) -> bool {
        self.pending & self.enable & ERROR_MASK != 0
    }

    #[cfg(test)]
    pub fn tick(&mut self) -> UtopiaEvents {
        self.divide += 1;
        if self.divide == CLOCK_DIVIDER {
            self.divide = 0;
            self.tick_sysclk3()
        } else {
            UtopiaEvents::default()
        }
    }

    pub(crate) fn tick_sysclk3(&mut self) -> UtopiaEvents {
        let transmit_count = (self.clock_detect >> 16) & 0xff;
        let receive_count = self.clock_detect & 0xff;
        if self.control & UCR_UXEN != 0 {
            self.transmit_idle += 1;
            // A count of zero disables the detector; otherwise the missing UXCLK edge fails the
            // section once the count elapses, which resets the transmit port (SPRUE48 Table 8).
            if transmit_count != 0 && self.transmit_idle >= transmit_count {
                self.transmit_idle = 0;
                self.pending |= XCFP;
            }
        } else {
            self.transmit_idle = 0;
        }
        if self.control & UCR_UREN != 0 {
            self.receive_idle += 1;
            if receive_count != 0 && self.receive_idle >= receive_count {
                self.receive_idle = 0;
                self.pending |= RCFP;
            }
        } else {
            self.receive_idle = 0;
        }
        UtopiaEvents {
            transmit: self.control & UCR_UXEN != 0 && self.transmit_event,
            receive: self.control & UCR_UREN != 0 && self.receive_event,
        }
    }
}

impl Sysclk3Block for Utopia {
    /// An edge is quiet when it raises no synchronisation event, fails no section and leaves UINT
    /// deasserted: the device tick counts an asserted UINT as movement on every edge, because it
    /// drives the EDMA3 activity recompute and the level relatch. What remains is an enabled
    /// section without its event counting towards the clock failure its count sets.
    fn quiet_edges(&self) -> u64 {
        if self.interrupt_pending() {
            return 0;
        }
        let transmit = Self::section_quiet_edges(
            self.control & UCR_UXEN != 0,
            self.transmit_event,
            self.transmit_idle,
            (self.clock_detect >> 16) & 0xff,
        );
        let receive = Self::section_quiet_edges(
            self.control & UCR_UREN != 0,
            self.receive_event,
            self.receive_idle,
            self.clock_detect & 0xff,
        );
        transmit.min(receive)
    }

    fn skip_quiet(&mut self, edges: u64) {
        if edges == 0 {
            return;
        }
        // Quiet edges never reach a section's count, and a disabled section holds its counter
        // at zero. The counter wraps like the per-edge increment does.
        self.transmit_idle = if self.control & UCR_UXEN != 0 {
            self.transmit_idle.wrapping_add(edges as u32)
        } else {
            0
        };
        self.receive_idle = if self.control & UCR_UREN != 0 {
            self.receive_idle.wrapping_add(edges as u32)
        } else {
            0
        };
    }
}

impl Utopia {
    /// The quiet edges one enabled section allows: none while its event is asserted, and
    /// otherwise the edges before its clock-detect counter reaches a non-zero count. A disabled
    /// section does neither.
    fn section_quiet_edges(enabled: bool, event: bool, idle: u32, count: u32) -> u64 {
        if !enabled {
            u64::MAX
        } else if event {
            0
        } else if count == 0 {
            u64::MAX
        } else {
            u64::from(count.saturating_sub(idle).max(1) - 1)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    use crate::sysclk3::{assert_skip_matches_steps, TestRng};
    const RECEIVE_QUEUE: u32 = C6455_RECEIVE_QUEUE;
    const TRANSMIT_QUEUE: u32 = C6455_RECEIVE_QUEUE + QUEUE_WINDOW;
    fn clocks(utopia: &mut Utopia, peripheral_clocks: u32) -> u32 {
        let mut transmit_events = 0;
        for _ in 0..peripheral_clocks * CLOCK_DIVIDER {
            if utopia.tick().transmit {
                transmit_events += 1;
            }
        }
        transmit_events
    }

    #[test]
    fn the_registers_come_up_the_way_the_manual_draws_them() {
        // SPRUE48 Figures 15 to 18: MPHY defaults to multi-PHY, the reserved field above BEND
        // reads one, both clock counts start at the longest wait and no error is pending.
        let mut utopia = Utopia::default();
        assert_eq!(utopia.read(UCR, TEST_CONTEXT), Some(0x2000_4000));
        assert_eq!(utopia.read(CDR, TEST_CONTEXT), Some(0x00ff_00ff));
        assert_eq!(utopia.read(EIER, TEST_CONTEXT), Some(0));
        assert_eq!(utopia.read(EIPR, TEST_CONTEXT), Some(0));
        // Writing every bit leaves the reserved fields at their documented values.
        utopia.write(UCR, 0xffff_ffff, TEST_CONTEXT);
        assert_eq!(utopia.read(UCR, TEST_CONTEXT), Some(0xbf3d_403d));
        utopia.write(CDR, 0xffff_ffff, TEST_CONTEXT);
        assert_eq!(utopia.read(CDR, TEST_CONTEXT), Some(0x00ff_00ff));
    }

    #[test]
    fn an_enabled_section_without_a_master_clock_fails_after_its_count() {
        // SPRUE48 Table 8 and Table 10: with no UXCLK edge inside XCCNT peripheral clocks the
        // transmit section fails, and the interrupt only reaches the CPU once EIER allows it.
        let mut utopia = Utopia::default();
        utopia.write(CDR, (3 << 16) | 5, TEST_CONTEXT);
        utopia.write(UCR, UCR_RESET | UCR_UXEN, TEST_CONTEXT);
        assert_eq!(clocks(&mut utopia, 2), 2);
        assert_eq!(utopia.read(EIPR, TEST_CONTEXT), Some(0));
        assert!(!utopia.interrupt_pending());
        clocks(&mut utopia, 1);
        assert_eq!(utopia.read(EIPR, TEST_CONTEXT), Some(XCFP));
        assert!(!utopia.interrupt_pending());
        utopia.write(EIER, XCFP, TEST_CONTEXT);
        assert!(utopia.interrupt_pending());
        // The pending bit is write-one-to-clear, and the receive section is still quiet because
        // its own count has not elapsed.
        utopia.write(EIPR, XCFP, TEST_CONTEXT);
        assert_eq!(utopia.read(EIPR, TEST_CONTEXT), Some(0));
        assert!(!utopia.interrupt_pending());
    }

    #[test]
    fn the_queue_ports_stall_a_read_and_never_stall_a_write() {
        // SPRUE48 section 12.4: a read from the empty receive queue sets RQSP, which is
        // read-only, so writing it back does not clear it.
        let mut utopia = Utopia::default();
        assert_eq!(utopia.read(RECEIVE_QUEUE, TEST_CONTEXT), Some(0));
        assert_eq!(utopia.read(EIPR, TEST_CONTEXT), Some(RQSP));
        utopia.write(EIPR, RQSP, TEST_CONTEXT);
        assert_eq!(utopia.read(EIPR, TEST_CONTEXT), Some(RQSP));
        // The transmit queue accepts every write, so the stall bit stays clear.
        utopia.write(TRANSMIT_QUEUE, 0x1122_3344, TEST_CONTEXT);
        assert_eq!(utopia.read(EIPR, TEST_CONTEXT), Some(RQSP));
        // A disabled transmitter raises no synchronisation event at all.
        assert_eq!(clocks(&mut utopia, 4), 0);
    }

    #[test]
    fn only_the_slave_id_answers_the_address_phase() {
        // SPRUE48 sections 3.1 and 3.7: in multi-PHY mode the slave drives its cell-available
        // pins at its own address only, and leaves them high-impedance for every other one.
        let cell = [0x5a; CELL_BYTES];
        let mut utopia = Utopia::default();
        let slave = 0x0b;
        let control = UCR_RESET | (slave << UCR_SLID_SHIFT) | UCR_UXEN | UCR_UREN;
        utopia.write(UCR, control, TEST_CONTEXT);
        for _ in 0..packet_words(CELL_BYTES) {
            utopia.write(TRANSMIT_QUEUE, 0x1122_3344, TEST_CONTEXT);
        }
        for address in 0..=NULL_PHY {
            let answer = (u32::from(address) == slave).then_some(true);
            assert_eq!(utopia.master_poll_transmit(address), answer, "{address:#x}");
            assert_eq!(utopia.master_poll_receive(address), answer, "{address:#x}");
        }
        // Selecting another PHY moves nothing: the cell waits for this slave's address, and the
        // receive queue does not take a cell meant for someone else.
        assert!(utopia.master_take_cell(0x0a).is_none());
        assert!(!utopia.master_send_cell(0x0c, &cell));
        assert_eq!(
            utopia.master_take_cell(0x0b).map(|cell| cell.bytes().len()),
            Some(CELL_BYTES)
        );
        assert_eq!(utopia.master_poll_transmit(0x0b), Some(false));
        assert!(utopia.master_send_cell(0x0b, &cell));
        assert!(utopia.master_send_cell(0x0b, &cell));
        assert_eq!(utopia.master_poll_receive(0x0b), Some(false));

        // SLID 1Fh is the null PHY, which no address reaches - not even 1Fh itself.
        utopia.write(UCR, control | (0x1f << UCR_SLID_SHIFT), TEST_CONTEXT);
        assert!((0..=NULL_PHY).all(|address| utopia.master_poll_receive(address).is_none()));
        // Single-PHY mode does not decode the bus, and a section in reset tri-states its pin.
        utopia.write(UCR, control & !UCR_MPHY & !UCR_UXEN, TEST_CONTEXT);
        assert_eq!(utopia.master_poll_receive(0x03), Some(false));
        assert_eq!(utopia.master_poll_receive(NULL_PHY), Some(false));
        assert_eq!(utopia.master_poll_transmit(slave as u8), None);
    }

    #[test]
    fn skipping_quiet_edges_matches_stepping_them() {
        // Either section enabled or not, with or without its event, clock counts from disabled
        // to a few edges away - or already passed - and errors pending with or without their
        // enable. A quiet edge also leaves UINT deasserted, since an asserted one counts as
        // movement on every edge.
        let mut rng = TestRng::new(0x0710_91a5);
        for _ in 0..3000 {
            let clock_detect = ((rng.below(8) as u32) << 16) | rng.below(8) as u32;
            let mut utopia = Utopia {
                control: UCR_RESET
                    | (u32::from(rng.below(2) == 0) * UCR_UXEN)
                    | (u32::from(rng.below(2) == 0) * UCR_UREN),
                clock_detect: if rng.below(4) == 0 {
                    CDR_RESET
                } else {
                    clock_detect
                },
                transmit_idle: rng.below(12) as u32,
                receive_idle: rng.below(12) as u32,
                transmit_event: rng.below(2) == 0,
                receive_event: rng.below(2) == 0,
                pending: rng.u32() & rng.u32() & ERROR_MASK,
                enable: rng.u32() & rng.u32() & ERROR_MASK,
                ..Utopia::default()
            };
            for _ in 0..4 {
                assert_skip_matches_steps(&utopia, 600, &mut rng, |utopia| {
                    utopia.tick_sysclk3() == UtopiaEvents::default() && !utopia.interrupt_pending()
                });
                utopia.skip_quiet(utopia.quiet_edges().min(600));
                utopia.tick_sysclk3();
            }
        }
    }

    const SLAVE: u8 = 0x05;

    /// A C6472 receiver with the given extra UCR bits, taken out of reset.
    fn c6472(extra: u32) -> Utopia {
        let mut utopia = Utopia::new(&C6472_UTOPIA);
        let control = UCR_RESET | (u32::from(SLAVE) << UCR_SLID_SHIFT) | UCR_UREN | extra;
        utopia.write(UCR, control, TEST_CONTEXT);
        utopia
    }

    /// A standard cell's wire bytes numbered 1..: header bytes first, then the payload.
    fn numbered_cell(length: usize) -> Vec<u8> {
        (1..=length as u8).collect()
    }

    fn drain(utopia: &mut Utopia, words: usize) -> Vec<u32> {
        (0..words).map(|_| utopia.read_receive_queue()).collect()
    }

    #[test]
    fn the_c6472_registers_come_up_the_way_the_manual_draws_them() {
        // SPRUEG2E Figures 9, 13-16 and Tables 17 and 25.
        let mut utopia = Utopia::new(&C6472_UTOPIA);
        assert_eq!(utopia.read(UCR, TEST_CONTEXT), Some(0x2000_6000));
        utopia.write(UCR, 0xffff_ffff, TEST_CONTEXT);
        assert_eq!(utopia.read(UCR, TEST_CONTEXT), Some(0xbf3d_603d));
        assert_eq!(utopia.read(UPIDR, TEST_CONTEXT), Some(0x0003_0301));
        utopia.write(UPIDR, 0, TEST_CONTEXT);
        assert_eq!(utopia.read(UPIDR, TEST_CONTEXT), Some(0x0003_0301));
        utopia.write(UPWREMU, 0xffff_ffff, TEST_CONTEXT);
        assert_eq!(utopia.read(UPWREMU, TEST_CONTEXT), Some(3));
        utopia.write(RRSR, 0xffff_ffff, TEST_CONTEXT);
        assert_eq!(utopia.read(RRSR, TEST_CONTEXT), Some(0x003f_0007));
        for index in 0..RRMR_COUNT as u32 {
            utopia.write(RRMR0 + 4 * index, 0x1111_0000 * (index + 1), TEST_CONTEXT);
            assert_eq!(
                utopia.read(RRMR0 + 4 * index, TEST_CONTEXT),
                Some(0x1111_0000 * (index + 1))
            );
        }
        // SPRS612G Table 7-142 stops at RMMR5, and the C6455 has none of these registers.
        assert_eq!(utopia.read(RRMR0 + 24, TEST_CONTEXT), None);
        assert_eq!(
            utopia.write(RRMR0 + 24, 0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        let mut c6455 = Utopia::default();
        assert_eq!(c6455.read(RRSR, TEST_CONTEXT), None);
        assert_eq!(c6455.read(UPIDR, TEST_CONTEXT), None);
        // The C6472 maps no data ports for the CPU.
        assert_eq!(
            utopia.write(RECEIVE_QUEUE, 1, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(utopia.read(RECEIVE_QUEUE, TEST_CONTEXT), None);
        assert_eq!(utopia.read(TRANSMIT_QUEUE, TEST_CONTEXT), None);
    }

    #[test]
    fn the_eight_bit_packet_has_three_leading_dummy_bytes() {
        // SPRUEG2E Tables 6 and 7: Header 1 sits in the last byte of word 0 on the wire side
        // of the padding, so the payload starts at byte 8 of the packet.
        let cell = numbered_cell(53);
        let mut big = c6472(UCR_BEND);
        assert!(big.master_send_cell(SLAVE, &cell));
        let words = drain(&mut big, 14);
        assert_eq!(&words[..3], [0x0000_0001, 0x0203_0405, 0x0607_0809]);
        assert_eq!(words[13], u32::from_be_bytes([50, 51, 52, 53]));

        let mut little = c6472(0);
        assert!(little.master_send_cell(SLAVE, &cell));
        let words = drain(&mut little, 14);
        assert_eq!(&words[..3], [0x0100_0000, 0x0504_0302, 0x0908_0706]);
        assert_eq!(words[13], u32::from_le_bytes([50, 51, 52, 53]));
        assert_eq!(little.master_poll_receive(SLAVE), Some(true));
    }

    #[test]
    fn the_sixteen_bit_cell_is_fifty_four_bytes_behind_two_dummies() {
        // Table 14 without user bytes: U16M adds a second UDF byte and the packet two dummies.
        let cell = numbered_cell(54);
        let mut utopia = c6472(UCR_BEND | UCR_U16M);
        assert!(utopia.master_send_cell(SLAVE, &cell));
        let words = drain(&mut utopia, 14);
        assert_eq!(&words[..3], [0x0000_0102, 0x0304_0506, 0x0708_090A]);
        // The 48-byte payload is the last 48 bytes of the 56-byte packet in either bus width.
        assert_eq!(words[2], u32::from_be_bytes([7, 8, 9, 10]));
        // User-defined bytes extend the cell: RUDC = 6 on the wide bus is exactly 15 words.
        let mut wide = c6472(UCR_BEND | UCR_U16M | (6 << UCR_RUDC_SHIFT));
        assert!(wide.master_send_cell(SLAVE, &numbered_cell(60)));
        let words = drain(&mut wide, 15);
        assert_eq!(words[0], u32::from_be_bytes([1, 2, 3, 4]));
        assert_eq!(wide.master_poll_receive(SLAVE), Some(true));
        // And an 8-bit cell with RUDC = 7 is 60 bytes in 15 words, little-endian (Table 9).
        let mut narrow = c6472(7 << UCR_RUDC_SHIFT);
        assert!(narrow.master_send_cell(SLAVE, &numbered_cell(60)));
        let words = drain(&mut narrow, 15);
        assert_eq!(words[0], u32::from_le_bytes([1, 2, 3, 4]));
        assert_eq!(words[14], u32::from_le_bytes([57, 58, 59, 60]));
    }

    #[test]
    fn a_cell_is_padded_or_cut_to_the_length_ucr_expects() {
        let mut utopia = c6472(UCR_BEND);
        assert!(utopia.master_send_cell(SLAVE, &[0xaa; 10]));
        let words = drain(&mut utopia, 14);
        assert_eq!(words[1], 0xaaaa_aaaa);
        assert_eq!(words[4], 0);
        assert!(utopia.master_send_cell(SLAVE, &[0xbb; 100]));
        assert_eq!(drain(&mut utopia, 14)[13], 0xbbbb_bbbb);
        // Nothing is left over from the over-long cell.
        assert_eq!(utopia.receive_cells.len(), 0);
    }

    #[test]
    fn only_the_slave_id_receives_and_a_wrong_phy_leaves_no_trace() {
        let mut utopia = c6472(0);
        for address in 0..=NULL_PHY {
            let answer = (address == SLAVE).then_some(true);
            assert_eq!(utopia.master_poll_receive(address), answer, "{address:#x}");
        }
        for wrong in [0, SLAVE + 1, NULL_PHY] {
            assert!(!utopia.master_send_cell(wrong, &[1; 54]));
        }
        assert!(utopia.receive_cells.is_empty());
        assert_eq!(utopia.receive_event_route(), None);
        assert!(!utopia.tick_sysclk3().receive);
    }

    #[test]
    fn two_cells_fill_the_receive_queue_until_the_first_is_read() {
        // SPRUEG2E sections 2 and 3.5: URCLAV drops with two cells waiting, UREVT is one event
        // that clears on the first read of the head cell and returns for the next.
        let mut utopia = c6472(0);
        assert_eq!(utopia.master_poll_receive(SLAVE), Some(true));
        assert!(utopia.master_send_cell(SLAVE, &[1; 54]));
        assert!(utopia.tick_sysclk3().receive);
        assert!(utopia.master_send_cell(SLAVE, &[2; 54]));
        assert_eq!(utopia.master_poll_receive(SLAVE), Some(false));
        assert!(!utopia.master_send_cell(SLAVE, &[3; 54]));
        assert_eq!(utopia.receive_cells.len(), 2);
        assert_eq!(utopia.read_receive_queue(), 0x0100_0000);
        assert!(!utopia.tick_sysclk3().receive);
        // Still no room: the cell is half read.
        assert_eq!(utopia.master_poll_receive(SLAVE), Some(false));
        drain(&mut utopia, 13);
        assert_eq!(utopia.master_poll_receive(SLAVE), Some(true));
        assert!(utopia.tick_sysclk3().receive);
        assert!(utopia.master_send_cell(SLAVE, &[3; 54]));
    }

    #[test]
    fn the_routing_unit_sends_each_cell_to_the_first_matching_event() {
        // SPRUEG2E section 3.8: VCI routing is RRSR = 0x00040004. The VCI is bits 4 up of header
        // byte 4 together with header bytes 3 and the low nibble of header byte 2.
        let cell = |vci: u16| {
            let mut cell = vec![0u8; 53];
            cell[1] = (vci >> 12) as u8 & 0xf;
            cell[2] = (vci >> 4) as u8;
            cell[3] = ((vci & 0xf) << 4) as u8;
            cell
        };
        let mut utopia = c6472(0);
        utopia.write(RRSR, 0x0004_0004, TEST_CONTEXT);
        utopia.write(RRMR0, 0x0123_ffff, TEST_CONTEXT);
        utopia.write(RRMR0 + 4, 0x0456_ffff, TEST_CONTEXT);
        // RMMR2 matches on the low byte only, so it also catches 0x0156 - after RMMR1 had its
        // turn, which is what keeps one cell on one channel.
        utopia.write(RRMR0 + 8, 0x0056_00ff, TEST_CONTEXT);
        for register in 3..RRMR_COUNT as u32 {
            // Only a VCI of 0xffff would match these.
            utopia.write(RRMR0 + 4 * register, 0xffff_ffff, TEST_CONTEXT);
        }
        for (vci, route) in [(0x0123, 0), (0x0456, 1), (0x0156, 2)] {
            assert!(utopia.master_send_cell(SLAVE, &cell(vci)));
            assert_eq!(utopia.receive_event_route(), Some(route), "vci {vci:#x}");
            drain(&mut utopia, 14);
        }
        // No register claims VCI 0x0789, so the cell belongs to no channel and is dropped.
        utopia.write(RRMR0 + 12, 0xffff_ffff, TEST_CONTEXT);
        assert!(utopia.master_send_cell(SLAVE, &cell(0x0789)));
        assert_eq!(utopia.receive_event_route(), None);
        assert!(utopia.receive_cells.is_empty());
        // A register with a zero mask matches every VCI, so it catches the stray cell.
        utopia.write(RRMR0 + 12, 0x0001_0000, TEST_CONTEXT);
        assert!(utopia.master_send_cell(SLAVE, &cell(0x0789)));
        assert_eq!(utopia.receive_event_route(), Some(3));
        drain(&mut utopia, 14);
        assert_eq!(utopia.master_poll_receive(SLAVE), Some(true));
        // RBYT = 0 is no routing, whatever the registers say.
        utopia.write(RRSR, 0, TEST_CONTEXT);
        assert!(utopia.master_send_cell(SLAVE, &cell(0x0789)));
        assert_eq!(utopia.receive_event_route(), Some(0));
    }
}
