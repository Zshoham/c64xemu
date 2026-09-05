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
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const BASE: u32 = 0x02b4_0000;
const UCR: u32 = BASE;
const CDR: u32 = BASE + 0x14;
const EIER: u32 = BASE + 0x18;
const EIPR: u32 = BASE + 0x1c;

/// The receive and transmit data ports are 1 KB windows of their own (SPRS276M Table 2-2).
const RECEIVE_QUEUE: u32 = 0x3c00_0000;
const TRANSMIT_QUEUE: u32 = 0x3c00_0400;
const QUEUE_WINDOW: u32 = 0x400;

/// UCR bit 31 selects big-endian cell assembly, bits 30-29 read back the reserved one, MPHY in
/// bit 14 comes up set, and everything else resets clear (SPRUE48 Figure 15).
const UCR_RESERVED_ONES: u32 = 1 << 29;
const UCR_WRITABLE: u32 =
    (1 << 31) | (0x1f << 24) | (0xf << 18) | (1 << 16) | (1 << 14) | (0xf << 2) | 1;
const UCR_RESET: u32 = UCR_RESERVED_ONES | (1 << 14);
const UCR_UXEN: u32 = 1 << 16;
const UCR_UREN: u32 = 1 << 0;

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
/// UCR's user-defined-cell fields: extra header bytes on top of the standard 53-byte cell.
const UCR_XUDC_SHIFT: u32 = 18;
#[cfg(test)]
const UCR_RUDC_SHIFT: u32 = 2;
/// A standard ATM cell is a five-byte header and a 48-byte payload (SPRUE48 section 2).
const CELL_BYTES: usize = 53;
/// Both queues hold two cells (SPRS276M section 7.19.1).
const QUEUE_CELLS: usize = 2;

/// The words a cell packet occupies: the wire cell rounded up to a word (SPRUE48 section 4.1).
fn packet_words(wire_bytes: usize) -> usize {
    wire_bytes.div_ceil(4)
}

/// What one peripheral clock of the interface produced.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct UtopiaEvents {
    /// UXEVT: the transmit queue has room for a cell packet (SPRUE48 Table 2).
    pub transmit: bool,
    /// UREVT: a complete cell is waiting in the receive queue.
    pub receive: bool,
}

pub struct Utopia {
    control: u32,
    clock_detect: u32,
    enable: u32,
    pending: u32,
    /// Peripheral clocks since each section last saw a clock edge - which never happens here.
    transmit_idle: u32,
    receive_idle: u32,
    /// Complete cell packets waiting for the master, and the packet being written.
    #[cfg(test)]
    divide: u32,
    transmit_cells: std::collections::VecDeque<Vec<u32>>,
    transmit_partial: Vec<u32>,
    /// A write that arrived with the queue full: the bus stalls, so exactly one word can be in
    /// flight, and it lands as soon as the master drains a cell.
    stalled_write: Option<u32>,
    /// Cells the master has sent, and how far the reader has got through the head one.
    receive_cells: std::collections::VecDeque<Vec<u32>>,
    receive_read: usize,
    /// Whether each event is currently asserted: a cell's first access clears it, and it comes
    /// back when the queue is ready again (SPRUE48 sections 3.3 and 3.5).
    transmit_event: bool,
    receive_event: bool,
}

impl Default for Utopia {
    fn default() -> Self {
        Self {
            control: UCR_RESET,
            clock_detect: CDR_RESET,
            enable: 0,
            pending: 0,
            transmit_idle: 0,
            receive_idle: 0,
            transmit_cells: std::collections::VecDeque::new(),
            #[cfg(test)]
            divide: 0,
            transmit_partial: Vec::new(),
            stalled_write: None,
            receive_cells: std::collections::VecDeque::new(),
            receive_read: 0,
            transmit_event: true,
            receive_event: false,
        }
    }
}

impl SlavePort for Utopia {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        if (RECEIVE_QUEUE..RECEIVE_QUEUE + QUEUE_WINDOW).contains(&address) {
            return Some(self.read_receive_queue());
        }
        if (TRANSMIT_QUEUE..TRANSMIT_QUEUE + QUEUE_WINDOW).contains(&address) {
            // The transmit queue is write-only to the CPU and the EDMA; reads see nothing.
            return Some(0);
        }
        let value = match address {
            UCR => self.control,
            CDR => self.clock_detect,
            EIER => self.enable,
            EIPR => self.pending,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if (RECEIVE_QUEUE..RECEIVE_QUEUE + QUEUE_WINDOW).contains(&address) {
            return PortWrite::Done;
        }
        if (TRANSMIT_QUEUE..TRANSMIT_QUEUE + QUEUE_WINDOW).contains(&address) {
            self.write_transmit_queue(value);
            return PortWrite::Done;
        }
        match address {
            UCR => {
                let previous = self.control;
                self.control = UCR_RESERVED_ONES | (value & UCR_WRITABLE);
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
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl Utopia {
    /// The wire length of a transmitted cell: the standard 53 bytes plus the user-defined header
    /// bytes XUDC asks for, which are one to eleven (SPRUE48 Table 7).
    fn transmit_cell_bytes(&self) -> usize {
        CELL_BYTES + ((self.control >> UCR_XUDC_SHIFT) & 0xf).min(11) as usize
    }

    #[cfg(test)]
    fn receive_cell_bytes(&self) -> usize {
        CELL_BYTES + ((self.control >> UCR_RUDC_SHIFT) & 0xf).min(11) as usize
    }

    /// How many of the two cell slots are taken: complete cells, plus the one being written.
    fn transmit_slots(&self) -> usize {
        self.transmit_cells.len() + usize::from(!self.transmit_partial.is_empty())
    }

    /// One 32-bit write into the transmit queue. A word that completes a packet moves it to the
    /// queue; a word that would need a slot the queue does not have is held, which is what the
    /// stall in SPRUE48 section 12.4 does, and XQSP says so until room appears.
    fn write_transmit_queue(&mut self, value: u32) {
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
        self.transmit_partial.push(value);
        if self.transmit_partial.len() == packet_words(self.transmit_cell_bytes()) {
            let packet = std::mem::take(&mut self.transmit_partial);
            self.transmit_cells.push_back(packet);
        }
        self.update_transmit_event();
    }

    /// One 32-bit read out of the receive queue. The first read of a cell clears the event, an
    /// exhausted cell leaves the queue, and a read with no cell waiting stalls: RQSP records it
    /// (SPRUE48 sections 3.5 and 12.4).
    fn read_receive_queue(&mut self) -> u32 {
        let Some(cell) = self.receive_cells.front() else {
            self.pending |= RQSP;
            return 0;
        };
        if self.receive_read == 0 {
            self.receive_event = false;
        }
        let value = cell[self.receive_read];
        self.receive_read += 1;
        if self.receive_read == cell.len() {
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
            self.transmit_event = true;
        }
    }

    fn update_receive_event(&mut self) {
        // A complete cell is available whenever one is queued, and the reader has not started it.
        if self.receive_read == 0 && !self.receive_cells.is_empty() {
            self.receive_event = true;
        }
    }

    /// The master polls for a cell and takes it if the transmitter has one, which is what UXCLAV
    /// answers (SPRUE48 section 3.3). The cell comes off the wire in the byte order UCR.BEND
    /// selects, and a write the queue had stalled lands in the space that frees up.
    #[cfg(test)]
    pub fn master_take_cell(&mut self) -> Option<Vec<u8>> {
        if self.control & UCR_UXEN == 0 {
            return None;
        }
        let packet = self.transmit_cells.pop_front()?;
        let mut bytes = Vec::with_capacity(packet.len() * 4);
        for word in &packet {
            bytes.extend_from_slice(&self.word_bytes(*word));
        }
        bytes.truncate(self.transmit_cell_bytes());
        if let Some(value) = self.stalled_write.take() {
            // The stalled write completes now that there is space, so the stall is over: XQSP
            // reports a write waiting, not a full queue (SPRUE48 section 12.4).
            self.pending &= !XQSP;
            self.write_transmit_queue(value);
        }
        self.update_transmit_event();
        Some(bytes)
    }

    /// The master offers a cell, which the receiver takes if it has room for one - the URCLAV
    /// answer of SPRUE48 section 3.5. A short cell is padded, a long one truncated to what RUDC
    /// says the interface expects.
    #[cfg(test)]
    pub fn master_send_cell(&mut self, bytes: &[u8]) -> bool {
        if self.control & UCR_UREN == 0 || self.receive_cells.len() >= QUEUE_CELLS {
            return false;
        }
        let wire = self.receive_cell_bytes();
        let mut cell = bytes.to_vec();
        cell.resize(packet_words(wire) * 4, 0);
        let words = cell
            .chunks(4)
            .map(|chunk| self.bytes_word([chunk[0], chunk[1], chunk[2], chunk[3]]))
            .collect();
        self.receive_cells.push_back(words);
        self.update_receive_event();
        true
    }

    /// UCR.BEND decides which end of a word the first byte on the wire sits at (SPRUE48 Table 7).
    #[cfg(test)]
    fn word_bytes(&self, word: u32) -> [u8; 4] {
        if self.control & (1 << 31) != 0 {
            word.to_be_bytes()
        } else {
            word.to_le_bytes()
        }
    }

    #[cfg(test)]
    fn bytes_word(&self, bytes: [u8; 4]) -> u32 {
        if self.control & (1 << 31) != 0 {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
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
}
