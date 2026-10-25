//! The host port interface of the C6455 and the C6472, both sides of it.
//!
//! SPRU969B (C6455) and SPRUEG1B (C6472): the host owns HPIC, HPID and the two address
//! registers, and reaches the device through the HPI's own DMA logic; the CPU has read-only
//! access to almost all of it and exactly three things it may write - the software reset that
//! clears the FIFOs, the HINT bit that interrupts the host, and a one to DSPINT that
//! acknowledges an interrupt from the host.
//!
//! The host side is driven from outside the device model rather than by a pin interface: a host
//! access names one of the four types the HCNTL pins select (Table 4) and the model performs it
//! at once. What is modelled is everything the protocol makes observable - the word-addressed
//! HPIA registers and their autoincrement, single and dual HPIA mode, the eight-word FIFOs with
//! their four-word bursts and documented flush conditions, and the two interrupt handshakes.

use crate::port::{PortContext, PortWrite, SlavePort};
use crate::system::Fabric;
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const BASE: u32 = 0x0288_0000;
const PWREMU_MGMT: u32 = BASE + 0x04;
const HPIC: u32 = BASE + 0x30;
const HPID: u32 = BASE + 0x34;
const HPIA: u32 = BASE + 0x38;

/// HPIC bits (SPRU969B Table 8).
const HPIC_HWOB: u32 = 1 << 0;
const HPIC_DSPINT: u32 = 1 << 1;
const HPIC_HINT: u32 = 1 << 2;
const HPIC_HRDY: u32 = 1 << 3;
const HPIC_FETCH: u32 = 1 << 4;
const HPIC_HPIRST: u32 = 1 << 7;
const HPIC_HWOBSTAT: u32 = 1 << 8;
const HPIC_DUALHPIA: u32 = 1 << 9;
const HPIC_HPIARWSEL: u32 = 1 << 11;

/// Both FIFOs are eight words deep and the DMA moves four at a time (SPRU969B section 6).
const FIFO_WORDS: usize = 8;
const BURST_WORDS: usize = 4;

/// BOOTMODE[3:0] 0001b is the host boot through the HPI on both devices (SPRS276M section 2.4 for
/// the C6455, SPRUEC6G section 4.3.2 for the C6472).
pub(crate) const BOOTMODE_HPI: u8 = 1;

/// Where the CPU starts when an HPI boot releases it: "the internal L2 SRAM located at
/// 0x80 0000" (SPRS276M section 2.4).
pub(crate) const HPI_BOOT_ENTRY: u32 = 0x0080_0000;

/// Whether, and how, the device was strapped to HPI boot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HpiBoot {
    /// Not strapped: HPIRST comes up set and the firmware opens the port.
    #[default]
    Off,
    /// The C6455's boot: the host's first DSPINT takes the held core out of reset (SPRS276M
    /// section 2.4).
    DspintRelease,
    /// The C6472's boot: the host releases each core through BOOT_COMPLETE_STAT, and DSPINT is
    /// an ordinary interrupt (SPRUEC6G section 4.3.2).
    BootController,
}

/// The access types the host selects on HCNTL[1:0] (SPRU969B Table 4).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum HostPort {
    /// HCNTL = 00b: the control register.
    Control,
    /// HCNTL = 01b: the data register, with the address register incremented afterwards.
    DataAutoIncrement,
    /// HCNTL = 10b: the address register the mode and HPIARWSEL select.
    Address,
    /// HCNTL = 11b: the data register, leaving the address alone.
    Data,
}

#[derive(Default)]
pub struct Hpi {
    power: u32,
    /// The host interrupt the CPU raises, and the host-to-CPU interrupt the CPU acknowledges.
    host_interrupt: bool,
    dsp_interrupt: bool,
    /// The CPU's software reset. While it is set the FIFOs are held clear and every host access
    /// is held off by HRDY (SPRU969B section 8.3).
    hpirst: bool,
    /// A `DspintRelease` boot whose held CPU the host's first DSPINT has not yet released.
    boot_pending: bool,
    /// Host-configured HPIC bits.
    hwob: bool,
    dual_hpia: bool,
    hpiarwsel: bool,
    /// Word addresses, exactly as the host writes them (SPRU969B section 8.5).
    hpiar: u32,
    hpiaw: u32,
    /// Posted host writes, and the word address the first of them belongs to.
    write_fifo: Vec<u32>,
    write_base: u32,
    /// Prefetched words, and the word address the DMA will fetch next.
    read_fifo: std::collections::VecDeque<u32>,
    prefetch: u32,
    /// The 16-bit multiplexed mode's half-finished cycles: the first halfword of a write, and
    /// the word a read fetched so the second halfword comes from the same value rather than
    /// from a second access (SPRU969B section 3.5).
    #[cfg(test)]
    half_write: Option<u16>,
    #[cfg(test)]
    half_read: Option<u32>,
}

impl Hpi {
    /// Reset leaves both interrupts cleared. HPIRST "defaults to 0 when HPI boot is selected;
    /// otherwise it defaults to 1" (SPRU969B Figure 31, SPRUEG1B Figure 22): a booting host has
    /// to reach memory before any code has run to clear it.
    pub fn new(strap: HpiBoot) -> Self {
        Self {
            hpirst: strap == HpiBoot::Off,
            boot_pending: strap == HpiBoot::DspintRelease,
            ..Self::default()
        }
    }

    /// Whether this DSPINT is the one that ends an HPI boot, which the hardware answers by
    /// taking the CPU out of reset. Only the first DSPINT after reset is: every later one is an
    /// ordinary interrupt for the application. DSPINT itself stays set either way; "the CPU
    /// needs to clear the DSPINT, otherwise, no more DSPINTs can be received" (SPRS276M
    /// section 2.4).
    pub(crate) fn take_boot_release(&mut self) -> bool {
        std::mem::take(&mut self.boot_pending)
    }

    /// Whether the device is strapped to an HPI boot the host has not finished with DSPINT.
    pub(crate) fn boot_pending(&self) -> bool {
        self.boot_pending
    }
}

impl SlavePort for Hpi {
    // ---- the CPU's side ---------------------------------------------------------------------

    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address {
            PWREMU_MGMT => self.power,
            // The CPU reads zero for FETCH and HRDY whatever the host sees (Table 8).
            HPIC => {
                self.host_configured()
                    | (u32::from(self.host_interrupt) * HPIC_HINT)
                    | (u32::from(self.dsp_interrupt) * HPIC_DSPINT)
                    | (u32::from(self.hpirst) * HPIC_HPIRST)
            }
            // The CPU cannot reach the data register, and the address registers are read-only.
            HPID => 0,
            HPIA => self.hpiar,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            PWREMU_MGMT => self.power = value & 3,
            HPIC => {
                // A write of one to HINT raises the interrupt to the host; a write of one to
                // DSPINT acknowledges the host's interrupt; writes of zero do nothing.
                if value & HPIC_HINT != 0 {
                    self.host_interrupt = true;
                }
                if value & HPIC_DSPINT != 0 {
                    self.dsp_interrupt = false;
                }
                // The software reset clears both FIFOs and holds the host off until the CPU
                // writes it back to zero (SPRU969B section 7.2).
                self.hpirst = value & HPIC_HPIRST != 0;
                if self.hpirst {
                    self.write_fifo.clear();
                    self.read_fifo.clear();
                }
            }
            HPID | HPIA => {}
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl Hpi {
    // ---- the host's side -------------------------------------------------------------------

    /// A host read cycle. Returns None while the software reset holds the host off.
    pub fn host_read(&mut self, port: HostPort, bus: &mut Fabric) -> Option<u32> {
        if self.hpirst {
            return None;
        }
        let value = match port {
            HostPort::Control => {
                self.host_configured()
                    | HPIC_HRDY
                    | (u32::from(self.host_interrupt) * HPIC_HINT)
                    | (u32::from(self.dsp_interrupt) * HPIC_DSPINT)
            }
            HostPort::Address => {
                if self.dual_hpia && !self.hpiarwsel {
                    self.hpiaw
                } else {
                    self.hpiar
                }
            }
            HostPort::Data => {
                // A read without autoincrementing flushes the read FIFO and fetches the
                // addressed word itself (SPRU969B section 6.3).
                self.flush_read();
                Self::fetch(bus, self.hpiar)
            }
            HostPort::DataAutoIncrement => {
                // Reading while posted writes are outstanding flushes them first in single-HPIA
                // mode, which is what keeps the host from reading stale memory (section 6.3).
                if !self.dual_hpia && !self.write_fifo.is_empty() {
                    self.flush_write(bus);
                }
                if self.read_fifo.is_empty() {
                    self.prefetch = self.hpiar;
                    self.fill_read(bus, FIFO_WORDS);
                }
                let value = self.read_fifo.pop_front().unwrap_or(0);
                self.advance(true);
                if FIFO_WORDS - self.read_fifo.len() >= BURST_WORDS {
                    self.fill_read(bus, BURST_WORDS);
                }
                value
            }
        };
        Some(value)
    }

    /// A host write cycle. Returns true when the host-to-CPU interrupt fires on this access, so
    /// the caller can raise the system event.
    pub fn host_write(&mut self, port: HostPort, value: u32, bus: &mut Fabric) -> bool {
        if self.hpirst {
            return false;
        }
        match port {
            HostPort::Control => {
                self.hwob = value & HPIC_HWOB != 0;
                self.dual_hpia = value & HPIC_DUALHPIA != 0;
                self.hpiarwsel = value & HPIC_HPIARWSEL != 0;
                if value & HPIC_HINT != 0 {
                    self.host_interrupt = false;
                }
                if value & HPIC_FETCH != 0 {
                    // A prefetch command flushes the read FIFO and starts filling it again.
                    self.flush_read();
                    self.fill_read(bus, FIFO_WORDS);
                }
                // A zero-to-one transition on DSPINT is the interrupt; while it stands, further
                // host writes do nothing (SPRU969B section 5.1).
                if value & HPIC_DSPINT != 0 && !self.dsp_interrupt {
                    self.dsp_interrupt = true;
                    return true;
                }
            }
            HostPort::Address => {
                // Writing an address register flushes the FIFO it belongs to, and in single-HPIA
                // mode both of them (SPRU969B section 6.3).
                if self.dual_hpia {
                    if self.hpiarwsel {
                        self.hpiar = value;
                        self.flush_read();
                    } else {
                        self.flush_write(bus);
                        self.hpiaw = value;
                    }
                } else {
                    self.flush_write(bus);
                    self.flush_read();
                    self.hpiar = value;
                    self.hpiaw = value;
                }
            }
            HostPort::Data => {
                // A write without autoincrementing is a flush condition, so it reaches memory
                // itself rather than waiting for a burst.
                self.flush_write(bus);
                bus.hpi_write32(self.hpiaw << 2, value);
            }
            HostPort::DataAutoIncrement => {
                if !self.dual_hpia && !self.read_fifo.is_empty() {
                    self.flush_read();
                }
                if self.write_fifo.is_empty() {
                    self.write_base = self.hpiaw;
                }
                self.write_fifo.push(value);
                self.advance(false);
                // The DMA is not asked for anything until four words are posted, and then it
                // moves them as one burst (SPRU969B section 6.2).
                if self.write_fifo.len() >= BURST_WORDS {
                    self.flush_write(bus);
                }
            }
        }
        false
    }

    /// One halfword of a 16-bit multiplexed write cycle. HHWIL is low for the first halfword and
    /// high for the second; HWOB decides which end of the word each carries (SPRU969B Table 8).
    /// The underlying access happens when the cycle completes, on the second halfword.
    #[cfg(test)]
    pub fn host_write_halfword(
        &mut self,
        port: HostPort,
        value: u16,
        second: bool,
        bus: &mut Fabric,
    ) -> bool {
        if self.hpirst {
            return false;
        }
        if !second {
            self.half_write = Some(value);
            return false;
        }
        let first = self.half_write.take().unwrap_or(0);
        let word = if self.hwob {
            // The first halfword was the least significant.
            (u32::from(value) << 16) | u32::from(first)
        } else {
            (u32::from(first) << 16) | u32::from(value)
        };
        self.host_write(port, word, bus)
    }

    /// One halfword of a 16-bit multiplexed read cycle. The first halfword performs the access
    /// and latches the whole word; the second takes its other half from that latch.
    #[cfg(test)]
    pub fn host_read_halfword(
        &mut self,
        port: HostPort,
        second: bool,
        bus: &mut Fabric,
    ) -> Option<u16> {
        if self.hpirst {
            return None;
        }
        let word = if second {
            self.half_read.take()?
        } else {
            let word = self.host_read(port, bus)?;
            self.half_read = Some(word);
            word
        };
        // HWOB = 0 sends the most significant halfword first.
        let high = (word >> 16) as u16;
        let low = word as u16;
        Some(match (self.hwob, second) {
            (false, false) | (true, true) => high,
            (false, true) | (true, false) => low,
        })
    }

    // ---- internals -------------------------------------------------------------------------

    fn host_configured(&self) -> u32 {
        (u32::from(self.hwob) * HPIC_HWOB)
            | (u32::from(self.hwob) * HPIC_HWOBSTAT)
            | (u32::from(self.dual_hpia) * HPIC_DUALHPIA)
            | (u32::from(self.hpiarwsel) * HPIC_HPIARWSEL)
    }

    /// Autoincrement. Single-HPIA mode moves both registers, dual mode only the one this
    /// direction uses (SPRU969B Table 8).
    fn advance(&mut self, reading: bool) {
        if self.dual_hpia {
            if reading {
                self.hpiar = self.hpiar.wrapping_add(1);
            } else {
                self.hpiaw = self.hpiaw.wrapping_add(1);
            }
        } else {
            self.hpiar = self.hpiar.wrapping_add(1);
            self.hpiaw = self.hpiaw.wrapping_add(1);
        }
    }

    /// Posted writes are forced to memory, in order, from the address the first one belongs to.
    fn flush_write(&mut self, bus: &mut Fabric) {
        let words = std::mem::take(&mut self.write_fifo);
        for (index, word) in words.into_iter().enumerate() {
            bus.hpi_write32((self.write_base + index as u32) << 2, word);
        }
        self.write_base = self.hpiaw;
    }

    /// A read flush discards prefetched data and resets the DMA pointer to the host's own.
    fn flush_read(&mut self) {
        self.read_fifo.clear();
        self.prefetch = self.hpiar;
    }

    fn fill_read(&mut self, bus: &mut Fabric, words: usize) {
        for _ in 0..words.min(FIFO_WORDS - self.read_fifo.len()) {
            let word = Self::fetch(bus, self.prefetch);
            self.read_fifo.push_back(word);
            self.prefetch = self.prefetch.wrapping_add(1);
        }
    }

    /// One word the DMA logic reads at a word address, with privilege ID 0.
    /// A refused read completes with zero and no HRDY hold-off. For host-inaccessible
    /// register windows TI specifies no response; zero is the emulator's policy,
    /// not a promised silicon value.
    fn fetch(bus: &mut Fabric, word_address: u32) -> u32 {
        bus.hpi_read32(word_address << 2).unwrap_or(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cpu_sees_the_documented_reset_state() {
        let mut hpi = Hpi::new(HpiBoot::Off);
        // HPIRST comes up set, both interrupts clear, and the CPU reads zero for FETCH and HRDY
        // whatever the host would see (SPRU969B Table 8 and Figure 31).
        assert_eq!(hpi.read(HPIC, TEST_CONTEXT), Some(HPIC_HPIRST));
        assert_eq!(hpi.read(HPID, TEST_CONTEXT), Some(0));
        assert_eq!(hpi.read(HPIA, TEST_CONTEXT), Some(0));
        assert_eq!(hpi.read(PWREMU_MGMT, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn the_cpu_may_raise_the_host_interrupt_and_acknowledge_its_own() {
        let mut hpi = Hpi::new(HpiBoot::Off);
        hpi.write(HPIC, HPIC_HINT, TEST_CONTEXT);
        assert_eq!(hpi.read(HPIC, TEST_CONTEXT), Some(HPIC_HINT));
        // A write of zero to either interrupt bit does nothing; only the host clears HINT.
        hpi.write(HPIC, 0, TEST_CONTEXT);
        assert_eq!(hpi.read(HPIC, TEST_CONTEXT), Some(HPIC_HINT));
        hpi.dsp_interrupt = true;
        hpi.write(HPIC, HPIC_DSPINT, TEST_CONTEXT);
        assert_eq!(hpi.read(HPIC, TEST_CONTEXT), Some(HPIC_HINT));
    }

    #[test]
    fn a_host_burst_reaches_memory_only_when_four_words_are_posted() {
        let mut system = Fabric::default();
        system.core(0).write32(HPIC, 0); // the CPU releases the software reset
        let base = 0x0080_2000u32;
        system.hpi_host_write(HostPort::Control, 0);
        system.hpi_host_write(HostPort::Address, base >> 2);
        for word in 0..3u32 {
            system.hpi_host_write(HostPort::DataAutoIncrement, 0x1000 + word);
        }
        assert_eq!(system.core(0).read32(base), 0);
        system.hpi_host_write(HostPort::DataAutoIncrement, 0x1003);
        for word in 0..4u32 {
            assert_eq!(system.core(0).read32(base + word * 4), 0x1000 + word);
        }
        // The address register advanced one word per access, as the host sees it.
        assert_eq!(
            system.hpi_host_read(HostPort::Address),
            Some((base >> 2) + 4)
        );
    }

    #[test]
    fn a_16_bit_read_cycle_fetches_one_word_and_advances_once() {
        let mut system = Fabric::default();
        system.core(0).write32(HPIC, 0);
        let base = 0x0080_2100u32;
        system.core(0).write32(base, 0xAAAA_BBBB);
        system.core(0).write32(base + 4, 0xCCCC_DDDD);
        system.hpi_host_write(HostPort::Control, 0); // HWOB clear: most significant first
        system.hpi_host_write(HostPort::Address, base >> 2);
        // Two halfwords make one cycle, so the autoincrement steps one word, not two, and the
        // second halfword comes from the word the first one fetched (SPRU969B section 3.5).
        assert_eq!(
            system.hpi_host_read_halfword(HostPort::DataAutoIncrement, false),
            Some(0xAAAA)
        );
        assert_eq!(
            system.hpi_host_read_halfword(HostPort::DataAutoIncrement, true),
            Some(0xBBBB)
        );
        assert_eq!(
            system.hpi_host_read_halfword(HostPort::DataAutoIncrement, false),
            Some(0xCCCC)
        );
        assert_eq!(
            system.hpi_host_read_halfword(HostPort::DataAutoIncrement, true),
            Some(0xDDDD)
        );
    }
}
