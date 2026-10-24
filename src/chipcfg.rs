//! The C6472's chip-level configuration registers at 0x02A8 0000 (SPRS612G sections 3.1-3.11).
//!
//! This is the device's own register file, as distinct from a megamodule's: the latched pin
//! configuration, the switch-fabric priority allocation, the peripheral enables and their key
//! locks, the supervisor/user permission map, and - the part a multicore device cannot work
//! without - the **inter-processor interrupt registers**. A core (or the host) raises an
//! interrupt in core *n* by writing IPCGR*n*, and the interrupt arrives in that core's
//! megamodule as system event 84, IPC_LOCAL; the receiving core clears the source bits through
//! IPCAR*n* (section 3.7.2). NMIGR*n* is the same shape for a non-maskable interrupt.
//!
//! The C6455's equivalent block lives in `devcfg.rs` at a different address with different
//! fields; the two devices share no register here beyond the idea.
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const BASE: u32 = 0x02a8_0000;
/// The chip-level register window is 128 KB (SPRS612G Table 2-2).
const WINDOW: u32 = 0x0002_0000;

const DEVSTAT: u32 = BASE;
const PRI_ALLOC: u32 = BASE + 0x004;
const DEVICE_ID: u32 = BASE + 0x008;
const DEVCTL: u32 = BASE + 0x200;
const DEVCTL_KEY: u32 = BASE + 0x204;
const RMIIRESET0: u32 = BASE + 0x208;
const RMIIRESET1: u32 = BASE + 0x20c;
const HOSTPRIV: u32 = BASE + 0x40c;
const PRIVPERM: u32 = BASE + 0x41c;
const PRIVKEY: u32 = BASE + 0x420;
const NMIGR0: u32 = BASE + 0x500;
const IPCGR0: u32 = BASE + 0x540;
const IPCGR_HOST: u32 = BASE + 0x57c;
const IPCAR0: u32 = BASE + 0x580;
const IPCAR_HOST: u32 = BASE + 0x5bc;
const MACID0: u32 = BASE + 0x700;
const MACID1: u32 = BASE + 0x704;
const TPMGR: u32 = BASE + 0x714;
const RSTMUX0: u32 = BASE + 0x718;

/// The six megamodules the per-core register arrays cover.
pub(crate) const CORES: usize = 6;

/// DEVCTL is writable only immediately after its key (SPRS612G section 3.3).
const DEVCTL_UNLOCK: u32 = 0x0a1e_183a;
/// PRIVPERM is writable only immediately after its key (section 3.6).
const PRIVKEY_UNLOCK: u32 = 0xbea7;

/// DEVCTL: one enable per pin-shared peripheral - EMAC1_EN bit 12, TSIP2_EN 11:9, TSIP1_EN 8:6,
/// TSIP0_EN 5:3, UTOPIA_EN 2:1, HPI_EN 0 (SPRS612G Table 3-4).
const DEVCTL_WRITABLE: u32 = 0x0000_1fff;

/// PRI_ALLOC: PDMA bit 15, EMAC1 14:12, HPI 5:3, EMAC0 2:0 are writable priorities, 000 highest
/// and 111 lowest; every reserved bit reads as one (SPRS612G section 4.3, Figure 4-2).
const PRI_ALLOC_WRITABLE: u32 = (1 << 15) | (0b111 << 12) | (0b111 << 3) | 0b111;
const PRI_ALLOC_RESERVED: u32 = !PRI_ALLOC_WRITABLE;

/// HOSTPRIV: bit 0 HPI, bit 1 SRIO, 1 meaning that master's accesses are supervisor
/// (section 3.6). Reserved bits read as ones.
const HOSTPRIV_WRITABLE: u32 = 0b11;

/// IPCGR: SRCS in 31:4, IPCG the write-one pulse in bit 0 (section 3.7.2).
const IPC_SOURCE_MASK: u32 = 0xffff_fff0;
const IPC_GENERATE: u32 = 1 << 0;

/// RSTMUX: DELAY 8:6 (reset 100b), EVTSTAT bit 4 cleared by writing zero, OMODE 3:1, LOCK bit 0
/// (section 3.8.2).
const RSTMUX_RESET: u32 = 0b100 << 6;
const RSTMUX_DELAY: u32 = 0b111 << 6;
const RSTMUX_EVTSTAT: u32 = 1 << 4;
const RSTMUX_OMODE: u32 = 0b111 << 1;
const RSTMUX_LOCK: u32 = 1 << 0;

/// DEVSTAT.LENDIAN, bit 6: 1 selects little endian (SPRS612G Table 3-16).
pub(crate) const DEVSTAT_LITTLE_ENDIAN: u32 = 1 << 6;

pub(crate) struct ChipConfig {
    /// The latched pin state, read-only to software (section 3.4).
    devstat: u32,
    priority: u32,
    device_id: u32,
    devctl: u32,
    devctl_unlocked: bool,
    rmii_reset: [u32; 2],
    host_privilege: u32,
    permissions: u32,
    permissions_unlocked: bool,
    /// IPCGR*n*.SRCS: the source flags a sender sets and the destination clears.
    ipc_sources: [u32; CORES],
    ipc_host_sources: u32,
    /// Destination cores whose IPCG bit has been written since the device last looked.
    ipc_pulses: u32,
    nmi_pulses: u32,
    /// UNCERTAIN: SPRS612G Table 3-2 lists "MAC ID" at 0x02A8 0700 with no field table, so how
    /// the address is split across the two words is not documented here. Both words are
    /// modelled as read-only and zero until a front end fuses an address into them.
    mac_id: [u32; 2],
    timer_output: u32,
    reset_mux: [u32; CORES],
}

impl ChipConfig {
    pub(crate) fn new(devstat: u32, device_id: u32) -> Self {
        Self {
            devstat,
            priority: u32::MAX,
            device_id,
            devctl: 0,
            devctl_unlocked: false,
            rmii_reset: [0; 2],
            host_privilege: u32::MAX,
            permissions: 0,
            permissions_unlocked: false,
            ipc_sources: [0; CORES],
            ipc_host_sources: 0,
            ipc_pulses: 0,
            nmi_pulses: 0,
            mac_id: [0; 2],
            timer_output: 0,
            reset_mux: [RSTMUX_RESET; CORES],
        }
    }

    /// Strap the pins a UTOPIA boot reads: the PHY ID on CFGGP[4:0], DEVSTAT bits 21:17
    /// (Figure 3-3, SPRUEC6G Table 22), and the UTOPIA_EN pins, which "initialize at reset from
    /// GP01/UTOPIA_EN" into DEVCTL bits 2:1 (SPRS612G Table 3-4): 01b powers the 8-bit port's
    /// I/O and 11b the upper byte as well (the 11/01 columns of the pin multiplex table).
    pub(crate) fn strap_utopia(&mut self, phy: u8, wide: bool) {
        const CFGGP_SHIFT: u32 = 17;
        const CFGGP_MASK: u32 = 0x1f << CFGGP_SHIFT;
        const UTOPIA_EN_SHIFT: u32 = 1;
        self.devstat = (self.devstat & !CFGGP_MASK) | (u32::from(phy & 0x1f) << CFGGP_SHIFT);
        let enable = if wide { 0b11 } else { 0b01 };
        self.devctl = (self.devctl & !(0b11 << UTOPIA_EN_SHIFT)) | (enable << UTOPIA_EN_SHIFT);
    }
}

impl SlavePort for ChipConfig {
    /// None when the address is not ours. Inside the window an unimplemented word reads zero,
    /// which is what the reserved rows of Table 3-2 say.
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        if !(BASE..BASE + WINDOW).contains(&address) {
            return None;
        }
        if let Some(core) = index_of(address, IPCGR0) {
            // IPCG reads zero; the source flags read back (section 3.7.2).
            return Some(self.ipc_sources[core]);
        }
        if let Some(core) = index_of(address, IPCAR0) {
            return Some(self.ipc_sources[core]);
        }
        if index_of(address, NMIGR0).is_some() {
            return Some(0);
        }
        if let Some(core) = index_of(address, RSTMUX0) {
            return Some(self.reset_mux[core]);
        }
        let value = match address {
            DEVSTAT => self.devstat,
            PRI_ALLOC => self.priority,
            DEVICE_ID => self.device_id,
            DEVCTL => self.devctl,
            // A key register reads zero: it exists to be written (section 3.3).
            DEVCTL_KEY | PRIVKEY => 0,
            RMIIRESET0 => self.rmii_reset[0],
            RMIIRESET1 => self.rmii_reset[1],
            HOSTPRIV => self.host_privilege,
            PRIVPERM => self.permissions,
            IPCGR_HOST | IPCAR_HOST => self.ipc_host_sources,
            MACID0 => self.mac_id[0],
            MACID1 => self.mac_id[1],
            TPMGR => self.timer_output,
            _ => 0,
        };
        Some(value)
    }

    /// `PortWrite::Unhandled` when the address is not ours.
    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if !(BASE..BASE + WINDOW).contains(&address) {
            return PortWrite::Unhandled;
        }
        if let Some(core) = index_of(address, IPCGR0) {
            // SRCS is set by writing ones; a zero leaves the flag alone, so two senders do not
            // clear each other's source (section 3.7.2).
            self.ipc_sources[core] |= value & IPC_SOURCE_MASK;
            if value & IPC_GENERATE != 0 {
                self.ipc_pulses |= 1 << core;
            }
            return PortWrite::Done;
        }
        if let Some(core) = index_of(address, IPCAR0) {
            self.ipc_sources[core] &= !(value & IPC_SOURCE_MASK);
            return PortWrite::Done;
        }
        if let Some(core) = index_of(address, NMIGR0) {
            if value & 1 != 0 {
                self.nmi_pulses |= 1 << core;
            }
            return PortWrite::Done;
        }
        if let Some(core) = index_of(address, RSTMUX0) {
            if self.reset_mux[core] & RSTMUX_LOCK == 0 {
                // EVTSTAT is cleared by writing zero to it and is never set by software.
                let event = self.reset_mux[core] & RSTMUX_EVTSTAT & value;
                self.reset_mux[core] =
                    (value & (RSTMUX_DELAY | RSTMUX_OMODE | RSTMUX_LOCK)) | event;
            }
            return PortWrite::Done;
        }
        match address {
            // DEVSTAT latches pins, the device identification is hard-wired, and the MAC ID is
            // fused.
            DEVSTAT | DEVICE_ID | MACID0 | MACID1 => {}
            PRI_ALLOC => self.priority = (value & PRI_ALLOC_WRITABLE) | PRI_ALLOC_RESERVED,
            DEVCTL => {
                if self.devctl_unlocked {
                    self.devctl = value & DEVCTL_WRITABLE;
                }
                // The key unlocks exactly one write, whether or not that write was accepted.
                self.devctl_unlocked = false;
            }
            DEVCTL_KEY => self.devctl_unlocked = value == DEVCTL_UNLOCK,
            RMIIRESET0 => self.rmii_reset[0] = value & 1,
            RMIIRESET1 => self.rmii_reset[1] = value & 1,
            HOSTPRIV => {
                self.host_privilege = (value & HOSTPRIV_WRITABLE) | !HOSTPRIV_WRITABLE;
            }
            PRIVPERM => {
                if self.permissions_unlocked {
                    self.permissions = value;
                }
                self.permissions_unlocked = false;
            }
            PRIVKEY => self.permissions_unlocked = value & 0xffff == PRIVKEY_UNLOCK,
            IPCGR_HOST => self.ipc_host_sources |= value & IPC_SOURCE_MASK,
            IPCAR_HOST => self.ipc_host_sources &= !(value & IPC_SOURCE_MASK),
            TPMGR => self.timer_output = value & 0xf,
            _ => {}
        }
        PortWrite::Done
    }
}

impl ChipConfig {
    /// The cores whose IPCG bit has been written since the last call, consumed once. The device
    /// raises system event 84 in each of them (SPRS612G Table 7-10, IPC_LOCAL).
    pub(crate) fn take_ipc_pulse(&mut self) -> u32 {
        std::mem::take(&mut self.ipc_pulses)
    }

    /// Fuse the device's Ethernet address into the MAC ID words, which is what a board does
    /// before the ROM bootloader's Ethernet boot can name itself (SPRUEC6G section 4.3.5).
    ///
    /// UNCERTAIN: SPRS612G Table 3-2 gives the two words no field table, so the split is a
    /// choice. The address is held most significant byte first - MACID0 the four most
    /// significant bytes, MACID1 the two least significant in bits 15-0 - which is the order
    /// the EMAC's own MACADDRHI/MACADDRLO pair uses for the same six bytes.
    pub(crate) fn fuse_mac_id(&mut self, mac: [u8; 6]) {
        self.mac_id = [
            u32::from_be_bytes([mac[0], mac[1], mac[2], mac[3]]),
            u32::from_be_bytes([0, 0, mac[4], mac[5]]),
        ];
    }

    /// The cores whose NMIGR has been written since the last call, consumed once. NMIGR drives
    /// the core's NMI pin rather than a system event (section 3.7.1).
    pub(crate) fn take_nmi_pulse(&mut self) -> u32 {
        std::mem::take(&mut self.nmi_pulses)
    }
}

/// The core a word address in a six-register array selects, if it is one exactly.
fn index_of(address: u32, base: u32) -> Option<usize> {
    let offset = address.checked_sub(base)?;
    if offset % 4 != 0 || offset / 4 >= CORES as u32 {
        return None;
    }
    Some((offset / 4) as usize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    #[test]
    fn an_address_outside_the_window_is_not_ours() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN, 0x0009_102f);
        assert_eq!(chip.read(BASE - 4, TEST_CONTEXT), None);
        assert_eq!(chip.read(BASE + WINDOW, TEST_CONTEXT), None);
        assert_eq!(chip.write(BASE - 4, 1, TEST_CONTEXT), PortWrite::Unhandled);
        assert_eq!(
            chip.write(BASE + WINDOW, 1, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        // A reserved word inside the window answers with zero rather than falling through to
        // memory.
        assert_eq!(chip.read(BASE + 0x100, TEST_CONTEXT), Some(0));
        assert_ne!(
            chip.write(BASE + 0x100, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
    }

    /// The inter-processor interrupt: a sender sets a source flag and pulses the destination,
    /// the destination reads the flags and clears them (SPRS612G section 3.7.2).
    #[test]
    fn an_ipc_write_pulses_once_and_its_source_survives_until_acknowledged() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN, 0x0009_102f);
        assert_ne!(
            chip.write(IPCGR0 + 4 * 3, (1 << 5) | IPC_GENERATE, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(chip.read(IPCGR0 + 4 * 3, TEST_CONTEXT), Some(1 << 5));

        assert_eq!(chip.take_ipc_pulse(), 1 << 3);
        assert_eq!(chip.take_ipc_pulse(), 0, "the pulse is consumed once");

        // A second sender adds its own flag without disturbing the first.
        assert_ne!(
            chip.write(IPCGR0 + 4 * 3, (1 << 9) | IPC_GENERATE, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            chip.read(IPCGR0 + 4 * 3, TEST_CONTEXT),
            Some((1 << 5) | (1 << 9))
        );
        assert_eq!(chip.take_ipc_pulse(), 1 << 3);

        // The destination acknowledges one source at a time through IPCAR.
        assert_ne!(
            chip.write(IPCAR0 + 4 * 3, 1 << 5, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(chip.read(IPCGR0 + 4 * 3, TEST_CONTEXT), Some(1 << 9));
        assert_ne!(
            chip.write(IPCAR0 + 4 * 3, 1 << 9, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(chip.read(IPCGR0 + 4 * 3, TEST_CONTEXT), Some(0));
        // Acknowledging never pulses anything.
        assert_eq!(chip.take_ipc_pulse(), 0);
    }

    #[test]
    fn an_nmi_generate_is_an_edge_per_core() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN, 0x0009_102f);
        assert_ne!(
            chip.write(NMIGR0 + 4 * 5, 1, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_ne!(chip.write(NMIGR0, 1, TEST_CONTEXT), PortWrite::Unhandled);
        assert_eq!(chip.take_nmi_pulse(), (1 << 5) | 1);
        assert_eq!(chip.take_nmi_pulse(), 0);
        assert_eq!(chip.read(NMIGR0, TEST_CONTEXT), Some(0), "NMIGR reads zero");
    }

    #[test]
    fn devctl_needs_its_key_for_every_write() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN, 0x0009_102f);
        assert_ne!(
            chip.write(DEVCTL, 1 << 12, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(chip.read(DEVCTL, TEST_CONTEXT), Some(0), "no key, no write");

        assert_ne!(
            chip.write(DEVCTL_KEY, DEVCTL_UNLOCK, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_ne!(
            chip.write(DEVCTL, 1 << 12, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(chip.read(DEVCTL, TEST_CONTEXT), Some(1 << 12));

        // The key unlocked exactly one write.
        assert_ne!(chip.write(DEVCTL, 0, TEST_CONTEXT), PortWrite::Unhandled);
        assert_eq!(chip.read(DEVCTL, TEST_CONTEXT), Some(1 << 12));

        // A wrong key unlocks nothing.
        assert_ne!(
            chip.write(DEVCTL_KEY, 0x1234_5678, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_ne!(chip.write(DEVCTL, 0, TEST_CONTEXT), PortWrite::Unhandled);
        assert_eq!(chip.read(DEVCTL, TEST_CONTEXT), Some(1 << 12));
    }

    #[test]
    fn the_permission_map_needs_its_own_key() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN, 0x0009_102f);
        assert_ne!(
            chip.write(PRIVPERM, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            chip.read(PRIVPERM, TEST_CONTEXT),
            Some(0),
            "no key, no write"
        );
        assert_ne!(
            chip.write(PRIVKEY, PRIVKEY_UNLOCK, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_ne!(
            chip.write(PRIVPERM, 0x0000_00c0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(chip.read(PRIVPERM, TEST_CONTEXT), Some(0x0000_00c0));
        assert_ne!(chip.write(PRIVPERM, 0, TEST_CONTEXT), PortWrite::Unhandled);
        assert_eq!(
            chip.read(PRIVPERM, TEST_CONTEXT),
            Some(0x0000_00c0),
            "one write per key"
        );
    }

    #[test]
    fn priority_allocation_resets_lowest_with_reserved_bits_reading_as_ones() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN, 0x0009_102f);
        assert_eq!(chip.read(PRI_ALLOC, TEST_CONTEXT), Some(0xffff_ffff));
        // Give EMAC0 the highest priority and leave the rest alone.
        assert_ne!(
            chip.write(PRI_ALLOC, 0xffff_fff8, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        let value = chip.read(PRI_ALLOC, TEST_CONTEXT).unwrap();
        assert_eq!(value & 0b111, 0, "EMAC0 is now priority 0");
        assert_eq!(value & PRI_ALLOC_RESERVED, PRI_ALLOC_RESERVED);
    }

    #[test]
    fn a_locked_reset_mux_ignores_every_later_write() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN, 0x0009_102f);
        assert_eq!(chip.read(RSTMUX0, TEST_CONTEXT), Some(RSTMUX_RESET));
        // Local reset on a watchdog timeout, then lock.
        assert_ne!(
            chip.write(RSTMUX0, (0b010 << 1) | RSTMUX_LOCK, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        let locked = chip.read(RSTMUX0, TEST_CONTEXT).unwrap();
        assert_ne!(
            chip.write(RSTMUX0, 0b101 << 1, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(chip.read(RSTMUX0, TEST_CONTEXT), Some(locked));
    }

    #[test]
    fn devstat_is_read_only_and_reports_the_pins() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN | 2, 0x0009_102f);
        assert_ne!(chip.write(DEVSTAT, 0, TEST_CONTEXT), PortWrite::Unhandled);
        assert_eq!(
            chip.read(DEVSTAT, TEST_CONTEXT),
            Some(DEVSTAT_LITTLE_ENDIAN | 2)
        );
        assert_eq!(chip.read(DEVICE_ID, TEST_CONTEXT), Some(0x0009_102f));
    }

    #[test]
    fn the_timer_pin_manager_keeps_only_its_selector() {
        let mut chip = ChipConfig::new(DEVSTAT_LITTLE_ENDIAN, 0x0009_102f);
        assert_ne!(
            chip.write(TPMGR, 0xffff_fff3, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(chip.read(TPMGR, TEST_CONTEXT), Some(3));
    }
}
