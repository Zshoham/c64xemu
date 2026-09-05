//! The C6455 MDIO module and the PHY on the other end of it.
//!
//! The register layout is SPRU975E §4: a control register with the enable and clock divider,
//! the ALIVE and LINK bitmaps, the link-change interrupt, and two user access registers that
//! carry clause-22 management transactions. Transactions do not complete instantly - the
//! driver is expected to poll the GO bit - so this model finishes them a fixed number of
//! cycles later, which keeps that polling loop honest.
use crate::port::{PortContext, PortWrite, SlavePort};
use std::collections::BTreeMap;
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const CONTROL: u32 = 0x02c8_1804;
const ALIVE: u32 = 0x02c8_1808;
const LINK: u32 = 0x02c8_180c;
const LINKINTRAW: u32 = 0x02c8_1810;
const LINKINTMASKED: u32 = 0x02c8_1814;
const USERINTRAW: u32 = 0x02c8_1820;
const USERINTMASKED: u32 = 0x02c8_1824;
const USERINTMASKSET: u32 = 0x02c8_1828;
const USERINTMASKCLEAR: u32 = 0x02c8_182c;
const USERACCESS0: u32 = 0x02c8_1880;
const USERPHYSEL0: u32 = 0x02c8_1884;
const USERACCESS1: u32 = 0x02c8_1888;
const USERPHYSEL1: u32 = 0x02c8_188c;
const MMD_CONTROL: u32 = 13;
const MMD_DATA: u32 = 14;
const VERSION: u32 = 0x02c8_1800;

const GO: u32 = 1 << 31;
const WRITE: u32 = 1 << 30;
const ACK: u32 = 1 << 29;
const CONTROL_ENABLE: u32 = 1 << 30;

/// Cycles a management transaction takes before the GO bit clears. Real hardware needs the
/// MDIO clock to shift 64 bits; the exact count is a divider setting, so this is just long
/// enough that firmware has to poll.
const TRANSACTION_CYCLES: u32 = 16;

/// The address the board's PHY answers on.
const PHY_ADDRESS: u32 = 0;

/// Clause-22 register numbers this model implements.
const BMCR: u32 = 0;
const BMSR: u32 = 1;
const PHYID1: u32 = 2;
const PHYID2: u32 = 3;
const ANAR: u32 = 4;
const ANLPAR: u32 = 5;
const GBCR: u32 = 9;
const GBSR: u32 = 10;

/// BMCR bits the model honours.
const BMCR_RESET: u32 = 1 << 15;
const BMCR_RESTART_AN: u32 = 1 << 9;

/// BMSR: extended capability, auto-negotiation complete, link up, both 100 and 10 abilities.
const BMSR_LINK: u32 = 1 << 2;
const BMSR_AN_COMPLETE: u32 = 1 << 5;
const BMSR_DEFAULT: u32 = (1 << 0)
    | (1 << 3)
    | (1 << 6)
    | (1 << 11)
    | (1 << 12)
    | (1 << 13)
    | (1 << 14)
    | BMSR_AN_COMPLETE
    | BMSR_LINK;

pub struct Mdio {
    /// MDIOVER, which differs by device revision.
    version: u32,
    control: u32,
    link_int_raw: u32,
    user_int_raw: u32,
    user_int_mask: u32,
    access: [u32; 2],
    phy_select: [u32; 2],
    /// Cycles left before each pending transaction completes, and the value it will report.
    pending: [Option<(u32, u32)>; 2],
    phy: [u32; 32],
    mmd_control: u16,
    mmd_address: [u16; 32],
    mmd: BTreeMap<(u8, u16), u16>,
    link: bool,
}

impl Default for Mdio {
    fn default() -> Self {
        let mut phy = [0u32; 32];
        // A plausible gigabit PHY: 1000/100/10 full and half duplex advertised on both sides,
        // auto-negotiation already complete and the link up.
        phy[BMCR as usize] = 0x1140;
        phy[BMSR as usize] = BMSR_DEFAULT;
        phy[PHYID1 as usize] = 0x0141;
        phy[PHYID2 as usize] = 0x0cc2;
        phy[ANAR as usize] = 0x01e1;
        phy[ANLPAR as usize] = 0x01e1;
        phy[GBCR as usize] = 0x0300;
        phy[GBSR as usize] = 0x0c00;
        let mut mmd = BTreeMap::new();
        // Clause-45 PMA/PMD identity and link status for the same gigabit PHY.
        mmd.insert((1, 1), BMSR_LINK as u16);
        mmd.insert((1, 2), 0x0141);
        mmd.insert((1, 3), 0x0cc2);
        Self {
            version: 0x0007_0102,
            control: 0,
            link_int_raw: 0,
            user_int_raw: 0,
            user_int_mask: 0,
            access: [0; 2],
            phy_select: [0; 2],
            pending: [None, None],
            phy,
            mmd_control: 0,
            mmd_address: [0; 32],
            mmd,
            link: true,
        }
    }
}

impl Mdio {
    /// A module reporting the revision this device's guide documents: SPRU975E Table 41 gives
    /// 0x0007_0102 for the C645x and SPRUEF8F section 4.2 gives 0x0007_0103 for the C647x.
    pub fn with_version(version: u32) -> Self {
        Self {
            version,
            ..Self::default()
        }
    }
}

impl SlavePort for Mdio {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address {
            VERSION => self.version,
            CONTROL => self.control,
            ALIVE => self.alive(),
            LINK => self.link_bits(),
            LINKINTRAW => self.link_int_raw,
            LINKINTMASKED => self.link_int_raw,
            USERINTRAW => self.user_int_raw,
            USERINTMASKED => self.user_int_raw & self.user_int_mask,
            USERINTMASKSET | USERINTMASKCLEAR => self.user_int_mask,
            USERACCESS0 => self.access[0],
            USERACCESS1 => self.access[1],
            USERPHYSEL0 => self.phy_select[0],
            USERPHYSEL1 => self.phy_select[1],
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            CONTROL => self.control = value,
            LINKINTRAW => self.link_int_raw &= !value,
            USERINTRAW => self.user_int_raw &= !value,
            USERINTMASKSET => self.user_int_mask |= value & 3,
            USERINTMASKCLEAR => self.user_int_mask &= !(value & 3),
            USERACCESS0 => self.start(0, value),
            USERACCESS1 => self.start(1, value),
            USERPHYSEL0 => self.phy_select[0] = value,
            USERPHYSEL1 => self.phy_select[1] = value,
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl Mdio {
    pub fn active(&self) -> bool {
        self.pending.iter().any(Option::is_some)
    }

    pub fn tick(&mut self) {
        for channel in 0..2 {
            if let Some((remaining, result)) = self.pending[channel] {
                if remaining <= 1 {
                    self.access[channel] = result;
                    self.pending[channel] = None;
                    self.user_int_raw |= 1 << channel;
                } else {
                    self.pending[channel] = Some((remaining - 1, result));
                }
            }
        }
    }

    /// The user-access interrupt is shared with the EMAC's MACINT line.
    pub fn interrupt_pending(&self) -> bool {
        self.user_int_raw & self.user_int_mask != 0
    }

    /// The two MDIO bits the EMAC control module reports in MACINVECTOR (SPRU975E Table 40).
    pub fn link_interrupt_pending(&self) -> bool {
        self.link_int_raw != 0
    }

    pub fn user_interrupt_pending(&self) -> bool {
        self.user_int_raw & self.user_int_mask != 0
    }

    fn alive(&self) -> u32 {
        if self.control & CONTROL_ENABLE == 0 {
            0
        } else {
            1 << PHY_ADDRESS
        }
    }

    fn link_bits(&self) -> u32 {
        if self.control & CONTROL_ENABLE != 0 && self.link {
            1 << PHY_ADDRESS
        } else {
            0
        }
    }

    fn start(&mut self, channel: usize, request: u32) {
        if request & GO == 0 {
            self.access[channel] = request & !GO;
            return;
        }
        // The transaction is in flight: GO stays set until it finishes.
        self.access[channel] = request;
        let phy = (request >> 16) & 0x1f;
        let register = (request >> 21) & 0x1f;
        if self.control & CONTROL_ENABLE == 0 || phy != PHY_ADDRESS {
            // No PHY answers, so the transaction completes without an acknowledge.
            self.pending[channel] = Some((TRANSACTION_CYCLES, 0));
            return;
        }
        let result = if request & WRITE != 0 {
            self.write_phy(register, request & 0xffff);
            ACK
        } else {
            ACK | self.read_phy(register)
        };
        self.pending[channel] = Some((TRANSACTION_CYCLES, result));
    }

    fn read_phy(&mut self, register: u32) -> u32 {
        if register == MMD_DATA {
            return u32::from(self.read_mmd());
        }
        self.phy[(register & 0x1f) as usize] & 0xffff
    }

    fn write_phy(&mut self, register: u32, value: u32) {
        if register == MMD_CONTROL {
            self.mmd_control = value as u16;
            self.phy[MMD_CONTROL as usize] = value & 0xc01f;
            return;
        }
        if register == MMD_DATA {
            self.write_mmd(value as u16);
            return;
        }
        let index = (register & 0x1f) as usize;
        if register == BMCR {
            if value & BMCR_RESET != 0 {
                // A reset returns the control register to its default and re-runs
                // auto-negotiation, which completes immediately in this model.
                self.phy[BMCR as usize] = 0x1140;
                self.phy[BMSR as usize] = BMSR_DEFAULT;
                return;
            }
            if value & BMCR_RESTART_AN != 0 {
                self.phy[BMSR as usize] = BMSR_DEFAULT;
            }
            self.phy[index] = value & !BMCR_RESTART_AN;
            return;
        }
        if matches!(register, BMSR | PHYID1 | PHYID2 | ANLPAR | GBSR) {
            // Status and identifier registers are read-only.
            return;
        }
        self.phy[index] = value;
    }
    fn mmd_device_and_function(&self) -> (u8, u8) {
        (
            (self.mmd_control & 0x1f) as u8,
            ((self.mmd_control >> 14) & 3) as u8,
        )
    }

    fn read_mmd(&mut self) -> u16 {
        let (device, function) = self.mmd_device_and_function();
        let address = self.mmd_address[device as usize];
        let mut value = self.mmd.get(&(device, address)).copied().unwrap_or(0);
        if device == 1 && address == 1 {
            value = (value & !(BMSR_LINK as u16)) | if self.link { BMSR_LINK as u16 } else { 0 };
        }
        // Function 2 post-increments after reads and writes; function 3 only after writes.
        if function == 2 {
            self.mmd_address[device as usize] = address.wrapping_add(1);
        }
        value
    }

    fn write_mmd(&mut self, value: u16) {
        let (device, function) = self.mmd_device_and_function();
        if function == 0 {
            self.mmd_address[device as usize] = value;
            return;
        }
        let address = self.mmd_address[device as usize];
        self.mmd.insert((device, address), value);
        if matches!(function, 2 | 3) {
            self.mmd_address[device as usize] = address.wrapping_add(1);
        }
    }

    /// Change the link state, which latches the link-change interrupt.
    #[cfg(test)]
    pub fn set_link(&mut self, up: bool) {
        if self.link != up {
            self.link_int_raw |= 1 << PHY_ADDRESS;
        }
        self.link = up;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    fn settle(mdio: &mut Mdio) {
        for _ in 0..TRANSACTION_CYCLES {
            mdio.tick();
        }
    }

    #[test]
    fn a_read_transaction_keeps_go_set_until_it_finishes() {
        let mut mdio = Mdio::default();
        mdio.write(CONTROL, CONTROL_ENABLE | 0xff, TEST_CONTEXT);
        assert_eq!(mdio.read(ALIVE, TEST_CONTEXT), Some(1));
        assert_eq!(mdio.read(LINK, TEST_CONTEXT), Some(1));

        mdio.write(USERACCESS0, GO | (PHYID1 << 21), TEST_CONTEXT);
        assert_eq!(mdio.read(USERACCESS0, TEST_CONTEXT).unwrap() & GO, GO);
        settle(&mut mdio);
        let done = mdio.read(USERACCESS0, TEST_CONTEXT).unwrap();
        assert_eq!(done & GO, 0);
        assert_eq!(done & ACK, ACK);
        assert_eq!(done & 0xffff, 0x0141);
    }

    #[test]
    fn a_write_round_trips_through_a_writable_phy_register() {
        let mut mdio = Mdio::default();
        mdio.write(CONTROL, CONTROL_ENABLE | 0xff, TEST_CONTEXT);
        mdio.write(
            USERACCESS0,
            GO | WRITE | (ANAR << 21) | 0x0061,
            TEST_CONTEXT,
        );
        settle(&mut mdio);
        mdio.write(USERACCESS0, GO | (ANAR << 21), TEST_CONTEXT);
        settle(&mut mdio);
        assert_eq!(
            mdio.read(USERACCESS0, TEST_CONTEXT).unwrap() & 0xffff,
            0x0061
        );

        // Status registers ignore writes.
        mdio.write(USERACCESS0, GO | WRITE | (BMSR << 21), TEST_CONTEXT);
        settle(&mut mdio);
        mdio.write(USERACCESS0, GO | (BMSR << 21), TEST_CONTEXT);
        settle(&mut mdio);
        assert_eq!(
            mdio.read(USERACCESS0, TEST_CONTEXT).unwrap() & 0xffff,
            BMSR_DEFAULT
        );
    }

    #[test]
    fn no_phy_answers_when_the_module_is_disabled_or_the_address_is_wrong() {
        let mut mdio = Mdio::default();
        mdio.write(USERACCESS0, GO | (PHYID1 << 21), TEST_CONTEXT);
        settle(&mut mdio);
        assert_eq!(mdio.read(USERACCESS0, TEST_CONTEXT), Some(0));
        assert_eq!(mdio.read(ALIVE, TEST_CONTEXT), Some(0));

        mdio.write(CONTROL, CONTROL_ENABLE, TEST_CONTEXT);
        mdio.write(USERACCESS0, GO | (PHYID1 << 21) | (7 << 16), TEST_CONTEXT);
        settle(&mut mdio);
        assert_eq!(mdio.read(USERACCESS0, TEST_CONTEXT).unwrap() & ACK, 0);
    }

    #[test]
    fn a_link_change_latches_the_link_interrupt() {
        let mut mdio = Mdio::default();
        mdio.write(CONTROL, CONTROL_ENABLE, TEST_CONTEXT);
        assert_eq!(mdio.read(LINKINTRAW, TEST_CONTEXT), Some(0));
        mdio.set_link(false);
        assert_eq!(mdio.read(LINK, TEST_CONTEXT), Some(0));
        assert_eq!(mdio.read(LINKINTRAW, TEST_CONTEXT), Some(1));
        mdio.write(LINKINTRAW, 1, TEST_CONTEXT);
        assert_eq!(mdio.read(LINKINTRAW, TEST_CONTEXT), Some(0));
        mdio.set_link(true);
        assert_eq!(mdio.read(LINK, TEST_CONTEXT), Some(1));
        assert_eq!(mdio.read(LINKINTRAW, TEST_CONTEXT), Some(1));
    }
}
