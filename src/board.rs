//! The board around the device and the boot it straps.
//!
//! A board straps the BOOTMODE and configuration pins, puts a serial EEPROM on the I2C bus,
//! fuses a MAC ID and cables an ATM master or a host to the device. The strap is resolved once
//! into a [`BootSource`] and checked against what the board actually supplies, and from then on
//! this module is the only one that knows which boot a device runs: the device starts the
//! [`Boot`] when it is built, ticks it every device cycle and asks it what it did; the machine
//! asks it what to report.

use crate::bootctl::MAX_CORES;
use crate::device::DeviceProfile;
use crate::hpi::HPI_BOOT_ENTRY;
use crate::i2cboot::{I2cBoot, Progress as I2cProgress, Rom as I2cRom};
use crate::machine::MachineArgs;
use crate::romboot::{BootParameters, EmacBoot, Progress as EthernetProgress};
use crate::system::Fabric;
use crate::utopiaboot::{Progress as UtopiaProgress, UtopiaBoot};
use serde_json::{json, Map, Value};

/// What the strapped boot mode asks for, resolved once against the device profile. The pins
/// themselves still reach the device, which latches them into its registers; every host-side
/// decision - which arguments a boot needs, whether the ELF is loaded, which boot the ROM
/// runs - reads this instead.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BootSource {
    /// No ROM or host boot: the machine places the ELF image itself.
    Image,
    /// HPI host boot, mode 1 on either device (SPRS276M section 2.4, SPRUEC6G section 4.3.2).
    Hpi,
    /// The master I2C boot from a serial EEPROM: the C6455's mode 5, the C6472's modes 2 and
    /// 3. Which EEPROM address the mode names is the boot ROM's own reading of the pins.
    I2c,
    /// The C6472 ROM's UTOPIA boot (SPRUEC6G section 4.3.4). The C6455's modes 5-8
    /// are other boots (section 2.3).
    Utopia,
    /// Only the C6472 ROM boots over Ethernet (SPRUEC6G section 4.3.5, modes 9 and 10).
    /// The C6455 boot mode table has no Ethernet entry (section 2.3), so these pins
    /// are refused rather than quietly held in reset.
    Ethernet { port: usize, base: u32 },
}

impl BootSource {
    pub(crate) fn resolve(profile: &'static DeviceProfile, strap: u8) -> Result<Self, String> {
        if strap == crate::hpi::BOOTMODE_HPI {
            return Ok(Self::Hpi);
        }
        if profile.has_boot_controller && crate::utopiaboot::is_boot_mode(strap) {
            return Ok(Self::Utopia);
        }
        if crate::i2cboot::eeprom_address(profile.has_boot_controller, strap).is_some() {
            return Ok(Self::I2c);
        }
        if let Some(port) = crate::romboot::mac_boot_port(strap) {
            if !profile.has_boot_controller || profile.emac_bases.len() <= port {
                return Err(format!(
                    "boot mode {strap} is the C6472's boot over MAC port {port}; the {} has no \
                     Ethernet boot mode (SPRUEC6G sections 2.3 and 4.3.5)",
                    profile.name
                ));
            }
            return Ok(Self::Ethernet {
                port,
                base: profile.emac_bases[port],
            });
        }
        Ok(Self::Image)
    }
}

/// The board: which device it carries, what it straps and fuses, and what the strapped boot
/// reads. Held by the machine so a reload builds the same board again.
pub(crate) struct Board {
    profile: &'static DeviceProfile,
    mode: u8,
    source: BootSource,
    /// The address fused into the MAC ID words; a board built for a test may fuse none.
    mac_id: Option<[u8; 6]>,
    /// The EEPROM on the I2C bus, which only the I2C boot reads.
    i2c_eeprom: Option<Vec<u8>>,
    /// The configuration pins an I2C boot reads: the parameter block index in the low bits,
    /// and on the C6472 the PLL1 select in CFG[4] (SPRUEC6G Table 22).
    cfggp: u8,
    /// The PHY address a UTOPIA boot answers to, on CFGGP[4:0].
    utopia_phy: u8,
    /// The UDP port an Ethernet boot accepts boot-table frames on.
    udp_port: u16,
}

impl Board {
    /// The board the command line describes, refused when it straps a boot it does not supply
    /// the source for or supplies something nothing it straps reads.
    pub(crate) fn from_args(
        profile: &'static DeviceProfile,
        args: &MachineArgs,
    ) -> Result<Self, String> {
        // The strap is an Ethernet boot the device does not have: nothing else about the
        // command line matters until that is fixed.
        let source = BootSource::resolve(profile, args.boot_mode)?;
        let i2c_boot = source == BootSource::I2c;
        if source == BootSource::Utopia {
            if args.utopia_unix.is_none() {
                return Err("C6472 UTOPIA boot requires an ATM master on --utopia-unix".into());
            }
            if args.i2c_eeprom.is_some() || args.boot_config.is_some() {
                return Err(
                    "UTOPIA boot receives its image from the ATM master; --i2c-eeprom and \
                     --boot-config cannot be combined with it"
                        .into(),
                );
            }
        } else if args.utopia_phy.is_some() {
            return Err(
                "--utopia-phy straps the PHY address a C6472 UTOPIA boot reads (--device c6472 \
                 --boot-mode 5-8); nothing else reads it"
                    .into(),
            );
        }
        if source == BootSource::Hpi {
            if args.hpi_unix.is_none() && args.hpi_replay.is_none() {
                return Err(format!(
                    "boot mode {} is the {}'s HPI host boot; give it a host with --hpi-unix or \
                     --hpi-replay",
                    args.boot_mode, profile.name
                ));
            }
            if args.i2c_eeprom.is_some() {
                return Err("--i2c-eeprom and HPI host boot are two boot sources; use one".into());
            }
            if args.boot_config.is_some() {
                return Err(
                    "--boot-config cannot be used with HPI host boot; the host loads memory directly"
                        .into(),
                );
            }
        }
        let i2c_eeprom = match &args.i2c_eeprom {
            Some(path) => {
                Some(std::fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?)
            }
            None => None,
        };
        if i2c_boot && i2c_eeprom.is_none() {
            return Err(format!(
                "boot mode {} is the {}'s master I2C boot; give it the EEPROM with --i2c-eeprom",
                args.boot_mode, profile.name
            ));
        }
        if !i2c_boot && i2c_eeprom.is_some() {
            return Err(format!(
                "--i2c-eeprom is read by the master I2C boot, which is {} on the {}",
                if profile.has_boot_controller {
                    "--boot-mode 2 (EEPROM at 0x50) or 3 (0x51)"
                } else {
                    "--boot-mode 5"
                },
                profile.name
            ));
        }
        // The configuration pins: CFGGP[2:0] are the C6455's index, CFG[3:0] the C6472's and
        // CFG[4] its PLL select (SPRUEC6G section 2.3.4.2 and Table 22).
        let index_limit = if profile.has_boot_controller { 16 } else { 8 };
        if args.boot_param_index >= index_limit {
            return Err(format!(
                "--boot-param-index {} is beyond the {} parameter blocks the {}'s pins select",
                args.boot_param_index, index_limit, profile.name
            ));
        }
        if args.i2c_pll_x19 && !(i2c_boot && profile.has_boot_controller) {
            return Err(
                "--i2c-pll-x19 straps CFG[4] for a C6472 I2C boot (--device c6472 \
                        --boot-mode 2 or 3); nothing else reads it"
                    .into(),
            );
        }
        if source != BootSource::Image {
            // Two boot sources are one too many.
            if args.boot_table.is_some() {
                return Err("--boot-table and a ROM boot are two boot sources; use one".into());
            }
            // A capture file cannot be the Ethernet boot's source: the boot code resets the MAC
            // before it configures it, so frames handed over before the announcement are gone.
            if matches!(source, BootSource::Ethernet { .. }) && args.pcap_in.is_some() {
                return Err(
                    "--pcap-in frames arrive before an Ethernet boot has configured the \
                            MAC and are lost with its reset; drive the boot through --net-unix"
                        .into(),
                );
            }
        }
        Ok(Self {
            profile,
            mode: args.boot_mode,
            source,
            mac_id: Some(args.mac_id),
            i2c_eeprom,
            cfggp: args.boot_param_index as u8 | (u8::from(args.i2c_pll_x19) << 4),
            utopia_phy: args.utopia_phy.unwrap_or(0),
            udp_port: args.boot_udp_port,
        })
    }

    /// A board that straps `mode` and supplies nothing else: no EEPROM, no fused MAC ID.
    #[cfg(test)]
    pub(crate) fn pins(profile: &'static DeviceProfile, mode: u8) -> Self {
        Self {
            profile,
            mode,
            source: BootSource::resolve(profile, mode).expect("a strap the device has"),
            mac_id: None,
            i2c_eeprom: None,
            cfggp: 0,
            utopia_phy: 0,
            udp_port: BootParameters::default().udp_destination_port,
        }
    }

    /// Put an EEPROM on the I2C bus and drive the configuration pins the I2C boot reads.
    #[cfg(test)]
    pub(crate) fn with_i2c_eeprom(mut self, cfggp: u8, contents: Vec<u8>) -> Self {
        self.cfggp = cfggp;
        self.i2c_eeprom = Some(contents);
        self
    }

    /// Strap the PHY address a UTOPIA boot answers to.
    #[cfg(test)]
    pub(crate) fn with_utopia_phy(mut self, phy: u8) -> Self {
        self.utopia_phy = phy;
        self
    }

    /// Fuse a MAC ID into the device.
    #[cfg(test)]
    pub(crate) fn with_mac_id(mut self, mac: [u8; 6]) -> Self {
        self.mac_id = Some(mac);
        self
    }

    pub(crate) fn profile(&self) -> &'static DeviceProfile {
        self.profile
    }

    /// The BOOTMODE pins, which the fabric latches into its registers.
    pub(crate) fn mode(&self) -> u8 {
        self.mode
    }

    pub(crate) fn source(&self) -> BootSource {
        self.source
    }

    /// The MAC the board cables its one wire to when the command line names none: MAC 0, or
    /// the port an Ethernet boot was strapped to, because that is the port it boots from.
    pub(crate) fn default_net_port(&self) -> usize {
        match self.source {
            BootSource::Ethernet { port, .. } => port,
            _ => 0,
        }
    }

    /// Fuse, strap and attach what the board supplies, and start the boot it straps. Core 0
    /// is held while a ROM boot runs in its place; the other cores stay as the strap left them
    /// until the boot table releases them.
    pub(crate) fn start(&self, fabric: &mut Fabric) -> Boot {
        // The board's fused Ethernet address: what the ROM's Ethernet boot names itself by, and
        // what any firmware reads out of the MAC ID words (SPRS612G Table 3-2).
        if let Some(mac) = self.mac_id {
            fabric.fuse_mac_id(mac);
        }
        let endian = fabric.endian();
        let rom = match self.source {
            BootSource::Image => None,
            BootSource::Hpi => {
                // SPRUEC6G Table 2 and Table 21: an HPI boot leaves 32 KB of each L1 cache and
                // all of L2 as SRAM, in every core there is. Written through the registers,
                // like every other cache configuration.
                fabric.configure_caches(4, 4, 0);
                None
            }
            // SPRUEC6G section 4.3.5.
            BootSource::Ethernet { base, .. } => Some(RomBoot::Emac(EmacBoot::new(
                base,
                endian,
                BootParameters {
                    udp_destination_port: self.udp_port,
                    ..BootParameters::default()
                },
            ))),
            // SPRUEC6G section 4.3.4: the mode picks the port width and PLL multiplier, and the
            // ATM master reaches the receive queue through the fabric's UTOPIA master side.
            BootSource::Utopia => {
                let boot = UtopiaBoot::cold(self.mode, self.utopia_phy, endian);
                fabric.strap_utopia(self.utopia_phy, boot.is_wide());
                Some(RomBoot::Utopia(boot))
            }
            // SPRUEC6G sections 2.3.4 and 4.3.3. The C6455 reads its parameter block index from
            // CFGGP[2:0]; the C6472 from CFG[3:0], with CFG[4] choosing PLL1's x9 or x19.
            BootSource::I2c => self.i2c_eeprom.as_ref().map(|contents| {
                let c6472 = self.profile.has_boot_controller;
                let slave = crate::i2cboot::eeprom_address(c6472, self.mode)
                    .expect("the boot mode is the device's master I2C boot");
                let (rom, index) = if c6472 {
                    let pll_multiplier = if self.cfggp & 0x10 != 0 { 19 } else { 9 };
                    fabric.strap_cfggp(self.cfggp);
                    (I2cRom::C6472 { pll_multiplier }, self.cfggp & 0xf)
                } else {
                    (I2cRom::C6455, self.cfggp & 0x7)
                };
                fabric.attach_i2c_eeprom(slave, contents.clone());
                RomBoot::I2c(I2cBoot::new(rom, slave, u32::from(index), endian))
            }),
        };
        if rom.is_some() {
            fabric.set_held_in_reset(0, true);
        }
        Boot {
            rom,
            hpi: self.source == BootSource::Hpi,
            hpi_entries: [None; MAX_CORES],
        }
    }
}

/// Which of the ROM's boot modes is running in place of core 0. They are separate state
/// machines because they share nothing but the boot table they end in and the core they hold.
enum RomBoot {
    Emac(EmacBoot),
    I2c(I2cBoot),
    Utopia(UtopiaBoot),
}

/// The boot a board strapped, as it runs and after it has: the ROM bootloader running in place
/// of core 0, or the HPI boot holding the cores for the host, or nothing. It outlives the boot
/// it performed, because what it did is what a front end reports afterwards.
pub(crate) struct Boot {
    rom: Option<RomBoot>,
    /// Strapped to HPI boot, which is hardware rather than ROM: the cores are held in reset for
    /// the host. The C6455's one core starts at the base of L2 on the host's DSPINT (SPRS276M
    /// section 2.4); each C6472 core starts at its DSP_BOOT_ADDR when the host sets its BC bit
    /// in BOOT_COMPLETE_STAT (SPRUEC6G section 4.3.2).
    hpi: bool,
    /// Where the host's release started each core of an HPI boot, `None` while it is held.
    hpi_entries: [Option<u32>; MAX_CORES],
}

impl Boot {
    /// No boot: a device built around a CPU a test made itself.
    #[cfg(test)]
    pub(crate) fn none() -> Self {
        Self {
            rom: None,
            hpi: false,
            hpi_entries: [None; MAX_CORES],
        }
    }

    /// Whether the boot holds core 0 - the ROM bootloader running in its place, or the HPI
    /// boot holding it in reset for the host. A run loop has to advance the device rather than
    /// the core while it does, and keeps counting in device cycles afterwards.
    pub(crate) fn holds_core_zero(&self) -> bool {
        self.rom.is_some() || self.hpi
    }

    /// One device cycle of the ROM boot, answering the entry point core 0 branches to when the
    /// boot table ends.
    pub(crate) fn tick(&mut self, fabric: &mut Fabric) -> Option<u32> {
        match self.rom.as_mut()? {
            RomBoot::Emac(boot) => boot.tick(fabric),
            RomBoot::I2c(boot) => boot.tick(fabric),
            RomBoot::Utopia(boot) => boot.tick(fabric),
        }
    }

    /// Whether a boot-controller release of `core` is the boot's to apply rather than the
    /// device's. BC0 never starts core 0 during a ROM boot: core 0 is the core that runs the
    /// boot code, so its bit only signals that the boot finished (SPRUEC6G section 4.1), and
    /// where core 0 goes is the boot table's entry point, which `tick` answers.
    pub(crate) fn owns_release(&self, core: usize) -> bool {
        core == 0 && self.rom.is_some()
    }

    /// The device has let `core` out of reset at `address`, or at wherever it was waiting when
    /// the device has no boot address for it. An HPI boot records where the host's release
    /// started a core it was holding; a C6455 has no boot controller, and its core is already
    /// waiting at the base of L2.
    pub(crate) fn released(&mut self, core: usize, was_held: bool, address: Option<u32>) {
        if self.hpi && was_held {
            self.hpi_entries[core] = Some(address.unwrap_or(HPI_BOOT_ENTRY));
        }
    }

    /// What the Ethernet boot has done, when the board strapped one.
    pub(crate) fn ethernet(&self) -> Option<EthernetProgress> {
        match self.rom.as_ref()? {
            RomBoot::Emac(boot) => Some(boot.progress()),
            _ => None,
        }
    }

    /// What the I2C boot has done, when the board strapped one.
    pub(crate) fn i2c(&self) -> Option<I2cProgress> {
        match self.rom.as_ref()? {
            RomBoot::I2c(boot) => Some(boot.progress()),
            _ => None,
        }
    }

    /// What the UTOPIA boot has done, when the board strapped one or an I2C boot's parameter
    /// block handed over to one.
    pub(crate) fn utopia(&self) -> Option<UtopiaProgress> {
        match self.rom.as_ref()? {
            RomBoot::Utopia(boot) => Some(boot.progress()),
            RomBoot::I2c(boot) => boot.utopia(),
            RomBoot::Emac(_) => None,
        }
    }

    /// Where the host's release started a core of an HPI boot, once it has: the base of L2 for
    /// the C6455's core 0, the core's DSP_BOOT_ADDR as the host left it for the C6472's. `None`
    /// while the core is still held, and for a core that no HPI boot released.
    pub(crate) fn hpi_entry(&self, core: usize) -> Option<u32> {
        self.hpi_entries.get(core).copied().flatten()
    }

    /// The machine state's boot members, keyed as the state JSON names them, for a device of
    /// `cores` cores. Empty when the board strapped no boot.
    pub(crate) fn report(&self, cores: usize) -> Map<String, Value> {
        let mut report = Map::new();
        if let Some(boot) = self.ethernet() {
            // What the ROM's Ethernet boot made of the wire, which is the only way to tell a
            // boot that completed from one still waiting for its next frame.
            report.insert(
                "ethernet_boot".into(),
                json!({
                    "frames_accepted": boot.accepted,
                    "frames_discarded": boot.discarded,
                    "table_bytes": boot.table_bytes,
                    "entry": boot.entry,
                }),
            );
        }
        if let Some(boot) = self.i2c() {
            // What the ROM's I2C boot made of the EEPROM, which is how a case tells a boot that
            // read its table from one that stopped on a bad block.
            report.insert(
                "i2c_boot".into(),
                json!({
                    "parameter_blocks": boot.parameter_blocks,
                    "config_blocks": boot.config_blocks,
                    "table_blocks": boot.table_blocks,
                    "table_bytes": boot.table_bytes,
                    "checksum_failures": boot.checksum_failures,
                    "entry": boot.entry,
                    "error": boot.error,
                }),
            );
        }
        if let Some(boot) = self.utopia() {
            report.insert(
                "utopia_boot".into(),
                json!({
                    "cells_accepted": boot.accepted,
                    "cells_discarded": boot.discarded,
                    "table_bytes": boot.table_bytes,
                    "entry": boot.entry,
                    "error": boot.error,
                }),
            );
        }
        if self.hpi {
            report.insert(
                "hpi_boot".into(),
                json!({
                    "entry": self.hpi_entry(0),
                    "entries": (0..cores).map(|core| self.hpi_entry(core)).collect::<Vec<_>>(),
                }),
            );
        }
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolved_strap_table() {
        use crate::device::{C6455, C6472};

        for profile in [&C6455, &C6472] {
            for strap in 0..=u8::MAX {
                let expected = match (profile.name, strap) {
                    (_, 1) => Some(BootSource::Hpi),
                    ("c6455", 5) | ("c6472", 2..=3) => Some(BootSource::I2c),
                    ("c6472", 5..=8) => Some(BootSource::Utopia),
                    ("c6472", 9..=10) => {
                        let port = usize::from(strap - 9);
                        Some(BootSource::Ethernet {
                            port,
                            base: profile.emac_bases[port],
                        })
                    }
                    // The C6455 has no Ethernet boot; its pins are refused, not ignored.
                    ("c6455", 9..=10) => None,
                    _ => Some(BootSource::Image),
                };
                assert_eq!(
                    BootSource::resolve(profile, strap).ok(),
                    expected,
                    "{} strap {strap}",
                    profile.name
                );
            }
        }
    }
}
