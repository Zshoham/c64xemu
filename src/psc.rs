//! The C6472's power/sleep controller (PSC) at 0x02AE_0000.
//!
//! SPRUEG3B (TMS320C6472/TMS320TCI6486 Power/Sleep Controller User's Guide) with the device
//! register map in SPRS612G section 7.4, Table 7-3. The PSC is a global PSC (GPSC), which owns
//! every memory-mapped register, plus one local PSC (LPSC) per clock domain, which owns the
//! clock and the reset of one module. Software stages a module's next state in MDCTLn.NEXT, then
//! writes the GO bit of that module's power domain in PTCMD; PTSTAT.GOSTAT reads 1 while the
//! transition is in flight and MDSTATn.STATE reports the state the module actually reached
//! (SPRUEG3B sections 4.1.2 and 9.6).
//!
//! Module numbering. SPRUEG3B Table 3 names the fourteen clock domains LPSC1-LPSC14, while the
//! register map has MDSTAT0-13/MDCTL0-13 numbered from zero. The two software examples in
//! SPRUEG3B section 8 pin the offset down: Example 3 clock-gates a C64x+ megamodule through
//! MDCTL0 (megamodule0 is LPSC1) and Example 4 clock-gates TSIP0 through MDCTL9 (TSIP0 is
//! LPSC10). So MDCTLn/MDSTATn belong to LPSC(n+1), and the module numbers this file uses are:
//!
//! | module | LPSC | block            | module | LPSC  | block  |
//! |--------|------|------------------|--------|-------|--------|
//! | 0-5    | 1-6  | C64x+ megamodule | 9-11   | 10-12 | TSIP0-2|
//! | 6      | 7    | SRIO             | 12     | 13    | HPI    |
//! | 7      | 8    | EMAC0            | 13     | 14    | UTOPIA |
//! | 8      | 9    | EMAC1            |        |       |        |
//!
//! Power domains. SPRUEG3B section 1.1 gives nine power domains including the always-on domain,
//! and PTCMD/PTSTAT accordingly have GO0-GO8/GOSTAT0-GOSTAT8 (Tables 11 and 12). SPRUEG3B
//! Figure 2, redrawn as SPRS612G Figure 7-6, names the eight switchable ones A-H: GEM0-GEM5 LL2
//! and SMC page 0 and page 1, that is the six local L2 and two shared L2 memory power domains of
//! SPRUEG3B section 3. Every LPSC, and the GPSC itself, sits in the always-on domain (SPRUEG3B
//! sections 2.1.1 and 2.1.2), so all fourteen clock domains transition on GO0 - which is what
//! every example in the guide writes, for megamodules (section 4.1.2, Example 3) and for
//! peripherals (section 4.2, Example 4) alike. The eight memory domains carry no LPSC: they are
//! mastered by the UMC and the SMC through L2PDSLEEP/SL2PDSLEEP, and the PSC only lends them its
//! request-grant counter (SPRUEG3B section 2.2.1), so a GO to one of them completes the handshake
//! without changing any module.
//!
//! UNCERTAIN: no section maps a PTCMD bit number onto a Figure 2 domain letter. This file numbers
//! them in the order Figure 2 draws them - 0 always-on, 1-6 GEM0-GEM5 LL2, 7 SMC page 0, 8 SMC
//! page 1 - which is unambiguous only for domain 0, the one every documented sequence uses.
//!
//! Interrupts. The only error source is the emulation pair EMURST/EMUIHB reported per module in
//! MDSTATn and enabled per module by MDCTLn.EMURSTIE/EMUIHBIE; a condition latches in MERRPR and
//! stays until software writes the matching MERRCR bit (SPRUEG3B sections 7.1 and 7.2). ALLINT is
//! asserted with ERRINT and holds until ERRINT clears (section 7.3), and on this device ALLINT is
//! the line the megamodules see: system event 65, PSC_ALLINT (SPRS612G section 7.6, Table 7-10).
//! `interrupt_pending` reports that line.
//!
//! Not modelled, deliberately: the clock-stop handshake. SPRUEG3B section 4.1.1 gates a
//! megamodule's clock only once the megamodule has executed IDLE, and section 4.2 makes TSIPs
//! wait until they are disabled; every other peripheral acknowledges at once. This model always
//! acknowledges at once, so a transition completes one device cycle after its GO. Software that
//! follows the documented sequence - IDLE first, then request the disable - sees exactly the
//! documented result. The IcePick InhibitSleep and ForceActive overrides of section 6.1 are not
//! modelled either.

use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const BASE: u32 = 0x02ae_0000;
const PID: u32 = BASE;
const INTEVAL: u32 = BASE + 0x018;
const MERRPR: u32 = BASE + 0x040;
const MERRCR: u32 = BASE + 0x050;
const PTCMD: u32 = BASE + 0x120;
const PTSTAT: u32 = BASE + 0x128;
/// MDSTAT0-13 at 800h + 4n and MDCTL0-13 at A00h + 4n (SPRS612G Table 7-3).
const MDSTAT0: u32 = BASE + 0x800;
const MDCTL0: u32 = BASE + 0xa00;

/// The fourteen software-controlled clock domains, in the order MDSTAT0-13 and MDCTL0-13 name
/// them: the six C64x+ megamodules are LPSC1-6, then SRIO, the two Ethernet MACs, the three
/// TSIPs, the host port and the UTOPIA (SPRUEG3B section 1.1 and Table 3).
pub(crate) const MODULES: usize = 14;
pub(crate) const MODULE_GEM0: usize = 0;
#[cfg(test)]
pub(crate) const MODULE_GEM2: usize = 2;
#[cfg(test)]
pub(crate) const MODULE_GEM3: usize = 3;
pub(crate) const MODULE_GEM5: usize = 5;
#[cfg(test)]
pub(crate) const MODULE_SRIO: usize = 6;
#[cfg(test)]
pub(crate) const MODULE_EMAC0: usize = 7;
#[cfg(test)]
pub(crate) const MODULE_EMAC1: usize = 8;
#[cfg(test)]
pub(crate) const MODULE_TSIP0: usize = 9;
#[cfg(test)]
pub(crate) const MODULE_HPI: usize = 12;
#[cfg(test)]
pub(crate) const MODULE_UTOPIA: usize = 13;

/// The nine power domains (SPRUEG3B section 1.1, Tables 11 and 12). Domain 0 holds the GPSC and
/// every LPSC; domains 1-8 are the memory domains the UMC and the SMC master.
pub(crate) const DOMAINS: usize = 9;
pub(crate) const DOMAIN_ALWAYS_ON: usize = 0;
#[cfg(test)]
pub(crate) const DOMAIN_GEM0_LL2: usize = 1;
#[cfg(test)]
pub(crate) const DOMAIN_SMC_PAGE1: usize = 8;

/// MERRPR/MERRCR carry one bit per module in 13:0 (SPRUEG3B Tables 9 and 10).
const MODULE_MASK: u32 = (1 << MODULES) - 1;
/// PTCMD/PTSTAT carry one bit per power domain in 8:0 (SPRUEG3B Tables 11 and 12).
const DOMAIN_MASK: u32 = (1 << DOMAINS) - 1;

/// PID: SCHEME 01b, FUNC 2808h, MAJOR_REVISION 001b, every other field zero (SPRUEG3B Table 7).
const PID_VALUE: u32 = (0b01 << 30) | (0x2808 << 16) | (0b001 << 8);

/// INTEVAL, a command register with no storage (SPRUEG3B Table 8).
const INTEVAL_ERRSET: u32 = 1 << 17;
const INTEVAL_ALLSET: u32 = 1 << 16;
const INTEVAL_ERREV: u32 = 1 << 1;
const INTEVAL_ALLEV: u32 = 1 << 0;

/// MDSTAT fields (SPRUEG3B Table 13).
const MDSTAT_EMUHIB: u32 = 1 << 17;
const MDSTAT_EMURST: u32 = 1 << 16;
const MDSTAT_MCKOUT: u32 = 1 << 12;
const MDSTAT_MRSTDONE: u32 = 1 << 11;
const MDSTAT_MRSTZ: u32 = 1 << 10;
const MDSTAT_LRSTDONE: u32 = 1 << 9;
const MDSTAT_LRSTZ: u32 = 1 << 8;

/// MDCTL fields (SPRUEG3B Table 14). NEXT is five bits wide with only 0-3 defined.
const MDCTL_EMUIHBIE: u32 = 1 << 10;
const MDCTL_EMURSTIE: u32 = 1 << 9;
const MDCTL_LRSTZ: u32 = 1 << 8;
const MDCTL_NEXT: u32 = 0x1f;
const MDCTL_WRITABLE: u32 = MDCTL_EMUIHBIE | MDCTL_EMURSTIE | MDCTL_LRSTZ | MDCTL_NEXT;

/// The four user-defined states and the transitional codes this model reports (SPRUEG3B Table 5
/// and Table 13). A state with bit 5 set is transitional.
const STATE_SW_RST_DISABLE: u32 = 0b00_0000;
const STATE_SYNC_RST: u32 = 0b00_0001;
const STATE_DISABLE: u32 = 0b00_0010;
const STATE_ENABLE: u32 = 0b00_0011;
const STATE_DISABLE_IN_PROG: u32 = 0b10_0000;
const STATE_ENABLE_IN_PROG: u32 = 0b10_0100;
/// ClkOn, documented for SwRstDisable to SyncRst.
const STATE_CLK_ON: u32 = 0b10_0111;
/// ClkOff, documented for SyncRst to SwRstDisable.
const STATE_CLK_OFF: u32 = 0b10_1000;

/// The transitional STATE reported between the GO and its completion.
///
/// UNCERTAIN: Table 5 names the two ClkOn/ClkOff codes only for the SwRstDisable-SyncRst pair, so
/// a request that reaches SyncRst or SwRstDisable from elsewhere reports the same code here for
/// want of a documented one. The In-Prog codes for Enable and Disable are unambiguous.
fn transitional_state(target: u32) -> u32 {
    match target {
        STATE_ENABLE => STATE_ENABLE_IN_PROG,
        STATE_DISABLE => STATE_DISABLE_IN_PROG,
        STATE_SYNC_RST => STATE_CLK_ON,
        _ => STATE_CLK_OFF,
    }
}

/// The module a word address in a fourteen-register array selects, if it is one exactly.
fn module_of(address: u32, base: u32) -> Option<usize> {
    let offset = address.checked_sub(base)?;
    if offset % 4 != 0 || offset / 4 >= MODULES as u32 {
        return None;
    }
    Some((offset / 4) as usize)
}

pub(crate) struct PowerSleep {
    mdctl: [u32; MODULES],
    /// The state each module has settled in. MDSTAT.STATE reports a transitional code instead
    /// while `transition` holds one, but the clock and reset bits keep reporting this state.
    state: [u32; MODULES],
    transition: [Option<u32>; MODULES],
    /// PTSTAT.GOSTAT, one level bit per power domain.
    gostat: u32,
    /// MERRPR, latched error conditions.
    merrpr: u32,
    /// The emulation reset and inhibit-sleep levels the device drives into each module, reported
    /// in MDSTAT.EMURST and MDSTAT.EMUHIB. One bit per module.
    emu_reset: u32,
    emu_inhibit: u32,
    local_reset_edges: Vec<(usize, bool)>,
    /// A one-cycle PSC_ALLINT pulse forced by INTEVAL, cleared by the next `tick`.
    pulse: bool,
}

impl Default for PowerSleep {
    /// Power-on reset. SPRUEG3B section 5.1: every power domain but the always-on one is off and
    /// every LPSC module domain is in SwRstDisable, which section 4.1.2 defines as clocks gated
    /// and module reset asserted - so MDSTAT reads MCKOUT = 0, MRSTZ = 0, MRSTDONE = 0.
    ///
    /// No figure in SPRUEG3B carries a "-n" reset annotation, so MDCTL has no documented reset
    /// value; this model chooses 0, which agrees with the SwRstDisable module state (NEXT = 0),
    /// leaves both error-interrupt enables off, and leaves LRSTZ = 0, local reset asserted.
    ///
    /// That last bit is a level, not an edge: `take_local_reset_edges` starts empty, and a device
    /// must not read the reset-time level as an instruction to hold its cores in reset. Releasing
    /// a core from reset after a device reset belongs to the boot controller (SPRS612G section
    /// 2.4.1); the PSC only ever *changes* a core's local reset, which is what the edge reports.
    fn default() -> Self {
        Self {
            mdctl: [0; MODULES],
            state: [STATE_SW_RST_DISABLE; MODULES],
            transition: [None; MODULES],
            gostat: 0,
            merrpr: 0,
            emu_reset: 0,
            emu_inhibit: 0,
            local_reset_edges: Vec::new(),
            pulse: false,
        }
    }
}

impl PowerSleep {
    /// The controller as the boot ROM leaves it: every module enabled and out of reset.
    ///
    /// SPRUEG3B section 5.1 has power-on reset put every LPSC in SwRstDisable, and the boot
    /// code then enables what the device needs. An emulator that loads an image directly never
    /// runs that code, so it starts from the state the image is entitled to assume; firmware
    /// that drives the PSC afterwards still sees every documented transition.
    pub(crate) fn booted() -> Self {
        let mut psc = Self::default();
        for module in 0..MODULES {
            psc.state[module] = STATE_ENABLE;
            psc.mdctl[module] = MDCTL_LRSTZ | STATE_ENABLE;
        }
        psc
    }
}

impl SlavePort for PowerSleep {
    /// None when the address is not ours. Every register answers at its exact word address only;
    /// the reserved gaps of SPRS612G Table 7-3 are not ours.
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        if let Some(module) = module_of(address, MDSTAT0) {
            return Some(self.mdstat(module));
        }
        if let Some(module) = module_of(address, MDCTL0) {
            return Some(self.mdctl[module]);
        }
        let value = match address {
            PID => PID_VALUE,
            // INTEVAL, MERRCR and PTCMD are command registers with no storage and read 0
            // (SPRUEG3B sections 9.2, 9.4 and 9.5).
            INTEVAL | MERRCR | PTCMD => 0,
            MERRPR => self.merrpr,
            PTSTAT => self.gostat,
            _ => return None,
        };
        Some(value)
    }

    /// `PortWrite::Unhandled` when the address is not ours.
    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if module_of(address, MDSTAT0).is_some() {
            // MDSTAT is read-only (SPRUEG3B section 7.1). UNCERTAIN: Figure 12 marks its EMUHIB,
            // EMURST, MCKOUT, MRSTDONE, MRSTZ, LRSTDONE, LRSTZ and STATE fields R/W, which
            // contradicts both that sentence and the register's purpose; this model drops writes.
            return PortWrite::Done;
        }
        if let Some(module) = module_of(address, MDCTL0) {
            self.write_mdctl(module, value);
            return PortWrite::Done;
        }
        match address {
            // PID is a hard-coded constant and MERRPR and PTSTAT are status registers.
            PID | MERRPR | PTSTAT => {}
            INTEVAL => self.write_inteval(value),
            MERRCR => self.write_merrcr(value),
            PTCMD => self.write_ptcmd(value),
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl PowerSleep {
    /// Advance one device cycle: complete the transitions a GO started and drop a forced
    /// interrupt pulse.
    ///
    /// A transition takes exactly one cycle here, so GOSTAT is observably 1 between the GO and
    /// the next cycle. SPRUEG3B section 9.6 only requires that GOSTAT be 1 while a module of the
    /// domain differs from its NEXT and revert to 0 when every state machine of the domain has
    /// finished (section 4.1.2); the guide gives no cycle count.
    pub(crate) fn tick(&mut self) {
        self.pulse = false;
        if self.gostat == 0 {
            return;
        }
        for module in 0..MODULES {
            if self.transition[module].take().is_some() {
                self.state[module] = self.mdctl[module] & MDCTL_NEXT;
            }
        }
        self.gostat = 0;
    }

    /// The PSC_ALLINT line, system event 65 (SPRS612G section 7.6, Table 7-10).
    ///
    /// High while any MERRPR bit is set - SPRUEG3B section 7.1 keeps the internal pending signal
    /// high until every enabled condition is cleared, and section 7.3 holds ALLINT with it - and
    /// for the one cycle after an INTEVAL write that pulses the output.
    pub(crate) fn interrupt_pending(&self) -> bool {
        self.merrpr != 0 || self.pulse
    }

    /// The LRSTZ edges since the last call, as (module, asserted): true when the module's local
    /// reset went from de-asserted to asserted, false when it was released. Cleared on read.
    ///
    /// The device consumes these exactly once, resetting the matching C64x+ megamodule core on an
    /// asserted edge and releasing it on the other. Only modules `MODULE_GEM0..=MODULE_GEM5`ever
    /// appear: SPRUEG3B section 5.2 gives the LPSC soft reset to the megamodules alone.
    pub(crate) fn take_local_reset_edges(&mut self) -> Vec<(usize, bool)> {
        std::mem::take(&mut self.local_reset_edges)
    }

    /// Whether a module's functional clock is running with the module out of reset, so the device
    /// can decide whether the peripheral responds at all.
    ///
    /// True only in Enable: SPRUEG3B section 4.1.2 has Disable gate the functional clocks and
    /// route new commands to null end points, SwRstDisable gate them and hold reset, and SyncRst
    /// run the clocks but hold the module in reset.
    pub(crate) fn module_enabled(&self, module: usize) -> bool {
        matches!(self.state.get(module), Some(&STATE_ENABLE))
    }

    /// Level state of a module's local reset: true while it is asserted (LRSTZ = 0). Only the six
    /// megamodules can be locally reset, so every other module reads false.
    pub(crate) fn local_reset_asserted_state(&self, module: usize) -> bool {
        module <= MODULE_GEM5 && self.mdctl[module] & MDCTL_LRSTZ == 0
    }

    /// The emulation levels the IcePick drives into a module, reported in MDSTAT.EMURST and
    /// MDSTAT.EMUHIB and, when the matching MDCTL enable is set, latched into MERRPR
    /// (SPRUEG3B section 7.1).
    #[cfg(test)]
    pub(crate) fn set_emulation_state(&mut self, module: usize, reset: bool, inhibit: bool) {
        let bit = 1 << module;
        self.emu_reset = (self.emu_reset & !bit) | if reset { bit } else { 0 };
        self.emu_inhibit = (self.emu_inhibit & !bit) | if inhibit { bit } else { 0 };
        self.latch_errors();
    }

    fn mdstat(&self, module: usize) -> u32 {
        let mut value = self.transition[module].unwrap_or(self.state[module]);
        let state = self.state[module];
        // SyncRst runs the module clock while holding the module in reset; Disable and
        // SwRstDisable gate it (SPRUEG3B section 4.1.2).
        if state == STATE_ENABLE || state == STATE_SYNC_RST {
            value |= MDSTAT_MCKOUT;
        }
        // Module reset is de-asserted, and its initialisation therefore done, in the two states
        // section 4.1.2 describes as out of reset.
        if state == STATE_ENABLE || state == STATE_DISABLE {
            value |= MDSTAT_MRSTZ | MDSTAT_MRSTDONE;
        }
        // MDSTAT.LRSTZ is the actual local reset (Table 13); it follows MDCTL.LRSTZ.
        //
        // UNCERTAIN: SPRUEG3B never states what MDSTAT.LRSTZ reads for a module that has no local
        // reset. Since section 5.2 says peripherals get no soft reset assertion, they read
        // de-asserted and done here.
        if !self.local_reset_asserted_state(module) {
            value |= MDSTAT_LRSTZ | MDSTAT_LRSTDONE;
        }
        if self.emu_reset & (1 << module) != 0 {
            value |= MDSTAT_EMURST;
        }
        if self.emu_inhibit & (1 << module) != 0 {
            value |= MDSTAT_EMUHIB;
        }
        value
    }

    /// MDCTL stages NEXT for the next GO, and carries the local reset control and the two error
    /// interrupt enables.
    ///
    /// UNCERTAIN: the guide does not say whether LRSTZ needs a GO to take effect. Section 4.1.2
    /// only requires LRSTZ to be set for the reset to be de-asserted and Example 4 happens to
    /// write it alongside NEXT, before a GO. LRSTZ is a control bit, not a state-machine request,
    /// so this model applies it as soon as it is written.
    fn write_mdctl(&mut self, module: usize, value: u32) {
        let value = value & MDCTL_WRITABLE;
        let changed = self.mdctl[module] ^ value;
        self.mdctl[module] = value;
        if module <= MODULE_GEM5 && changed & MDCTL_LRSTZ != 0 {
            self.local_reset_edges
                .push((module, value & MDCTL_LRSTZ == 0));
        }
        // A newly set EMURSTIE or EMUIHBIE latches a condition that is already present.
        self.latch_errors();
    }

    /// A GO bit sets its domain's GOSTAT and starts every staged transition in that domain
    /// (SPRUEG3B sections 4.1.2 and 9.5). Writing 0 to a GO bit does nothing.
    fn write_ptcmd(&mut self, value: u32) {
        let go = value & DOMAIN_MASK;
        if go == 0 {
            return;
        }
        self.gostat |= go;
        if go & (1 << DOMAIN_ALWAYS_ON) == 0 {
            // A memory power domain: the handshake completes on the next cycle with no module
            // affected (SPRUEG3B section 2.2.1).
            return;
        }
        for module in 0..MODULES {
            let target = self.mdctl[module] & MDCTL_NEXT;
            // NEXT 4-15 is reserved (SPRUEG3B Table 14) and requests nothing.
            if target > STATE_ENABLE || target == self.state[module] {
                continue;
            }
            self.transition[module] = Some(transitional_state(target));
        }
    }

    /// MERRCR is write-one-to-clear over MERRPR and reads 0 (SPRUEG3B section 7.2).
    fn write_merrcr(&mut self, value: u32) {
        self.merrpr &= !(value & MODULE_MASK);
        // Section 7.2 gives an incoming condition priority over a simultaneous clear, so a
        // condition that is still asserted re-latches at once.
        self.latch_errors();
    }

    /// INTEVAL has no storage: a written 1 pulses or re-evaluates an interrupt output, a written 0
    /// does nothing (SPRUEG3B sections 7.4 and 9.2).
    ///
    /// ERRSET and ALLSET pulse unconditionally; ERREV and ALLEV re-pulse only if a condition is
    /// pending. Either error-side bit also raises ALLINT, which section 7.3 sets with ERRINT.
    fn write_inteval(&mut self, value: u32) {
        let forced = value & (INTEVAL_ERRSET | INTEVAL_ALLSET) != 0;
        let re_evaluated = value & (INTEVAL_ERREV | INTEVAL_ALLEV) != 0 && self.merrpr != 0;
        if forced || re_evaluated {
            self.pulse = true;
        }
    }

    /// Latch every enabled emulation condition into MERRPR (SPRUEG3B section 7.1).
    fn latch_errors(&mut self) {
        for module in 0..MODULES {
            let bit = 1 << module;
            let control = self.mdctl[module];
            let reset_error = self.emu_reset & bit != 0 && control & MDCTL_EMURSTIE != 0;
            let inhibit_error = self.emu_inhibit & bit != 0 && control & MDCTL_EMUIHBIE != 0;
            if reset_error || inhibit_error {
                self.merrpr |= bit;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    fn mdstat(module: usize) -> u32 {
        MDSTAT0 + 4 * module as u32
    }

    fn mdctl(module: usize) -> u32 {
        MDCTL0 + 4 * module as u32
    }

    /// What MDSTAT reports for a module whose local reset is de-asserted, or that has none at
    /// all: LRSTZ and LRSTDONE both 1 (SPRUEG3B Table 13 and section 5.2).
    const LOCAL_RESET_OFF: u32 = MDSTAT_LRSTZ | MDSTAT_LRSTDONE;

    #[test]
    fn pid_reads_the_documented_constant() {
        // SPRUEG3B Table 7: SCHEME 01b, FUNC 2808h, MAJOR_REVISION 001b, RTL, CUSTOM and
        // MINOR_REVISION zero. Writes have no effect (section 9.1).
        let mut psc = PowerSleep::default();
        assert_eq!(psc.read(PID, TEST_CONTEXT), Some(0x6808_0100));
        assert_ne!(
            psc.write(PID, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(PID, TEST_CONTEXT), Some(0x6808_0100));
    }

    #[test]
    fn every_module_resets_to_swrstdisable() {
        // SPRUEG3B section 5.1: a power-on reset leaves every LPSC module domain in
        // SwRstDisable, which section 4.1.2 defines as clocks gated and module reset asserted.
        let mut psc = PowerSleep::default();
        for module in 0..MODULES {
            // Only the six megamodules have a local reset, and at reset it is asserted.
            let local_reset = if module <= MODULE_GEM5 {
                0
            } else {
                LOCAL_RESET_OFF
            };
            assert_eq!(psc.read(mdstat(module), TEST_CONTEXT), Some(local_reset));
            assert_eq!(psc.read(mdctl(module), TEST_CONTEXT), Some(0));
            assert!(!psc.module_enabled(module));
        }
        assert_eq!(psc.read(PTSTAT, TEST_CONTEXT), Some(0));
        assert_eq!(psc.read(MERRPR, TEST_CONTEXT), Some(0));
        assert!(psc.take_local_reset_edges().is_empty());
    }

    #[test]
    fn a_next_and_go_sequence_walks_the_documented_states() {
        // SPRUEG3B section 4.2, Example 4: 103h into MDCTLn enables the module and 102h disables
        // it, each followed by GO0. Section 4.1.2: GOSTAT is 1 while the LPSC transitions and
        // reverts to 0 when the domain's state machines are done, and MDSTAT.STATE then reports
        // the state reached.
        let mut psc = PowerSleep::default();
        let module = MODULE_TSIP0;
        assert_eq!(mdctl(module), BASE + 0xa24, "TSIP0 is LPSC10, so MDCTL9");

        // Staging NEXT alone moves nothing.
        assert_ne!(
            psc.write(mdctl(module), 0x103, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(mdctl(module), TEST_CONTEXT), Some(0x103));
        assert_eq!(
            psc.read(mdstat(module), TEST_CONTEXT),
            Some(LOCAL_RESET_OFF | STATE_SW_RST_DISABLE)
        );
        assert_eq!(psc.read(PTSTAT, TEST_CONTEXT), Some(0));
        psc.tick();
        assert_eq!(
            psc.read(mdstat(module), TEST_CONTEXT),
            Some(LOCAL_RESET_OFF | STATE_SW_RST_DISABLE)
        );

        // The GO starts it: GOSTAT reads 1 and STATE reports EnableIn-Prog (Table 5).
        assert_ne!(
            psc.write(PTCMD, 1 << DOMAIN_ALWAYS_ON, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(PTSTAT, TEST_CONTEXT), Some(1));
        assert_eq!(
            psc.read(mdstat(module), TEST_CONTEXT),
            Some(LOCAL_RESET_OFF | STATE_ENABLE_IN_PROG)
        );
        assert!(!psc.module_enabled(module));

        // One cycle later the transition is complete: clock on, module reset de-asserted and its
        // initialisation done, local reset de-asserted (Table 13).
        psc.tick();
        assert_eq!(psc.read(PTSTAT, TEST_CONTEXT), Some(0));
        assert_eq!(
            psc.read(mdstat(module), TEST_CONTEXT),
            Some(
                MDSTAT_MCKOUT
                    | MDSTAT_MRSTDONE
                    | MDSTAT_MRSTZ
                    | MDSTAT_LRSTDONE
                    | MDSTAT_LRSTZ
                    | STATE_ENABLE
            )
        );
        assert!(psc.module_enabled(module));

        // Disabling gates the clock again but leaves the module out of reset (section 4.1.2).
        // While the transition is in flight the module is still Enabled, so its status bits are
        // unchanged and only STATE reports Disable In-Prog (Table 5).
        assert_ne!(
            psc.write(mdctl(module), 0x102, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_ne!(
            psc.write(PTCMD, 1 << DOMAIN_ALWAYS_ON, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            psc.read(mdstat(module), TEST_CONTEXT),
            Some(
                MDSTAT_MCKOUT
                    | MDSTAT_MRSTDONE
                    | MDSTAT_MRSTZ
                    | LOCAL_RESET_OFF
                    | STATE_DISABLE_IN_PROG
            )
        );
        psc.tick();
        assert_eq!(psc.read(PTSTAT, TEST_CONTEXT), Some(0));
        assert_eq!(
            psc.read(mdstat(module), TEST_CONTEXT),
            Some(MDSTAT_MRSTDONE | MDSTAT_MRSTZ | MDSTAT_LRSTDONE | MDSTAT_LRSTZ | STATE_DISABLE)
        );
        assert!(!psc.module_enabled(module));

        // Nothing else in the domain moved, and PTCMD itself has no storage (section 9.5).
        assert_eq!(
            psc.read(mdstat(MODULE_UTOPIA), TEST_CONTEXT),
            Some(LOCAL_RESET_OFF | STATE_SW_RST_DISABLE)
        );
        assert_eq!(psc.read(PTCMD, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn a_memory_domain_go_only_completes_the_handshake() {
        // SPRUEG3B section 2.2.1: the eight memory power domains carry no LPSC - the UMC and SMC
        // master them - so their GO sets and clears GOSTAT without touching a module.
        let mut psc = PowerSleep::default();
        // 103h also sets LRSTZ, which releases this megamodule's local reset at once.
        assert_ne!(
            psc.write(mdctl(MODULE_GEM0), 0x103, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_ne!(
            psc.write(
                PTCMD,
                (1 << DOMAIN_GEM0_LL2) | (1 << DOMAIN_SMC_PAGE1),
                TEST_CONTEXT
            ),
            PortWrite::Unhandled
        );
        assert_eq!(
            psc.read(PTSTAT, TEST_CONTEXT),
            Some((1 << DOMAIN_GEM0_LL2) | (1 << DOMAIN_SMC_PAGE1))
        );
        assert_eq!(
            psc.read(mdstat(MODULE_GEM0), TEST_CONTEXT),
            Some(LOCAL_RESET_OFF | STATE_SW_RST_DISABLE)
        );
        psc.tick();
        assert_eq!(psc.read(PTSTAT, TEST_CONTEXT), Some(0));
        assert_eq!(
            psc.read(mdstat(MODULE_GEM0), TEST_CONTEXT),
            Some(LOCAL_RESET_OFF | STATE_SW_RST_DISABLE)
        );
        // Bits above GO8 are reserved (Table 11) and set no status.
        assert_ne!(
            psc.write(PTCMD, 1 << DOMAINS, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(PTSTAT, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn local_reset_edges_are_reported_exactly_once() {
        // SPRUEG3B section 5.2: clearing LRSTZ in an LPSC1-6 MDCTL asserts a megamodule's local
        // reset. Each change is one edge for the device to consume.
        let mut psc = PowerSleep::default();
        assert!(psc.local_reset_asserted_state(MODULE_GEM3));

        assert_ne!(
            psc.write(mdctl(MODULE_GEM3), MDCTL_LRSTZ | STATE_ENABLE, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert!(!psc.local_reset_asserted_state(MODULE_GEM3));
        assert_eq!(psc.take_local_reset_edges(), vec![(MODULE_GEM3, false)]);
        assert!(psc.take_local_reset_edges().is_empty());

        // Rewriting the same LRSTZ level is not an edge.
        assert_ne!(
            psc.write(
                mdctl(MODULE_GEM3),
                MDCTL_LRSTZ | STATE_DISABLE,
                TEST_CONTEXT
            ),
            PortWrite::Unhandled
        );
        assert!(psc.take_local_reset_edges().is_empty());

        // Clearing it asserts the local reset again.
        assert_ne!(
            psc.write(mdctl(MODULE_GEM3), STATE_DISABLE, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert!(psc.local_reset_asserted_state(MODULE_GEM3));
        assert_eq!(
            psc.read(mdstat(MODULE_GEM3), TEST_CONTEXT).unwrap() & (MDSTAT_LRSTZ | MDSTAT_LRSTDONE),
            0
        );
        assert_eq!(psc.take_local_reset_edges(), vec![(MODULE_GEM3, true)]);
    }

    #[test]
    fn only_the_megamodules_are_locally_resettable() {
        // SPRUEG3B section 5.2: "There is no soft reset assertion to peripherals". MDCTL.LRSTZ is
        // still a read/write bit for them (Table 14), it just drives nothing.
        //
        // UNCERTAIN: section 5.2 says LPSC1-5 while the device has six megamodules on LPSC1-6
        // (Table 3, Figure 2, and section 4.1.2, which names MDCTL1-6). All six are implemented.
        let mut psc = PowerSleep::default();
        for module in [MODULE_GEM0, MODULE_GEM5] {
            assert_ne!(
                psc.write(mdctl(module), MDCTL_LRSTZ, TEST_CONTEXT),
                PortWrite::Unhandled
            );
            assert_eq!(psc.take_local_reset_edges(), vec![(module, false)]);
        }
        for module in [MODULE_SRIO, MODULE_EMAC1, MODULE_UTOPIA] {
            assert_ne!(
                psc.write(mdctl(module), MDCTL_LRSTZ, TEST_CONTEXT),
                PortWrite::Unhandled
            );
            assert_ne!(
                psc.write(mdctl(module), 0, TEST_CONTEXT),
                PortWrite::Unhandled
            );
            assert!(psc.take_local_reset_edges().is_empty());
            assert!(!psc.local_reset_asserted_state(module));
            // A peripheral has no local reset, so MDSTAT reports it de-asserted and done.
            let status = psc.read(mdstat(module), TEST_CONTEXT).unwrap();
            assert_eq!(status & MDSTAT_LRSTZ, MDSTAT_LRSTZ);
            assert_eq!(status & MDSTAT_LRSTDONE, MDSTAT_LRSTDONE);
        }
    }

    #[test]
    fn merrcr_clears_merrpr_one_bit_at_a_time() {
        // SPRUEG3B sections 7.1 and 7.2: an emulation condition with its MDCTL enable set latches
        // in the read-only MERRPR, a 1 in MERRCR clears that bit, and MERRCR reads 0.
        let mut psc = PowerSleep::default();
        psc.set_emulation_state(MODULE_EMAC0, true, false);
        // With EMURSTIE clear the condition shows in MDSTAT but raises nothing.
        assert_eq!(
            psc.read(mdstat(MODULE_EMAC0), TEST_CONTEXT).unwrap() & MDSTAT_EMURST,
            MDSTAT_EMURST
        );
        assert_eq!(psc.read(MERRPR, TEST_CONTEXT), Some(0));
        assert!(!psc.interrupt_pending());

        // Enabling it latches the pending bit and asserts PSC_ALLINT.
        assert_ne!(
            psc.write(mdctl(MODULE_EMAC0), MDCTL_EMURSTIE, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(MERRPR, TEST_CONTEXT), Some(1 << MODULE_EMAC0));
        assert!(psc.interrupt_pending());
        assert_eq!(psc.read(MERRCR, TEST_CONTEXT), Some(0));

        // Section 7.2: a condition still asserted wins over a simultaneous clear.
        assert_ne!(
            psc.write(MERRCR, 1 << MODULE_EMAC0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(MERRPR, TEST_CONTEXT), Some(1 << MODULE_EMAC0));

        // With the condition gone the clear sticks, and only the written bit clears.
        psc.set_emulation_state(MODULE_EMAC0, false, false);
        psc.set_emulation_state(MODULE_HPI, false, true);
        assert_ne!(
            psc.write(mdctl(MODULE_HPI), MDCTL_EMUIHBIE, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            psc.read(MERRPR, TEST_CONTEXT),
            Some((1 << MODULE_EMAC0) | (1 << MODULE_HPI))
        );
        assert_ne!(
            psc.write(MERRCR, 1 << MODULE_EMAC0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(MERRPR, TEST_CONTEXT), Some(1 << MODULE_HPI));
        assert!(psc.interrupt_pending());

        // MERRPR is read-only (section 9.3): a write to it clears nothing.
        assert_ne!(
            psc.write(MERRPR, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(MERRPR, TEST_CONTEXT), Some(1 << MODULE_HPI));
        psc.set_emulation_state(MODULE_HPI, false, false);
        assert_ne!(
            psc.write(MERRCR, MODULE_MASK, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(MERRPR, TEST_CONTEXT), Some(0));
        assert!(!psc.interrupt_pending());
    }

    #[test]
    fn inteval_has_no_readable_storage_and_pulses_allint() {
        // SPRUEG3B sections 7.4 and 9.2: INTEVAL has no storage and reads 0; a written 1 in
        // ALLSET/ERRSET pulses the output, and ALLEV/ERREV re-pulse it only when a condition is
        // pending. Section 7.3 sets ALLINT along with ERRINT.
        let mut psc = PowerSleep::default();
        assert_ne!(
            psc.write(
                INTEVAL,
                INTEVAL_ALLSET | INTEVAL_ERRSET | INTEVAL_ALLEV,
                TEST_CONTEXT
            ),
            PortWrite::Unhandled
        );
        assert_eq!(psc.read(INTEVAL, TEST_CONTEXT), Some(0));
        assert!(psc.interrupt_pending());
        // The pulse lasts one device cycle, and nothing was pending behind it.
        psc.tick();
        assert!(!psc.interrupt_pending());

        // A write of 0 has no effect.
        assert_ne!(psc.write(INTEVAL, 0, TEST_CONTEXT), PortWrite::Unhandled);
        assert!(!psc.interrupt_pending());

        // Re-evaluation with nothing pending does not pulse.
        assert_ne!(
            psc.write(INTEVAL, INTEVAL_ALLEV | INTEVAL_ERREV, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert!(!psc.interrupt_pending());

        // With a condition pending it re-pulses, and the level keeps ALLINT asserted anyway.
        psc.set_emulation_state(MODULE_GEM2, true, false);
        assert_ne!(
            psc.write(mdctl(MODULE_GEM2), MDCTL_EMURSTIE, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_ne!(
            psc.write(INTEVAL, INTEVAL_ERREV, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert!(psc.interrupt_pending());
        psc.tick();
        assert!(psc.interrupt_pending());
    }

    #[test]
    fn addresses_outside_the_register_set_are_not_ours() {
        // SPRS612G Table 7-3: the PSC window is 02AE 0000 - 02AF FFFF but only the listed word
        // addresses are registers; the router uses None to mean "not mine".
        let mut psc = PowerSleep::default();
        for address in [
            BASE - 4,
            BASE + 0x004,
            BASE + 0x014,
            BASE + 0x01c,
            BASE + 0x044,
            BASE + 0x124,
            BASE + 0x12c,
            BASE + 0x7fc,
            // One past MDSTAT13 and one past MDCTL13.
            MDSTAT0 + 4 * MODULES as u32,
            MDCTL0 + 4 * MODULES as u32,
            // Unaligned inside both arrays.
            MDSTAT0 + 2,
            MDCTL0 + 1,
            BASE + 0x2_0000,
        ] {
            assert_eq!(psc.read(address, TEST_CONTEXT), None, "read {address:#x}");
            assert_eq!(
                psc.write(address, 1, TEST_CONTEXT),
                PortWrite::Unhandled,
                "write {address:#x}"
            );
        }
        // The last register of each array is ours. MDSTAT13 (UTOPIA) reports its local reset as
        // de-asserted, because only the megamodules have one (section 5.2).
        assert_eq!(
            psc.read(MDSTAT0 + 4 * (MODULES as u32 - 1), TEST_CONTEXT),
            Some(LOCAL_RESET_OFF | STATE_SW_RST_DISABLE)
        );
        assert_eq!(
            psc.read(MDCTL0 + 4 * (MODULES as u32 - 1), TEST_CONTEXT),
            Some(0)
        );
    }
}
