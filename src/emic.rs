//! Ethernet Multicore Interrupt Combiner (EMIC) of the TMS320C6472.
//!
//! The C6472 replaces the single `EWCTL`/`EWINTTCNT` gate of the C6455 EMAC
//! control module (SPRUE48 §3.2, §3.3) with one EMIC block per EMAC. Each EMIC
//! routes one set of EMAC + MDIO interrupts to all six C64x+ megamodules with a
//! per-core enable register, and it paces the transmit and receive pulse
//! interrupts (SPRUEF8F §2.7).
//!
//! Documents used:
//!   * SPRUEF8F — *TMS320C6472/TMS320TCI6486 DSP EMAC/MDIO Module User's Guide*,
//!     §2.7 (EMIC architecture), §2.16.2 (EMIC initialization), §2.17.3/§2.17.4
//!     (interrupt processing), §3 (EMIC module registers).
//!   * SPRS612G — *TMS320C6472 Fixed-Point DSP* data manual, §7.6 Table 7-10
//!     (system event map), §7.15.3 Tables 7-105/7-106 (EMIC register maps).
//!
//! # Blocks modelled
//!
//! * Prescaler: divides the peripheral clock (CPU/6, SPRUEF8F §2.16.2) with the
//!   `PSCFG.PRESCALE_CFG` reload value and produces `PS_TICK` (§2.7, Figure 14).
//! * Pacing block per event: a timed-delay state machine (TDSM, §2.7.2) and a
//!   divide-by-N state machine (DSM, §2.7.3). Eight receive pacers (RPIC,
//!   §2.7.5) and eight transmit pacers (TPIC, §2.7.4). The pacer configuration
//!   is *shared* by all megamodules (§3.2.1, §3.3.1: "This register
//!   configuration is common to all C64x+ megamodules"); only the enable gate
//!   downstream of the pacer is per core (Figures 18 and 19).
//! * Common interrupt combiner (CIC, §2.7.6): HOST, STAT, MDIO_LINT, MDIO_USER,
//!   level-to-pulse converted, per-core enabled, **not** paced — Figure 20 shows
//!   no pacing block and no `PS_TICK` in the common path.
//!
//! # Outputs
//!
//! Per core the EMIC drives MACRXINT*n*, MACTXINT*n* and MACINT*n*. On the C6472
//! these reach the megamodule as system events (SPRS612G Table 7-10): EMAC0 uses
//! event 5 (MACRXINT0), event 6 (MACTXINT0) and event 70 (MACINT0); EMAC1 uses
//! event 7 (MACRXINT1), event 8 (MACTXINT1) and event 71 (MACINT1). The
//! megamodule expects *pulse* interrupts (§2.7.6), so [`Lines`] describes a
//! single-cycle pulse, not a level; see [`EmacInterruptControl::lines`].
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

/// Number of transmit and of receive events per EMAC (SPRUEF8F §2.7.4/§2.7.5:
/// TX0-TX7 and RX0-RX7).
const EVENTS: usize = 8;

/// EW_INTCTL0 - EW_INTCTL5, one per C64x+ megamodule (SPRUEF8F §3.1).
const MAX_CORES: usize = 6;

/// The EMIC occupies 2 KB: EMIC0 at 0x02C8_1000-0x02C8_17FF, EMIC1 at
/// 0x02CC_1000-0x02CC_17FF (SPRS612G §7.4 device memory map).
const WINDOW: u32 = 0x800;

/// Peripheral ID register (SPRS612G Table 7-105, offset 0h).
const OFF_PID: u32 = 0x000;
/// Prescalar configuration register (SPRUEF8F §3.4, SPRS612G Table 7-105).
const OFF_PSCFG: u32 = 0x004;
/// EW_INTCTL0..5, one per core, stride 4 (SPRS612G Table 7-105, offset 20h).
const OFF_EW_INTCTL: u32 = 0x020;
/// RPCFG0..7, one per receive event (SPRS612G Table 7-105, offset 100h).
const OFF_RPCFG: u32 = 0x100;
/// RPSTAT0..7, one per receive event (SPRS612G Table 7-105, offset 120h).
const OFF_RPSTAT: u32 = 0x120;
/// TPCFG0..7, one per transmit event (SPRS612G Table 7-105, offset 140h).
const OFF_TPCFG: u32 = 0x140;
/// TPSTAT0..7, one per transmit event (SPRS612G Table 7-105, offset 160h).
const OFF_TPSTAT: u32 = 0x160;

/// EMIC0 base address (SPRS612G §7.4: EMIC0 2K at 0x02C8_1000).
#[cfg(test)]
pub(crate) const EMIC0_BASE: u32 = 0x02c8_1000;
/// EMIC1 base address (SPRS612G §7.4: EMIC1 2K at 0x02CC_1000).
#[cfg(test)]
pub(crate) const EMIC1_BASE: u32 = 0x02cc_1000;

/// Writable bits of EW_INTCTL: RX7:0 in 23:16, TX7:0 in 15:8, MDIO_USER 4,
/// MDIO_LINT 3, STAT 2, HOST 1. Bits 31:24, 7:5 and 0 are reserved
/// (SPRUEF8F Figure 23).
const EW_INTCTL_MASK: u32 = 0x00ff_ff1e;

/// EW_INTCTL bit 1 - HOST (EMAC host error) enable (SPRUEF8F Figure 23,
/// Figure 20).
const COMMON_HOST: u8 = 1 << 1;
/// EW_INTCTL bit 2 - STAT (statistics) enable.
const COMMON_STAT: u8 = 1 << 2;
/// EW_INTCTL bit 3 - MDIO_LINT (MDIO link change) enable.
const COMMON_MDIO_LINT: u8 = 1 << 3;
/// EW_INTCTL bit 4 - MDIO_USER (MDIO user command complete) enable.
const COMMON_MDIO_USER: u8 = 1 << 4;
/// All four common sources of the CIC (SPRUEF8F §2.7.6: EW_INTCTL[4:1]).
const COMMON_ALL: u8 = COMMON_HOST | COMMON_STAT | COMMON_MDIO_LINT | COMMON_MDIO_USER;

/// RPCFG/TPCFG bit 3 - TU: write key that enables writes to TIME_CFG
/// (SPRUEF8F Table 17/Table 19).
const CFG_TU: u32 = 1 << 3;
/// RPCFG/TPCFG bit 2 - CU: write key that enables writes to CNT_CFG.
const CFG_CU: u32 = 1 << 2;
/// RPCFG/TPCFG bit 1 - TR: resets the timer counter.
const CFG_TR: u32 = 1 << 1;
/// RPCFG/TPCFG bit 0 - CR: resets the divide-by-N count.
const CFG_CR: u32 = 1 << 0;

/// TIME_CFG / TIME occupy bits 27:16, i.e. 12 bits (SPRUEF8F Figures 24-27).
const TIME_MASK: u32 = 0x0fff;
/// CNT_CFG / CNT occupy bits 15:8, i.e. 8 bits (SPRUEF8F Figures 24-27).
const CNT_MASK: u32 = 0xff;

/// The unmasked interrupt state of one EMAC plus the shared MDIO, as the EMIC
/// sees it.
///
/// `rx` and `tx` are the eight per-channel receive and transmit events
/// (RXEVT[7:0]/TXEVT[7:0] of SPRUEF8F Figures 18 and 19). The EMIC consumes
/// these as *pulses* (§2.7: "The EMIC module uses the pulse interrupts to
/// incorporate some of its pacing functionality"), so the model derives one
/// event pulse from each rising edge of a bit.
///
/// `stat`, `host`, `mdio_link` and `mdio_user` are the four level-driven common
/// sources; the CIC performs the level-to-pulse conversion itself (§2.7.6).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Sources {
    /// RXEVT[7:0]: EMAC receive channel interrupts (RXPEND*n*).
    pub rx: u8,
    /// TXEVT[7:0]: EMAC transmit channel interrupts (TXPEND*n*).
    pub tx: u8,
    /// EMAC statistics interrupt (STATPEND).
    pub stat: bool,
    /// EMAC host error interrupt (HOSTPEND).
    pub host: bool,
    /// MDIO link change interrupt (LINKINT).
    pub mdio_link: bool,
    /// MDIO user command complete interrupt (USERINT).
    pub mdio_user: bool,
}

/// Per-core lines the EMIC drives: the receive, transmit and common interrupts.
///
/// Each field is a one-cycle pulse produced by the cycle's
/// [`EmacInterruptControl::update`]; the device model must forward it to the
/// megamodule INTC of that core in the same cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Lines {
    /// MACRXINT*n* (SPRS612G Table 7-10 event 5 for EMAC0, event 7 for EMAC1).
    pub rx: bool,
    /// MACTXINT*n* (event 6 for EMAC0, event 8 for EMAC1).
    pub tx: bool,
    /// MACINT*n* (event 70 for EMAC0, event 71 for EMAC1).
    pub common: bool,
}

/// State of the timed-delay state machine (SPRUEF8F §2.7.2) as reported by
/// `RPSTAT.TIM_SM`/`TPSTAT.TIM_SM` (Table 18/Table 20).
///
/// The OUTPUT state (`10b`) is a transitional state - SPRUEF8F §2.7.2 notes the
/// grayed states are left immediately - so it is never observable in a register
/// read and is therefore not represented.
const SM_WAITING: u32 = 0b00;
/// TIM_SM = DELAY / DIV_SM = COUNT.
///
/// UNCERTAIN: SPRUEF8F Table 18/Table 20 label DIV_SM = `01b` "Divide by N SM in
/// DELAY state" while §2.7.3 names the DSM states WAITING, COUNT and OUTPUT.
/// The encoding is the same either way, so only the name is in doubt.
const SM_ACTIVE: u32 = 0b01;

/// One pacing block: a timed-delay state machine and a divide-by-N state
/// machine sharing one event input and one pulse output (SPRUEF8F §2.7.1,
/// Figure 15).
#[derive(Clone, Copy, Debug, Default)]
struct Pacer {
    /// TIME_CFG, bits 27:16 of RPCFG/TPCFG. Reset 0, which disables the TDSM
    /// (SPRUEF8F §2.7.2).
    time_cfg: u16,
    /// CNT_CFG, bits 15:8 of RPCFG/TPCFG. Reset 0 (SPRUEF8F §2.7.3).
    cnt_cfg: u8,
    /// TIME, bits 27:16 of RPSTAT/TPSTAT: prescaler periods counted so far.
    time: u16,
    /// CNT, bits 15:8 of RPSTAT/TPSTAT: events counted so far.
    cnt: u8,
    /// True while the TDSM is in the DELAY state.
    delaying: bool,
}

impl Pacer {
    /// Feed one event pulse in. Returns true when the pacer emits an output
    /// pulse in the same cycle.
    ///
    /// SPRUEF8F §2.7.2 and §2.7.3 make the two state machines mutually
    /// exclusive:
    ///   * `TIME_CFG != 0` -> time-based pacing; the DSM is disabled ("When the
    ///     TIME_CFG is set to non-zero, it then disables the divide-by-N state
    ///     machine"). The first event moves the TDSM from WAITING to DELAY and
    ///     is *held off*; further events during DELAY are absorbed.
    ///   * `TIME_CFG == 0 && CNT_CFG != 0` -> count-based pacing; "the CNT_CFG
    ///     number of events are counted before an output pulse is generated" and
    ///     "the counter resets (and reloads) every time when a divide-by-N pulse
    ///     is generated".
    ///   * `TIME_CFG == 0 && CNT_CFG == 0` -> reset state, "immediately
    ///     generates a pulse (basically means a zero delay)".
    ///
    /// UNCERTAIN: SPRUEF8F §2.7.1 also claims the block provides "time-based or
    /// count-based pacing of interrupts, in any combination", which contradicts
    /// the mutual exclusion stated in §2.7.3 for the very same fields. The
    /// register-level statement of §2.7.3 is implemented here.
    fn event(&mut self) -> bool {
        if self.time_cfg != 0 {
            if !self.delaying {
                self.delaying = true;
                self.time = 0;
            }
            return false;
        }
        if self.cnt_cfg == 0 {
            return true;
        }
        self.cnt = self.cnt.wrapping_add(1);
        if self.cnt >= self.cnt_cfg {
            self.cnt = 0;
            return true;
        }
        false
    }

    /// Advance the TDSM by one prescaler output period (`PS_TICK`). Returns true
    /// when the delay expires and the pacer emits its output pulse.
    ///
    /// SPRUEF8F §2.7.2: "When the TIME_CFG is set to a non-zero value, an output
    /// pulse is generated after the TIME_CFG number of prescalar output periods.
    /// The counter starts counting on the first event."
    ///
    /// UNCERTAIN: the TDSM transition diagram (Figure 16) increments TIME while
    /// `TIME < TIME_CFG` and leaves DELAY on `PS_TICK && TIME >= TIME_CFG`,
    /// which would take `TIME_CFG + 1` ticks. The prose of §2.7.2 says
    /// `TIME_CFG` periods. The prose is implemented (increment, then compare),
    /// so exactly `TIME_CFG` `PS_TICK`s elapse between the first event and the
    /// output pulse.
    fn ps_tick(&mut self) -> bool {
        if self.time_cfg == 0 || !self.delaying {
            return false;
        }
        self.time += 1;
        if u32::from(self.time) >= u32::from(self.time_cfg) {
            self.time = 0;
            self.delaying = false;
            return true;
        }
        false
    }

    /// RPCFG/TPCFG read value. TU, CU, TR and CR are write-only and read back as
    /// 0 (SPRUEF8F Table 17/Table 19).
    fn read_cfg(&self) -> u32 {
        (u32::from(self.time_cfg) << 16) | (u32::from(self.cnt_cfg) << 8)
    }

    /// RPCFG/TPCFG write.
    ///
    /// TIME_CFG and CNT_CFG are key-protected: SPRUEF8F Table 17/Table 19 define
    /// TU as "Enables writes to TIME_CFG" and CU as "Enables writes to CNT_CFG",
    /// so a write without the matching key leaves the field untouched. TR
    /// "Resets the timer counter" and CR "Resets the divide by N count".
    fn write_cfg(&mut self, value: u32) {
        if value & CFG_TU != 0 {
            self.time_cfg = ((value >> 16) & TIME_MASK) as u16;
        }
        if value & CFG_CU != 0 {
            self.cnt_cfg = ((value >> 8) & CNT_MASK) as u8;
        }
        if value & CFG_TR != 0 {
            self.time = 0;
            self.delaying = false;
        }
        if value & CFG_CR != 0 {
            self.cnt = 0;
        }
    }

    /// RPSTAT/TPSTAT read value: TIME 27:16, CNT 15:8, TIM_SM 3:2, DIV_SM 1:0
    /// (SPRUEF8F Table 18/Table 20).
    ///
    /// The DSM is in its active (COUNT) state exactly while a partial count is
    /// held, per the DSM transition diagram (Figure 17), which enters COUNT on
    /// the first event and returns to WAITING when the count is consumed.
    fn read_stat(&self) -> u32 {
        let tim_sm = if self.delaying { SM_ACTIVE } else { SM_WAITING };
        let div_sm = if self.cnt != 0 { SM_ACTIVE } else { SM_WAITING };
        (u32::from(self.time) << 16) | (u32::from(self.cnt) << 8) | (tim_sm << 2) | div_sm
    }
}

/// One EMIC instance: the pacers and combiners of a single EMAC, shared by all
/// cores of the device.
///
/// The device model is expected to drive it once per peripheral-clock cycle
/// (CPU/6, SPRUEF8F §2.16.2):
///
/// ```text
/// emic.tick();                 // prescaler + timed pacing
/// emic.update(sources);        // sample EMAC/MDIO state, recompute the lines
/// let l = emic.lines(core);    // pulses belonging to this cycle
/// ```
#[derive(Clone, Debug)]
pub(crate) struct EmacInterruptControl {
    /// Base address of the 2 KB EMIC window.
    base: u32,
    /// Number of implemented EW_INTCTL registers, i.e. cores (6 on the C6472).
    cores: usize,
    /// PSCFG.PRESCALE_CFG, bits 15:0, reset 0 (SPRUEF8F §3.4, Figure 28).
    pscfg: u32,
    /// Free-running prescaler counter feeding `PS_TICK`.
    prescale: u32,
    /// EW_INTCTL per core, reset 0 (SPRUEF8F Figure 23).
    ew_intctl: [u32; MAX_CORES],
    /// Receive pacing blocks RPCFG0/RPSTAT0 .. RPCFG7/RPSTAT7 (§2.7.5).
    rx_pacer: [Pacer; EVENTS],
    /// Transmit pacing blocks TPCFG0/TPSTAT0 .. TPCFG7/TPSTAT7 (§2.7.4).
    tx_pacer: [Pacer; EVENTS],
    /// Previous RXEVT levels, for rising-edge (pulse) extraction.
    prev_rx: u8,
    /// Previous TXEVT levels, for rising-edge (pulse) extraction.
    prev_tx: u8,
    /// Previous per-core *gated* common source levels, for the CIC
    /// level-to-pulse conversion (§2.7.6).
    prev_common: [u8; MAX_CORES],
    /// Receive pacer output pulses produced since the last `update`.
    fired_rx: u8,
    /// Transmit pacer output pulses produced since the last `update`.
    fired_tx: u8,
    /// Lines computed by the last `update`.
    lines: [Lines; MAX_CORES],
}

impl EmacInterruptControl {
    /// Create an EMIC at `base` ([`EMIC0_BASE`] or [`EMIC1_BASE`]) with `cores`
    /// EW_INTCTL registers (6 on the C6472, SPRUEF8F §3.1).
    ///
    /// All registers reset to 0 (SPRUEF8F Figures 23-28).
    ///
    /// UNCERTAIN: neither SPRS612G Table 7-105/7-106 nor SPRUEF8F documents the
    /// EMIC `PID` (Peripheral ID) register contents or its reset value; SPRUEF8F
    /// §3 does not describe the register at all. It reads as 0 here.
    pub(crate) fn new(base: u32, cores: usize) -> Self {
        assert!(
            cores <= MAX_CORES,
            "the EMIC implements EW_INTCTL0-EW_INTCTL5 only"
        );
        Self {
            base,
            cores,
            pscfg: 0,
            prescale: 0,
            ew_intctl: [0; MAX_CORES],
            rx_pacer: [Pacer::default(); EVENTS],
            tx_pacer: [Pacer::default(); EVENTS],
            prev_rx: 0,
            prev_tx: 0,
            prev_common: [0; MAX_CORES],
            fired_rx: 0,
            fired_tx: 0,
            lines: [Lines::default(); MAX_CORES],
        }
    }
}

impl SlavePort for EmacInterruptControl {
    /// None when the address is not ours.
    ///
    /// Every word of the documented 2 KB EMIC window answers; the reserved holes
    /// listed in SPRS612G Table 7-105 (008h-01Fh, 038h-0FCh and everything above
    /// 17Fh) read as 0 because the block owns them.
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let offset = address.checked_sub(self.base)?;
        if offset >= WINDOW {
            return None;
        }
        if offset & 3 != 0 {
            return Some(0);
        }
        Some(match offset {
            OFF_PID => 0,
            OFF_PSCFG => self.pscfg,
            _ => {
                if let Some(core) = index(offset, OFF_EW_INTCTL, self.cores) {
                    self.ew_intctl[core]
                } else if let Some(n) = index(offset, OFF_RPCFG, EVENTS) {
                    self.rx_pacer[n].read_cfg()
                } else if let Some(n) = index(offset, OFF_RPSTAT, EVENTS) {
                    self.rx_pacer[n].read_stat()
                } else if let Some(n) = index(offset, OFF_TPCFG, EVENTS) {
                    self.tx_pacer[n].read_cfg()
                } else if let Some(n) = index(offset, OFF_TPSTAT, EVENTS) {
                    self.tx_pacer[n].read_stat()
                } else {
                    0
                }
            }
        })
    }

    /// `PortWrite::Unhandled` when the address is not ours.
    ///
    /// RPSTAT and TPSTAT are read-only (SPRUEF8F Figures 25 and 27), as is PID;
    /// writes to them and to reserved words are accepted and discarded.
    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        let Some(offset) = address.checked_sub(self.base) else {
            return PortWrite::Unhandled;
        };
        if offset >= WINDOW {
            return PortWrite::Unhandled;
        }
        if offset & 3 != 0 {
            return PortWrite::Done;
        }
        match offset {
            OFF_PID => {}
            OFF_PSCFG => self.pscfg = value & 0xffff,
            _ => {
                if let Some(core) = index(offset, OFF_EW_INTCTL, self.cores) {
                    self.ew_intctl[core] = value & EW_INTCTL_MASK;
                } else if let Some(n) = index(offset, OFF_RPCFG, EVENTS) {
                    self.rx_pacer[n].write_cfg(value);
                } else if let Some(n) = index(offset, OFF_TPCFG, EVENTS) {
                    self.tx_pacer[n].write_cfg(value);
                }
            }
        }
        PortWrite::Done
    }
}

impl EmacInterruptControl {
    /// Advance one peripheral-clock cycle: run the prescaler and, on `PS_TICK`,
    /// every timed-delay state machine.
    ///
    /// SPRUEF8F §3.4: PSCFG "contains the reload value for the prescalar
    /// configuration value", and §2.7.4/§2.7.5: the TPIC and RPIC each "receive
    /// a single PS_TICK and forward it to all the pacing blocks".
    ///
    /// UNCERTAIN: the documents give neither the width of the `PS_TICK` period
    /// in terms of PRESCALE_CFG nor any special meaning for PRESCALE_CFG = 0.
    /// A reload down-counter is modelled, so `PS_TICK` occurs every
    /// `PRESCALE_CFG + 1` peripheral-clock cycles and PRESCALE_CFG = 0 ticks
    /// every cycle.
    pub(crate) fn tick(&mut self) {
        if self.prescale >= self.pscfg {
            self.prescale = 0;
            for n in 0..EVENTS {
                if self.rx_pacer[n].ps_tick() {
                    self.fired_rx |= 1 << n;
                }
                if self.tx_pacer[n].ps_tick() {
                    self.fired_tx |= 1 << n;
                }
            }
        } else {
            self.prescale += 1;
        }
    }

    /// Sample the EMAC and MDIO interrupt state and recompute every core's
    /// lines.
    ///
    /// Receive and transmit: each rising edge of an RXEVT/TXEVT bit is one event
    /// pulse into that channel's shared pacing block; a pacer output pulse is
    /// then ANDed with EW_INTCTL[16+n] / EW_INTCTL[8+n] of each core and ORed
    /// into MACRXINT/MACTXINT (SPRUEF8F Figures 18 and 19, §3.1: MACRXINT*x*
    /// requires "at least one of the pulse interrupts in the receive group" to
    /// be both enabled and asserted).
    ///
    /// Common: the CIC converts the *gated* level to a pulse (§2.7.6), i.e. the
    /// pulse follows the rising edge of `source & enable`. That is what makes the
    /// §2.17.3 idiom work - clearing and re-setting the EW_INTCTL bit inside the
    /// ISR re-triggers MACINT while the level source is still asserted. Pacing
    /// does not apply to this group: Figure 20 shows no pacing block in the
    /// common path.
    pub(crate) fn update(&mut self, sources: Sources) {
        let rx_edges = sources.rx & !self.prev_rx;
        let tx_edges = sources.tx & !self.prev_tx;
        self.prev_rx = sources.rx;
        self.prev_tx = sources.tx;
        for n in 0..EVENTS {
            let bit = 1u8 << n;
            if rx_edges & bit != 0 && self.rx_pacer[n].event() {
                self.fired_rx |= bit;
            }
            if tx_edges & bit != 0 && self.tx_pacer[n].event() {
                self.fired_tx |= bit;
            }
        }

        let mut common = 0u8;
        if sources.host {
            common |= COMMON_HOST;
        }
        if sources.stat {
            common |= COMMON_STAT;
        }
        if sources.mdio_link {
            common |= COMMON_MDIO_LINT;
        }
        if sources.mdio_user {
            common |= COMMON_MDIO_USER;
        }

        for core in 0..self.cores {
            let enable = self.ew_intctl[core];
            let gated = common & (enable as u8) & COMMON_ALL;
            let pulse = gated & !self.prev_common[core];
            self.prev_common[core] = gated;
            self.lines[core] = Lines {
                rx: self.fired_rx & ((enable >> 16) as u8) != 0,
                tx: self.fired_tx & ((enable >> 8) as u8) != 0,
                common: pulse != 0,
            };
        }
        self.fired_rx = 0;
        self.fired_tx = 0;
    }

    /// The MACRXINT/MACTXINT/MACINT pulses the EMIC drives to `core` in the
    /// cycle computed by the last [`update`](Self::update).
    ///
    /// The device model consumes each asserted field exactly once, as a pulse
    /// into that core's megamodule INTC. Cores beyond the configured count read
    /// as idle.
    pub(crate) fn lines(&self, core: usize) -> Lines {
        self.lines.get(core).copied().unwrap_or_default()
    }
}

/// Index of a register in an array of `count` words starting at `start`.
fn index(offset: u32, start: u32, count: usize) -> Option<usize> {
    let delta = offset.checked_sub(start)?;
    let n = (delta / 4) as usize;
    if delta % 4 == 0 && n < count {
        Some(n)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    /// RX channel `n` asserted, everything else idle.
    fn rx(bits: u8) -> Sources {
        Sources {
            rx: bits,
            ..Sources::default()
        }
    }

    fn emic() -> EmacInterruptControl {
        EmacInterruptControl::new(EMIC0_BASE, 6)
    }

    /// SPRS612G Table 7-105: PID 0h, PSCFG 4h, EW_INTCTL0-5 at 20h, RPCFG0-7 at
    /// 100h, RPSTAT0-7 at 120h, TPCFG0-7 at 140h, TPSTAT0-7 at 160h.
    #[test]
    fn register_offsets_answer_at_their_exact_addresses() {
        let mut e = emic();
        assert_eq!(e.read(EMIC0_BASE, TEST_CONTEXT), Some(0));
        assert_ne!(
            e.write(EMIC0_BASE + 0x004, 0x1234, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(e.read(EMIC0_BASE + 0x004, TEST_CONTEXT), Some(0x1234));
        assert_ne!(
            e.write(EMIC0_BASE + 0x020 + 4 * 5, 0x00ff_ff1e, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            e.read(EMIC0_BASE + 0x020 + 4 * 5, TEST_CONTEXT),
            Some(0x00ff_ff1e)
        );
        // TIME_CFG=1 with the TU key, on the last receive and transmit pacer.
        assert_ne!(
            e.write(EMIC0_BASE + 0x100 + 4 * 7, (1 << 16) | CFG_TU, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            e.read(EMIC0_BASE + 0x100 + 4 * 7, TEST_CONTEXT),
            Some(1 << 16)
        );
        assert_eq!(e.read(EMIC0_BASE + 0x120 + 4 * 7, TEST_CONTEXT), Some(0));
        assert_ne!(
            e.write(EMIC0_BASE + 0x140 + 4 * 7, (2 << 16) | CFG_TU, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            e.read(EMIC0_BASE + 0x140 + 4 * 7, TEST_CONTEXT),
            Some(2 << 16)
        );
        assert_eq!(e.read(EMIC0_BASE + 0x160 + 4 * 7, TEST_CONTEXT), Some(0));
    }

    /// SPRS612G §7.4: the EMIC window is 2 KB. Anything outside is not ours.
    #[test]
    fn out_of_window_address_is_rejected() {
        let mut e = emic();
        assert_eq!(e.read(EMIC0_BASE - 4, TEST_CONTEXT), None);
        assert_eq!(
            e.write(EMIC0_BASE - 4, 1, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(e.read(EMIC0_BASE + WINDOW, TEST_CONTEXT), None);
        assert_eq!(
            e.write(EMIC0_BASE + WINDOW, 1, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        // Reserved inside the window still belongs to the block
        // (SPRS612G Table 7-105 lists 008h-01Fh and 038h-0FCh as reserved).
        assert_eq!(e.read(EMIC0_BASE + 0x008, TEST_CONTEXT), Some(0));
        assert_eq!(e.read(EMIC0_BASE + 0x038, TEST_CONTEXT), Some(0));
        assert_eq!(e.read(EMIC0_BASE + 0x7fc, TEST_CONTEXT), Some(0));
        assert_ne!(
            e.write(EMIC0_BASE + 0x038, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(e.read(EMIC0_BASE + 0x038, TEST_CONTEXT), Some(0));
    }

    /// SPRUEF8F Figure 23: bits 31:24, 7:5 and 0 of EW_INTCTL are reserved;
    /// reset value is 0 for every field.
    #[test]
    fn ew_intctl_resets_to_zero_and_masks_reserved_bits() {
        let mut e = emic();
        for core in 0..6 {
            assert_eq!(e.read(EMIC0_BASE + 0x020 + 4 * core, TEST_CONTEXT), Some(0));
        }
        e.write(EMIC0_BASE + 0x020, 0xffff_ffff, TEST_CONTEXT);
        assert_eq!(
            e.read(EMIC0_BASE + 0x020, TEST_CONTEXT),
            Some(EW_INTCTL_MASK)
        );
    }

    /// SPRUEF8F §3.1: MACRXINT*x* is generated only when one of the receive
    /// pulse interrupts is both enabled in EW_INTCTL and asserted.
    #[test]
    fn core_sees_only_the_channels_its_ew_intctl_enables() {
        let mut e = emic();
        // Core 0 enables RX0 and RX3 (bits 16 and 19) and TX2 (bit 10).
        e.write(
            EMIC0_BASE + 0x020,
            (1 << 16) | (1 << 19) | (1 << 10),
            TEST_CONTEXT,
        );

        e.update(rx(0b0000_0001));
        assert_eq!(
            e.lines(0),
            Lines {
                rx: true,
                tx: false,
                common: false
            }
        );

        // RX1 is not enabled for this core.
        e.update(rx(0b0000_0011));
        assert!(!e.lines(0).rx);

        // RX3 is.
        e.update(rx(0b0000_1011));
        assert!(e.lines(0).rx);

        // TX2 enabled, TX0 not.
        e.update(Sources {
            tx: 0b0000_0001,
            ..Sources::default()
        });
        assert!(!e.lines(0).tx);
        e.update(Sources {
            tx: 0b0000_0101,
            ..Sources::default()
        });
        assert!(e.lines(0).tx);
    }

    /// SPRUEF8F §3.1 and Figure 19: each core has its own EW_INTCTL, so the
    /// per-core enable selects which of the shared pacer outputs reach it.
    #[test]
    fn two_cores_select_disjoint_channel_sets() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, 1 << 16, TEST_CONTEXT); // core 0: RX0
        e.write(EMIC0_BASE + 0x024, 1 << 17, TEST_CONTEXT); // core 1: RX1

        e.update(rx(0b01));
        assert!(e.lines(0).rx);
        assert!(!e.lines(1).rx);

        e.update(rx(0b11));
        assert!(!e.lines(0).rx, "RX0 produced no new pulse");
        assert!(e.lines(1).rx);

        // Cores that enabled nothing stay quiet.
        assert_eq!(e.lines(2), Lines::default());
        assert_eq!(e.lines(5), Lines::default());
    }

    /// SPRUEF8F §2.7.6: the CIC combines HOST and STAT from the EMAC with
    /// MDIO_LINT and MDIO_USER from the MDIO into one MACINT per core, gated by
    /// EW_INTCTL[4:1], and converts level to pulse.
    #[test]
    fn common_line_carries_mdio_and_statistics_sources() {
        let mut e = emic();
        // Core 0 enables STAT (bit 2) and MDIO_LINT (bit 3) only.
        e.write(
            EMIC0_BASE + 0x020,
            COMMON_STAT as u32 | COMMON_MDIO_LINT as u32,
            TEST_CONTEXT,
        );

        let stat = Sources {
            stat: true,
            ..Sources::default()
        };
        e.update(stat);
        assert!(e.lines(0).common, "STAT drives MACINT");

        // Level-to-pulse: the still-asserted level does not re-pulse.
        e.update(stat);
        assert!(!e.lines(0).common);

        // MDIO link change adds a fresh edge.
        e.update(Sources {
            stat: true,
            mdio_link: true,
            ..Sources::default()
        });
        assert!(e.lines(0).common, "MDIO_LINT drives MACINT");

        // HOST and MDIO_USER are masked for this core.
        e.update(Sources {
            host: true,
            mdio_user: true,
            ..Sources::default()
        });
        assert!(!e.lines(0).common);

        // SPRUEF8F §2.17.3: clearing and re-setting the enable re-triggers the
        // interrupt while the level source is still asserted.
        e.update(Sources {
            stat: true,
            ..Sources::default()
        });
        assert!(e.lines(0).common);
        e.write(EMIC0_BASE + 0x020, 0, TEST_CONTEXT);
        e.update(stat);
        assert!(!e.lines(0).common);
        e.write(EMIC0_BASE + 0x020, COMMON_STAT as u32, TEST_CONTEXT);
        e.update(stat);
        assert!(e.lines(0).common);
    }

    /// SPRUEF8F §2.7.6 / Figure 20: the common path has no pacing block, so a
    /// non-zero pacer configuration never delays MACINT.
    #[test]
    fn pacing_does_not_gate_the_common_line() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, COMMON_HOST as u32, TEST_CONTEXT);
        for n in 0..EVENTS as u32 {
            e.write(
                EMIC0_BASE + 0x100 + 4 * n,
                (0xff << 16) | CFG_TU,
                TEST_CONTEXT,
            );
            e.write(
                EMIC0_BASE + 0x140 + 4 * n,
                (0xff << 16) | CFG_TU,
                TEST_CONTEXT,
            );
        }
        e.update(Sources {
            host: true,
            ..Sources::default()
        });
        assert!(e.lines(0).common);
    }

    /// SPRUEF8F §2.7.3: with TIME_CFG = 0 and CNT_CFG = 0 the pacer "immediately
    /// generates a pulse (basically means a zero delay)".
    #[test]
    fn reset_configuration_paces_nothing() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, 1 << 16, TEST_CONTEXT);
        e.update(rx(1));
        assert!(e.lines(0).rx);
        e.update(rx(0));
        e.update(rx(1));
        assert!(e.lines(0).rx);
    }

    /// SPRUEF8F §2.7.3: "When the CNT_CFG is set to non-zero, the CNT_CFG number
    /// of events are counted before an output pulse is generated. The counter
    /// resets (and reloads) every time when a divide-by-N pulse is generated."
    #[test]
    fn count_pacing_holds_the_line_until_the_count_is_reached() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, 1 << 16, TEST_CONTEXT); // core 0 enables RX0
        e.write(EMIC0_BASE + 0x100, (3 << 8) | CFG_CU, TEST_CONTEXT); // RPCFG0: CNT_CFG = 3

        // RPSTAT0 CNT/DIV_SM after one and two of the three events
        // (SPRUEF8F Table 18).
        for expected_cnt in [1u32, 2] {
            e.update(rx(1));
            assert!(!e.lines(0).rx);
            e.update(rx(0));
            assert_eq!(
                e.read(EMIC0_BASE + 0x120, TEST_CONTEXT),
                Some((expected_cnt << 8) | SM_ACTIVE)
            );
        }

        e.update(rx(1));
        assert!(e.lines(0).rx, "third event releases the paced interrupt");
        // Counter reloaded.
        assert_eq!(e.read(EMIC0_BASE + 0x120, TEST_CONTEXT), Some(SM_WAITING));

        // And it paces the next group the same way.
        e.update(rx(0));
        e.update(rx(1));
        assert!(!e.lines(0).rx);
    }

    /// SPRUEF8F Table 17 bit 0 CR: "Resets the divide by N count".
    #[test]
    fn cr_resets_the_divide_by_n_count() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, 1 << 16, TEST_CONTEXT);
        e.write(EMIC0_BASE + 0x100, (4 << 8) | CFG_CU, TEST_CONTEXT);
        e.update(rx(1));
        e.update(rx(0));
        assert_eq!(
            e.read(EMIC0_BASE + 0x120, TEST_CONTEXT),
            Some((1 << 8) | SM_ACTIVE)
        );
        e.write(EMIC0_BASE + 0x100, CFG_CR, TEST_CONTEXT);
        assert_eq!(e.read(EMIC0_BASE + 0x120, TEST_CONTEXT), Some(SM_WAITING));
        // CNT_CFG survived a write without the CU key.
        assert_eq!(e.read(EMIC0_BASE + 0x100, TEST_CONTEXT), Some(4 << 8));
    }

    /// SPRUEF8F Table 17: TU "Enables writes to TIME_CFG", CU "Enables writes to
    /// CNT_CFG"; a locked write leaves the field alone, and TU/CU/TR/CR are
    /// write-only bits that read back as 0.
    #[test]
    fn cfg_fields_ignore_writes_without_their_key() {
        let mut e = emic();
        e.write(
            EMIC0_BASE + 0x100,
            (0x123 << 16) | (0x45 << 8),
            TEST_CONTEXT,
        );
        assert_eq!(e.read(EMIC0_BASE + 0x100, TEST_CONTEXT), Some(0));

        e.write(
            EMIC0_BASE + 0x100,
            (0x123 << 16) | (0x45 << 8) | CFG_TU,
            TEST_CONTEXT,
        );
        assert_eq!(e.read(EMIC0_BASE + 0x100, TEST_CONTEXT), Some(0x123 << 16));

        e.write(
            EMIC0_BASE + 0x100,
            (0x777 << 16) | (0x45 << 8) | CFG_CU,
            TEST_CONTEXT,
        );
        assert_eq!(
            e.read(EMIC0_BASE + 0x100, TEST_CONTEXT),
            Some((0x123 << 16) | (0x45 << 8)),
            "CU write must not disturb TIME_CFG"
        );

        // Same keys on the transmit side (SPRUEF8F Table 19).
        e.write(
            EMIC0_BASE + 0x140,
            (0x0aa << 16) | (0x0b << 8),
            TEST_CONTEXT,
        );
        assert_eq!(e.read(EMIC0_BASE + 0x140, TEST_CONTEXT), Some(0));
        e.write(
            EMIC0_BASE + 0x140,
            (0x0aa << 16) | (0x0b << 8) | CFG_TU | CFG_CU,
            TEST_CONTEXT,
        );
        assert_eq!(
            e.read(EMIC0_BASE + 0x140, TEST_CONTEXT),
            Some((0x0aa << 16) | (0x0b << 8))
        );
    }

    /// SPRUEF8F §2.7.2: "an output pulse is generated after the TIME_CFG number
    /// of prescalar output periods. The counter starts counting on the first
    /// event."  With PRESCALE_CFG = 0 a prescaler output period is one
    /// peripheral-clock cycle.
    #[test]
    fn time_pacing_holds_the_line_until_the_delay_elapses() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, 1 << 16, TEST_CONTEXT);
        e.write(EMIC0_BASE + 0x100, (3 << 16) | CFG_TU, TEST_CONTEXT); // TIME_CFG = 3

        e.update(rx(1));
        assert!(!e.lines(0).rx, "the first event only starts the timer");
        // TIM_SM = DELAY (SPRUEF8F Table 18).
        assert_eq!(
            e.read(EMIC0_BASE + 0x120, TEST_CONTEXT),
            Some(SM_ACTIVE << 2)
        );

        for expected_time in [1u32, 2] {
            e.tick();
            e.update(rx(1));
            assert!(!e.lines(0).rx);
            assert_eq!(
                e.read(EMIC0_BASE + 0x120, TEST_CONTEXT),
                Some((expected_time << 16) | (SM_ACTIVE << 2))
            );
        }

        e.tick();
        e.update(rx(1));
        assert!(e.lines(0).rx, "the third PS_TICK releases the interrupt");
        assert_eq!(e.read(EMIC0_BASE + 0x120, TEST_CONTEXT), Some(SM_WAITING));

        // The pulse lasts exactly one cycle.
        e.tick();
        e.update(rx(1));
        assert!(!e.lines(0).rx);
    }

    /// SPRUEF8F Table 17 bit 1 TR: "Resets the timer counter".
    #[test]
    fn tr_resets_the_timer_counter() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, 1 << 16, TEST_CONTEXT);
        e.write(EMIC0_BASE + 0x100, (4 << 16) | CFG_TU, TEST_CONTEXT);
        e.update(rx(1));
        e.tick();
        e.update(rx(1));
        assert_eq!(
            e.read(EMIC0_BASE + 0x120, TEST_CONTEXT),
            Some((1 << 16) | (SM_ACTIVE << 2))
        );
        e.write(EMIC0_BASE + 0x100, CFG_TR, TEST_CONTEXT);
        assert_eq!(e.read(EMIC0_BASE + 0x120, TEST_CONTEXT), Some(SM_WAITING));
        assert_eq!(e.read(EMIC0_BASE + 0x100, TEST_CONTEXT), Some(4 << 16));
    }

    /// SPRUEF8F §2.7.3: "When the TIME_CFG is set to non-zero, it then disables
    /// the divide-by-N state machine."
    #[test]
    fn nonzero_time_cfg_disables_the_divide_by_n_machine() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, 1 << 16, TEST_CONTEXT);
        e.write(
            EMIC0_BASE + 0x100,
            (2 << 16) | (2 << 8) | CFG_TU | CFG_CU,
            TEST_CONTEXT,
        );
        for _ in 0..4 {
            e.update(rx(1));
            assert!(!e.lines(0).rx);
            e.update(rx(0));
        }
        // DIV_SM never left WAITING and CNT stayed 0.
        assert_eq!(
            e.read(EMIC0_BASE + 0x120, TEST_CONTEXT).unwrap() & 0x0000_ff03,
            SM_WAITING
        );
    }

    /// SPRUEF8F §3.4: PSCFG holds the reload value of the prescaler that
    /// produces PS_TICK, and §2.7.4/§2.7.5 forward that one PS_TICK to all
    /// pacing blocks. With PRESCALE_CFG = 3 a prescaler output period is four
    /// peripheral-clock cycles, so TIME_CFG = 2 delays the interrupt by eight.
    #[test]
    fn pscfg_scales_the_pacing_clock() {
        let mut e = emic();
        e.write(EMIC0_BASE + 0x020, 1 << 16, TEST_CONTEXT);
        e.write(EMIC0_BASE + 0x004, 3, TEST_CONTEXT); // PRESCALE_CFG = 3
        e.write(EMIC0_BASE + 0x100, (2 << 16) | CFG_TU, TEST_CONTEXT);

        e.update(rx(1));
        assert!(!e.lines(0).rx);
        for cycle in 1..=7 {
            e.tick();
            e.update(rx(1));
            assert!(!e.lines(0).rx, "cycle {cycle} must stay paced");
        }
        e.tick();
        e.update(rx(1));
        assert!(
            e.lines(0).rx,
            "two prescaler periods of four cycles elapsed"
        );
    }

    /// SPRUEF8F §2.7.4/§2.7.5: the transmit and receive combiners are separate
    /// paths with their own pacers, and §3.2.1/§3.3.1: the pacer configuration is
    /// shared by all megamodules, so one pacer output feeds every enabled core.
    #[test]
    fn transmit_and_receive_paths_are_independent_and_pacers_are_shared() {
        let mut e = emic();
        // Cores 0 and 4 both enable RX2; core 0 also enables TX2.
        e.write(EMIC0_BASE + 0x020, (1 << 18) | (1 << 10), TEST_CONTEXT);
        e.write(EMIC0_BASE + 0x020 + 4 * 4, 1 << 18, TEST_CONTEXT);
        e.write(EMIC0_BASE + 0x100 + 4 * 2, (2 << 8) | CFG_CU, TEST_CONTEXT); // RPCFG2 CNT_CFG=2

        let ev = Sources {
            rx: 1 << 2,
            tx: 1 << 2,
            ..Sources::default()
        };
        e.update(ev);
        assert!(!e.lines(0).rx, "receive is paced");
        assert!(e.lines(0).tx, "transmit pacer is still zero-delay");
        assert!(!e.lines(4).rx);

        e.update(Sources::default());
        e.update(ev);
        assert!(e.lines(0).rx);
        assert!(e.lines(4).rx, "the shared pacer output reaches both cores");
        assert!(!e.lines(4).tx, "core 4 did not enable TX2");
    }

    /// A device configured with fewer cores than EW_INTCTL5 leaves the surplus
    /// registers reserved, and `lines` for an unimplemented core is idle.
    #[test]
    fn core_count_bounds_the_ew_intctl_array() {
        let mut e = EmacInterruptControl::new(EMIC1_BASE, 2);
        assert_ne!(
            e.write(EMIC1_BASE + 0x020 + 4 * 2, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(e.read(EMIC1_BASE + 0x020 + 4 * 2, TEST_CONTEXT), Some(0));
        e.update(rx(0xff));
        assert_eq!(e.lines(2), Lines::default());
        assert_eq!(e.lines(99), Lines::default());
    }
}
