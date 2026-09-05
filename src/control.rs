use std::cell::Cell;

const CSR_CPU_ID: u32 = 0x10 << 24;
const CSR_ENDIAN: u32 = 1 << 8;
const TSR_GIE: u32 = 1 << 0;
const TSR_SGIE: u32 = 1 << 1;
const TSR_GEE: u32 = 1 << 2;
const TSR_INT: u32 = 1 << 9;
/// TSR bit 3: enables the maskable external exception input `EXCEP`.
const TSR_XEN: u32 = 1 << 3;
/// TSR bit 10: set while an exception is being processed (SPRU732J Table 2-21).
const TSR_EXC: u32 = 1 << 10;
/// TSR bits 7-6: the current execution mode, 0 Supervisor and 1 User (SPRU732J 8.1).
const TSR_CXM: u32 = 1 << 6;
/// TSR bit 14: set by hardware while the SPLOOP buffer is executing (SPRUFE8B Table 2-23).
const TSR_SPLX: u32 = 1 << 14;
/// NTSR bit 16: the exception was taken by hardware (NMI, EXCEP or internal) rather than by
/// `SWE`/`SWENR`, which is how a handler tells a service request from a fault (SPRU732J
/// Table 2-23).
const NTSR_HWE: u32 = 1 << 16;
/// EFR bit 0: a software exception from `SWE` or `SWENR`.
const EFR_SXF: u32 = 1 << 0;
/// EFR bit 1: an internal exception, with the cause in IERR.
const EFR_IXF: u32 = 1 << 1;
/// EFR bit 31: the NMI pin was recognised while exceptions were enabled.
const EFR_NXF: u32 = 1 << 31;
/// EFR bit 30: the maskable external exception input `EXCEP`.
const EFR_EXF: u32 = 1 << 30;
/// `IER` bit 1 is NMIE, which also gates every maskable interrupt and external exception.
const IER_NMIE: u32 = 1 << 1;

/// GFPGFR after reset: field size 7h in bits 24-26 and polynomial 1Dh in bits 0-7
/// (SPRUFE8B section 2.8.5).
const GFPGFR_RESET: u32 = 0x0700_001d;

pub struct Control {
    amr: u32,
    csr_misc: u32,
    big_endian: bool,
    /// CSR bit 9: set only by a saturating functional unit, cleared only by MVC.
    sat: bool,
    ifr: u16,
    ier: u16,
    istp_base: u32,
    irp: u32,
    nrp: u32,
    tscl: u32,
    tsch: u32,
    tsc_enabled: bool,
    tsch_latch: Cell<u32>,
    dier: u32,
    ilc: u32,
    rilc: u32,
    rep: u32,
    ssr: u32,
    gplya: u32,
    gplyb: u32,
    gfpgfr: u32,
    tsr: u32,
    itsr: u32,
    ntsr: u32,
    efr: u32,
    ierr: u32,
    set_due: [u64; 16],
    set_due_mask: u16,
    line_high: u16,
    /// DNUM, the megamodule number this CPU reports (read-only to software).
    dnum: u32,
}

impl Default for Control {
    fn default() -> Self {
        Self {
            amr: 0,
            csr_misc: 0,
            big_endian: false,
            sat: false,
            ifr: 0,
            ier: 0,
            istp_base: 0,
            irp: 0,
            nrp: 0,
            tscl: 0,
            tsch: 0,
            tsc_enabled: false,
            ilc: 0,
            tsch_latch: Cell::new(0),
            dier: 0,
            rilc: 0,
            rep: 0,
            ssr: 0,
            gplya: 0,
            gplyb: 0,
            gfpgfr: GFPGFR_RESET,
            tsr: 0,
            itsr: 0,
            ntsr: 0,
            efr: 0,
            ierr: 0,
            set_due: [u64::MAX; 16],
            set_due_mask: 0,
            line_high: 0,
            dnum: 0,
        }
    }
}

impl Control {
    pub fn set_big_endian(&mut self, big: bool) {
        self.big_endian = big;
    }

    /// Which megamodule this is, which DNUM reports. Set once, when the device builds its
    /// cores (SPRS612G section 5.7).
    pub fn set_core_number(&mut self, core: u32) {
        self.dnum = core;
    }

    pub fn read(&self, register: u8, pce1: u32) -> u32 {
        match register {
            0x00 => self.amr,
            0x01 => {
                CSR_CPU_ID
                    | self.csr_misc
                    | if self.big_endian { CSR_ENDIAN } else { 0 }
                    | u32::from(self.sat) << 9
                    | (self.itsr & 1) << 1
                    | (self.tsr & 1)
            }
            0x02 => self.ifr as u32,
            0x04 => self.ier as u32,
            0x05 => self.istp_base | (self.highest_pending().unwrap_or(0) as u32) << 5,
            0x06 => self.irp,
            0x07 => self.nrp,
            0x0a => {
                self.tsch_latch.set(self.tsch);
                self.tscl
            }
            0x0b => self.tsch_latch.get(),
            0x0d => self.ilc,
            0x0e => self.rilc,
            0x0f => self.rep,
            0x10 => pce1,
            // DNUM: the number of the megamodule this CPU is, which is how one image running on
            // six cores tells them apart (SPRU871K section 2.9.4, SPRS612G section 5.7).
            0x11 => self.dnum,
            0x15 => self.ssr,
            0x16 => self.gplya,
            0x17 => self.gplyb,
            0x19 => self.dier,
            0x18 => self.gfpgfr,
            0x1a => self.tsr,
            0x1b => self.itsr,
            0x1c => self.ntsr,
            0x1d => self.efr,
            0x1f => self.ierr,
            _ => 0,
        }
    }

    pub fn write(&mut self, register: u8, value: u32) {
        match register {
            0x00 => self.amr = value & 0x03ff_ffff,
            0x01 => {
                self.csr_misc = value & 0x0000_fc00;
                // Writing zero to the SAT bit clears it; writing one has no effect
                // (SPRUFE8B Table 2-9).
                if value & (1 << 9) == 0 {
                    self.sat = false;
                }
                self.tsr = (self.tsr & !TSR_GIE) | (value & TSR_GIE);
                self.itsr = (self.itsr & !TSR_GIE) | ((value >> 1) & TSR_GIE);
            }
            0x02 => self.ifr |= (value & 0xfff0) as u16,
            0x03 => self.ifr &= !(value as u16),
            0x04 => {
                // Software can set NMIE but cannot clear it; only NMI entry and reset clear
                // it. Maskable enable bits remain ordinary read/write bits.
                let nmie = self.ier & IER_NMIE as u16 | (value & IER_NMIE) as u16;
                self.ier = (value & 0xfff0) as u16 | nmie;
            }
            0x05 => self.istp_base = value & !0x3ff,
            0x06 => self.irp = value & !1,
            0x07 => self.nrp = value & !1,
            0x0a => self.tsc_enabled = true,
            0x0d => self.ilc = value,
            0x0e => self.rilc = value,
            0x0f => self.rep = value & !1,
            0x19 => {
                if value & 2 == 0 {
                    self.dier = value & 0xc000_fff0;
                }
            }
            0x15 => self.ssr = value,
            0x16 => self.gplya = value,
            0x17 => self.gplyb = value,
            0x18 => self.gfpgfr = value,
            0x1a => {
                let gee = self.tsr & TSR_GEE | value & TSR_GEE;
                self.tsr = value & 0x0000_c65f | gee;
            }
            0x1b => self.itsr = value & 0x0000_c65f,
            0x1c => self.ntsr = value & 0x0001_c65f,
            // Writing ECR clears the EFR flags named by the ones written, while IERR is a
            // plain read/write register: a handler walks the causes and writes back the ones
            // it has not dealt with yet (linux-c6x `process_iexcept` does exactly that).
            0x1d => self.efr &= !value,
            0x1f => self.ierr = value & 0x1ff,
            _ => {}
        }
    }

    #[inline(always)]
    pub fn tick(&mut self, cycle: u64, lines: u16) -> Option<u8> {
        if self.tsc_enabled {
            let (low, carry) = self.tscl.overflowing_add(1);
            self.tscl = low;
            self.tsch = self.tsch.wrapping_add(u32::from(carry));
        }
        if lines == self.line_high && self.set_due_mask == 0 {
            return None;
        }
        let rising = lines & !self.line_high;
        let mut incoming = rising & 0xfff0;
        while incoming != 0 {
            let interrupt = incoming.trailing_zeros() as usize;
            let bit = 1u16 << interrupt;
            if self.set_due[interrupt] == u64::MAX {
                self.set_due[interrupt] = cycle + 2;
                self.set_due_mask |= bit;
            }
            incoming &= !bit;
        }
        let mut dropped = None;
        let mut due = self.set_due_mask;
        while due != 0 {
            let interrupt = due.trailing_zeros() as usize;
            let bit = 1u16 << interrupt;
            if self.set_due[interrupt] == cycle {
                if self.ifr & bit != 0 {
                    dropped.get_or_insert(interrupt as u8);
                } else {
                    self.ifr |= bit;
                }
                self.set_due[interrupt] = u64::MAX;
                self.set_due_mask &= !bit;
            }
            due &= !bit;
        }
        self.line_high = lines;
        dropped
    }

    /// `DINT` clears TSR.GIE, saving its previous value in TSR.SGIE (bit 1).
    pub fn disable_interrupts(&mut self) {
        let gie = self.tsr & TSR_GIE;
        self.tsr = (self.tsr & !(TSR_GIE | TSR_SGIE)) | (gie << 1);
    }

    /// `RINT` restores TSR.GIE from TSR.SGIE.
    pub fn restore_interrupts(&mut self) {
        let saved = (self.tsr & TSR_SGIE) >> 1;
        self.tsr = (self.tsr & !TSR_GIE) | saved;
    }

    /// The Galois field polynomial for the given side, used by `GMPY`.
    pub fn polynomial(&self, side_b: bool) -> u32 {
        if side_b {
            self.gplyb
        } else {
            self.gplya
        }
    }

    /// The addressing modes and block sizes that drive circular addressing.
    pub fn amr(&self) -> u32 {
        self.amr
    }

    /// The Galois field polynomial and field size that control `GMPY4`.
    pub fn gfpgfr(&self) -> u32 {
        self.gfpgfr
    }

    pub fn rilc(&self) -> u32 {
        self.rilc
    }

    pub fn ilc(&self) -> u32 {
        self.ilc
    }

    pub fn set_ilc(&mut self, value: u32) {
        self.ilc = value;
    }

    pub fn raise_ierr(&mut self, mask: u32) {
        self.ierr |= mask & 0x1ff;
    }

    #[inline(always)]
    pub fn takeable_interrupt(&self, branches_clear: bool) -> Option<u8> {
        if !branches_clear || self.tsr & TSR_GIE == 0 || self.ier & (1 << 1) == 0 {
            return None;
        }
        let pending = self.ifr & self.ier & 0xfff0;
        (pending != 0).then(|| pending.trailing_zeros() as u8)
    }

    /// `in_sploop` records whether the interrupted code was inside a drained SPLOOP: ITSR
    /// keeps the SPLX bit so the return pipes the loop back up (SPRUFE8B 7.13.4).
    pub fn enter_interrupt(&mut self, interrupt: u8, return_pc: u32, in_sploop: bool) -> u32 {
        self.itsr = if in_sploop {
            self.tsr | TSR_SPLX
        } else {
            self.tsr & !TSR_SPLX
        };
        self.tsr = self.tsr & TSR_GEE | TSR_INT;
        self.irp = return_pc;
        self.ifr &= !(1 << interrupt);
        self.istp_base.wrapping_add((interrupt as u32) << 5)
    }
    pub fn return_from_irp(&mut self) -> u32 {
        self.tsr = self.itsr;
        self.irp
    }

    /// A saturating functional unit sets its bit in SSR and the SAT bit in CSR; both take
    /// precedence over an MVC write in the same cycle (SPRUFE8B Table 2-9 and Table 2-22).
    pub fn note_saturation(&mut self, unit_bit: u32) {
        self.sat = true;
        self.ssr |= 1 << unit_bit;
    }

    /// True when the TSR restored by the last return names an interrupted SPLOOP.
    pub fn sploop_executing(&self) -> bool {
        self.tsr & TSR_SPLX != 0
    }

    pub fn set_sploop_executing(&mut self, active: bool) {
        if active {
            self.tsr |= TSR_SPLX;
        } else {
            self.tsr &= !TSR_SPLX;
        }
    }

    /// `B NRP` restores the task state saved on the exception. `NTSR.HWE` lives above the
    /// TSR fields and does not come back with it, and the return sets NMIE again the way
    /// `B IRP` does not (SPRU732J 6.2.4).
    pub fn return_from_nrp(&mut self) -> u32 {
        self.tsr = self.ntsr & 0x0000_ffff;
        self.ier |= IER_NMIE as u16;
        self.nrp
    }

    /// Exceptions are enabled solely by `TSR.GEE`, which software can set but never clear.
    pub fn exceptions_enabled(&self) -> bool {
        self.tsr & TSR_GEE != 0
    }

    /// True while an exception is being processed, which is what makes the next one nested.
    pub fn in_exception(&self) -> bool {
        self.tsr & TSR_EXC != 0
    }

    /// The execution mode the last context restore installed. The mode a instruction actually
    /// runs in changes at the branch target, not in the delay slots, so the CPU tracks that
    /// separately; this is the architectural bit.
    pub fn user_mode(&self) -> bool {
        self.tsr & TSR_CXM != 0
    }

    /// An `MVC` write in User mode. The fully restricted registers never reach here - they
    /// raise an exception at the instruction - but CSR and TSR are partially restricted:
    /// User code may clear CSR.SAT and change GIE, and may change TSR.GIE and TSR.SGIE, while
    /// PGIE, PWRD, PCC, DCC and every hardware-maintained TSR bit stay put (SPRU732J 8.2.4.2).
    pub fn write_user(&mut self, register: u8, value: u32) {
        match register {
            0x01 => {
                if value & (1 << 9) == 0 {
                    self.sat = false;
                }
                self.tsr = (self.tsr & !TSR_GIE) | (value & TSR_GIE);
            }
            0x1a => {
                let writable = TSR_GIE | TSR_SGIE;
                self.tsr = (self.tsr & !writable) | (value & writable);
            }
            _ => self.write(register, value),
        }
    }

    /// Take an exception: save TSR into NTSR and the address of the first annulled execute
    /// packet into NRP, then enter the exception context (SPRU732J Table 6-3 - GIE, SGIE,
    /// XEN, INT and SPLX clear, EXC set, CXM back to Supervisor, GEE untouched).  `hardware`
    /// distinguishes an NMI, EXCEP or internal exception, which sets `NTSR.HWE`, from a
    /// `SWE`/`SWENR` service request, which clears it.  `in_sploop` records that a loop was
    /// terminated, which is one of the conditions that make the exception unresumable.
    /// Returns the vector address.
    pub fn enter_exception(
        &mut self,
        flag: u32,
        return_pc: u32,
        hardware: bool,
        in_sploop: bool,
    ) -> u32 {
        self.efr |= flag;
        self.ntsr = (self.tsr & !TSR_SPLX)
            | if in_sploop { TSR_SPLX } else { 0 }
            | if hardware { NTSR_HWE } else { 0 };
        self.tsr = self.tsr & TSR_GEE | TSR_EXC;
        self.nrp = return_pc;
        self.istp_base.wrapping_add(0x20)
    }

    /// `SWENR` behaves like `SWE` but enters at `REP` instead of the exception vector.
    pub fn enter_software_exception(&mut self, return_pc: u32, restricted: bool) -> u32 {
        let vector = self.enter_exception(EFR_SXF, return_pc, false, false);
        if restricted {
            self.rep
        } else {
            vector
        }
    }

    /// An internal exception reports its cause in IERR, which the raising site has already
    /// set, and flags itself in `EFR.IXF`.
    pub fn enter_internal_exception(&mut self, return_pc: u32, in_sploop: bool) -> u32 {
        self.enter_exception(EFR_IXF, return_pc, true, in_sploop)
    }
    /// Whether a rising NMI edge can be latched. Once an NMI exception has set NXF, another
    /// edge is ignored until the handler acknowledges the flag through ECR.
    pub fn nmi_edge_allowed(&self) -> bool {
        self.tsr & TSR_GEE == 0 || self.efr & EFR_NXF == 0
    }

    /// With exceptions enabled, NMI is an exception and is not blocked by branch delay slots.
    pub fn nmi_exception_ready(&self) -> bool {
        self.tsr & TSR_GEE != 0 && self.ier & IER_NMIE as u16 != 0 && self.efr & EFR_NXF == 0
    }

    /// With exceptions disabled, NMI follows the interrupt path and waits for branch shadows.
    pub fn nmi_interrupt_ready(&self, branches_clear: bool) -> bool {
        branches_clear && self.tsr & TSR_GEE == 0 && self.ier & IER_NMIE as u16 != 0
    }

    /// Enter NMI through the legacy interrupt path. Unlike a maskable interrupt, NMI saves
    /// NTSR/NRP and returns with `B NRP`; NTSR.HWE identifies the hardware source.
    pub fn enter_nmi_interrupt(&mut self, return_pc: u32, in_sploop: bool) -> u32 {
        self.ntsr = (self.tsr & !TSR_SPLX) | if in_sploop { TSR_SPLX } else { 0 } | NTSR_HWE;
        self.tsr = self.tsr & TSR_GEE | TSR_INT;
        self.nrp = return_pc;
        self.ier &= !(IER_NMIE as u16);
        self.istp_base.wrapping_add(0x20)
    }

    /// With GEE set, NMI uses the exception context and reports EFR.NXF.
    pub fn enter_nmi_exception(&mut self, return_pc: u32, in_sploop: bool) -> u32 {
        let vector = self.enter_exception(EFR_NXF, return_pc, true, in_sploop);
        self.ier &= !(IER_NMIE as u16);
        vector
    }

    /// The maskable external exception input. It needs GEE, `TSR.XEN` and `IER.NMIE`, and is
    /// ignored while `EFR.EXF` is still set: taking the exception does not clear the flag, so
    /// an input that is still asserted cannot re-enter a handler that has not acknowledged it
    /// through ECR (SPRU732J 6.1.2).
    pub fn external_exception_ready(&self) -> bool {
        self.tsr & TSR_GEE != 0
            && self.tsr & TSR_XEN != 0
            && self.ier & IER_NMIE as u16 != 0
            && self.efr & EFR_EXF == 0
    }

    /// Taking `EXCEP` also clears `TSR.XEN`, which `enter_exception` does along with the rest
    /// of the context, so a second external exception needs the handler to enable it again.
    pub fn enter_external_exception(&mut self, return_pc: u32, in_sploop: bool) -> u32 {
        self.enter_exception(EFR_EXF, return_pc, true, in_sploop)
    }

    /// An exception recognised while `TSR.EXC` is already set cannot save its context without
    /// destroying the outer one, so the CPU leaves NTSR and NRP alone, saves TSR and the PC
    /// into the interrupt pair instead, clears NMIE and vectors through the service table's
    /// RESET entry, which boot code inspects to tell this apart from a pin reset
    /// (SPRU732J 6.3.4).
    pub fn enter_nested_exception(&mut self, return_pc: u32) -> u32 {
        self.itsr = self.tsr;
        self.irp = return_pc;
        self.ier &= !(IER_NMIE as u16);
        self.istp_base
    }
    #[inline(always)]
    fn highest_pending(&self) -> Option<u8> {
        let pending = self.ifr & self.ier & !1;
        (pending != 0).then(|| pending.trailing_zeros() as u8)
    }
}

/// The control registers User mode may not touch at all: any `MVC` naming one raises an
/// exception reporting both `IERR.RAX` and `IERR.PRX`. SPRU732J 8.2.4.1 lists ECR, EFR, ICR,
/// IER, IERR, IFR, ISTP, ITSR, NTSR and REP; Table 3-22 additionally marks ISR and DIER, and
/// leaving ISR open to User mode would let it raise interrupt flags, so this is the union.
/// EFR and ECR share 0x1d, and IFR and ISR share 0x02.
pub fn restricted_in_user(register: u8) -> bool {
    matches!(
        register,
        0x02 | 0x03 | 0x04 | 0x05 | 0x0f | 0x19 | 0x1b | 0x1c | 0x1d | 0x1f
    )
}

/// The control registers reachable through `MVC`, keyed by the register number the encoding
/// carries (SPRU732J Table 3-22). Names are the ones TI's tools print, which is what a trace
/// line or a debugger register dump should say.
pub fn register_name(register: u8) -> Option<&'static str> {
    Some(match register {
        0x00 => "amr",
        0x01 => "csr",
        0x02 => "ifr",
        0x03 => "icr",
        0x04 => "ier",
        0x05 => "istp",
        0x06 => "irp",
        0x07 => "nrp",
        0x0a => "tscl",
        0x0b => "tsch",
        0x0d => "ilc",
        0x0e => "rilc",
        0x0f => "rep",
        0x10 => "pce1",
        0x11 => "dnum",
        0x15 => "ssr",
        0x16 => "gplya",
        0x17 => "gplyb",
        0x18 => "gfpgfr",
        0x19 => "dier",
        0x1a => "tsr",
        0x1b => "itsr",
        0x1c => "ntsr",
        0x1d => "efr",
        0x1f => "ierr",
        _ => return None,
    })
}

/// The name TI's tools print for a control register that `MVC` writes. Two encodings name one
/// register when read and another when written: 0x02 reads IFR and writes ISR, and 0x1d reads
/// EFR and writes ECR (SPRU732J Table 3-22), which is exactly what `write` implements - a
/// write to 0x02 sets interrupt flags and a write to 0x1d clears exception flags.
pub fn written_register_name(register: u8) -> Option<&'static str> {
    Some(match register {
        0x02 => "isr",
        0x1d => "ecr",
        _ => return register_name(register),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Control register numbers: CSR is 1 and IER is 4.
    const CSR: u8 = 0x01;
    const IER: u8 = 0x04;

    #[test]
    fn a_line_that_stays_asserted_does_not_re_enter_the_handler() {
        // The CPU recognises an interrupt request as a 0-to-1 transition of its routed line, and
        // SPRU871K 7.4.2 says a routine servicing a directly selected event need neither read
        // nor clear EVTFLAG - which is only consistent if a line that is still asserted on the
        // way out of the handler does not announce the interrupt again. A new assertion does.
        let mut control = Control::default();
        control.write(IER, (1 << 1) | (1 << 4));
        control.write(CSR, 1);

        let line = 1u16 << 4;
        for cycle in 0..4 {
            control.tick(cycle, line);
        }
        assert_eq!(control.takeable_interrupt(true), Some(4));
        control.enter_interrupt(4, 0x1000, false);
        assert_eq!(control.takeable_interrupt(true), None);

        // The routine returns with the line still high: nothing new is pending.
        control.return_from_irp();
        for cycle in 4..8 {
            control.tick(cycle, line);
        }
        assert_eq!(control.takeable_interrupt(true), None);

        // The source is quieted and asserts again, which is a fresh transition.
        control.tick(8, 0);
        for cycle in 9..13 {
            control.tick(cycle, line);
        }
        assert_eq!(control.takeable_interrupt(true), Some(4));
    }
}
