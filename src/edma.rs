//! The EDMA3 channel controller (EDMA3CC) of the C6455.
//!
//! SPRU966C: a RAM-based transfer engine. Every channel names a 32-byte PaRAM set holding the
//! source, destination, three counts, four indexes and a link address; a trigger submits one
//! transfer request per synchronisation event, and the channel controller updates the set
//! between requests. This model performs one transfer request per tick, which is what makes a
//! driver's "wait for the completion interrupt" loop behave like hardware.

use crate::edma_tc::ServicedRequest;
use crate::port::{PortContext, PortWrite, SlavePort};
use crate::system::Privilege;
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

/// PRIV (OPT bit 31, set for supervisor) and PRIVID (OPT bits 27-24), the fields the channel
/// controller stamps on a PaRAM set when a master writes it (SPRU966C Table 2-3).
const OPT_STAMP: u32 = (1 << 31) | (0xf << 24);

fn opt_stamp(privilege: Privilege) -> u32 {
    (u32::from(!privilege.user) << 31) | (u32::from(privilege.id & 0xf) << 24)
}

/// The identity a PaRAM set carries, which its transfer requests are issued under: whoever
/// last wrote any word of it (SPRU966C section 2.10.2), not the controller.
fn stamped_privilege(opt: u32) -> Privilege {
    Privilege {
        id: ((opt >> 24) & 0xf) as u8,
        user: opt >> 31 == 0,
    }
}

/// What the transfer controllers move data through. Each access names the privilege it is
/// made with, so the memory system can apply that identity's protection.
pub trait TransferMemory {
    fn read(&mut self, privilege: Privilege, address: u32, length: usize) -> Vec<u8>;
    fn write(&mut self, privilege: Privilege, address: u32, bytes: &[u8]);
}

const BASE: u32 = 0x02a0_0000;
const PARAM_BASE: u32 = 0x02a0_4000;
const PARAM_WORDS: usize = 256 * 8;
const CHANNELS: usize = 64;

const PID: u32 = BASE;
const CCCFG: u32 = BASE + 0x0004;
const DCHMAP_BASE: u32 = BASE + 0x0100;
const QCHMAP_BASE: u32 = BASE + 0x0200;
const DMAQNUM_BASE: u32 = BASE + 0x0240;
const QDMAQNUM: u32 = BASE + 0x0260;
const QUETCMAP: u32 = BASE + 0x0280;
const QUEPRI: u32 = BASE + 0x0284;
const EMR: u32 = BASE + 0x0300;
const EMRH: u32 = BASE + 0x0304;
const EMCR: u32 = BASE + 0x0308;
const EMCRH: u32 = BASE + 0x030c;
const QEMR: u32 = BASE + 0x0310;
const QEMCR: u32 = BASE + 0x0314;
const CCERR: u32 = BASE + 0x0318;
const CCERRCLR: u32 = BASE + 0x031c;
const EEVAL: u32 = BASE + 0x0320;
const DRAE_BASE: u32 = BASE + 0x0340;
const CCSTAT: u32 = BASE + 0x0640;
const QRAE_BASE: u32 = BASE + 0x0380;
const ER: u32 = BASE + 0x1000;
const ERH: u32 = BASE + 0x1004;
const ECR: u32 = BASE + 0x1008;
const ECRH: u32 = BASE + 0x100c;
const ESR: u32 = BASE + 0x1010;
const ESRH: u32 = BASE + 0x1014;
const CER: u32 = BASE + 0x1018;
const CERH: u32 = BASE + 0x101c;
const EER: u32 = BASE + 0x1020;
const EERH: u32 = BASE + 0x1024;
const EECR: u32 = BASE + 0x1028;
const EECRH: u32 = BASE + 0x102c;
const EESR: u32 = BASE + 0x1030;
const MPFAR: u32 = BASE + 0x0800;
const MPFSR: u32 = BASE + 0x0804;
const MPFCR: u32 = BASE + 0x0808;
const MPPAG: u32 = BASE + 0x080c;
const MPPA_BASE: u32 = BASE + 0x0810;
const EESRH: u32 = BASE + 0x1034;
const SER: u32 = BASE + 0x1038;
const SERH: u32 = BASE + 0x103c;
const SECR: u32 = BASE + 0x1040;
const SECRH: u32 = BASE + 0x1044;
const IER: u32 = BASE + 0x1050;
const IERH: u32 = BASE + 0x1054;
const IECR: u32 = BASE + 0x1058;
const IECRH: u32 = BASE + 0x105c;
const IESR: u32 = BASE + 0x1060;
const IESRH: u32 = BASE + 0x1064;
const IPR: u32 = BASE + 0x1068;
const IPRH: u32 = BASE + 0x106c;
const ICR: u32 = BASE + 0x1070;
const ICRH: u32 = BASE + 0x1074;
const IEVAL: u32 = BASE + 0x1078;
const QER: u32 = BASE + 0x1080;
const QEER: u32 = BASE + 0x1084;
const QEECR: u32 = BASE + 0x1088;
const QEESR: u32 = BASE + 0x108c;
const QSER: u32 = BASE + 0x1090;
const QSECR: u32 = BASE + 0x1094;
/// The eight shadow regions are 0x200 apart and repeat the channel registers of the global
/// region, filtered by that region's access enables (SPRU966C §2.7 and Table 2-8).
const SHADOW_BASE: u32 = BASE + 0x2000;
const SHADOW_STRIDE: u32 = 0x200;
const SHADOW_LIMIT: u32 = 0x0098;
const REGIONS: usize = 8;
const QDMA_CHANNELS: usize = 4;
const QUEUE_ENTRY_BASE: u32 = BASE + 0x0400;
const QSTAT_BASE: u32 = BASE + 0x0600;
const QWMTHRA: u32 = BASE + 0x0620;
const QUEUES: usize = 4;
const QUEUE_DEPTH: usize = 16;
/// Every queue's watermark threshold resets to the maximum, sixteen (SPRU966C Table 4-24).
const QWMTHRA_RESET: u32 = 0x1010_1010;
/// The event types an entry records: a peripheral event, a manual trigger, a chained trigger
/// and a QDMA trigger (SPRU966C Table 4-22).
const ETYPE_EVENT: u32 = 0;
const ETYPE_MANUAL: u32 = 1;
const ETYPE_CHAIN: u32 = 2;
const ETYPE_QDMA: u32 = 3;
/// A page attribute register resets with every requestor id allowed, external ids allowed, and
/// read and write permitted at both privilege levels but not execute (SPRU966C Figure 4-28).
const MPPA_RESET: u32 = 0xfef6;
const MPPA_SR: u32 = 1 << 5;
const MPPA_SW: u32 = 1 << 4;
const MPPA_UR: u32 = 1 << 2;
const MPPA_UW: u32 = 1 << 1;
/// MPFSR error bits, and FID: the PrivID of the request that faulted (SPRU966C Table 4-27).
const MPFSR_SRE: u32 = 1 << 5;
const MPFSR_SWE: u32 = 1 << 4;
const MPFSR_URE: u32 = 1 << 2;
const MPFSR_UWE: u32 = 1 << 1;
const MPFSR_FID: u32 = 0xf << 9;

/// One event queue: sixteen entry slots the debug windows read directly, plus the head, the
/// occupancy, the high-water mark and the threshold flag QSTAT reports (SPRU966C §4.2.4).
#[derive(Clone, Copy)]
struct Queue {
    slots: [u32; QUEUE_DEPTH],
    head: usize,
    length: usize,
    watermark: u32,
    exceeded: bool,
}

impl Default for Queue {
    fn default() -> Self {
        Self {
            slots: [0; QUEUE_DEPTH],
            head: 0,
            length: 0,
            watermark: 0,
            exceeded: false,
        }
    }
}

impl Queue {
    fn push(&mut self, entry: u32) -> bool {
        if self.length == QUEUE_DEPTH {
            return false;
        }
        self.slots[(self.head + self.length) % QUEUE_DEPTH] = entry;
        self.length += 1;
        self.watermark = self.watermark.max(self.length as u32);
        true
    }

    fn pop(&mut self) -> Option<u32> {
        if self.length == 0 {
            return None;
        }
        let entry = self.slots[self.head];
        self.head = (self.head + 1) % QUEUE_DEPTH;
        self.length -= 1;
        Some(entry)
    }

    fn status(&self) -> u32 {
        (u32::from(self.exceeded) << 24)
            | (self.watermark << 16)
            | ((self.length as u32) << 8)
            | self.head as u32
    }
}

/// The reset values SPRU966C Figures 4-1 and 4-2 give for this device: 256 PaRAM sets, 64 DMA
/// and 64 interrupt channels, 4 QDMA channels, 4 queues, 8 regions, memory protection present.
const PID_VALUE: u32 = 0x4001_1b00;
const CCCFG_VALUE: u32 =
    (1 << 25) | (1 << 24) | (3 << 20) | (3 << 16) | (4 << 12) | (4 << 8) | (2 << 4) | 5;

/// OPT fields (SPRU966C Table 2-3).
const OPT_ITCCHEN: u32 = 1 << 23;
const OPT_TCCHEN: u32 = 1 << 22;
const OPT_ITCINTEN: u32 = 1 << 21;
const OPT_TCINTEN: u32 = 1 << 20;
const OPT_STATIC: u32 = 1 << 3;
const OPT_SYNCDIM: u32 = 1 << 2;
const OPT_DAM: u32 = 1 << 1;
const OPT_SAM: u32 = 1 << 0;
/// A null link writes a null PaRAM set back over the channel's own set.
const NULL_LINK: u32 = 0xffff;

/// One PaRAM set, unpacked from its eight words.
struct ParamSet {
    opt: u32,
    src: u32,
    acnt: u32,
    bcnt: u32,
    dst: u32,
    src_bidx: i32,
    dst_bidx: i32,
    link: u32,
    bcntrld: u32,
    src_cidx: i32,
    dst_cidx: i32,
    ccnt: u32,
}

fn sign16(value: u32) -> i32 {
    value as u16 as i16 as i32
}

impl ParamSet {
    fn load(words: &[u32]) -> Self {
        Self {
            opt: words[0],
            src: words[1],
            acnt: words[2] & 0xffff,
            bcnt: words[2] >> 16,
            dst: words[3],
            src_bidx: sign16(words[4]),
            dst_bidx: sign16(words[4] >> 16),
            link: words[5] & 0xffff,
            bcntrld: words[5] >> 16,
            src_cidx: sign16(words[6]),
            dst_cidx: sign16(words[6] >> 16),
            ccnt: words[7] & 0xffff,
        }
    }

    fn store(&self, words: &mut [u32]) {
        words[1] = self.src;
        words[2] = (self.bcnt << 16) | self.acnt;
        words[3] = self.dst;
        words[7] = self.ccnt;
    }

    fn is_null(&self) -> bool {
        self.acnt == 0 && self.bcnt == 0 && self.ccnt == 0
    }

    fn is_dummy(&self) -> bool {
        !self.is_null() && (self.acnt == 0 || self.bcnt == 0 || self.ccnt == 0)
    }

    fn transfer_code(&self) -> usize {
        ((self.opt >> 12) & 0x3f) as usize
    }
}

pub struct Edma {
    param: Vec<u32>,
    dchmap: [u32; CHANNELS],
    dmaqnum: [u32; 8],
    qdmaqnum: u32,
    quetcmap: u32,
    quepri: u32,
    /// The request a controller has just serviced, waiting to reach its debug windows.
    serviced: Option<(usize, ServicedRequest)>,
    /// Latched hardware events, the enables that gate them, and the software-set events. All
    /// three trigger sources are independent (SPRU966C §2.4.1.3).
    event: u64,
    event_enable: u64,
    chained: u64,
    secondary: u64,
    missed: u64,
    /// Channels queued for service, whichever trigger source queued them.
    queued: u64,
    interrupt_enable: u64,
    interrupt_pending: u64,
    ccerr: u32,
    drae: [u64; 8],
    qchmap: [u32; QDMA_CHANNELS],
    /// The QDMA event, enable, secondary and missed registers. A QDMA channel is triggered by
    /// a write to the trigger word of its PaRAM set (SPRU966C §2.4.2).
    qdma_event: u32,
    qdma_enable: u32,
    qdma_secondary: u32,
    qdma_missed: u32,
    qdma_queued: u32,
    /// Active memory protection: the global page attributes, one per shadow region, the fault
    /// address and status, and the privilege of the access in flight (SPRU966C §2.10).
    mppag: u32,
    mppa: [u32; REGIONS],
    fault_address: u32,
    fault_status: u32,
    fault_pulse: bool,
    privilege: Privilege,
    /// The error interrupt only pulses when the latched error state goes from clear to set.
    error_latched: bool,
    error_pulse: bool,
    queues: [Queue; QUEUES],
    qwmthra: u32,
    qrae: [u32; REGIONS],
    /// A transfer request the transfer controller must reject, and which controller it went to.
    transfer_error: Option<(usize, u32)>,
}

impl Default for Edma {
    fn default() -> Self {
        Self {
            param: vec![0; PARAM_WORDS],
            dchmap: [0; CHANNELS],
            dmaqnum: [0; 8],
            qdmaqnum: 0,
            // Queue n maps to transfer controller n out of reset (SPRU966C Table 4-9).
            quetcmap: 0x3210,
            quepri: 0,
            serviced: None,
            event: 0,
            event_enable: 0,
            chained: 0,
            secondary: 0,
            missed: 0,
            queued: 0,
            interrupt_enable: 0,
            interrupt_pending: 0,
            ccerr: 0,
            drae: [0; 8],
            qchmap: [0; QDMA_CHANNELS],
            qdma_event: 0,
            qdma_enable: 0,
            qdma_secondary: 0,
            qdma_missed: 0,
            qdma_queued: 0,
            error_latched: false,
            error_pulse: false,
            qrae: [0; REGIONS],
            queues: [Queue::default(); QUEUES],
            qwmthra: QWMTHRA_RESET,
            transfer_error: None,
            mppag: MPPA_RESET,
            mppa: [MPPA_RESET; REGIONS],
            fault_address: 0,
            fault_status: 0,
            fault_pulse: false,
            privilege: Privilege::ID0_SUPERVISOR,
        }
    }
}

/// Split a 64-bit channel mask into the low and high halves the register pairs expose.
fn low(mask: u64) -> u32 {
    mask as u32
}

fn high(mask: u64) -> u32 {
    (mask >> 32) as u32
}

impl Edma {
    /// The identity the access in flight carries. Every access to the controller's registers
    /// presents it first, so the model can apply active memory protection and stamp a PaRAM
    /// set with its programmer's identity (SPRU966C sections 2.10.1 and 2.10.2).
    pub fn set_privilege(&mut self, privilege: Privilege) {
        self.privilege = privilege;
    }
}

impl SlavePort for Edma {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        if !self.permitted(address, false) {
            // The access is denied and reads return zero (SPRU966C §2.10.1).
            return Some(0);
        }
        if let Some(index) = param_word(address) {
            return Some(self.param[index]);
        }
        if let Some((global, region)) = shadow(address) {
            // A shadow view reads the same physical register through that region's access
            // enables (SPRU966C Table 2-8).
            let value = self.read(global, _context)?;
            return Some(value & self.region_mask(global, region));
        }
        let value = match address {
            PID => PID_VALUE,
            CCCFG => CCCFG_VALUE,
            QDMAQNUM => self.qdmaqnum,
            QUETCMAP => self.quetcmap,
            QUEPRI => self.quepri,
            QWMTHRA => self.qwmthra,
            EMR | EMCR => low(self.missed),
            EMRH | EMCRH => high(self.missed),
            CCERR | CCERRCLR => self.ccerr,
            EEVAL | IEVAL => 0,
            // CCSTAT reports which queues hold entries and whether any enabled event is
            // waiting; a transfer request never spans a tick boundary here, so TRACTV, the
            // write-status interface and the completion count are always idle (Table 4-25).
            CCSTAT => {
                let queues = self
                    .queues
                    .iter()
                    .enumerate()
                    .fold(0, |mask, (queue, entry)| {
                        mask | (u32::from(entry.length > 0) << (16 + queue))
                    });
                let events = u32::from(self.event & self.event_enable != 0);
                let qdma_events = u32::from(self.qdma_event & self.qdma_enable != 0);
                let active = u32::from(queues != 0 || events != 0 || qdma_events != 0);
                queues | (active << 4) | (qdma_events << 1) | events
            }
            ER | ECR => low(self.event),
            ERH | ECRH => high(self.event),
            // The event set registers read back the same latched events they set.
            ESR => low(self.event),
            ESRH => high(self.event),
            CER => low(self.chained),
            CERH => high(self.chained),
            QEMR | QEMCR => self.qdma_missed,
            QER => self.qdma_event,
            QEER | QEECR | QEESR => self.qdma_enable,
            QSER | QSECR => self.qdma_secondary,
            EER | EECR | EESR => low(self.event_enable),
            EERH | EECRH | EESRH => high(self.event_enable),
            SER | SECR => low(self.secondary),
            SERH | SECRH => high(self.secondary),
            IER | IECR | IESR => low(self.interrupt_enable),
            IERH | IECRH | IESRH => high(self.interrupt_enable),
            IPR | ICR => low(self.interrupt_pending),
            IPRH | ICRH => high(self.interrupt_pending),
            _ => return self.indexed_read(address),
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if !self.permitted(address, true) {
            // A denied write leaves the register alone (SPRU966C §2.10.1); the address still
            // belongs to this module, so the bus access is complete.
            return if self.owns(address) {
                PortWrite::Done
            } else {
                PortWrite::Unhandled
            };
        }
        if let Some(index) = param_word(address) {
            self.param[index] = value;
            // The set carries its programmer's privilege and requestor id from then on
            // (SPRU966C §2.10.2 and Table 2-3).
            let option = index & !7;
            self.param[option] = (self.param[option] & !OPT_STAMP) | opt_stamp(self.privilege);
            // A write to the trigger word of a PaRAM set a QCHMAP names is a QDMA event, when
            // the channel is enabled (SPRU966C §2.4.2.1).
            self.check_qdma_trigger(index);
            self.evaluate_errors();
            return PortWrite::Done;
        }
        if let Some((global, region)) = shadow(address) {
            let masked = value & self.region_mask(global, region);
            return self.write(global, masked, _context);
        }
        let mask = u64::from(value);
        match address {
            PID | CCCFG | CCSTAT => {}
            QDMAQNUM => self.qdmaqnum = value,
            QUETCMAP => self.quetcmap = value,
            QUEPRI => self.quepri = value,
            // The clear registers take a write of one per bit; the status halves are read-only.
            EMR | EMRH | ER | ERH | SER | SERH | IPR | IPRH | CER | CERH | QER | QEMR | QSER => {}
            QEECR => self.qdma_enable &= !value,
            QEESR => self.qdma_enable |= value & 0xf,
            QEER => {}
            QEMCR => self.qdma_missed &= !value,
            QSECR => {
                self.qdma_secondary &= !value;
                self.qdma_queued &= !value;
            }
            EMCR => self.missed &= !mask,
            MPFAR | MPFSR => {}
            MPFCR => {
                if value & 1 != 0 {
                    self.fault_address = 0;
                    self.fault_status = 0;
                }
            }
            MPPAG => self.mppag = value & 0xffff,
            EMCRH => self.missed &= !(mask << 32),
            CCERR => {}
            CCERRCLR => {
                self.ccerr &= !value;
                // Clearing a queue threshold error also clears that queue's watermark and its
                // threshold flag (SPRU966C Table 4-18).
                for (queue, entry) in self.queues.iter_mut().enumerate() {
                    if value & (1 << queue) != 0 {
                        entry.watermark = 0;
                        entry.exceeded = false;
                    }
                }
            }
            QWMTHRA => self.qwmthra = value & 0x1f1f_1f1f,
            // Writing the evaluate bit re-pulses the error interrupt while any error stands
            // (SPRU966C §2.9.4), and the interrupt evaluate register does the same for
            // completion interrupts, which this model reports as a level.
            EEVAL => {
                if value & 1 != 0 && self.error_state() {
                    self.error_pulse = true;
                }
            }
            IEVAL => {}
            ECR => self.event &= !mask,
            ECRH => self.event &= !(mask << 32),
            // A manual trigger queues the channel whatever the event enable says, and a second
            // trigger before the first is serviced is a missed event (SPRU966C §2.4.1.2).
            ESR => self.trigger(mask, ETYPE_MANUAL),
            ESRH => self.trigger(mask << 32, ETYPE_MANUAL),
            EER | EERH => {}
            EECR => self.event_enable &= !mask,
            EECRH => self.event_enable &= !(mask << 32),
            EESR => self.enable_events(mask),
            EESRH => self.enable_events(mask << 32),
            SECR => {
                self.secondary &= !mask;
                self.queued &= !mask;
            }
            SECRH => {
                self.secondary &= !(mask << 32);
                self.queued &= !(mask << 32);
            }
            IECR => self.interrupt_enable &= !mask,
            IECRH => self.interrupt_enable &= !(mask << 32),
            IESR => self.interrupt_enable |= mask,
            IESRH => self.interrupt_enable |= mask << 32,
            ICR => self.interrupt_pending &= !mask,
            ICRH => self.interrupt_pending &= !(mask << 32),
            _ => {
                if let Some(channel) = indexed(address, DCHMAP_BASE, CHANNELS) {
                    self.dchmap[channel] = value & 0x3fe0;
                } else if let Some(index) = indexed(address, DMAQNUM_BASE, 8) {
                    self.dmaqnum[index] = value;
                } else if let Some(index) = indexed(address, DRAE_BASE, 16) {
                    let region = index / 2;
                    if index % 2 == 0 {
                        self.drae[region] = (self.drae[region] & !0xffff_ffff) | mask;
                    } else {
                        self.drae[region] = (self.drae[region] & 0xffff_ffff) | (mask << 32);
                    }
                } else if let Some(channel) = indexed(address, QCHMAP_BASE, QDMA_CHANNELS) {
                    self.qchmap[channel] = value & 0x3ffc;
                } else if let Some(region) = indexed(address, QRAE_BASE, REGIONS) {
                    self.qrae[region] = value & 0xf;
                } else if let Some(region) = indexed(address, MPPA_BASE, REGIONS) {
                    // A page attribute register is supervisor-writable only (Table 2-13).
                    if !self.privilege.user {
                        self.mppa[region] = value & 0xffff;
                    }
                } else {
                    return PortWrite::Unhandled;
                }
            }
        }
        self.evaluate_errors();
        PortWrite::Done
    }
}

impl Edma {
    /// The register blocks indexed by channel, queue or region.
    fn indexed_read(&self, address: u32) -> Option<u32> {
        if let Some(channel) = indexed(address, DCHMAP_BASE, CHANNELS) {
            return Some(self.dchmap[channel]);
        }
        if let Some(index) = indexed(address, DMAQNUM_BASE, 8) {
            return Some(self.dmaqnum[index]);
        }
        if let Some(index) = indexed(address, DRAE_BASE, 16) {
            let region = index / 2;
            return Some(if index % 2 == 0 {
                low(self.drae[region])
            } else {
                high(self.drae[region])
            });
        }
        if address == MPFAR {
            return Some(self.fault_address);
        }
        if address == MPFSR {
            return Some(self.fault_status);
        }
        if address == MPFCR {
            return Some(0);
        }
        if address == MPPAG {
            return Some(self.mppag);
        }
        if let Some(region) = indexed(address, MPPA_BASE, REGIONS) {
            return Some(self.mppa[region]);
        }
        if let Some(channel) = indexed(address, QCHMAP_BASE, QDMA_CHANNELS) {
            return Some(self.qchmap[channel]);
        }
        if let Some(region) = indexed(address, QRAE_BASE, REGIONS) {
            return Some(self.qrae[region]);
        }
        if let Some(index) = indexed(address, QUEUE_ENTRY_BASE, QUEUES * QUEUE_DEPTH) {
            return Some(self.queues[index / QUEUE_DEPTH].slots[index % QUEUE_DEPTH]);
        }
        if let Some(queue) = indexed(address, QSTAT_BASE, QUEUES) {
            return Some(self.queues[queue].status());
        }
        None
    }
    /// Which page attribute register guards an address: the shadow region window and the PaRAM
    /// octant that belong to a region use that region's, everything else the global one
    /// (SPRU966C Table 2-14).
    fn page_attributes(&self, address: u32) -> u32 {
        if let Some((_, region)) = shadow(address) {
            return self.mppa[region];
        }
        if let Some(index) = param_word(address) {
            return self.mppa[(index * 4 / 0x400).min(REGIONS - 1)];
        }
        self.mppag
    }

    fn owns(&self, address: u32) -> bool {
        param_word(address).is_some()
            || shadow(address).is_some()
            || (BASE..BASE + 0x2000).contains(&address) && address % 4 == 0
    }

    /// Active memory protection: a request needs its PrivID allowed and the permission its
    /// privilege level asks for, and a refusal records the address, the failing permission and
    /// the PrivID for the fault registers (SPRU966C §2.10.1 and Table 4-27).
    fn permitted(&mut self, address: u32, write: bool) -> bool {
        if !self.owns(address) {
            return true;
        }
        // The fault registers and the command that clears them are always reachable, otherwise
        // a denied access could never be diagnosed.
        if matches!(address, MPFAR | MPFSR | MPFCR) {
            return true;
        }
        let attributes = self.page_attributes(address);
        let Privilege { id, user } = self.privilege;
        let permission = match (user, write) {
            (false, false) => MPPA_SR,
            (false, true) => MPPA_SW,
            (true, false) => MPPA_UR,
            (true, true) => MPPA_UW,
        };
        if attributes & self.privilege.allowed_id_bit() != 0 && attributes & permission != 0 {
            return true;
        }
        self.fault_address = address;
        let error = match (user, write) {
            (false, false) => MPFSR_SRE,
            (false, true) => MPFSR_SWE,
            (true, false) => MPFSR_URE,
            (true, true) => MPFSR_UWE,
        };
        // FID names the requestor of the fault the address describes.
        self.fault_status = (self.fault_status & !MPFSR_FID) | (u32::from(id & 0xf) << 9) | error;
        self.fault_pulse = true;
        false
    }

    /// The memory protection interrupt the megamodule sees as EDMA3CC_MPINT.
    pub fn take_fault_pulse(&mut self) -> bool {
        std::mem::take(&mut self.fault_pulse)
    }

    /// A trigger source queues each named channel in the event queue its DMAQNUM field points
    /// at. A channel already waiting for service records a missed event instead
    /// (SPRU966C §2.4.1.2).
    fn trigger(&mut self, mask: u64, etype: u32) {
        self.missed |= mask & self.queued;
        for channel in 0..CHANNELS {
            let bit = 1u64 << channel;
            if mask & bit == 0 || self.queued & bit != 0 {
                continue;
            }
            self.enqueue(self.dma_queue(channel), (etype << 6) | channel as u32);
        }
        self.queued |= mask;
        self.event |= mask;
    }

    /// A peripheral synchronisation event is latched in ER whatever EER says; it is only
    /// queued while the channel is enabled, and an event that arrives before the previous one
    /// was serviced is a missed event (SPRU966C §2.4.1.1).
    pub fn trigger_event(&mut self, channel: usize) {
        let bit = 1u64 << channel;
        self.missed |= bit & self.event;
        self.event |= bit;
        if self.event_enable & bit != 0 && self.queued & bit == 0 {
            self.queued |= bit;
            self.enqueue(self.dma_queue(channel), (ETYPE_EVENT << 6) | channel as u32);
        }
        self.evaluate_errors();
    }

    /// Enabling an event a peripheral already latched queues it there and then
    /// (SPRU966C §2.4.1.1).
    fn enable_events(&mut self, mask: u64) {
        self.event_enable |= mask;
        for channel in 0..CHANNELS {
            let bit = 1u64 << channel;
            if mask & bit == 0 || self.event & bit == 0 || self.queued & bit != 0 {
                continue;
            }
            self.queued |= bit;
            self.enqueue(self.dma_queue(channel), (ETYPE_EVENT << 6) | channel as u32);
        }
    }

    /// Put an entry in a queue, tracking the watermark and the threshold the queue exceeds
    /// (SPRU966C Tables 4-23 and 4-24).
    fn enqueue(&mut self, queue: usize, entry: u32) {
        let queue = queue.min(QUEUES - 1);
        self.queues[queue].push(entry);
        let threshold = (self.qwmthra >> (queue * 8)) & 0x1f;
        if threshold <= QUEUE_DEPTH as u32 && self.queues[queue].length as u32 >= threshold {
            self.queues[queue].exceeded = true;
            self.ccerr |= 1 << queue;
        }
        self.evaluate_errors();
    }

    /// The global completion interrupt the megamodule sees as EDMA3CC_GINT.
    pub fn interrupt_pending(&self) -> bool {
        self.interrupt_pending & self.interrupt_enable != 0
    }

    /// A shadow region's completion interrupt is gated by IER and that region's DRAE as a
    /// second level of masking (SPRU966C §2.9.1.1).
    pub fn region_interrupt_pending(&self, region: usize) -> bool {
        self.interrupt_pending & self.interrupt_enable & self.drae[region] != 0
    }

    /// The error interrupt pulses when the latched error state goes from clear to set; taking
    /// the pulse is what latches the system event (SPRU966C §2.9.4).
    pub fn take_error_pulse(&mut self) -> bool {
        std::mem::take(&mut self.error_pulse)
    }

    fn error_state(&self) -> bool {
        self.missed != 0 || self.qdma_missed != 0 || self.ccerr != 0
    }

    fn evaluate_errors(&mut self) {
        let state = self.error_state();
        if state && !self.error_latched {
            self.error_pulse = true;
        }
        self.error_latched = state;
    }

    /// A write to the trigger word a QCHMAP names is a QDMA event, provided the channel is
    /// enabled in QEER; a second one before service is a missed event (SPRU966C §2.4.2.1).
    fn check_qdma_trigger(&mut self, word: usize) {
        for channel in 0..QDMA_CHANNELS {
            let map = self.qchmap[channel];
            let entry = ((map >> 5) & 0x1ff) as usize;
            let trigger_word = ((map >> 2) & 7) as usize;
            if entry * 8 + trigger_word != word || self.qdma_enable & (1 << channel) == 0 {
                continue;
            }
            let bit = 1u32 << channel;
            self.qdma_missed |= bit & self.qdma_queued;
            if self.qdma_queued & bit == 0 {
                let queue = ((self.qdmaqnum >> (channel * 4)) & 7) as usize;
                self.enqueue(queue, (ETYPE_QDMA << 6) | channel as u32);
            }
            self.qdma_queued |= bit;
            self.qdma_event |= bit;
        }
    }

    /// Service one transfer request per tick, taking the head entry of the highest-priority
    /// non-empty event queue. QUEPRI ranks the queues, zero highest, and ties go to the lower
    /// queue number (SPRU966C sections 2.4 and 4.2.1.8).
    /// A tick only services queued requests; configured channels with no event are dormant.
    pub fn active(&self) -> bool {
        self.queues.iter().any(|queue| queue.length != 0)
    }

    pub fn tick(&mut self, memory: &mut impl TransferMemory) {
        let Some(queue) = (0..QUEUES)
            .filter(|queue| self.queues[*queue].length > 0)
            .min_by_key(|queue| ((self.quepri >> (queue * 4)) & 7, *queue))
        else {
            return;
        };
        let Some(entry) = self.queues[queue].pop() else {
            return;
        };
        let number = (entry & 0x3f) as usize;
        if (entry >> 6) & 3 == ETYPE_QDMA {
            let bit = 1u32 << number;
            self.qdma_queued &= !bit;
            self.qdma_event &= !bit;
            let base = ((self.qchmap[number] >> 5) & 0x1ff) as usize * 8;
            let controller = self.controller_for_queue(queue);
            if self.request(memory, base, controller, self.queue_priority(queue)) {
                self.qdma_missed |= bit;
                self.qdma_secondary |= bit;
            } else {
                self.qdma_secondary &= !bit;
            }
        } else {
            let bit = 1u64 << number;
            self.queued &= !bit;
            self.event &= !bit;
            self.chained &= !bit;
            let base = self.param_base(number);
            let controller = self.controller_for_queue(queue);
            if self.request(memory, base, controller, self.queue_priority(queue)) {
                // A null set moves nothing: the discarded event is reported in EMR and stays
                // in SER until software clears it (SPRU966C §2.3.3).
                self.missed |= bit;
                self.secondary |= bit;
            } else {
                self.secondary &= !bit;
            }
        }
        self.evaluate_errors();
    }

    /// Which event queue a DMA channel submits to, and which transfer controller serves that
    /// queue (SPRU966C Tables 4-6, 4-7 and 4-9).
    fn dma_queue(&self, channel: usize) -> usize {
        let register = self.dmaqnum[channel / 8];
        ((register >> ((channel % 8) * 4)) & 7) as usize
    }

    /// The priority QUEPRI gives a queue, which is what the transfer controller's windows
    /// report in the PRI field of their options registers (SPRU966C Table 4-75).
    fn queue_priority(&self, queue: usize) -> u32 {
        (self.quepri >> (queue * 4)) & 7
    }

    fn controller_for_queue(&self, queue: usize) -> usize {
        ((self.quetcmap >> (queue * 4)) & 7) as usize
    }

    /// The transfer request error the controller has to be told about, if any.
    pub fn take_transfer_error(&mut self) -> Option<(usize, u32)> {
        self.transfer_error.take()
    }

    /// The request a transfer controller has just serviced, for its debug windows.
    pub fn take_serviced_request(&mut self) -> Option<(usize, ServicedRequest)> {
        self.serviced.take()
    }

    /// One transfer request out of the PaRAM set at `base`: move the data, update the set as
    fn request(
        &mut self,
        memory: &mut impl TransferMemory,
        base: usize,
        controller: usize,
        priority: u32,
    ) -> bool {
        let mut set = ParamSet::load(&self.param[base..base + 8]);
        if set.is_null() {
            if set.opt & OPT_STATIC == 0 {
                self.link(base, set.link);
            }
            return true;
        }
        let ab_synchronised = set.opt & OPT_SYNCDIM != 0;
        let arrays = if ab_synchronised { set.bcnt } else { 1 };
        // A constant-addressing transfer must be 256-bit aligned; the transfer controller
        // rejects the request and signals a TR error when it is not, so no data moves
        // (SPRU966C sections 2.3.2.2, 2.3.2.3 and Table 4-69).
        let misaligned = (set.opt & OPT_SAM != 0 && set.src & 0x1f != 0)
            || (set.opt & OPT_DAM != 0 && set.dst & 0x1f != 0);
        if misaligned {
            // The details register keeps the completion code and the two completion enables
            // the failing request carried (SPRU966C Table 4-72).
            let detail = (u32::from(set.opt & OPT_TCCHEN != 0) << 17)
                | (u32::from(set.opt & OPT_TCINTEN != 0) << 16)
                | ((set.transfer_code() as u32) << 8);
            self.transfer_error = Some((controller, detail));
        } else if !set.is_dummy() {
            self.transfer(memory, &set, arrays);
            self.serviced = Some((controller, self.serviced_request(&set, arrays, priority)));
        }
        let final_request = if set.is_dummy() {
            true
        } else if ab_synchronised {
            set.ccnt <= 1
        } else {
            set.bcnt <= 1 && set.ccnt <= 1
        };
        if set.opt & OPT_STATIC == 0 {
            self.advance(&mut set, ab_synchronised, final_request);
            if final_request {
                self.link(base, set.link);
            } else {
                set.store(&mut self.param[base..base + 8]);
            }
        }
        let code = set.transfer_code();
        let (interrupt, chain) = if final_request {
            (set.opt & OPT_TCINTEN != 0, set.opt & OPT_TCCHEN != 0)
        } else {
            (set.opt & OPT_ITCINTEN != 0, set.opt & OPT_ITCCHEN != 0)
        };
        if interrupt {
            self.interrupt_pending |= 1 << code;
        }
        if chain {
            // Chaining does not touch the chained channel's parameters; it only queues it
            // (SPRU966C §2.8).
            self.chained |= 1 << code;
            self.trigger(1 << code, ETYPE_CHAIN);
        }
        false
    }

    /// Table 2-8: a shadow view of a channel register is filtered by that region's DRAE, the
    /// QDMA registers by its QRAE, and ICR, ICRH and IEVAL are not filtered at all.
    fn region_mask(&self, global: u32, region: usize) -> u32 {
        match global {
            ICR | ICRH | IEVAL => u32::MAX,
            QER | QEER | QEECR | QEESR | QSER | QSECR => self.qrae[region],
            ERH | ECRH | ESRH | CERH | EERH | EECRH | EESRH | SERH | SECRH | IERH | IECRH
            | IESRH | IPRH => high(self.drae[region]),
            _ => low(self.drae[region]),
        }
    }

    /// What the transfer controller's debug windows say about the request just serviced. The
    /// source and destination references are the start of the last array read and written, and
    /// this model issues one read and one write command per array, so the last write command's
    /// address is that destination reference. Constant addressing holds an address still, so
    /// its pointer never walks past the start (SPRU966C §2.3.8 and Tables 4-76 to 4-92). Every
    /// The privilege the windows report is the one the PaRAM set was stamped with when it was
    /// programmed - bit 31 of its OPT word for the level and bits 27-24 for the requestor id -
    /// because that is the identity the controller issues its reads and writes under.
    fn serviced_request(&self, set: &ParamSet, arrays: u32, priority: u32) -> ServicedRequest {
        let last = arrays.saturating_sub(1);
        let source_reference = if set.opt & OPT_SAM != 0 {
            set.src
        } else {
            offset_address(set.src, set.src_bidx, last)
        };
        let destination_reference = if set.opt & OPT_DAM != 0 {
            set.dst
        } else {
            offset_address(set.dst, set.dst_bidx, last)
        };
        ServicedRequest {
            opt: ServicedRequest::options(set.opt, priority),
            acnt: set.acnt,
            src_bidx: set.src_bidx as u32,
            dst_bidx: set.dst_bidx as u32,
            source_reference,
            source_end: if set.opt & OPT_SAM != 0 {
                set.src
            } else {
                source_reference.wrapping_add(set.acnt)
            },
            destination_reference,
            mpprxy: ((set.opt >> 31) << 8) | ((set.opt >> 24) & 0xf),
        }
    }

    /// One transfer request: `arrays` arrays of ACNT bytes each, stepping by the B indexes.
    /// Constant addressing mode holds the address still (SPRU966C §2.3.8).
    fn transfer(&self, memory: &mut impl TransferMemory, set: &ParamSet, arrays: u32) {
        let source_fixed = set.opt & OPT_SAM != 0;
        let destination_fixed = set.opt & OPT_DAM != 0;
        for array in 0..arrays {
            let source = if source_fixed {
                set.src
            } else {
                offset_address(set.src, set.src_bidx, array)
            };
            let destination = if destination_fixed {
                set.dst
            } else {
                offset_address(set.dst, set.dst_bidx, array)
            };
            let privilege = stamped_privilege(set.opt);
            let bytes = memory.read(privilege, source, set.acnt as usize);
            memory.write(privilege, destination, &bytes);
        }
    }

    /// The B, C and link updates of Table 2-5. An A-synchronised request consumes one array;
    /// an AB-synchronised request consumes a whole frame.
    fn advance(&self, set: &mut ParamSet, ab_synchronised: bool, final_request: bool) {
        if final_request {
            return;
        }
        if ab_synchronised {
            set.ccnt -= 1;
            set.src = offset_address(set.src, set.src_cidx, 1);
            set.dst = offset_address(set.dst, set.dst_cidx, 1);
            return;
        }
        if set.bcnt > 1 {
            set.bcnt -= 1;
            set.src = offset_address(set.src, set.src_bidx, 1);
            set.dst = offset_address(set.dst, set.dst_bidx, 1);
        } else {
            set.ccnt -= 1;
            set.bcnt = set.bcntrld;
            set.src = offset_address(set.src, set.src_cidx, 1);
            set.dst = offset_address(set.dst, set.dst_cidx, 1);
        }
    }

    /// A link update copies the eight words the LINK field names over the channel's own set; a
    /// null link writes a null set back instead (SPRU966C §2.3.7).
    fn link(&mut self, base: usize, link: u32) {
        if link == NULL_LINK {
            for word in 0..8 {
                self.param[base + word] = 0;
            }
            self.param[base + 5] = NULL_LINK;
            self.link_triggers_qdma(base);
            return;
        }
        // The upper two bits are ignored, so a literal address and a PaRAM-relative offset both
        // work; the offset names a 32-byte aligned set.
        let source = ((link & 0x3fff) / 4) as usize & !7;
        if source + 8 <= PARAM_WORDS {
            for word in 0..8 {
                self.param[base + word] = self.param[source + word];
            }
        }
        self.link_triggers_qdma(base);
    }

    /// A link update onto a PaRAM set a QCHMAP names counts as a write to its trigger word, so
    /// a linked list of QDMA transfers keeps itself going (SPRU966C §2.4.2.1).
    fn link_triggers_qdma(&mut self, base: usize) {
        for channel in 0..QDMA_CHANNELS {
            let map = self.qchmap[channel];
            if ((map >> 5) & 0x1ff) as usize * 8 == base {
                self.check_qdma_trigger(base + ((map >> 2) & 7) as usize);
            }
        }
    }

    fn param_base(&self, channel: usize) -> usize {
        let entry = ((self.dchmap[channel] >> 5) & 0x1ff) as usize;
        entry * 8
    }
}

/// Address stepping is a signed byte offset repeated `count` times, and wraps as programmed.
fn offset_address(base: u32, index: i32, count: u32) -> u32 {
    base.wrapping_add((index * count as i32) as u32)
}

fn param_word(address: u32) -> Option<usize> {
    if address % 4 != 0 {
        return None;
    }
    let offset = address.checked_sub(PARAM_BASE)? as usize / 4;
    (offset < PARAM_WORDS).then_some(offset)
}

fn indexed(address: u32, base: u32, count: usize) -> Option<usize> {
    if address % 4 != 0 || address < base {
        return None;
    }
    let index = ((address - base) / 4) as usize;
    (index < count).then_some(index)
}

/// A shadow region address names the same physical channel register as the global region.
fn shadow(address: u32) -> Option<(u32, usize)> {
    let offset = address.checked_sub(SHADOW_BASE)?;
    let region = (offset / SHADOW_STRIDE) as usize;
    let register = offset % SHADOW_STRIDE;
    if region >= REGIONS || register >= SHADOW_LIMIT || register % 4 != 0 {
        return None;
    }
    Some((BASE + 0x1000 + register, region))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    use std::collections::BTreeMap;

    struct Memory(BTreeMap<u32, u8>);

    impl TransferMemory for Memory {
        fn read(&mut self, _privilege: Privilege, address: u32, length: usize) -> Vec<u8> {
            self.dma_read_bytes(address, length)
        }

        fn write(&mut self, _privilege: Privilege, address: u32, bytes: &[u8]) {
            self.dma_write_bytes(address, bytes);
        }
    }

    impl Memory {
        fn dma_read_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
            (0..length)
                .map(|offset| {
                    *self
                        .0
                        .get(&address.wrapping_add(offset as u32))
                        .unwrap_or(&0)
                })
                .collect()
        }

        fn dma_write_bytes(&mut self, address: u32, bytes: &[u8]) {
            for (offset, byte) in bytes.iter().copied().enumerate() {
                self.0.insert(address.wrapping_add(offset as u32), byte);
            }
        }

        fn dma_read_u32(&mut self, address: u32) -> u32 {
            u32::from_le_bytes(self.dma_read_bytes(address, 4).try_into().unwrap())
        }
    }

    /// A memory that notes the privilege each access was made with.
    struct Witness {
        memory: Memory,
        seen: Vec<Privilege>,
    }

    impl TransferMemory for Witness {
        fn read(&mut self, privilege: Privilege, address: u32, length: usize) -> Vec<u8> {
            self.seen.push(privilege);
            self.memory.read(privilege, address, length)
        }

        fn write(&mut self, privilege: Privilege, address: u32, bytes: &[u8]) {
            self.seen.push(privilege);
            self.memory.write(privilege, address, bytes);
        }
    }

    fn program(edma: &mut Edma, set: usize, words: [u32; 8]) {
        for (index, word) in words.into_iter().enumerate() {
            edma.write(
                PARAM_BASE + (set as u32 * 32) + index as u32 * 4,
                word,
                TEST_CONTEXT,
            );
        }
    }

    #[test]
    fn a_transfer_runs_as_whoever_wrote_its_set_however_the_presented_privilege_has_changed() {
        // SPRU966C section 2.10.2 and Table 2-3: the set is stamped with the identity of the
        // master that wrote it, and that stamp - not the controller's current view of the bus -
        // is what the transfer is issued under.
        let user_core_three = Privilege { id: 3, user: true };
        let supervisor_core_one = Privilege { id: 1, user: false };
        let mut memory = Witness {
            memory: Memory(BTreeMap::new()),
            seen: Vec::new(),
        };
        let mut edma = Edma::default();
        edma.write(DCHMAP_BASE, 2 << 5, TEST_CONTEXT);
        edma.write(DCHMAP_BASE + 4, 4 << 5, TEST_CONTEXT);
        edma.set_privilege(user_core_three);
        program(
            &mut edma,
            2,
            [0, 0x1000, (1 << 16) | 4, 0x2000, 0, 0xffff, 0, 1],
        );
        edma.set_privilege(supervisor_core_one);
        program(
            &mut edma,
            4,
            [0, 0x1000, (1 << 16) | 4, 0x3000, 0, 0xffff, 0, 1],
        );
        // The next access is a different identity again, and neither transfer notices.
        edma.set_privilege(Privilege { id: 5, user: true });
        edma.write(ESR, 3, TEST_CONTEXT);
        edma.tick(&mut memory);
        edma.tick(&mut memory);
        assert_eq!(
            memory.seen,
            [
                user_core_three,
                user_core_three,
                supervisor_core_one,
                supervisor_core_one
            ]
        );
    }

    #[test]
    fn a_register_access_is_judged_by_its_privid_and_a_refusal_names_it() {
        // SPRU966C section 2.10.1, Figure 4-28 and Table 4-27: a page admits PrivIDs 0 to 5
        // by their own AID bit and every higher one by AIDX, and MPFSR's FID records whose
        // request faulted.
        const AIDX: u32 = 1 << 9;
        fn aid(id: u32) -> u32 {
            1 << (10 + id)
        }
        let mut edma = Edma::default();
        // PaRAM set 0 lies in region 0's page; only core 3 may touch it.
        edma.write(MPPA_BASE, aid(3) | MPPA_SR | MPPA_SW, TEST_CONTEXT);
        edma.set_privilege(Privilege { id: 1, user: false });
        edma.write(PARAM_BASE + 4, 0x1234, TEST_CONTEXT);
        assert_eq!(edma.read(MPFAR, TEST_CONTEXT), Some(PARAM_BASE + 4));
        assert_eq!(edma.read(MPFSR, TEST_CONTEXT), Some((1 << 9) | MPFSR_SWE));
        edma.write(MPFCR, 1, TEST_CONTEXT);
        edma.set_privilege(Privilege { id: 3, user: false });
        edma.write(PARAM_BASE + 4, 0x1234, TEST_CONTEXT);
        assert_eq!(edma.read(PARAM_BASE + 4, TEST_CONTEXT), Some(0x1234));
        assert_eq!(edma.read(MPFSR, TEST_CONTEXT), Some(0));
        // PrivID 7 is no AID of its own: refused until the page grants AIDX.
        edma.set_privilege(Privilege { id: 7, user: false });
        edma.write(PARAM_BASE + 8, 0x5678, TEST_CONTEXT);
        assert_eq!(edma.read(MPFSR, TEST_CONTEXT), Some((7 << 9) | MPFSR_SWE));
        edma.write(MPFCR, 1, TEST_CONTEXT);
        edma.write(MPPA_BASE, AIDX | MPPA_SR | MPPA_SW, TEST_CONTEXT);
        edma.write(PARAM_BASE + 8, 0x5678, TEST_CONTEXT);
        assert_eq!(edma.read(PARAM_BASE + 8, TEST_CONTEXT), Some(0x5678));
        assert_eq!(edma.read(MPFSR, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn an_a_synchronised_transfer_moves_one_array_per_request() {
        // SPRU966C Table 2-7: an A-synchronised set submits BCNT x CCNT requests of ACNT bytes,
        // and Table 2-5 steps SRC and DST by the B indexes between them.
        let mut memory = Memory(BTreeMap::new());
        for offset in 0..8u32 {
            memory.dma_write_bytes(0x1000 + offset, &[offset as u8 + 1]);
        }
        let mut edma = Edma::default();
        edma.write(DCHMAP_BASE, 3 << 5, TEST_CONTEXT);
        program(
            &mut edma,
            3,
            [
                OPT_TCINTEN | (5 << 12),
                0x1000,
                (2 << 16) | 4,
                0x2000,
                (4 << 16) | 4,
                0xffff,
                0,
                1,
            ],
        );
        edma.write(IESR, 1 << 5, TEST_CONTEXT);
        edma.write(ESR, 1, TEST_CONTEXT);
        edma.tick(&mut memory);
        // The first request moved four bytes and left the second array pending.
        assert_eq!(memory.dma_read_u32(0x2000), 0x0403_0201);
        assert_eq!(memory.dma_read_u32(0x2004), 0);
        assert!(!edma.interrupt_pending());
        assert_eq!(
            edma.read(PARAM_BASE + 3 * 32 + 8, TEST_CONTEXT),
            Some((1 << 16) | 4)
        );

        edma.write(ESR, 1, TEST_CONTEXT);
        edma.tick(&mut memory);
        assert_eq!(memory.dma_read_u32(0x2004), 0x0807_0605);
        // The final request raises the completion code and the null link empties the set.
        assert_eq!(edma.read(IPR, TEST_CONTEXT), Some(1 << 5));
        assert!(edma.interrupt_pending());
        assert_eq!(edma.read(PARAM_BASE + 3 * 32 + 8, TEST_CONTEXT), Some(0));
        assert_eq!(
            edma.read(PARAM_BASE + 3 * 32 + 20, TEST_CONTEXT),
            Some(0xffff)
        );
        edma.write(ICR, 1 << 5, TEST_CONTEXT);
        assert!(!edma.interrupt_pending());
    }

    #[test]
    fn an_ab_synchronised_transfer_moves_a_frame_per_request() {
        // SPRU966C Table 2-7: an AB-synchronised set submits CCNT requests of ACNT x BCNT
        // bytes, and only the C indexes step between them.
        let mut memory = Memory(BTreeMap::new());
        for offset in 0..16u32 {
            memory.dma_write_bytes(0x3000 + offset, &[offset as u8]);
        }
        let mut edma = Edma::default();
        edma.write(DCHMAP_BASE + 4, 1 << 5, TEST_CONTEXT);
        program(
            &mut edma,
            1,
            [
                OPT_SYNCDIM,
                0x3000,
                (2 << 16) | 2,
                0x4000,
                (2 << 16) | 4,
                0xffff,
                (8 << 16) | 8,
                2,
            ],
        );
        edma.write(ESR, 1 << 1, TEST_CONTEXT);
        edma.tick(&mut memory);
        // Two arrays of two bytes, four bytes apart at the source and two at the destination.
        assert_eq!(memory.dma_read_u32(0x4000), 0x0504_0100);
        assert_eq!(edma.read(PARAM_BASE + 32 + 28, TEST_CONTEXT), Some(1));
        assert_eq!(edma.read(PARAM_BASE + 32 + 4, TEST_CONTEXT), Some(0x3008));
        assert_eq!(edma.read(PARAM_BASE + 32 + 12, TEST_CONTEXT), Some(0x4008));
    }

    #[test]
    fn a_null_set_reports_a_missed_event() {
        // SPRU966C §2.3.3: a set whose three counts are zero moves nothing, sets the event
        // missed bit and leaves the channel in the secondary event register.
        let mut memory = Memory(BTreeMap::new());
        let mut edma = Edma::default();
        edma.write(DCHMAP_BASE + 4 * 7, 2 << 5, TEST_CONTEXT);
        edma.write(ESR, 1 << 7, TEST_CONTEXT);
        edma.tick(&mut memory);
        assert_eq!(edma.read(EMR, TEST_CONTEXT), Some(1 << 7));
        assert_eq!(edma.read(SER, TEST_CONTEXT), Some(1 << 7));
        edma.write(EMCR, 1 << 7, TEST_CONTEXT);
        edma.write(SECR, 1 << 7, TEST_CONTEXT);
        assert_eq!(edma.read(EMR, TEST_CONTEXT), Some(0));
        assert_eq!(edma.read(SER, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn null_and_dummy_sets_link_without_losing_their_event_status() {
        // SPRU966C §2.3.7 p.37: "A link update occurs for null and dummy transfers"
        // when STATIC is clear, including a null LINK. The copy triggers QDMA; §2.3.3
        // and Table 2-4 still require missed/secondary bits only for the null set.
        for counts in [(0, 0), (0, 3), (3 << 16, 2)] {
            for static_bit in [0, OPT_STATIC] {
                for link in [32, NULL_LINK] {
                    for qdma in [false, true] {
                        let mut memory = Memory(BTreeMap::new());
                        let mut edma = Edma::default();
                        let original = [static_bit, 0x1000, counts.0, 0x2000, 4, link, 8, counts.1];
                        let replacement = [OPT_STATIC, 0x3000, 0x10004, 0x4000, 4, NULL_LINK, 8, 1];
                        program(&mut edma, 0, original);
                        program(&mut edma, 1, replacement);
                        if qdma {
                            for channel in 1..QDMA_CHANNELS {
                                edma.write(
                                    QCHMAP_BASE + channel as u32 * 4,
                                    (channel as u32 + 1) << 5,
                                    TEST_CONTEXT,
                                );
                            }
                            edma.write(QEESR, 1, TEST_CONTEXT);
                            // QCHMAP's reset trigger word is OPT in set 0.
                            edma.write(PARAM_BASE, static_bit, TEST_CONTEXT);
                        } else {
                            edma.write(ESR, 1, TEST_CONTEXT);
                        }
                        edma.tick(&mut memory);
                        let expected = if static_bit != 0 {
                            original
                        } else if link == NULL_LINK {
                            [0, 0, 0, 0, 0, NULL_LINK, 0, 0]
                        } else {
                            replacement
                        };
                        for (word, mut expected) in expected.into_iter().enumerate() {
                            // CPU programming stamps supervisor PRIV into OPT; a null link
                            // instead writes literal zeros (SPRU966C §2.10.2 and §2.3.7).
                            if word == 0 && (static_bit != 0 || link != NULL_LINK) {
                                expected |= 1 << 31;
                            }
                            assert_eq!(edma.read(PARAM_BASE + word as u32 * 4, TEST_CONTEXT), Some(expected),
                                "counts {counts:?}, STATIC {static_bit}, LINK {link}, QDMA {qdma}, word {word}");
                        }
                        let null = u32::from(counts == (0, 0));
                        let (missed, secondary) = if qdma { (QEMR, QSER) } else { (EMR, SER) };
                        assert_eq!(
                            edma.read(missed, TEST_CONTEXT),
                            Some(null),
                            "counts {counts:?}, STATIC {static_bit}, LINK {link}, QDMA {qdma}"
                        );
                        assert_eq!(edma.read(secondary, TEST_CONTEXT), Some(null));
                        if qdma {
                            assert_eq!(
                                edma.read(QER, TEST_CONTEXT),
                                Some(u32::from(static_bit == 0))
                            );
                        }
                        assert!(memory.0.is_empty(), "zero-byte sets never move data");
                    }
                }
            }
        }
    }

    #[test]
    fn events_wait_in_the_queue_their_channel_maps_to() {
        // SPRU966C sections 4.2.1.5, 4.2.4.1 and 4.2.4.2: a channel's DMAQNUM field picks the
        // queue, the entry records the event type and channel number, QSTAT reports the head
        // and the occupancy, and QUEPRI decides which queue is served first.
        let mut memory = Memory(BTreeMap::new());
        let mut edma = Edma::default();
        // Channel 1 to queue 0, channel 2 to queue 1.
        edma.write(DMAQNUM_BASE, 1 << 8, TEST_CONTEXT);
        // Queue 1 outranks queue 0.
        edma.write(QUEPRI, 1, TEST_CONTEXT);
        edma.write(ESR, (1 << 1) | (1 << 2), TEST_CONTEXT);
        assert_eq!(
            edma.read(QSTAT_BASE, TEST_CONTEXT),
            Some((1 << 16) | (1 << 8))
        );
        assert_eq!(
            edma.read(QSTAT_BASE + 4, TEST_CONTEXT),
            Some((1 << 16) | (1 << 8))
        );
        // Manual triggers on channels 1 and 2, one entry in each queue.
        assert_eq!(
            edma.read(QUEUE_ENTRY_BASE, TEST_CONTEXT),
            Some((ETYPE_MANUAL << 6) | 1)
        );
        assert_eq!(
            edma.read(QUEUE_ENTRY_BASE + 4 * QUEUE_DEPTH as u32, TEST_CONTEXT),
            Some((ETYPE_MANUAL << 6) | 2)
        );
        assert_eq!(
            edma.read(CCSTAT, TEST_CONTEXT),
            Some((1 << 17) | (1 << 16) | (1 << 4))
        );
        // Queue 1 is served first, and its head advances.
        edma.tick(&mut memory);
        assert_eq!(edma.read(QSTAT_BASE + 4, TEST_CONTEXT), Some((1 << 16) | 1));
        assert_eq!(
            edma.read(QSTAT_BASE, TEST_CONTEXT),
            Some((1 << 16) | (1 << 8))
        );
        edma.tick(&mut memory);
        assert_eq!(edma.read(QSTAT_BASE, TEST_CONTEXT), Some((1 << 16) | 1));
        assert_eq!(edma.read(CCSTAT, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn a_queue_past_its_watermark_threshold_reports_an_error() {
        // SPRU966C Table 4-24: the queue threshold error stands once the occupancy reaches the
        // programmed watermark, and clearing it through CCERRCLR also clears the watermark.
        let mut memory = Memory(BTreeMap::new());
        let mut edma = Edma::default();
        edma.write(
            QWMTHRA,
            (0x10 << 24) | (0x10 << 16) | (0x10 << 8) | 2,
            TEST_CONTEXT,
        );
        edma.write(ESR, 1 << 1, TEST_CONTEXT);
        assert_eq!(edma.read(CCERR, TEST_CONTEXT), Some(0));
        edma.write(ESR, 1 << 2, TEST_CONTEXT);
        assert_eq!(edma.read(CCERR, TEST_CONTEXT), Some(1));
        assert_eq!(
            edma.read(QSTAT_BASE, TEST_CONTEXT),
            Some((1 << 24) | (2 << 16) | (2 << 8))
        );
        assert!(edma.take_error_pulse());
        edma.tick(&mut memory);
        edma.tick(&mut memory);
        edma.write(CCERRCLR, 1, TEST_CONTEXT);
        assert_eq!(edma.read(CCERR, TEST_CONTEXT), Some(0));
        assert_eq!(edma.read(QSTAT_BASE, TEST_CONTEXT), Some(2));
    }
}
