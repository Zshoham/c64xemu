//! The C6455's four EDMA3 transfer controllers.
//!
//! SPRU966C section 4.3: each transfer controller identifies itself, reports the FIFO geometry
//! it was built with, exposes its channel status and its error registers, and pulses its own
//! error interrupt. The source-active and destination-FIFO register sets are read-only debug
//! windows: SPRU966C §2.12.4 says they are there "to derive a brief history of TRs serviced
//! through the transfer controller", and that is what they report here. This model completes a
//! transfer request within the tick that submits it, so the activity fields - PROGBUSY,
//! SRCACTV, WSACTV and DSTACTV - always read idle, which for SRCACTV is documented as "source
//! active register set contains a previously processed transfer request", and the windows hold
//! the parameters of the requests that have been through. The destination FIFO is a circular
//! buffer of DSTREGDEPTH entries whose head DFSTRTPTR names, so the last two requests on TC0
//! and the last four elsewhere stay readable.
//!
//! One granularity note: the counts and pointers the manual defines in terms of read and write
//! *commands* are reported at this model's command granularity, which is one read and one write
//! per array.
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const PID_VALUE: u32 = 0x0333_4425;
const APERTURE: u32 = 0x8000;

const PID: u32 = 0x000;
const TCCFG: u32 = 0x004;
const TCSTAT: u32 = 0x100;
const ERRSTAT: u32 = 0x120;
const ERREN: u32 = 0x124;
const ERRCLR: u32 = 0x128;
const ERRDET: u32 = 0x12c;
const ERRCMD: u32 = 0x130;
const RDRATE: u32 = 0x140;
/// The source-active set (SPRU966C §4.3.6.1 to §4.3.6.9).
const SAOPT: u32 = 0x240;
const SASRC: u32 = 0x244;
const SACNT: u32 = 0x248;
const SADST: u32 = 0x24c;
const SABIDX: u32 = 0x250;
const SAMPPRXY: u32 = 0x254;
const SACNTRLD: u32 = 0x258;
const SASRCBREF: u32 = 0x25c;
const SADSTBREF: u32 = 0x260;
/// The destination FIFO's shared registers, and the per-entry set at 0x300 + 0x40 * n
/// (SPRU966C §4.3.6.10 to §4.3.6.18).
const DFCNTRLD: u32 = 0x280;
const DFSRCBREF: u32 = 0x284;
const DFDSTBREF: u32 = 0x288;
const DF_BASE: u32 = 0x300;
const DF_STRIDE: u32 = 0x40;
const DF_OPT: u32 = 0x00;
const DF_CNT: u32 = 0x08;
const DF_DST: u32 = 0x0c;
const DF_BIDX: u32 = 0x10;
const DF_MPPRXY: u32 = 0x14;

/// ERRSTAT and ERREN bits (SPRU966C Tables 4-69 and 4-70).
const BUSERR: u32 = 1 << 0;
const TRERR: u32 = 1 << 2;
const MMRAERR: u32 = 1 << 3;
const ERROR_MASK: u32 = BUSERR | TRERR | MMRAERR;
/// TCSTAT bit 8 always reads one, and DFSTRTPTR sits in bits 12-11 (SPRU966C Table 4-68).
const TCSTAT_IDLE: u32 = 1 << 8;
const TCSTAT_DFSTRTPTR_SHIFT: u32 = 11;
/// The options fields the source-active and destination-FIFO windows report, which are the
/// transfer's own bits plus the priority the channel controller's QUEPRI gave its queue
/// (SPRU966C Tables 4-75 and 4-84).
const OPT_REPORTED: u32 = (1 << 22) | (1 << 20) | (0x3f << 12) | (7 << 8) | 3;
const OPT_PRI_SHIFT: u32 = 4;

/// One transfer request as the debug windows describe it once it has been serviced.
#[derive(Clone, Copy, Default)]
pub struct ServicedRequest {
    /// The transfer's options, already reduced to the fields SAOPT and DFOPT report.
    pub opt: u32,
    /// The A count the reload registers report, and the B indexes the index registers do.
    pub acnt: u32,
    pub src_bidx: u32,
    pub dst_bidx: u32,
    /// Where the last array read began, and the address one past its last byte.
    pub source_reference: u32,
    pub source_end: u32,
    /// Where the last array written began, which is the address of the last write command this
    /// model issued.
    pub destination_reference: u32,
    /// The privilege the request was submitted with.
    pub mpprxy: u32,
}

pub struct TransferController {
    base: u32,
    config: u32,
    errstat: u32,
    erren: u32,
    errdet: u32,
    rdrate: u32,
    error_latched: bool,
    error_pulse: bool,
    /// How many entries the destination FIFO was built with, its head, and the requests the two
    /// windows hold.
    depth: u32,
    df_start: u32,
    source_active: ServicedRequest,
    destination_fifo: [ServicedRequest; 4],
}

impl ServicedRequest {
    /// The options both windows report: the request's own chaining, interrupt, code, FIFO width
    /// and addressing-mode bits, plus the priority QUEPRI gave the queue it came through.
    pub fn options(opt: u32, priority: u32) -> u32 {
        (opt & OPT_REPORTED) | ((priority & 7) << OPT_PRI_SHIFT)
    }
}

impl TransferController {
    /// SPRU966C section 4.3.6 and Table 4-67: TC0 has a two-entry destination FIFO and a
    /// 128-byte data FIFO, TC1 and TC2 four entries and 128 bytes, TC3 four entries and 256
    /// bytes; all four are 128 bits wide.
    pub fn new(index: usize, base: u32) -> Self {
        let depth = if index == 0 { 1 } else { 2 };
        let fifo = if index == 3 { 3 } else { 2 };
        Self {
            base,
            config: (depth << 8) | (2 << 4) | fifo,
            errstat: 0,
            erren: 0,
            errdet: 0,
            rdrate: 0,
            error_latched: false,
            error_pulse: false,
            depth: if index == 0 { 2 } else { 4 },
            df_start: 0,
            source_active: ServicedRequest::default(),
            destination_fifo: [ServicedRequest::default(); 4],
        }
    }
}

impl SlavePort for TransferController {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let offset = self.offset(address)?;
        let value = match offset {
            PID => PID_VALUE,
            TCCFG => self.config,
            TCSTAT => TCSTAT_IDLE | (self.df_start << TCSTAT_DFSTRTPTR_SHIFT),
            ERRSTAT => self.errstat,
            ERREN => self.erren,
            ERRCLR | ERRCMD => 0,
            ERRDET => self.errdet,
            RDRATE => self.rdrate,
            // The source-active window: the request the source controller last read through.
            // SACNT reads zero because a completed request has nothing left to read, SADST and
            // SADSTBREF are documented as always zero, and SASRC has walked to the end of the
            // last array (SPRU966C Tables 4-76 to 4-83).
            SAOPT => self.source_active.opt,
            SASRC => self.source_active.source_end,
            SACNT => 0,
            SADST | SADSTBREF | DFSRCBREF => 0,
            SABIDX => self.source_active.src_bidx & 0xffff,
            SAMPPRXY => self.source_active.mpprxy,
            SACNTRLD => self.source_active.acnt & 0xffff,
            SASRCBREF => self.source_active.source_reference,
            // The destination FIFO's shared registers describe the array most recently
            // written, which is the request that went through last rather than the entry the
            // head names - the head is where the next one lands.
            DFCNTRLD => self.latest().acnt & 0xffff,
            DFDSTBREF => self.latest().destination_reference,
            _ if entry_register(offset) => {
                let (entry, field) = ((offset - DF_BASE) / DF_STRIDE, offset & 0x3f);
                let request = self.destination_fifo[entry as usize];
                match field {
                    DF_OPT => request.opt,
                    DF_CNT => 0,
                    DF_DST => request.destination_reference,
                    DF_BIDX => (request.dst_bidx & 0xffff) << 16,
                    DF_MPPRXY => request.mpprxy,
                    // DFSRC carries no source address for the destination side.
                    _ => 0,
                }
            }
            _ if debug_register(offset) => 0,
            _ => {
                // Reading an address the configuration map does not define is an MMR address
                // error (SPRU966C Table 4-69).
                self.raise(MMRAERR);
                0
            }
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        let Some(offset) = self.offset(address) else {
            return PortWrite::Unhandled;
        };
        match offset {
            PID | TCCFG | TCSTAT | ERRSTAT | ERRDET => {}
            ERREN => {
                self.erren = value & ERROR_MASK;
                self.evaluate();
            }
            ERRCLR => {
                self.errstat &= !(value & (MMRAERR | TRERR | BUSERR));
                // Clearing a bus error clears the details it recorded; the other two do not.
                if value & BUSERR != 0 {
                    self.errdet = 0;
                }
                self.evaluate();
            }
            ERRCMD => {
                if value & 1 != 0 && self.errstat != 0 {
                    self.error_pulse = true;
                }
            }
            RDRATE => self.rdrate = value & 7,
            _ if debug_register(offset) => {}
            _ => self.raise(MMRAERR),
        }
        PortWrite::Done
    }
}

impl TransferController {
    /// The channel controller submitted a transfer request this controller cannot perform: the
    /// transfer request error stands and the options that came with it are recorded
    /// (SPRU966C Tables 4-69 and 4-72).
    pub fn report_transfer_error(&mut self, detail: u32) {
        self.errdet = detail;
        self.raise(TRERR);
    }

    /// The controller serviced a transfer request: both windows keep it, and the destination
    /// FIFO's head advances, so the last DSTREGDEPTH requests stay readable (SPRU966C §2.12.4).
    pub fn record_request(&mut self, request: ServicedRequest) {
        self.source_active = request;
        self.destination_fifo[self.df_start as usize] = request;
        self.df_start = (self.df_start + 1) % self.depth;
    }

    fn latest(&self) -> ServicedRequest {
        self.destination_fifo[((self.df_start + self.depth - 1) % self.depth) as usize]
    }

    pub fn take_error_pulse(&mut self) -> bool {
        std::mem::take(&mut self.error_pulse)
    }

    fn raise(&mut self, flag: u32) {
        self.errstat |= flag;
        self.evaluate();
    }

    /// The error interrupt pulses when an enabled error appears (SPRU966C §4.3.4.2).
    fn evaluate(&mut self) {
        let state = self.errstat & self.erren != 0;
        if state && !self.error_latched {
            self.error_pulse = true;
        }
        self.error_latched = state;
    }

    fn offset(&self, address: u32) -> Option<u32> {
        if address % 4 != 0 {
            return None;
        }
        let offset = address.checked_sub(self.base)?;
        (offset < APERTURE).then_some(offset)
    }
}

/// The read-only source-active and destination-FIFO windows (SPRU966C Table 4-65).
/// The per-entry destination FIFO registers, whichever entry they belong to.
fn entry_register(offset: u32) -> bool {
    (DF_BASE..=DF_BASE + 3 * DF_STRIDE + DF_MPPRXY).contains(&offset)
        && (offset & 0x3f) <= DF_MPPRXY
}

fn debug_register(offset: u32) -> bool {
    matches!(offset, 0x240..=0x260 | 0x280..=0x288)
        || (0x300..=0x3d4).contains(&offset) && (offset & 0x3f) <= 0x14
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    #[test]
    fn each_controller_reports_the_fifo_it_was_built_with() {
        // SPRU966C section 4.3.6: TC0's destination FIFO holds two entries, the others four,
        // and TC3 has the 256-byte data FIFO.
        let mut tc0 = TransferController::new(0, 0);
        let mut tc1 = TransferController::new(1, 0);
        let mut tc3 = TransferController::new(3, 0);
        assert_eq!(tc0.read(PID, TEST_CONTEXT), Some(0x0333_4425));
        assert_eq!(tc0.read(TCCFG, TEST_CONTEXT), Some(0x0122));
        assert_eq!(tc1.read(TCCFG, TEST_CONTEXT), Some(0x0222));
        assert_eq!(tc3.read(TCCFG, TEST_CONTEXT), Some(0x0223));
        assert_eq!(tc0.read(TCSTAT, TEST_CONTEXT), Some(1 << 8));
    }

    #[test]
    fn an_undefined_configuration_address_is_an_mmr_error() {
        // SPRU966C Table 4-69: a read or write to an address the map does not define sets
        // MMRAERR, which only pulses the error line once it is enabled.
        let mut tc = TransferController::new(0, 0);
        tc.write(ERREN, MMRAERR, TEST_CONTEXT);
        assert!(!tc.take_error_pulse());
        assert_eq!(tc.read(0x200, TEST_CONTEXT), Some(0));
        assert_eq!(tc.read(ERRSTAT, TEST_CONTEXT), Some(MMRAERR));
        assert!(tc.take_error_pulse());
        // A second error while the first stands does not pulse again, but the command does.
        tc.write(0x204, 1, TEST_CONTEXT);
        assert!(!tc.take_error_pulse());
        tc.write(ERRCMD, 1, TEST_CONTEXT);
        assert!(tc.take_error_pulse());
        tc.write(ERRCLR, MMRAERR, TEST_CONTEXT);
        assert_eq!(tc.read(ERRSTAT, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn a_transfer_request_error_records_its_options() {
        // SPRU966C Table 4-72: the details register keeps the completion code and the enables
        // that came with the failing request, and only a bus error clears it.
        let mut tc = TransferController::new(1, 0);
        tc.write(ERREN, ERROR_MASK, TEST_CONTEXT);
        tc.report_transfer_error((1 << 16) | (9 << 8));
        assert_eq!(tc.read(ERRSTAT, TEST_CONTEXT), Some(TRERR));
        assert_eq!(tc.read(ERRDET, TEST_CONTEXT), Some((1 << 16) | (9 << 8)));
        assert!(tc.take_error_pulse());
        tc.write(ERRCLR, TRERR, TEST_CONTEXT);
        assert_eq!(tc.read(ERRSTAT, TEST_CONTEXT), Some(0));
        assert_eq!(tc.read(ERRDET, TEST_CONTEXT), Some((1 << 16) | (9 << 8)));
        tc.write(ERRCLR, BUSERR, TEST_CONTEXT);
        assert_eq!(tc.read(ERRDET, TEST_CONTEXT), Some(0));
    }
}
