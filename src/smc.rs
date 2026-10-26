//! The TMS320C6472 shared-memory controller (SMC): its register file, the shared-L2 (SL2) request
//! path that drives it, and the six per-core SMC profilers that sit on the UMAP1 interface between
//! each megamodule and the SMC.
//!
//! SPRUEG5D is the SMC user's guide; it deliberately does not give the base addresses of either
//! register block, so both come from SPRS612G Table 2-2: the SMC control registers occupy 32K at
//! 0x02AA8000, and SMCP0-5 occupy 16K each at 0x02C20000 + 0x4000 * core.
//!
//! The fabric puts this block on the SL2 access path (`Fabric::read32` and its siblings, the
//! master transfer stage and the three linked-atomic instructions), so what the SMC documents runs
//! on real accesses: the atomic-access monitors answer LL, SL and CMTL (`load_link`,
//! `store_link`, `commit_link`), an access wakes the page it lands in and is timed and counted by
//! the requesting megamodule's profiler (`request`, `SharedMemoryProfiler::note_access`), and a
//! User-mode write to a supervisor-only register is refused and latched as a protection fault
//! (`refuse_user_write`). Every address this module takes is an offset into the shared L2 RAM -
//! "ADDR IN SMC" of SPRUEG5D Table 11 - so the local window and every core's global alias of the
//! same word are one address to the monitor.
//!
//! The storage itself is the backing store's. These parts of SPRUEG5D sections 2.3 to 2.5 are not
//! modelled: the prefetch buffers and their generators, the per-bank arbitration, the write token
//! controller, and every wait state beyond the fixed one `request` reports. Nothing prefetches, so
//! SL2PFCPEN and SL2PFCFLUSH hold values that change nothing observable (a flush has no buffer to
//! discard) and SL2PFCCNT never advances.
//!
//! Neither type has any time-dependent state of its own, so neither has a `tick`. Everything
//! derived - the merged page power state, the counter saturation status - is recomputed at the
//! write or the event that can change it.
//!
//! Two system events come out of this block (SPRS612G section 7.6 Table 7-10):
//!
//! * 63 SMCMPINT, the memory protection interrupt. SPRUEG5D section 6 says one common exception
//!   goes to all six megamodules, so a single level, `SharedMemory::fault_pending`, drives event
//!   63 on every core.
//! * 64 SMCPEVT, which Table 7-10 names "SMC Profiler Consolidated Event". It is the output of the
//!   profiler's 8-to-1 statistics event combiner (SPRUEG5D section 7.3.6), so it is per core and
//!   lives on `SharedMemoryProfiler`, not on the shared `SharedMemory` block.

/// SMC control register base, SPRS612G Table 2-2 ("Shared Memory Controller", 32K at 0x02AA8000).
/// SPRUEG5D Table 5 gives only the offsets. Only the exact register addresses below answer; the
/// rest of the 32K window is not this block's.
pub(crate) const BASE: u32 = 0x02aa_8000;

use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const SL2PFCPEN: u32 = BASE;
const SL2PFCFLUSH: u32 = BASE + 0x004;
const SL2MPFSR: u32 = BASE + 0x008;
const SL2MPFAR: u32 = BASE + 0x00c;
const SL2PDSTAT: u32 = BASE + 0x010;
const SL2AMLSTAT0: u32 = BASE + 0x100;
const SL2AMLDATA0: u32 = BASE + 0x120;
const SL2PDSLEEP0: u32 = BASE + 0x200;
const SL2PDWAKE0: u32 = BASE + 0x220;

/// Six megamodules are connected to the SMC (SPRUEG5D Figure 2).
pub(crate) const CORES: usize = 6;
/// Four memory banks, each 256 bits wide, each with its own atomic-access monitor (SPRUEG5D
/// Figure 2 and section 3).
pub(crate) const BANKS: usize = 4;
/// The SMC divides SL2 into four physical pages, of which the C6472 populates two with RAM; the
/// other two are ROM and need no power-down support (SPRUEG5D section 4.2, Figure 5). SPRS612G
/// section 7.4 names the two power domains "SMC Pg0" and "SMC Pg1".
pub(crate) const PAGES: usize = 2;

/// How SL2 RAM splits into the two power-down pages: page 0 is the first 64-bit x 4K x 16 = 512 KB
/// and page 1 the next 64-bit x 2K x 16 = 256 KB (SPRUEG5D section 4.2, Figure 5).
///
/// UNCERTAIN: the guide gives the two sizes and calls them the "first" and "second" physical
/// page, but never says which addresses either one covers. This module lays them out in address
/// order, the one reading that needs no invented interleave.
const PAGE0_BYTES: u32 = 512 * 1024;

/// SL2PDSTAT/SL2PDSLEEP/SL2PDWAKE carry page 0 in bit 2 (P2) and page 1 in bit 3 (P3).
///
/// UNCERTAIN: SPRUEG5D names the fields P2 and P3 throughout, and Tables 10 and 13 describe them
/// as "Physical Page 0" and "Physical Page 1" while Table 14 describes the same two bits of
/// SL2PDWAKE as "Physical Page 2" and "Physical Page 3". The bit positions are unambiguous; only
/// the field naming disagrees. This module takes the Table 10/13 reading - the two bits are the
/// two populated RAM pages - and indexes them as page 0 and page 1.
const PAGE_MASK: u32 = (1 << 2) | (1 << 3);

/// SL2MPFSR.CPU_ID resets to 111b (SPRUEG5D Figure 8), which is not one of the six megamodule ids,
/// so it doubles as the "no fault recorded" marker.
const NO_FAULT_CPU_ID: u32 = 7;

/// SL2AMLSTAT holds the monitored address in bits 23-5 (SPRUEG5D Table 11), i.e. the 256-bit
/// granule the monitor is watching.
///
/// Table 11 calls the field "ADDR IN SMC", and this module takes that literally: the address is
/// the offset into the shared L2 RAM the fabric resolved, so the local window and every global
/// alias of one word are the same address to the monitor. What is compared is the word, not the
/// granule: CMTL writes LinkData "into LinkAdr address" (section 3.1.3), which a granule address
/// cannot name, so an SL to a different word of the monitored granule is an address mismatch.
/// That is also what the version-tracking reservation the other memories use answers.
const AML_ADDR_MASK: u32 = 0x00ff_ffe0;

/// The C6472 shared-memory controller register file (SPRUEG5D section 5).
pub(crate) struct SharedMemory {
    /// SL2PFCPEN: one prefetchable-page enable bit per 1/32 of SL2, reset 0 - all of SL2 is
    /// non-prefetchable (SPRUEG5D section 5.1).
    prefetch_enable: u32,
    /// Set by a write of 1 to SL2PFCFLUSH.FLUSH, consumed by `take_prefetch_flush`.
    flush: bool,
    /// SMCMPINT level: a fault is recorded and not yet cleared through SL2MPFSR.CLEAR.
    fault: bool,
    /// SL2MPFSR.CPU_ID, reset 111b.
    fault_cpu_id: u32,
    /// SL2MPFSR.MODE: 0 = secure, 1 = nonsecure (SPRUEG5D Table 8).
    fault_nonsecure: bool,
    /// SL2MPFAR, reset 0 (SPRUEG5D section 5.4).
    fault_address: u32,
    /// One atomic-access monitor per bank (SPRUEG5D section 3).
    monitor: [Monitor; BANKS],
    /// SL2PDSLEEP0-5, masked to P3/P2: each core's permission to let a page sleep.
    sleep_permission: [u32; CORES],
    /// SL2PDWAKE0-5, masked to P3/P2: each core's request to keep a page awake.
    wake_request: [u32; CORES],
    /// Tracked sleep state behind SL2PDSTAT, reset asleep (see `Default`).
    asleep: [bool; PAGES],
}

/// One atomic-access monitor, i.e. the state SL2AMLSTATn and SL2AMLDATAn report (SPRUEG5D
/// sections 3.1, 5.6 and 5.7).
#[derive(Clone, Copy, Default)]
struct Monitor {
    /// LinkAdr: the monitored word, as an offset into SL2 RAM.
    address: u32,
    /// CPU_ID of the megamodule that took the link.
    cpu_id: u32,
    /// LinkV.
    link_valid: bool,
    /// LinkdtV, which SL2AMLSTAT calls LINKTV.
    data_valid: bool,
    /// LinkData, reported by SL2AMLDATAn.
    data: u32,
}

impl Default for SharedMemory {
    fn default() -> Self {
        Self {
            prefetch_enable: 0,
            flush: false,
            fault: false,
            fault_cpu_id: NO_FAULT_CPU_ID,
            fault_nonsecure: false,
            fault_address: 0,
            monitor: [Monitor::default(); BANKS],
            sleep_permission: [0; CORES],
            wake_request: [0; CORES],
            // SL2PDSTAT resets to 0, which SPRUEG5D section 5.5 states means both pages are in
            // sleep mode "(default)".
            //
            // UNCERTAIN: that documented default contradicts the merged permission rule of
            // section 4.1, because at reset no core has granted sleep permission and section 4.1
            // says the SMC keeps a page awake whenever any core wants it awake. This module keeps
            // the documented reset value and only re-evaluates the merged state when a
            // SL2PDSLEEP/SL2PDWAKE write can have changed it, so SL2PDSTAT reads 0 out of reset
            // and follows `page_may_sleep` from the first such write onwards.
            asleep: [true; PAGES],
        }
    }
}

impl SlavePort for SharedMemory {
    /// None when the address is not one of this block's registers.
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address {
            SL2PFCPEN => self.prefetch_enable,
            // "A read of the SL2PFCFLUSH register should return 0s" (SPRUEG5D section 5.2): the
            // write of 1 triggers the flush logic and does not latch.
            SL2PFCFLUSH => 0,
            // CLEAR is write-only, so it reads back 0 (SPRUEG5D Figure 8).
            SL2MPFSR => (self.fault_cpu_id << 2) | (u32::from(self.fault_nonsecure) << 1),
            SL2MPFAR => self.fault_address,
            SL2PDSTAT => self.page_status(),
            SL2AMLSTAT0..=SL2AMLSTAT3 if is_word(address) => {
                self.monitor[((address - SL2AMLSTAT0) / 4) as usize].status()
            }
            SL2AMLDATA0..=SL2AMLDATA3 if is_word(address) => {
                self.monitor[((address - SL2AMLDATA0) / 4) as usize].data
            }
            // "C64x+ megamodule(s) can write to SL2PDSLEEP0-5 registers only in secure supervisor
            // mode or supervisor mode, and a read returns 0" - likewise SL2PDWAKE0-5 (SPRUEG5D
            // sections 5.8 and 5.9).
            //
            // UNCERTAIN: Figures 13 and 14 mark P3 and P2 of both registers R/W-0, which would
            // imply the written value reads back. This module follows the prose, which states the
            // read behaviour explicitly for both registers; the written permissions are still
            // observable through SL2PDSTAT and `page_may_sleep`.
            SL2PDSLEEP0..=SL2PDSLEEP5 if is_word(address) => 0,
            SL2PDWAKE0..=SL2PDWAKE5 if is_word(address) => 0,
            _ => return None,
        };
        Some(value)
    }

    /// `PortWrite::Unhandled` when the address is not one of this block's registers.
    ///
    /// The supervisor-mode check of SPRUEG5D sections 5.1, 5.2, 5.8 and 5.9 is not made here,
    /// because the mode of the access is not visible at this interface: the CPU path applies it
    /// before the write is made, with `refuse_user_write`.
    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            SL2PFCPEN => self.prefetch_enable = value,
            SL2PFCFLUSH => {
                if value & 1 != 0 {
                    self.flush = true;
                }
            }
            // SL2MPFSR.CLEAR is write-one-to-clear over the fault status and the fault address
            // (SPRUEG5D section 5.3); the other fields are read-only.
            SL2MPFSR => {
                if value & 1 != 0 {
                    self.fault = false;
                    self.fault_cpu_id = NO_FAULT_CPU_ID;
                    self.fault_nonsecure = false;
                    self.fault_address = 0;
                }
            }
            // Read-only registers: the address is ours, the write does nothing.
            SL2MPFAR | SL2PDSTAT => {}
            SL2AMLSTAT0..=SL2AMLSTAT3 if is_word(address) => {}
            SL2AMLDATA0..=SL2AMLDATA3 if is_word(address) => {}
            SL2PDSLEEP0..=SL2PDSLEEP5 if is_word(address) => {
                self.sleep_permission[((address - SL2PDSLEEP0) / 4) as usize] = value & PAGE_MASK;
                self.merge_page_state();
            }
            SL2PDWAKE0..=SL2PDWAKE5 if is_word(address) => {
                self.wake_request[((address - SL2PDWAKE0) / 4) as usize] = value & PAGE_MASK;
                self.merge_page_state();
            }
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl SharedMemory {
    /// Whether a write the CPU makes in User mode to `address` is refused, and if so, latch it.
    ///
    /// SPRUEG5D sections 5.1, 5.2, 5.8 and 5.9: SL2PFCPEN, SL2PFCFLUSH, SL2PDSLEEP0-5 and
    /// SL2PDWAKE0-5 can be written "only in secure supervisor mode or supervisor mode". A write in
    /// secure user or user mode makes the SMC raise its exception towards every megamodule and
    /// record the latest exception status (section 5.1), and section 5.3 says what: the accessing
    /// megamodule id into SL2MPFSR.CPU_ID, the accessing mode into SL2MPFSR.MODE and the address
    /// into SL2MPFAR. "A non-supervisor mode write returns a memory protection error to the
    /// configuration bus interface", so the write does not happen.
    ///
    /// Only a megamodule can fault: it is the only requester with a megamodule id for CPU_ID, and
    /// a master's register write is not refused. The caller asks for User-mode writes only.
    ///
    /// MODE is recorded nonsecure (1, Table 8). The model has no secure mode - nothing in the
    /// device can enter one - so every User-mode write is the nonsecure kind.
    ///
    /// UNCERTAIN: section 5.3 says the faulting writes are those "to the writable registers", and
    /// SL2MPFSR.CLEAR is a writable bit. Only the four registers that sections 5.1, 5.2, 5.8
    /// and 5.9 name as supervisor-only are treated as faulting, so a User-mode write to SL2MPFSR
    /// is accepted. The atomic-access registers and SL2MPFAR are read-only and ignore writes.
    pub(crate) fn refuse_user_write(&mut self, core: usize, address: u32) -> bool {
        let word = address & !3;
        let supervisor_only = matches!(word, SL2PFCPEN | SL2PFCFLUSH)
            || (SL2PDSLEEP0..=SL2PDSLEEP5).contains(&word)
            || (SL2PDWAKE0..=SL2PDWAKE5).contains(&word);
        if supervisor_only {
            self.fault_cpu_id = core as u32 & 7;
            self.fault_nonsecure = true;
            self.fault_address = address;
            self.fault = true;
        }
        supervisor_only
    }

    /// True once for each write of 1 to SL2PFCFLUSH.FLUSH.
    ///
    /// Consumed by the memory path, which must invalidate the per-megamodule prefetch buffers of
    /// all six megamodules connected to the SMC (SPRUEG5D sections 2.3.6 and 5.2). The flush is
    /// global, so one edge covers every core. The model holds no prefetch buffer, so the fabric
    /// has nothing to discard and takes the edge only to keep the write one-shot.
    pub(crate) fn take_prefetch_flush(&mut self) -> bool {
        core::mem::take(&mut self.flush)
    }

    /// One request reaching SL2 RAM at `offset`, from a megamodule's L2 controller: the SMC wakes
    /// the page it lands in and says which bank serves it and in how many wait states.
    ///
    /// SPRUEG5D section 4.3: the SMC wakes a page "in response to ... a C64x+ megamodule accesses
    /// the sleeping/power-down SMC physical page(s)" and delays the acknowledgement until the page
    /// is awake. The wake takes "the SL2 memory wrapper" and the chip's power controller, neither
    /// of which the model has a duration for, so the access waits no longer for it. The page then
    /// stays awake until the next SL2PDSLEEP or SL2PDWAKE write merges the cores' permissions
    /// again; the guide does not say when a woken page may go back to sleep.
    ///
    /// UNCERTAIN: the guide never says which address bits select a bank. The banks are 256 bits
    /// (32 bytes) wide (Figure 2) and the profiler decodes the bank from
    /// megamodule_to_smc_addr[1:0] (Figure 17), so this module interleaves the four banks on
    /// 256-bit granules and selects with address bits 6-5, the same bits the monitors use.
    pub(crate) fn request(&mut self, offset: u32) -> Service {
        self.asleep[page_of(offset)] = false;
        Service {
            bank: monitor_index(offset),
            wait_states: READ_WAIT_STATES,
        }
    }

    /// Level state of system event 63, SMCMPINT.
    ///
    /// Asserted from the fault until software writes 1 to SL2MPFSR.CLEAR. SPRUEG5D section 6 and
    /// Figure 15: one common exception goes to all six megamodules, so this one level drives event
    /// 63 on every core.
    pub(crate) fn fault_pending(&self) -> bool {
        self.fault
    }

    /// True when every core has granted permission for the page to sleep and no core is holding a
    /// wake request for it.
    ///
    /// SPRUEG5D section 4.1: "SMC looks at the logical AND of all sleep inputs for each page from
    /// each C64x+ megamodule and use that to determine what pages may actually sleep. If any
    /// C64x+ megamodule connected to SMC wants a page awake, SMC keeps the page awake." Section
    /// 4.4 repeats it: the SMC sleeps a page when it has the permission from all the megamodules.
    ///
    /// `page` is 0 or 1; anything else is not a page of this device and can never sleep.
    ///
    /// UNCERTAIN: the guide describes SL2PDWAKE only as the other half of the
    /// "SL2PDWAKE/SL2PDSLEEP interface" that manually commands a page awake (sections 4.1, 4.3 and
    /// 5.9) and never states how it combines with SL2PDSLEEP, nor whether a wake request is a
    /// level or a self-clearing pulse. This module takes both registers as per-core levels, as
    /// their Enable/Disable field encodings suggest, and lets a set wake bit veto sleep - which is
    /// the section 4.1 rule that any core wanting a page awake keeps it awake.
    pub(crate) fn page_may_sleep(&self, page: usize) -> bool {
        let Some(bit) = page_bit(page) else {
            return false;
        };
        self.sleep_permission.iter().all(|grant| grant & bit != 0)
            && self.wake_request.iter().all(|wake| wake & bit == 0)
    }

    /// LL, load-linked (SPRUEG5D section 3.1.1), on the word at `offset` into SL2 RAM.
    ///
    /// "When the monitor sees an LL instruction from any of the C64x+ megamodules, the monitor logs
    /// the requester C64x+ megamodule id in the CPU_ID field, logs the address in LinkAdr field,
    /// set the LinkV flag to 1 and set the LinkdtV flag to 0, irrespective of the state of the
    /// monitor." The load itself is unaffected and is the memory path's business. LinkData is not
    /// documented as cleared, so it is left alone.
    pub(crate) fn load_link(&mut self, core: usize, offset: u32) {
        let monitor = &mut self.monitor[monitor_index(offset)];
        monitor.cpu_id = core as u32 & 7;
        monitor.address = offset & !3;
        monitor.link_valid = true;
        monitor.data_valid = false;
    }

    /// SL, store-linked (SPRUEG5D section 3.1.2, Table 3): `data` is the 32-bit datum the
    /// megamodule stores at `offset`.
    ///
    /// SL is a write to the monitor only, never to memory, so the memory path stores nothing for
    /// it. Table 3, for the rows that do anything at all - all of which need LinkV set and the
    /// megamodule id to match:
    ///
    /// * address mismatch: invalidate the link.
    /// * address match with LinkdtV clear: store the 32-bit datum and set LinkdtV.
    /// * address match with LinkdtV already set: invalidate the link.
    ///
    /// Every other combination discards the request.
    pub(crate) fn store_link(&mut self, core: usize, offset: u32, data: u32) {
        let monitor = &mut self.monitor[monitor_index(offset)];
        if !monitor.link_valid || monitor.cpu_id != core as u32 & 7 {
            return;
        }
        if monitor.address != offset & !3 || monitor.data_valid {
            monitor.link_valid = false;
            return;
        }
        monitor.data = data;
        monitor.data_valid = true;
    }

    /// CMTL, commit-linked (SPRUEG5D section 3.1.3, Table 4), on the word at `offset`.
    ///
    /// `Some(data)` means the commit succeeded, so the memory path writes `data` to the word and
    /// the megamodule gets 1; `None` means the commit failed and the megamodule gets 0. Either
    /// way the link is left invalid whenever the monitor belonged to this core, which is what
    /// makes the commit single-shot.
    pub(crate) fn commit_link(&mut self, core: usize, offset: u32) -> Option<u32> {
        let monitor = &mut self.monitor[monitor_index(offset)];
        if !monitor.link_valid || monitor.cpu_id != core as u32 & 7 {
            return None;
        }
        monitor.link_valid = false;
        if monitor.address != offset & !3 || !monitor.data_valid {
            return None;
        }
        Some(monitor.data)
    }

    /// SL2PDSTAT: 1 means awake, 0 means sleeping (SPRUEG5D Table 10).
    fn page_status(&self) -> u32 {
        let mut value = 0;
        for page in 0..PAGES {
            if !self.asleep[page] {
                value |= page_bit(page).unwrap_or(0);
            }
        }
        value
    }

    /// Re-evaluate the merged sleep state after a permission or wake request changed.
    fn merge_page_state(&mut self) {
        for page in 0..PAGES {
            self.asleep[page] = self.page_may_sleep(page);
        }
    }
}

impl Monitor {
    /// SL2AMLSTATn (SPRUEG5D Table 11): ADDR in bits 23-5, CPUID in 4-2, LINKV in 1, LINKTV in 0.
    ///
    /// UNCERTAIN: Table 11 encodes LINKV as 0 = Valid, 1 = Not valid, which contradicts section
    /// 3.1.1 ("set the LinkV flag to 1") and the whole of Tables 3 and 4, where LinkV = 1 is the
    /// valid link. This module reports LINKV = 1 for a valid link and treats the Table 11 enum
    /// ordering as a documentation error. LINKTV, whose Table 11 encoding is 1 = Valid, agrees
    /// with the rest of the guide either way.
    fn status(&self) -> u32 {
        (self.address & AML_ADDR_MASK)
            | ((self.cpu_id & 7) << 2)
            | (u32::from(self.link_valid) << 1)
            | u32::from(self.data_valid)
    }
}

const SL2AMLSTAT3: u32 = SL2AMLSTAT0 + 4 * (BANKS as u32 - 1);
const SL2AMLDATA3: u32 = SL2AMLDATA0 + 4 * (BANKS as u32 - 1);
const SL2PDSLEEP5: u32 = SL2PDSLEEP0 + 4 * (CORES as u32 - 1);
const SL2PDWAKE5: u32 = SL2PDWAKE0 + 4 * (CORES as u32 - 1);

/// Whether `address` lies in the span the register file's registers are spread over, a filter
/// the CPU path applies before it asks `refuse_user_write` about a User-mode write. The gaps
/// between the arrays are inside it; `refuse_user_write` is what knows which words are registers.
pub(crate) fn is_register(address: u32) -> bool {
    address.wrapping_sub(BASE) < SL2PDWAKE5 + 4 - BASE
}

/// The register arrays are word-spaced, so the addresses between them are not registers.
fn is_word(address: u32) -> bool {
    address % 4 == 0
}

/// The SL2PDSTAT/SL2PDSLEEP/SL2PDWAKE bit that carries a page.
fn page_bit(page: usize) -> Option<u32> {
    match page {
        0 => Some(1 << 2),
        1 => Some(1 << 3),
        _ => None,
    }
}

/// The atomic-access monitor, and the bank, an SL2 offset belongs to: there is one monitor per
/// bank, and "accesses to distinct banks ... are guaranteed to never hit the same monitor
/// address" (SPRUEG5D section 3). Which address bits select a bank is the UNCERTAIN choice
/// `request` documents.
fn monitor_index(offset: u32) -> usize {
    ((offset >> 5) & (BANKS as u32 - 1)) as usize
}

/// The power-down page an SL2 offset lives in; see `PAGE0_BYTES`.
fn page_of(offset: u32) -> usize {
    usize::from(offset >= PAGE0_BYTES)
}

/// How many SYSCLK9 wait states the SMC serves a read request in: the 3 of a request that
/// misses the prefetch buffers and is "serviced from the shared L2 memory as a 3-wait-state
/// memory" (SPRUEG5D section 2.4, P-1 stage and Figure 3).
///
/// A prefetch hit would be 0, but nothing prefetches here, so every read is a miss. Arbitration
/// against the other five megamodules and the wake of a sleeping page would add more; neither has
/// a duration the model can state, so a read never waits longer than the documented minimum.
pub(crate) const READ_WAIT_STATES: usize = 3;

/// What the SMC answers for one request reaching SL2 RAM: the bank that serves it and the wait
/// states the requester sees, which is what the profiler files the request under.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Service {
    pub(crate) bank: usize,
    pub(crate) wait_states: usize,
}

/// Base of the per-core SMC profiler register files, SPRS612G Table 2-2: SMCP0-5, 16K each, at
/// 0x02C20000 + 0x4000 * core. SPRUEG5D Table 17 gives only the offsets.
pub(crate) const PROFILER_BASE: u32 = 0x02c2_0000;
/// Distance between two SMCPn windows (SPRS612G Table 2-2).
pub(crate) const PROFILER_STRIDE: u32 = 0x4000;

const PROFILER_BNKMSK: u32 = 0x00;
const PROFILER_PWSCNT0: u32 = 0x04;
const PROFILER_PWSCNT7: u32 = PROFILER_PWSCNT0 + 4 * (WAIT_STATE_BINS as u32 - 1);
const PROFILER_PFCCNT: u32 = 0x24;
const PROFILER_PCMD: u32 = 0x28;
const PROFILER_PSTAT: u32 = 0x2c;
const PROFILER_STATMASK: u32 = 0x30;

/// Eight wait-state counters, one per SPCL count value (SPRUEG5D Table 15).
pub(crate) const WAIT_STATE_BINS: usize = 8;
/// The SPCL counter saturates at 7, so a request of seven or more wait states lands in the last
/// bin: "The maximum required value of counter/s is 7. When they reach the maximum value, the
/// count remains at the maximum value" (SPRUEG5D section 7.1.3), and Table 15 describes count 7 as
/// "Read request serviced in 0 or more wait state".
const MAX_WAIT_STATES: usize = WAIT_STATE_BINS - 1;

/// SL2BNKMSK resets with all four bank bits set, i.e. accepting statistics events from every bank
/// (SPRUEG5D Figure 21).
const BNKMSK_RESET: u32 = (1 << BANKS) - 1;
/// SL2PCMD.ENPROFILE.
const PCMD_ENPROFILE: u32 = 1 << 1;
/// SL2PCMD.CLEAR.
const PCMD_CLEAR: u32 = 1 << 0;

/// One core's SMC profiler (SPRUEG5D section 7): the statistics counters the profiler fills from
/// the UMAP1 interface between that megamodule and the SMC.
///
/// The profiler counts the read requests the memory path reports: `note_access` once per SMC
/// read request serviced for this core. Nothing is counted while profiling is disabled.
pub(crate) struct SharedMemoryProfiler {
    /// Start of this core's 16K window.
    base: u32,
    /// SL2BNKMSK bits 3-0, reset 1s (SPRUEG5D section 7.3.1).
    bank_mask: u32,
    /// SL2PWSCNT0-7, reset 0, saturating (SPRUEG5D section 7.3.2).
    wait_state_count: [u32; WAIT_STATE_BINS],
    /// SL2PCMD, reset 0: profiling disabled (SPRUEG5D section 7.3.4).
    command: u32,
    /// SL2STATMASK bits 7-0, reset 0: no statistics event is combined into the profiler event
    /// (SPRUEG5D section 7.3.6).
    event_mask: u32,
    /// An unconsumed combined statistics event, i.e. system event 64 SMCPEVT for this core.
    event: bool,
}

impl SharedMemoryProfiler {
    /// The profiler of megamodule `core`, at SMCP`core` (SPRS612G Table 2-2).
    pub(crate) fn new(core: usize) -> Self {
        Self {
            base: PROFILER_BASE + PROFILER_STRIDE * core as u32,
            bank_mask: BNKMSK_RESET,
            wait_state_count: [0; WAIT_STATE_BINS],
            command: 0,
            event_mask: 0,
            event: false,
        }
    }
}

impl SlavePort for SharedMemoryProfiler {
    /// None when the address is not one of this profiler's registers.
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address.wrapping_sub(self.base) {
            PROFILER_BNKMSK => self.bank_mask,
            offset @ PROFILER_PWSCNT0..=PROFILER_PWSCNT7 if offset % 4 == 0 => {
                self.wait_state_count[((offset - PROFILER_PWSCNT0) / 4) as usize]
            }
            // SPRUEG5D section 7.3.3: SL2PFCCNT counts the prefetch requests the SMC generated.
            // Nothing prefetches in this model (see the module documentation), so it never counts.
            PROFILER_PFCCNT => 0,
            PROFILER_PCMD => self.command,
            PROFILER_PSTAT => self.saturation_status(),
            PROFILER_STATMASK => self.event_mask,
            _ => return None,
        };
        Some(value)
    }

    /// `PortWrite::Unhandled` when the address is not one of this profiler's registers.
    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address.wrapping_sub(self.base) {
            PROFILER_BNKMSK => self.bank_mask = value & BNKMSK_RESET,
            // The counters and the saturation status are read-only (SPRUEG5D Figures 22, 23, 25).
            offset @ PROFILER_PWSCNT0..=PROFILER_PWSCNT7 if offset % 4 == 0 => {}
            PROFILER_PFCCNT | PROFILER_PSTAT => {}
            // SPRUEG5D section 7.3.4: "Writing 1 to CLEAR (bit 0) clears/resets all the SL2PWSCNT
            // registers and the SL2PFCCNT register."
            //
            // UNCERTAIN: the guide does not say whether CLEAR self-clears once the counters are
            // cleared. Figure 24 marks it R/W-0, so this module latches what was written and
            // performs the clear on each write of 1.
            PROFILER_PCMD => {
                self.command = value & (PCMD_ENPROFILE | PCMD_CLEAR);
                if value & PCMD_CLEAR != 0 {
                    self.wait_state_count = [0; WAIT_STATE_BINS];
                }
            }
            PROFILER_STATMASK => self.event_mask = value & 0xff,
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl SharedMemoryProfiler {
    /// Report one SMC read request serviced for this core, in `wait_states` SYSCLK9 wait states,
    /// against bank `bank`.
    ///
    /// SPRUEG5D sections 7.1.3 to 7.1.5: the SPCL counter saturates at 7, the SBML drops the
    /// request unless SL2BNKMSK accepts its bank, and the SSUL then bumps SL2PWSCNT[wait states]
    /// and drives the matching statistics event. Nothing happens while SL2PCMD.ENPROFILE is clear
    /// (section 7.3.4: "By default, the SMC profiler is disabled").
    pub(crate) fn note_access(&mut self, bank: usize, wait_states: usize) {
        if !self.enabled() || bank >= BANKS || self.bank_mask & (1 << bank) == 0 {
            return;
        }
        let bin = wait_states.min(MAX_WAIT_STATES);
        self.wait_state_count[bin] = self.wait_state_count[bin].saturating_add(1);
        if self.event_mask & (1 << bin) != 0 {
            self.event = true;
        }
    }

    /// Consume the combined statistics event, so the interrupt controller sees exactly one event
    /// per qualified read request.
    ///
    /// SPRUEG5D section 7.3.6: the profiler ORs the eight statistics events that SL2STATMASK
    /// admits into one event for the megamodule, which SPRS612G Table 7-10 lists as event 64,
    /// "SMC Profiler Consolidated Event". With SL2STATMASK at its reset value of 0 nothing is
    /// combined and the event never asserts.
    pub(crate) fn take_event(&mut self) -> bool {
        core::mem::take(&mut self.event)
    }

    /// SL2PSTAT (SPRUEG5D section 7.3.5), derived from the counters: a counter that has reached its
    /// maximum value has saturated and stopped advancing, so SATn reports it. PSAT, the SL2PFCCNT
    /// bit, stays 0 because that counter never advances.
    fn saturation_status(&self) -> u32 {
        let mut value = 0;
        for (bin, count) in self.wait_state_count.iter().enumerate() {
            if *count == u32::MAX {
                value |= 1 << bin;
            }
        }
        value
    }

    fn enabled(&self) -> bool {
        self.command & PCMD_ENPROFILE != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    #[test]
    fn the_control_registers_come_up_at_their_documented_reset_values() {
        // SPRUEG5D Figures 6 to 14: everything resets to 0 except SL2MPFSR.CPU_ID, which resets to
        // 111b, and SL2PDSTAT, whose 0s mean both pages start in sleep mode.
        let mut smc = SharedMemory::default();
        assert_eq!(smc.read(SL2PFCPEN, TEST_CONTEXT), Some(0));
        assert_eq!(smc.read(SL2PFCFLUSH, TEST_CONTEXT), Some(0));
        assert_eq!(smc.read(SL2MPFSR, TEST_CONTEXT), Some(NO_FAULT_CPU_ID << 2));
        assert_eq!(smc.read(SL2MPFAR, TEST_CONTEXT), Some(0));
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(0));
        for bank in 0..BANKS as u32 {
            assert_eq!(smc.read(SL2AMLSTAT0 + 4 * bank, TEST_CONTEXT), Some(0));
            assert_eq!(smc.read(SL2AMLDATA0 + 4 * bank, TEST_CONTEXT), Some(0));
        }
        for core in 0..CORES as u32 {
            assert_eq!(smc.read(SL2PDSLEEP0 + 4 * core, TEST_CONTEXT), Some(0));
            assert_eq!(smc.read(SL2PDWAKE0 + 4 * core, TEST_CONTEXT), Some(0));
        }
        assert!(!smc.fault_pending());
    }

    #[test]
    fn addresses_outside_the_register_file_are_not_ours() {
        // Only the exact register addresses answer; the rest of the 32K window at 0x02AA8000 and
        // the gaps between the register arrays are not this block's (SPRUEG5D Table 5).
        let mut smc = SharedMemory::default();
        for address in [
            BASE - 4,
            BASE + 0x014,
            BASE + 0x0fc,
            BASE + 0x102,
            BASE + 0x110,
            BASE + 0x130,
            BASE + 0x218,
            BASE + 0x238,
            BASE + 0x4000,
        ] {
            assert_eq!(smc.read(address, TEST_CONTEXT), None, "read {address:#x}");
            assert_eq!(
                smc.write(address, 1, TEST_CONTEXT),
                PortWrite::Unhandled,
                "write {address:#x}"
            );
        }
        // The last register of each array does answer, so the ranges above are the gaps.
        assert_eq!(smc.read(SL2AMLSTAT0 + 0x0c, TEST_CONTEXT), Some(0));
        assert_eq!(smc.read(SL2PDSLEEP0 + 0x14, TEST_CONTEXT), Some(0));
        assert_eq!(smc.read(SL2PDWAKE0 + 0x14, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn the_prefetch_page_enables_are_a_plain_read_write_level() {
        // SPRUEG5D section 5.1: 32 prefetchable-page bits, readable in any mode.
        let mut smc = SharedMemory::default();
        smc.write(SL2PFCPEN, 0x8000_0003, TEST_CONTEXT);
        assert_eq!(smc.read(SL2PFCPEN, TEST_CONTEXT), Some(0x8000_0003));
    }

    #[test]
    fn a_flush_write_triggers_one_edge_and_never_latches() {
        // SPRUEG5D section 5.2: writing 1 to FLUSH triggers the flush logic, "should not latch
        // into the SL2PFCFLUSH register", and a read returns 0s.
        let mut smc = SharedMemory::default();
        assert!(!smc.take_prefetch_flush());
        assert_ne!(
            smc.write(SL2PFCFLUSH, 1, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(smc.read(SL2PFCFLUSH, TEST_CONTEXT), Some(0));
        assert!(smc.take_prefetch_flush());
        assert!(!smc.take_prefetch_flush());
        // A write of 0 has no effect (Table 7).
        smc.write(SL2PFCFLUSH, 0, TEST_CONTEXT);
        assert!(!smc.take_prefetch_flush());
    }

    #[test]
    fn a_user_mode_write_to_a_supervisor_only_register_is_refused_and_latched_until_cleared() {
        // SPRUEG5D sections 5.1, 5.2, 5.8 and 5.9 make SL2PFCPEN, SL2PFCFLUSH, SL2PDSLEEP0-5 and
        // SL2PDWAKE0-5 supervisor-only; section 5.3: the SMC records the accessing megamodule id
        // in CPU_ID and the mode in MODE, latches the address in SL2MPFAR, raises the common
        // exception, and software clears both by writing 1 to CLEAR.
        let mut smc = SharedMemory::default();
        assert!(smc.refuse_user_write(3, SL2PFCPEN));
        assert_eq!(smc.read(SL2MPFSR, TEST_CONTEXT), Some((3 << 2) | (1 << 1)));
        assert_eq!(smc.read(SL2MPFAR, TEST_CONTEXT), Some(SL2PFCPEN));
        assert!(smc.fault_pending());
        // The latest fault overwrites the previous one (section 5.1).
        assert!(smc.refuse_user_write(5, SL2PDWAKE0 + 0x14));
        assert_eq!(smc.read(SL2MPFSR, TEST_CONTEXT), Some((5 << 2) | (1 << 1)));
        assert_eq!(smc.read(SL2MPFAR, TEST_CONTEXT), Some(SL2PDWAKE0 + 0x14));
        // A write of 0 to CLEAR has no effect (Table 8).
        smc.write(SL2MPFSR, 0, TEST_CONTEXT);
        assert!(smc.fault_pending());
        smc.write(SL2MPFSR, 1, TEST_CONTEXT);
        assert!(!smc.fault_pending());
        assert_eq!(smc.read(SL2MPFSR, TEST_CONTEXT), Some(NO_FAULT_CPU_ID << 2));
        assert_eq!(smc.read(SL2MPFAR, TEST_CONTEXT), Some(0));
        // CPU_ID and MODE are read-only: a write cannot set them.
        smc.write(SL2MPFSR, 0xffff_fffe, TEST_CONTEXT);
        assert_eq!(smc.read(SL2MPFSR, TEST_CONTEXT), Some(NO_FAULT_CPU_ID << 2));
        // SL2MPFAR is read-only.
        assert_ne!(
            smc.write(SL2MPFAR, 0xdead_beef, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(smc.read(SL2MPFAR, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn only_the_four_supervisor_only_register_families_refuse_a_user_mode_write() {
        let mut smc = SharedMemory::default();
        for core in 0..CORES as u32 {
            assert!(smc.refuse_user_write(core as usize, SL2PDSLEEP0 + 4 * core));
            assert!(smc.refuse_user_write(core as usize, SL2PDWAKE0 + 4 * core));
        }
        assert!(smc.refuse_user_write(0, SL2PFCFLUSH));
        // Read-only registers, the clear-only status register and the gaps are not faults, and a
        // refused write is the only thing that records one.
        smc.write(SL2MPFSR, 1, TEST_CONTEXT);
        for address in [
            SL2MPFSR,
            SL2MPFAR,
            SL2PDSTAT,
            SL2AMLSTAT0,
            SL2AMLDATA0,
            BASE + 0x014,
            SL2PDSLEEP0 + 0x18,
            SL2PDWAKE0 + 0x18,
        ] {
            assert!(!smc.refuse_user_write(1, address), "{address:#x}");
        }
        assert!(!smc.fault_pending());
        assert!(is_register(SL2PFCPEN) && is_register(SL2PDWAKE0 + 0x14));
        assert!(!is_register(BASE - 4) && !is_register(BASE + 0x238));
    }

    #[test]
    fn a_page_sleeps_only_when_all_six_cores_grant_permission() {
        // SPRUEG5D section 4.1: the SMC ANDs the sleep inputs of all six megamodules; section 4.4
        // repeats that it needs the permission from all of them. SL2PDSTAT then reports 0 for a
        // sleeping page and 1 for an awake one (Table 10).
        let mut smc = SharedMemory::default();
        let page1 = 1 << 3;
        for core in 0..CORES as u32 - 1 {
            assert_ne!(
                smc.write(SL2PDSLEEP0 + 4 * core, page1, TEST_CONTEXT),
                PortWrite::Unhandled
            );
            assert!(!smc.page_may_sleep(1));
            // Every page is awake as long as one core withholds permission.
            assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(PAGE_MASK));
        }
        smc.write(SL2PDSLEEP0 + 4 * (CORES as u32 - 1), page1, TEST_CONTEXT);
        assert!(smc.page_may_sleep(1));
        assert!(!smc.page_may_sleep(0));
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(1 << 2));
        // One core withdrawing its permission wakes the page again.
        smc.write(SL2PDSLEEP0 + 4 * 2, 0, TEST_CONTEXT);
        assert!(!smc.page_may_sleep(1));
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(PAGE_MASK));
        // There is no page 2 on this device (section 4.2: only two pages are RAM).
        assert!(!smc.page_may_sleep(2));
    }

    #[test]
    fn a_request_wakes_the_page_it_lands_in_until_the_permissions_are_merged_again() {
        // SPRUEG5D section 4.3: the SMC wakes a sleeping page when a megamodule accesses it. Both
        // pages come out of reset asleep (section 5.5), and the first two pages are laid out in
        // address order: page 0 is the first 512 KB, page 1 the 256 KB after it (section 4.2).
        let mut smc = SharedMemory::default();
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(0));
        smc.request(PAGE0_BYTES - 4);
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(1 << 2));
        smc.request(PAGE0_BYTES);
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(PAGE_MASK));
        // Every core granting permission lets both sleep again, which is the one merge there is.
        for core in 0..CORES as u32 {
            smc.write(SL2PDSLEEP0 + 4 * core, PAGE_MASK, TEST_CONTEXT);
        }
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(0));
        smc.request(0);
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(1 << 2));
    }

    #[test]
    fn a_request_is_served_by_the_bank_its_granule_selects_in_the_documented_wait_states() {
        // SPRUEG5D section 2.4: a read that misses the prefetch buffers is "serviced from the
        // share L2 memory as a 3-wait-state memory". The banks interleave on 256-bit granules.
        let mut smc = SharedMemory::default();
        for bank in 0..BANKS {
            let service = smc.request(32 * bank as u32);
            assert_eq!(service.bank, bank);
            assert_eq!(service.wait_states, 3);
        }
        assert_eq!(smc.request(32 * BANKS as u32).bank, 0);
        assert_eq!(
            smc.request(0x1c).bank,
            0,
            "the word offset is inside one granule"
        );
    }

    #[test]
    fn a_wake_request_from_any_core_keeps_the_page_awake() {
        // SPRUEG5D section 4.1: "If any C64x+ megamodule connected to SMC wants a page awake, SMC
        // keeps the page awake", and section 4.4 lets a program command a page awake through the
        // SL2PDWAKE/SL2PDSLEEP interface.
        let mut smc = SharedMemory::default();
        let page0 = 1 << 2;
        for core in 0..CORES as u32 {
            smc.write(SL2PDSLEEP0 + 4 * core, page0, TEST_CONTEXT);
        }
        assert!(smc.page_may_sleep(0));
        assert_ne!(
            smc.write(SL2PDWAKE0 + 4 * 4, page0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert!(!smc.page_may_sleep(0));
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(PAGE_MASK));
        smc.write(SL2PDWAKE0 + 4 * 4, 0, TEST_CONTEXT);
        assert!(smc.page_may_sleep(0));
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(1 << 3));
        // Only P3 and P2 exist; the reserved bits of a write are dropped, so a write of all ones
        // is a permission for both pages and nothing else.
        for core in 0..CORES as u32 {
            smc.write(SL2PDSLEEP0 + 4 * core, 0xffff_ffff, TEST_CONTEXT);
        }
        assert!(smc.page_may_sleep(0) && smc.page_may_sleep(1));
        assert_eq!(smc.read(SL2PDSTAT, TEST_CONTEXT), Some(0));
    }

    /// SL2AMLSTATn of the bank `offset` belongs to.
    fn monitor_status(smc: &mut SharedMemory, offset: u32) -> u32 {
        smc.read(SL2AMLSTAT0 + 4 * monitor_index(offset) as u32, TEST_CONTEXT)
            .unwrap()
    }

    /// SL2AMLDATAn of the bank `offset` belongs to.
    fn monitor_data(smc: &mut SharedMemory, offset: u32) -> u32 {
        smc.read(SL2AMLDATA0 + 4 * monitor_index(offset) as u32, TEST_CONTEXT)
            .unwrap()
    }

    const LINK_V: u32 = 1 << 1;
    const LINKDT_V: u32 = 1;

    #[test]
    fn the_atomic_monitor_runs_the_documented_link_store_commit_sequence() {
        // SPRUEG5D sections 3.1.1 to 3.1.3: LL logs the core, address and LinkV; SL stores the
        // datum and sets LinkdtV; CMTL with all four conditions met clears LinkV, returns the
        // datum for the memory write and reports success.
        let mut smc = SharedMemory::default();
        let offset = 0x0001_0044;
        smc.load_link(2, offset);
        assert_eq!(
            monitor_status(&mut smc, offset),
            (offset & AML_ADDR_MASK) | (2 << 2) | LINK_V
        );
        smc.store_link(2, offset, 0x1234_5678);
        assert_eq!(
            monitor_status(&mut smc, offset),
            (offset & AML_ADDR_MASK) | (2 << 2) | LINK_V | LINKDT_V
        );
        assert_eq!(monitor_data(&mut smc, offset), 0x1234_5678);
        assert_eq!(smc.commit_link(2, offset), Some(0x1234_5678));
        // The commit invalidated the link, so it cannot be taken twice (Table 4).
        assert_eq!(smc.commit_link(2, offset), None);
        // Both status registers are read-only (Figures 11 and 12).
        let status = monitor_status(&mut smc, offset);
        let bank = 4 * monitor_index(offset) as u32;
        assert_ne!(
            smc.write(SL2AMLSTAT0 + bank, 0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_ne!(
            smc.write(SL2AMLDATA0 + bank, 0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(monitor_status(&mut smc, offset), status);
        assert_eq!(monitor_data(&mut smc, offset), 0x1234_5678);
    }

    #[test]
    fn a_foreign_store_link_is_discarded_and_a_second_one_breaks_the_link() {
        // SPRUEG5D Table 3: with LinkV set, an SL whose megamodule id does not match is discarded,
        // an SL to another address invalidates the link, and an SL that arrives with LinkdtV
        // already set invalidates it too.
        let mut smc = SharedMemory::default();
        let offset = 0x0000_0100;
        smc.load_link(1, offset);
        smc.store_link(4, offset, 0xaaaa_aaaa);
        assert_eq!(
            monitor_data(&mut smc, offset),
            0,
            "a foreign SL stores nothing"
        );
        assert_eq!(
            monitor_status(&mut smc, offset) & (LINK_V | LINKDT_V),
            LINK_V
        );
        assert_eq!(smc.commit_link(4, offset), None);
        // Core 1 still owns a valid link with no data, so its commit fails but clears LinkV.
        assert_eq!(monitor_status(&mut smc, offset) & LINK_V, LINK_V);
        assert_eq!(smc.commit_link(1, offset), None);
        assert_eq!(monitor_status(&mut smc, offset) & LINK_V, 0);
        // An SL with no live link does nothing at all (Table 3, LinkV = 0 rows).
        smc.store_link(1, offset, 0x5555_5555);
        assert_eq!(monitor_data(&mut smc, offset), 0);
        assert_eq!(monitor_status(&mut smc, offset) & LINKDT_V, 0);
        // A second SL after a successful one invalidates the link instead of storing again.
        smc.load_link(1, offset);
        smc.store_link(1, offset, 0x0000_0001);
        smc.store_link(1, offset, 0x0000_0002);
        assert_eq!(monitor_status(&mut smc, offset) & LINK_V, 0);
        assert_eq!(smc.commit_link(1, offset), None);
        assert_eq!(monitor_data(&mut smc, offset), 0x0000_0001);
        // An SL to a different granule of the same bank invalidates the link (Table 3, row
        // LinkV = 1, id match, address mismatch).
        smc.load_link(1, offset);
        let other = offset + 4 * 32 * BANKS as u32;
        assert_eq!(monitor_index(other), monitor_index(offset));
        smc.store_link(1, other, 0x0000_0003);
        assert_eq!(monitor_status(&mut smc, offset) & LINK_V, 0);
        assert_eq!(smc.commit_link(1, offset), None);
        // A different word of the monitored granule is an address mismatch as well: CMTL writes
        // LinkData to the linked word, which a granule could not name.
        smc.load_link(1, offset);
        smc.store_link(1, offset + 4, 0x0000_0004);
        assert_eq!(monitor_status(&mut smc, offset) & LINK_V, 0);
    }

    #[test]
    fn each_bank_has_its_own_monitor() {
        // SPRUEG5D section 3: one monitor per per-BANK SMC logic, and accesses to distinct banks
        // never hit the same monitor address.
        let mut smc = SharedMemory::default();
        let bank0 = 0x0000_0000;
        let bank1 = bank0 + 32;
        assert_ne!(monitor_index(bank0), monitor_index(bank1));
        smc.load_link(0, bank0);
        smc.load_link(1, bank1);
        smc.store_link(0, bank0, 0xa);
        smc.store_link(1, bank1, 0xb);
        assert_eq!(smc.commit_link(0, bank0), Some(0xa));
        assert_eq!(smc.commit_link(1, bank1), Some(0xb));
    }

    #[test]
    fn the_profiler_registers_come_up_at_their_documented_reset_values() {
        // SPRUEG5D Figures 21 to 26: SL2BNKMSK resets with all four bank bits set, everything else
        // resets to 0, and the window is SMCPn from SPRS612G Table 2-2.
        for core in 0..CORES {
            let mut profiler = SharedMemoryProfiler::new(core);
            let base = PROFILER_BASE + PROFILER_STRIDE * core as u32;
            assert_eq!(
                profiler.read(base + PROFILER_BNKMSK, TEST_CONTEXT),
                Some(0xf)
            );
            for bin in 0..WAIT_STATE_BINS as u32 {
                assert_eq!(
                    profiler.read(base + PROFILER_PWSCNT0 + 4 * bin, TEST_CONTEXT),
                    Some(0)
                );
            }
            assert_eq!(profiler.read(base + PROFILER_PFCCNT, TEST_CONTEXT), Some(0));
            assert_eq!(profiler.read(base + PROFILER_PCMD, TEST_CONTEXT), Some(0));
            assert_eq!(profiler.read(base + PROFILER_PSTAT, TEST_CONTEXT), Some(0));
            assert_eq!(
                profiler.read(base + PROFILER_STATMASK, TEST_CONTEXT),
                Some(0)
            );
        }
    }

    #[test]
    fn profiler_addresses_outside_the_register_file_are_not_ours() {
        // The register file ends at offset 0x30 (SPRUEG5D Table 17); the rest of the 16K SMCP1
        // window and the neighbouring windows are not this profiler's.
        let mut profiler = SharedMemoryProfiler::new(1);
        let base = PROFILER_BASE + PROFILER_STRIDE;
        for address in [
            base - 4,
            base + 0x02,
            base + 0x34,
            base + 0x1000,
            PROFILER_BASE,
        ] {
            assert_eq!(
                profiler.read(address, TEST_CONTEXT),
                None,
                "read {address:#x}"
            );
            assert_eq!(
                profiler.write(address, 1, TEST_CONTEXT),
                PortWrite::Unhandled,
                "write {address:#x}"
            );
        }
    }

    #[test]
    fn the_counters_only_advance_while_profiling_is_enabled() {
        // SPRUEG5D section 7.3.4: "By default, the SMC profiler is disabled", and ENPROFILE
        // enables it to gather statistics.
        let mut profiler = SharedMemoryProfiler::new(0);
        profiler.note_access(0, 3);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0 + 12, TEST_CONTEXT),
            Some(0)
        );
        profiler.write(PROFILER_BASE + PROFILER_PCMD, PCMD_ENPROFILE, TEST_CONTEXT);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PCMD, TEST_CONTEXT),
            Some(PCMD_ENPROFILE)
        );
        profiler.note_access(0, 3);
        profiler.note_access(0, 3);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0 + 12, TEST_CONTEXT),
            Some(2)
        );
        // Writing 1 to CLEAR resets all the SL2PWSCNT registers (section 7.3.4).
        profiler.write(
            PROFILER_BASE + PROFILER_PCMD,
            PCMD_ENPROFILE | PCMD_CLEAR,
            TEST_CONTEXT,
        );
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0 + 12, TEST_CONTEXT),
            Some(0)
        );
        // Disabling stops counting again.
        profiler.write(PROFILER_BASE + PROFILER_PCMD, 0, TEST_CONTEXT);
        profiler.note_access(0, 3);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0 + 12, TEST_CONTEXT),
            Some(0)
        );
    }

    #[test]
    fn each_wait_state_count_lands_in_its_own_bin_and_seven_absorbs_the_rest() {
        // SPRUEG5D Table 15: count values 0 to 6 are their own statistics event, and count 7 is
        // "0 or more wait state" because the SPCL counter stops at its maximum (section 7.1.3).
        let mut profiler = SharedMemoryProfiler::new(0);
        profiler.write(PROFILER_BASE + PROFILER_PCMD, PCMD_ENPROFILE, TEST_CONTEXT);
        for wait_states in 0..WAIT_STATE_BINS {
            profiler.note_access(0, wait_states);
        }
        profiler.note_access(0, 12);
        for bin in 0..WAIT_STATE_BINS as u32 - 1 {
            assert_eq!(
                profiler.read(PROFILER_BASE + PROFILER_PWSCNT0 + 4 * bin, TEST_CONTEXT),
                Some(1)
            );
        }
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0 + 28, TEST_CONTEXT),
            Some(2)
        );
    }

    #[test]
    fn the_bank_mask_filters_statistics_events() {
        // SPRUEG5D sections 7.1.4 and 7.3.1: the SBML rejects statistics events from a bank whose
        // SL2BNKMSK bit is 0 before they reach the SSUL.
        let mut profiler = SharedMemoryProfiler::new(0);
        profiler.write(PROFILER_BASE + PROFILER_PCMD, PCMD_ENPROFILE, TEST_CONTEXT);
        profiler.write(PROFILER_BASE + PROFILER_BNKMSK, 1 << 2, TEST_CONTEXT);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_BNKMSK, TEST_CONTEXT),
            Some(1 << 2)
        );
        profiler.note_access(0, 0);
        profiler.note_access(3, 0);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0, TEST_CONTEXT),
            Some(0)
        );
        profiler.note_access(2, 0);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0, TEST_CONTEXT),
            Some(1)
        );
        // SL2PFCCNT counts prefetches the SMC generated, and nothing prefetches here.
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PFCCNT, TEST_CONTEXT),
            Some(0)
        );
    }

    #[test]
    fn a_saturated_counter_stops_and_reports_itself_in_pstat() {
        // SPRUEG5D section 7.3.5: SL2PSTAT reports the saturation of each SL2PWSCNT in SATn. PSAT,
        // the SL2PFCCNT bit, has nothing to report: nothing prefetches, so that counter is 0.
        let mut profiler = SharedMemoryProfiler::new(0);
        profiler.write(PROFILER_BASE + PROFILER_PCMD, PCMD_ENPROFILE, TEST_CONTEXT);
        profiler.wait_state_count[5] = u32::MAX - 1;
        profiler.note_access(0, 5);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0 + 20, TEST_CONTEXT),
            Some(u32::MAX)
        );
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PSTAT, TEST_CONTEXT),
            Some(1 << 5)
        );
        // Saturated counters hold their maximum instead of wrapping.
        profiler.note_access(0, 5);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PWSCNT0 + 20, TEST_CONTEXT),
            Some(u32::MAX)
        );
        // SL2PSTAT is read-only and CLEAR takes the counters, and so the status, back to 0.
        assert_ne!(
            profiler.write(PROFILER_BASE + PROFILER_PSTAT, 0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PSTAT, TEST_CONTEXT),
            Some(1 << 5)
        );
        profiler.write(
            PROFILER_BASE + PROFILER_PCMD,
            PCMD_ENPROFILE | PCMD_CLEAR,
            TEST_CONTEXT,
        );
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_PSTAT, TEST_CONTEXT),
            Some(0)
        );
    }

    #[test]
    fn the_masked_statistics_events_combine_into_one_consolidated_event() {
        // SPRUEG5D section 7.3.6: the eight statistics events are combined into the single SMC
        // profiler event, and SL2STATMASK selects which of them are combined. That event is
        // SPRS612G Table 7-10 event 64, SMCPEVT.
        let mut profiler = SharedMemoryProfiler::new(0);
        profiler.write(PROFILER_BASE + PROFILER_PCMD, PCMD_ENPROFILE, TEST_CONTEXT);
        // Nothing is combined at the reset mask of 0.
        profiler.note_access(0, 2);
        assert!(!profiler.event);
        profiler.write(PROFILER_BASE + PROFILER_STATMASK, 1 << 2, TEST_CONTEXT);
        assert_eq!(
            profiler.read(PROFILER_BASE + PROFILER_STATMASK, TEST_CONTEXT),
            Some(1 << 2)
        );
        profiler.note_access(0, 1);
        assert!(!profiler.event);
        profiler.note_access(0, 2);
        assert!(profiler.event);
        assert!(profiler.take_event());
        assert!(!profiler.take_event());
        // A masked-off bank produces no event either, because the SBML drops the request first.
        profiler.write(PROFILER_BASE + PROFILER_BNKMSK, 1, TEST_CONTEXT);
        profiler.note_access(1, 2);
        assert!(!profiler.event);
    }
}
