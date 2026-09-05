//! The memory hierarchy: the L1P, L1D and L2 caches, the MAR cacheability bits, the cache
//! configuration registers, and the storage behind them.
//!
//! One place for the rules that produced the codebase's most recent defect history: the
//! associativity of each cache, the write buffer, the miss costs, and the snoop protocol
//! (L2 SRAM is snooped against L1D, nothing else is, and L1P is never snooped at all -
//! SPRU862B chapter 2).
//!
//! The split between `MemoryHierarchy` and `Backing` is what makes a multicore device
//! expressible. A cache belongs to one megamodule and is tagged by the address that
//! megamodule issued; the storage behind it belongs to the device, because every core's L2
//! SRAM has a global address any other master can reach (SPRS612G Table 2-2). So each core
//! owns a `MemoryHierarchy` - three caches, its MAR bits, its cache configuration - and the
//! device owns one `Backing`, which holds every megamodule's SRAM, the shared L2 RAM and ROM,
//! the DDR2 window and everything sparse, and resolves local aliases through the
//! `AddressMap`. The peripheral fabric in `system.rs` holds both and hands the pair to every
//! access.

use crate::cache::{Cache, CacheView, EvictedLine};
use crate::space::{AddressMap, Placement, Requester};
use crate::system::Endian;
use std::collections::BTreeMap;

const DDR2_PAGE_BYTES: usize = 64 * 1024;
const DDR2_PAGE_SHIFT: u32 = 16;

pub(crate) const L2CFG: u32 = 0x0184_0000;
pub(crate) const L1PCFG: u32 = 0x0184_0020;
const L1PCC: u32 = 0x0184_0024;
pub(crate) const L1DCFG: u32 = 0x0184_0040;
const L1DCC: u32 = 0x0184_0044;
pub(crate) const L2WBAR: u32 = 0x0184_4000;
pub(crate) const L2WWC: u32 = 0x0184_4004;
const L2WIBAR: u32 = 0x0184_4010;
const L2WIWC: u32 = 0x0184_4014;
const L2IBAR: u32 = 0x0184_4018;
const L2IWC: u32 = 0x0184_401c;
const L1PIBAR: u32 = 0x0184_4020;
const L1PIWC: u32 = 0x0184_4024;
const L1DWIBAR: u32 = 0x0184_4030;
const L1DWIWC: u32 = 0x0184_4034;
const L1DWBAR: u32 = 0x0184_4040;
const L1DWWC: u32 = 0x0184_4044;
const L1DIBAR: u32 = 0x0184_4048;
const L1DIWC: u32 = 0x0184_404c;
pub(crate) const L2WB: u32 = 0x0184_5000;
const L2WBINV: u32 = 0x0184_5004;
const L2INV: u32 = 0x0184_5008;
const L1PINV: u32 = 0x0184_5028;
const L1DWB: u32 = 0x0184_5040;
const L1DWBINV: u32 = 0x0184_5044;
pub(crate) const L1DINV: u32 = 0x0184_5048;
pub(crate) const MAR_BASE: u32 = 0x0184_8000;

/// Whether an address is ordinary memory rather than a peripheral configuration space or a
/// peripheral data port. A DMA master reaches the port rather than memory when this is false.
pub(crate) fn is_memory(address: u32) -> bool {
    !(0x0180_0000..0x0400_0000).contains(&address) && !is_data_port(address)
}

/// The peripheral data ports that live outside the configuration space, so a DMA master reaches
/// the port rather than memory: the UTOPIA cell queues and the two McBSP data buses (SPRS276M
/// Tables 7-105, 7-57 and 7-58).
fn is_data_port(address: u32) -> bool {
    (0x3c00_0000..0x3c00_0800).contains(&address)
        || (0x3000_0000..0x3000_0014).contains(&address)
        || (0x3400_0000..0x3400_0014).contains(&address)
}

/// L1PCFG.MODE and L1DCFG.MODE: 4 KB of cache per step up to the 32 KB both caches stop at,
/// which is all the L1 memory a C64x+ megamodule has. Every encoding above 4 is that maximum,
/// which is how TI's own kernel turns the L1 caches on - `L1_cache_on` writes 7 to both
/// registers (Linux v5.10 `arch/c6x/platforms/cache.c:228`,
/// <https://github.com/torvalds/linux/blob/v5.10/arch/c6x/platforms/cache.c#L228>), and reading
/// those encodings as reserved left it running out of L2 with no L1 cache at all.
fn l1_capacity(mode: u32) -> usize {
    match mode & 7 {
        0 => 0,
        1 => 4 * 1024,
        2 => 8 * 1024,
        3 => 16 * 1024,
        _ => 32 * 1024,
    }
}

/// L2CFG.L2MODE: 32 KB of cache per step up to the 256 KB maximum every C64x+ megamodule stops
/// at, whatever the size of its L2 SRAM. The C6472's 608 KB L2 does not extend that limit - it
/// only leaves more SRAM behind it (SPRS612G section 5.1, SPRU871K Table 4-4). The encodings
/// above 4 are the maximum too, which is not a guess: TI's own kernel writes 7 for it -
/// `L2MODE_256K_CACHE 0x7` in Linux v5.10 `arch/c6x/platforms/cache.c:94`
/// (<https://github.com/torvalds/linux/blob/v5.10/arch/c6x/platforms/cache.c#L94>), set at
/// boot by `L2_cache_set_mode` under the comment "Set L2 caches on the the whole L2 SRAM
/// memory". Reading 7 as a reserved encoding left Linux running with no L2 cache at all.
fn l2_capacity(mode: u32) -> usize {
    match mode & 7 {
        0 => 0,
        1 => 32 * 1024,
        2 => 64 * 1024,
        3 => 128 * 1024,
        _ => 256 * 1024,
    }
}

/// How much of a local window is still SRAM once part of it has been converted into cache.
///
/// The conversion runs from the top of the window downwards: SPRU862B section 2.3 says of L2
/// that "when the L2 cache size is increased, the memory is taken from the high memory
/// addresses" and its Figure 2-1 places a 256 KB cache in a 2048 KB L2 at 009C 0000h, while
/// SPRU871K section 2.4 says L1P "converts memory from RAM to cache by starting at the top of
/// the L1P memory map and working downwards" and section 3.3 says the same of L1D. So the SRAM
/// that remains is always a prefix of the window, and a cache as large as the window - which is
/// what the C6455's 32 KB of L1P and L1D reach at their maximum size - leaves none of it.
fn sram_remaining(window: u32, cache: usize) -> u32 {
    window.saturating_sub(cache as u32)
}

/// What one data access tells the timing model.
///
/// The three facts travel together because the CPU charges them together: whether L1D serves the
/// access without a fill, whether a read of it would need one, and where the line the fill would
/// bring in comes from. Asking them apart let two consumers probe the cache at two points in the
/// cycle for the same line - the write buffer at the store, the miss cost at the end of the cycle
/// - and a correction to what counts as L2-served had to be made in both places.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct AccessTiming {
    /// Serviced by L1D without a line fill, which is what a bank conflict needs.
    pub(crate) resident: bool,
    /// A read of this address needs an L1D line fill. Writes never allocate: L1D is
    /// read-allocate and a write miss goes to the write buffer instead.
    pub(crate) read_miss: bool,
    /// When a fill is needed, whether its line comes out of the L2 cache rather than L2 SRAM,
    /// which SPRU862B Tables 1-3 and 3-2 both cost separately and which SPRU871K section 3.5.3
    /// also makes the write buffer's merge rule turn on. A line the L2 cache misses comes from
    /// external memory, whose latency neither table quantifies; it is charged as an L2 cache line
    /// rather than given an invented number. False when no fill is needed: an access that costs
    /// no line has no source, and both consumers only ask having seen `read_miss`.
    pub(crate) from_l2_cache: bool,
}

/// One L1P line fill and where it came from, reported to the fetch path's stall accounting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct LineFill {
    pub(crate) line: u32,
    pub(crate) from_l2_cache: bool,
}

/// How much of one megamodule's L2, L1P and L1D windows is still addressable SRAM.
#[derive(Clone, Copy)]
struct CoreSram {
    l2: u32,
    l1p: u32,
    l1d: u32,
}

/// The storage every master shares: each megamodule's SRAM, the shared L2 RAM and ROM, the
/// DDR2 window and everything sparse. Addresses arrive as the requester issued them and are
/// resolved through the device's address map, so a core's local window and the global alias of
/// the same memory land on the same bytes.
pub(crate) struct Backing {
    map: &'static AddressMap,
    endian: Endian,
    /// The base of the local L2 SRAM window, held here so the hottest access in the emulator -
    /// a load or store into the core's own L2 - is one subtraction and one bounds check against
    /// that core's remaining SRAM rather than a walk of the whole address map.
    l2_base: u32,
    /// Whether a DMA master's address resolves through the local windows, which it does on a
    /// device with no global aliases because there is only one megamodule to mean.
    dma_is_local: bool,
    /// Every core's L2, L1P and L1D SRAM in one flat array, `core_stride` bytes per core in
    /// that order. One allocation and one bounds check: this is the hottest memory in the
    /// emulator, because it is where code, stacks and most buffers live.
    core_sram: Box<[u8]>,
    core_stride: usize,
    /// How much of each core's three local windows is still SRAM. This is what makes the
    /// address map a function of the cache configuration: every master resolves an address
    /// here, so memory a megamodule has converted into cache is gone for the CPU, for a DMA
    /// master and for an image loader alike.
    sram: Box<[CoreSram]>,
    /// The shared L2 RAM and ROM of a multicore device; empty on a single-core one.
    shared_ram: Box<[u8]>,
    shared_rom: Box<[u8]>,
    /// The DDR2 window, dense in 64 KiB pages allocated on first touch. The NIMU DMA buffers
    /// of a network stack live here, so the per-byte tree path would cost a B-tree walk on
    /// every transferred byte.
    dram: Vec<Option<Box<[u8; DDR2_PAGE_BYTES]>>>,
    /// Everything with no dense home: external asynchronous memory, and addresses no memory
    /// answers at all.
    sparse: BTreeMap<u32, u8>,
    /// The version stamp each written word carries, which the linked-atomic instructions
    /// compare to detect an intervening store.
    memory_versions: BTreeMap<u32, u64>,
    write_serial: u64,
}

impl Backing {
    pub(crate) fn new(map: &'static AddressMap, endian: Endian) -> Self {
        let (l2_base, _) = map.local_l2();
        let core_stride = (map.l2_bytes() + map.l1p_bytes() + map.l1d_bytes()) as usize;
        Self {
            map,
            endian,
            l2_base,
            dma_is_local: !map.has_global_aliases(),
            core_sram: vec![0u8; core_stride * map.cores].into_boxed_slice(),
            core_stride,
            // Every cache comes out of reset disabled, so all of every window is SRAM
            // (SPRU871K sections 1.2.2 to 1.2.4: L2 is "always initiated to all SRAM after
            // reset", and the L1 reset state is per device - the C6455 boots all SRAM).
            sram: vec![
                CoreSram {
                    l2: map.l2_bytes(),
                    l1p: map.l1p_bytes(),
                    l1d: map.l1d_bytes(),
                };
                map.cores
            ]
            .into_boxed_slice(),
            shared_ram: vec![0u8; map.shared_ram_bytes() as usize].into_boxed_slice(),
            shared_rom: vec![0u8; map.shared_rom_bytes() as usize].into_boxed_slice(),
            dram: vec![None; map.dram_bytes() as usize / DDR2_PAGE_BYTES],
            sparse: BTreeMap::new(),
            memory_versions: BTreeMap::new(),
            write_serial: 0,
        }
    }

    pub(crate) fn set_endian(&mut self, endian: Endian) {
        self.endian = endian;
    }

    pub(crate) fn endian(&self) -> Endian {
        self.endian
    }

    pub(crate) fn endian_big(&self) -> bool {
        self.endian == Endian::Big
    }

    pub(crate) fn memory_version(&self, address: u32) -> u64 {
        self.memory_versions
            .get(&(address & !3))
            .copied()
            .unwrap_or(0)
    }

    pub(crate) fn track_memory_version(&mut self, address: u32) -> u64 {
        *self.memory_versions.entry(address & !3).or_insert(0)
    }

    /// How much of a core's three windows the cache configuration has left as SRAM. The
    /// megamodule reports it here on every configuration change, because the caches belong to
    /// one megamodule while the storage behind them belongs to the device.
    ///
    /// What is now cache also stops holding what was written to it, because the cache is using
    /// those bits for its lines and tags. SPRU871K section 2.4.3 names the consequence - "data
    /// loss due to portions of L1P RAM becoming cache" - and SPRU862B Table 2-6 asks software
    /// to copy anything it still needs out of those addresses before growing the cache. So the
    /// model loses it rather than keeping it safe behind the aperture: firmware that skips that
    /// copy has to see the loss here as it would on silicon, and shrinking the cache again
    /// hands back memory that reads as nothing rather than as what used to be there.
    pub(crate) fn set_core_sram(&mut self, core: usize, l2: u32, l1p: u32, l1d: u32) {
        self.sram[core] = CoreSram { l2, l1p, l1d };
        let base = core * self.core_stride;
        let l1p_base = base + self.map.l2_bytes() as usize;
        let l1d_base = l1p_base + self.map.l1p_bytes() as usize;
        self.core_sram[base + l2 as usize..l1p_base].fill(0);
        self.core_sram[l1p_base + l1p as usize..l1d_base].fill(0);
        self.core_sram[l1d_base + l1d as usize..base + self.core_stride].fill(0);
    }

    /// Where an access lands, with the cache configuration applied: the address map resolves the
    /// requester's aliases, and a local offset past the SRAM its megamodule has left is memory
    /// that is now cache. Every master goes through this, which is what keeps one answer for the
    /// CPU, the DMA masters, the snoop path and the image loader.
    #[inline]
    pub(crate) fn placement(&self, requester: Requester, address: u32) -> Placement {
        let placement = self.map.placement(requester, address);
        match placement {
            Placement::CoreL2 { core, offset } if offset >= self.sram[core].l2 => {
                Placement::ConvertedToCache
            }
            Placement::CoreL1P { core, offset } if offset >= self.sram[core].l1p => {
                Placement::ConvertedToCache
            }
            Placement::CoreL1D { core, offset } if offset >= self.sram[core].l1d => {
                Placement::ConvertedToCache
            }
            _ => placement,
        }
    }

    /// Where a core's L2, L1P or L1D SRAM starts in the flat array.
    #[inline]
    fn core_l2_index(&self, core: usize, offset: u32) -> usize {
        core * self.core_stride + offset as usize
    }

    #[inline]
    fn core_l1p_index(&self, core: usize, offset: u32) -> usize {
        core * self.core_stride + self.map.l2_bytes() as usize + offset as usize
    }

    #[inline]
    fn core_l1d_index(&self, core: usize, offset: u32) -> usize {
        core * self.core_stride
            + (self.map.l2_bytes() + self.map.l1p_bytes()) as usize
            + offset as usize
    }

    /// The dense slice a placement names, together with the offset into it, or `None` when the
    /// access is sparse or falls in the read-only ROM.
    fn dense(&self, placement: Placement) -> Option<(&[u8], usize)> {
        match placement {
            Placement::CoreL2 { core, offset } => {
                Some((&self.core_sram, self.core_l2_index(core, offset)))
            }
            Placement::CoreL1P { core, offset } => {
                Some((&self.core_sram, self.core_l1p_index(core, offset)))
            }
            Placement::CoreL1D { core, offset } => {
                Some((&self.core_sram, self.core_l1d_index(core, offset)))
            }
            Placement::Shared { offset } => Some((&self.shared_ram, offset as usize)),
            Placement::SharedRom { offset } => Some((&self.shared_rom, offset as usize)),
            Placement::Dram { offset } => {
                let page = self.dram[(offset >> DDR2_PAGE_SHIFT) as usize].as_deref()?;
                Some((page.as_slice(), (offset as usize) & (DDR2_PAGE_BYTES - 1)))
            }
            Placement::ConvertedToCache | Placement::Sparse => None,
        }
    }

    fn dense_mut(&mut self, placement: Placement) -> Option<(&mut [u8], usize)> {
        match placement {
            Placement::CoreL2 { core, offset } => {
                let index = self.core_l2_index(core, offset);
                Some((&mut self.core_sram, index))
            }
            Placement::CoreL1P { core, offset } => {
                let index = self.core_l1p_index(core, offset);
                Some((&mut self.core_sram, index))
            }
            Placement::CoreL1D { core, offset } => {
                let index = self.core_l1d_index(core, offset);
                Some((&mut self.core_sram, index))
            }
            Placement::Shared { offset } => Some((&mut self.shared_ram, offset as usize)),
            // The shared L2 ROM ignores writes, as a ROM does.
            Placement::SharedRom { .. } => None,
            Placement::Dram { offset } => {
                let index = (offset >> DDR2_PAGE_SHIFT) as usize;
                let page = self.dram[index].get_or_insert_with(|| Box::new([0; DDR2_PAGE_BYTES]));
                Some((
                    page.as_mut_slice(),
                    (offset as usize) & (DDR2_PAGE_BYTES - 1),
                ))
            }
            Placement::ConvertedToCache | Placement::Sparse => None,
        }
    }

    /// How many bytes from `address` stay inside the memory its placement named, so a bulk
    /// transfer can copy a run at a time instead of resolving every byte.
    fn run_length(&self, placement: Placement, remaining: usize) -> usize {
        let available = match placement {
            Placement::CoreL2 { core, offset } => {
                (self.sram[core].l2 as usize).saturating_sub(offset as usize)
            }
            Placement::CoreL1P { core, offset } => {
                (self.sram[core].l1p as usize).saturating_sub(offset as usize)
            }
            Placement::CoreL1D { core, offset } => {
                (self.sram[core].l1d as usize).saturating_sub(offset as usize)
            }
            Placement::Shared { offset } => self.shared_ram.len().saturating_sub(offset as usize),
            Placement::SharedRom { offset } => {
                self.shared_rom.len().saturating_sub(offset as usize)
            }
            Placement::Dram { offset } => {
                DDR2_PAGE_BYTES - ((offset as usize) & (DDR2_PAGE_BYTES - 1))
            }
            Placement::ConvertedToCache | Placement::Sparse => 1,
        };
        available.min(remaining).max(1)
    }

    /// The core whose local windows this requester resolves through, if any.
    #[inline]
    fn local_core(&self, requester: Requester) -> Option<usize> {
        match requester {
            Requester::Core(core) => Some(core),
            Requester::Dma => self.dma_is_local.then_some(0),
        }
    }

    /// The index in the flat array of a run that stays inside the requester's own L2 SRAM,
    /// which is where code, stacks and most buffers live: one bounds check and one copy instead
    /// of a walk of the address map. A run reaching past the SRAM the core has left takes the
    /// slow path, where the placement names those bytes as memory converted into cache.
    #[inline]
    fn local_l2_run(&self, requester: Requester, address: u32, length: usize) -> Option<usize> {
        let offset = address.wrapping_sub(self.l2_base);
        let core = self.local_core(requester)?;
        let sram = self.sram[core].l2;
        (offset < sram && (sram - offset) as usize >= length)
            .then(|| core * self.core_stride + offset as usize)
    }

    #[inline]
    fn local_l2(&self, requester: Requester, address: u32) -> Option<usize> {
        self.local_l2_run(requester, address, 1)
    }

    #[inline]
    pub(crate) fn read_byte(&self, requester: Requester, address: u32) -> Option<u8> {
        if let Some(index) = self.local_l2(requester, address) {
            return Some(self.core_sram[index]);
        }
        let placement = self.placement(requester, address);
        match self.dense(placement) {
            Some((slice, offset)) => Some(slice[offset]),
            None if placement == Placement::Sparse => self.sparse.get(&address).copied(),
            // A DDR2 page that has never been touched reads as zero without allocating, and so
            // does SRAM that is now cache.
            None => Some(0),
        }
    }

    #[inline]
    pub(crate) fn write_byte(&mut self, requester: Requester, address: u32, value: u8) {
        if let Some(index) = self.local_l2(requester, address) {
            self.core_sram[index] = value;
            return;
        }
        let placement = self.placement(requester, address);
        match self.dense_mut(placement) {
            Some((slice, offset)) => slice[offset] = value,
            None if placement == Placement::Sparse => {
                self.sparse.insert(address, value);
            }
            None => {}
        }
    }

    pub(crate) fn read_into(&self, requester: Requester, address: u32, data: &mut [u8]) {
        // A transfer that stays inside the core's own L2 - a cache line fill, a descriptor, a
        // packet buffer - is one copy.
        if let Some(index) = self.local_l2_run(requester, address, data.len()) {
            data.copy_from_slice(&self.core_sram[index..index + data.len()]);
            return;
        }
        let mut done = 0usize;
        while done < data.len() {
            let at = address.wrapping_add(done as u32);
            let placement = self.placement(requester, at);
            let run = self.run_length(placement, data.len() - done);
            match self.dense(placement) {
                Some((slice, offset)) => {
                    data[done..done + run].copy_from_slice(&slice[offset..offset + run]);
                }
                None if placement == Placement::Sparse => {
                    for index in 0..run {
                        let byte = at.wrapping_add(index as u32);
                        data[done + index] = self.sparse.get(&byte).copied().unwrap_or(0);
                    }
                }
                None => data[done..done + run].fill(0),
            }
            done += run;
        }
    }

    pub(crate) fn read_bytes(&self, requester: Requester, address: u32, length: usize) -> Vec<u8> {
        let mut data = vec![0u8; length];
        self.read_into(requester, address, &mut data);
        data
    }

    pub(crate) fn write_bytes(&mut self, requester: Requester, address: u32, bytes: &[u8]) {
        if let Some(index) = self.local_l2_run(requester, address, bytes.len()) {
            self.core_sram[index..index + bytes.len()].copy_from_slice(bytes);
            return;
        }
        let mut done = 0usize;
        while done < bytes.len() {
            let at = address.wrapping_add(done as u32);
            let placement = self.placement(requester, at);
            let run = self.run_length(placement, bytes.len() - done);
            match self.dense_mut(placement) {
                Some((slice, offset)) => {
                    slice[offset..offset + run].copy_from_slice(&bytes[done..done + run]);
                }
                None if placement == Placement::Sparse => {
                    for index in 0..run {
                        let byte = at.wrapping_add(index as u32);
                        self.sparse.insert(byte, bytes[done + index]);
                    }
                }
                None => {}
            }
            done += run;
        }
    }

    /// Load the shared L2 ROM, which is the only way its contents ever change: the boot
    /// controller starts core 0 inside it (SPRS612G Table 2-4).
    pub(crate) fn note_memory_write(&mut self, address: u32, length: usize) {
        if length == 0 || self.memory_versions.is_empty() {
            return;
        }
        let first = address & !3;
        let last = address.wrapping_add((length - 1).min(u32::MAX as usize) as u32) & !3;
        let mut word = first;
        let mut serial = None;
        loop {
            if let Some(version) = self.memory_versions.get_mut(&word) {
                let current = *serial.get_or_insert_with(|| {
                    self.write_serial = self.write_serial.wrapping_add(1).max(1);
                    self.write_serial
                });
                *version = current;
            }
            if word == last {
                break;
            }
            word = word.wrapping_add(4);
        }
    }
}

/// The cache side of one megamodule: its three caches, its MAR bits and its cache
/// configuration and maintenance registers.
pub(crate) struct MemoryHierarchy {
    /// Which megamodule this is, so an access can be resolved through the local windows that
    /// belong to it.
    core: usize,
    map: &'static AddressMap,
    /// This megamodule's own L1P and L1D memory, by every address this core can name it with:
    /// the two local windows, and their aliases inside this core's own global alias block on a
    /// device that has one. No cache in the megamodule may hold them (SPRU871K section 2.3.1 -
    /// "L1P memory can not be cached within Level 1 data (L1D) cache, Level 1 program (L1P)
    /// cache, or Level 2 (L2) cache within the same megamodule" - and section 3.2.1, which says
    /// the same of L1D memory). Another core's L1 through the global block is ordinary memory
    /// to this one, so only this core's own aliases are here.
    ///
    /// Held as the high bits of each window rather than as a base and a length, because every
    /// data access asks and a shift with four compares is measurably cheaper than four
    /// subtractions with four compares: both L1 windows are a power-of-two size aligned to it,
    /// so an address is inside one exactly when its top bits match. `u32::MAX` is the absent
    /// entry, which no address can produce for a shift of at least one.
    own_l1_shift: u32,
    own_l1_keys: [u32; 4],
    /// The memory protection attribute bits, one byte per 16 MB region. Bit 0 is the
    /// cacheability bit for L1D and L2; L1P caches external code regardless (SPRU862B 2.3).
    mar: [u32; 256],
    l1p_cfg: u32,
    l1d_cfg: u32,
    l2_cfg: u32,
    l1p_cc: u32,
    l1d_cc: u32,
    l1p: Cache,
    /// Invalidates the CPU's one-packet fetch buffer when software changes L1P state.
    l1p_epoch: u64,
    /// The L1P line fills since the fetch path last drained them, each with where the line came
    /// from, which is what the per-packet miss stalls charge on.
    l1p_fills: Vec<LineFill>,
    /// A dirty line left the victim buffer since the last drain, which disrupts miss
    /// pipelining (SPRU862B Table 3-2).
    l1d_victim_flush: bool,
    l1d: Cache,
    l2: Cache,
    /// The base addresses of the seven range cache operations.
    cache_bars: [u32; 7],
}

impl MemoryHierarchy {
    pub(crate) fn new(core: usize, map: &'static AddressMap) -> Self {
        let mut mar = [0; 256];
        mar[0] = 1;
        let [l1p, l1d] = map.local_l1();
        assert_eq!(l1p.1, l1d.1, "the two L1 windows share one shift");
        assert!(l1p.1.is_power_of_two());
        assert_eq!(l1p.0 % l1p.1, 0);
        assert_eq!(l1d.0 % l1d.1, 0);
        let own_l1_shift = l1p.1.trailing_zeros();
        let alias = map.global_alias_base(core);
        let own_l1_keys = [
            l1p.0 >> own_l1_shift,
            l1d.0 >> own_l1_shift,
            alias.map_or(u32::MAX, |base| (base + l1p.0) >> own_l1_shift),
            alias.map_or(u32::MAX, |base| (base + l1d.0) >> own_l1_shift),
        ];
        Self {
            core,
            map,
            own_l1_shift,
            own_l1_keys,
            mar,
            l1p_cfg: 0,
            l1d_cfg: 0,
            l2_cfg: 0,
            l1p_cc: 0,
            l1d_cc: 0,
            l1p_epoch: 0,
            // L1P is direct-mapped, L1D two-way LRU and L2 four-way (SPRU862B Table 1-2 and
            // section 1.8, SPRS276M section 4).
            l1p: Cache::new(32, 1),
            l1p_fills: Vec::new(),
            l1d_victim_flush: false,
            l1d: Cache::new(64, 2),
            l2: Cache::new(128, 4),
            cache_bars: [0; 7],
        }
    }
    #[inline]
    fn requester(&self) -> Requester {
        Requester::Core(self.core)
    }
    pub(crate) fn cache_views(&self) -> [CacheView; 3] {
        [
            self.l1p.view("L1P"),
            self.l1d.view("L1D"),
            self.l2.view("L2"),
        ]
    }
    /// One resident line for passive debugger inspection. Reading it must not affect LRU state or
    /// cache statistics.
    pub(crate) fn cache_line_data(&self, level: usize, address: u32) -> Option<&[u8]> {
        match level {
            0 => &self.l1p,
            1 => &self.l1d,
            2 => &self.l2,
            _ => return None,
        }
        .line_data(address)
    }
    /// The 256 MAR registers; bit 0 of each enables caching for its 16 MiB segment.
    pub(crate) fn mar_registers(&self) -> [u32; 256] {
        self.mar
    }
    /// Whether the L1D fills since this was last drained flushed a dirty line out of the victim
    /// buffer. SPRU862B Table 3-2 says such a flush "disrupts miss pipelining" and adds up to ten
    /// stalls; the disruption is modelled because it needs no number, the extra stalls are not.
    pub(crate) fn take_l1d_victim_flush(&mut self) -> bool {
        std::mem::take(&mut self.l1d_victim_flush)
    }
    /// Everything one data access tells the timing model, answered once for the address the
    /// access issued. Only the SRAM the L1D configuration has left answers in one cycle; the rest
    /// of the window is the cache itself.
    ///
    /// The source travels with the access instead of being re-derived later in the cycle: the
    /// write buffer's merge rule needs it for a store L1D holds no line for, and the miss cost
    /// needs it for a read that needs one, and each used to probe the cache again for a
    /// masked-off form of this address. It is a property of the 16 MB MAR region and of the local
    /// L1 windows, so a byte, its quad-word and its line all answer alike - which is what lets
    /// one answer serve both. Resolving it only when a fill is needed keeps the cost where it
    /// already was: a hit paid nothing before and pays nothing now.
    #[inline]
    pub(crate) fn access_timing(&self, address: u32) -> AccessTiming {
        if let Placement::CoreL1D { offset, .. } = self.map.placement(self.requester(), address) {
            if offset < sram_remaining(self.map.l1d_bytes(), l1_capacity(self.l1d_cfg)) {
                return AccessTiming {
                    resident: true,
                    read_miss: false,
                    from_l2_cache: false,
                };
            }
        }
        let resident = self.l1d.contains(address);
        let read_miss = !resident && self.l1d.enabled() && self.l1_cacheable(address);
        AccessTiming {
            resident,
            read_miss,
            from_l2_cache: read_miss && self.l2_cacheable(address),
        }
    }
    pub(crate) fn fetch_reusable(&self, address: u32) -> bool {
        self.l1p.enabled() && self.l1p_cacheable(address)
    }
    pub(crate) fn l1p_epoch(&self) -> u64 {
        self.l1p_epoch
    }
    /// Whether this address is this megamodule's own L1P or L1D memory, which none of its three
    /// caches may hold - see `own_l1` for the two sentences that say so. It covers the whole
    /// window, cache-converted part included: an address the L1D configuration turned into
    /// cache is not memory at all, so caching it would answer a read the storage refuses.
    #[inline]
    fn own_l1(&self, address: u32) -> bool {
        let key = address >> self.own_l1_shift;
        self.own_l1_keys.contains(&key)
    }
    /// Whether L1P caches instructions from this address. The MAR bits do not gate it: SPRU862B
    /// section 2.3 says that on C64x+ devices "program fetches to external memory addresses are
    /// always cached in L1P, regardless of the cacheability setting", unlike C64x where MAR
    /// governed all three caches. Since L1P is never snooped, that is also how external code
    /// modified behind the CPU's back goes stale with MAR left clear.
    #[inline]
    fn l1p_cacheable(&self, address: u32) -> bool {
        is_memory(address) && !self.own_l1(address)
    }
    #[inline]
    fn l1_cacheable(&self, address: u32) -> bool {
        is_memory(address) && self.mar[(address >> 24) as usize] & 1 != 0 && !self.own_l1(address)
    }
    #[inline]
    fn l2_cacheable(&self, address: u32) -> bool {
        if !self.l2.enabled() {
            return false;
        }
        let mar = (address >> 24) as usize;
        // The MAR range already keeps local memory out of L2 through the local windows, which
        // sit in region 0; the global alias block does not, so this core's own L1 is named
        // again there.
        mar >= 16 && self.mar[mar] & 1 != 0 && !self.own_l1(address)
    }
    /// The L1P line fills the program-fetch path performed since this was last drained, each with
    /// the memory that served it. SPRU862B Table 1-3 gives the average stalls per execute packet
    /// for a straight-line miss stream at each packet width, over two L2 configurations, and one
    /// occupancy per line explains all thirty-two numbers: with the fill of the next line
    /// overlapped against execution of the current one, a width-n packet consumes a 32-byte line
    /// in 8/n cycles and so stalls `max(0, k - 8/n)` per line, which is `max(0, k*n/8 - 1)` per
    /// packet. The C6455's L2 is 2048K at 0080 0000h with 2-cycle latency in a 4 x 128-bit bank
    /// configuration (SPRS276M section 4), which is the table's 1-wait-state column - the same
    /// one the L1D read-miss numbers come from - and solving against it gives k = 14/3 for a line
    /// out of L2 SRAM and k = 6 for one the L2 cache holds. The cost of a line the L2 cache
    /// misses is left at the cache figure: it comes from external memory, whose latency this
    /// table does not quantify, and an invented number would be worse than an understated one.
    ///
    /// Reporting the fills that happened, rather than answering a question about residency before
    /// the fetch, is what keeps the accounting exact: a C64x+ execute packet may span two fetch
    /// packets, so the fetch of one packet can fill two lines.
    pub(crate) fn drain_l1p_fills(&mut self) -> std::vec::Drain<'_, LineFill> {
        self.l1p_fills.drain(..)
    }
    pub(crate) fn cache_register_read(&self, address: u32) -> Option<u32> {
        let value = match address {
            L2CFG => self.l2_cfg,
            L1PCFG => self.l1p_cfg,
            L1DCFG => self.l1d_cfg,
            L1PCC => self.l1p_cc,
            L1DCC => self.l1d_cc,
            L2WBAR => self.cache_bars[0],
            L2WIBAR => self.cache_bars[1],
            L2IBAR => self.cache_bars[2],
            L1PIBAR => self.cache_bars[3],
            L1DWIBAR => self.cache_bars[4],
            L1DWBAR => self.cache_bars[5],
            L1DIBAR => self.cache_bars[6],
            L2WWC | L2WIWC | L2IWC | L1PIWC | L1DWIWC | L1DWWC | L1DIWC | L2WB | L2WBINV
            | L2INV | L1PINV | L1DWB | L1DWBINV | L1DINV => 0,
            _ => return None,
        };
        Some(value)
    }
    /// Apply this megamodule's L1D contents to bytes a master is reading out of memory. A dirty
    /// L1D line is forwarded to the master and stays dirty: the data is not written back on the
    /// way past (SPRU862B Figure 2-4). The caller decides which addresses are snooped at all -
    /// only L2 SRAM is - and passes each tag the line may carry, because a multicore device gives
    /// the same byte a local and a global address and the cache is tagged by whichever one the
    /// CPU used.
    pub(crate) fn snoop_read(&self, address: u32, data: &mut [u8]) {
        self.l1d.snoop_read_range(address, data);
    }
    /// Apply a master's write to this megamodule's L1D, so the CPU sees it without a cache
    /// operation. The write reaches L1D only: code a master puts in L2 SRAM keeps executing out
    /// of L1P until software invalidates it (SPRU862B sections 2.4 and 2.6).
    pub(crate) fn snoop_write(&mut self, address: u32, bytes: &[u8]) {
        self.l1d.snoop_write_range(address, bytes);
    }
    /// The debug port sees what the CPU wrote, so a line still held dirty in L1D or in the L2
    /// cache has not reached memory yet.
    pub(crate) fn host_snoop_read(&self, address: u32, data: &mut [u8]) {
        self.l2.snoop_read_range(address, data);
        self.l1d.snoop_read_range(address, data);
    }
    /// The MAR register at `address`, or `None` when the address is not one.
    pub(crate) fn mar_read(&self, address: u32) -> Option<u32> {
        mar_index(address).map(|index| self.mar[index])
    }
    /// Write the MAR register at `address`. Only the cacheability bit is writable, and only
    /// for the external regions the cache controller actually consults (index >= 16); the
    /// on-chip regions are read-only and always cacheable.
    pub(crate) fn mar_write(&mut self, address: u32, value: u32) -> bool {
        let Some(index) = mar_index(address) else {
            return false;
        };
        if index >= 16 {
            self.mar[index] = value & 1;
        }
        true
    }

    /// Bind this hierarchy to the storage behind it.
    ///
    /// The hierarchy holds no bytes - the backing store does, because every master on the device
    /// shares it - and binding the two here is what keeps that split out of the operations. A
    /// read, a write, a fetch and a cache maintenance name an address and nothing else, and no
    /// helper below this seam re-supplies the store.
    #[inline(always)]
    pub(crate) fn bind<'a>(&'a mut self, backing: &'a mut Backing) -> BoundHierarchy<'a> {
        BoundHierarchy {
            hierarchy: self,
            backing,
        }
    }
}

/// One megamodule's memory hierarchy bound to the storage behind it: the value every
/// operation that moves bytes is issued on.
///
/// It is two borrows and nothing else, so binding costs nothing and the fetch path keeps
/// the direct, inlined calls it had. What it buys is that the ownership split the device
/// needs - caches per megamodule, bytes shared by every master - stops being a parameter
/// that thirty-one functions have to agree about.
pub(crate) struct BoundHierarchy<'a> {
    hierarchy: &'a mut MemoryHierarchy,
    backing: &'a mut Backing,
}

impl BoundHierarchy<'_> {
    #[inline]
    pub(crate) fn read_u8(&mut self, address: u32) -> u8 {
        self.read_data_byte(address)
    }
    #[inline]
    pub(crate) fn write_u8(&mut self, address: u32, value: u8) {
        self.write_data_byte(address, value);
    }
    pub(crate) fn write_u16(&mut self, address: u32, value: u16) {
        let bytes = match self.backing.endian {
            Endian::Little => value.to_le_bytes(),
            Endian::Big => value.to_be_bytes(),
        };
        self.write_data_bytes(address, bytes);
    }
    pub(crate) fn read_u16(&mut self, address: u32) -> u16 {
        let bytes = self.read_data_bytes(address);
        match self.backing.endian {
            Endian::Little => u16::from_le_bytes(bytes),
            Endian::Big => u16::from_be_bytes(bytes),
        }
    }
    #[inline]
    pub(crate) fn read_u32(&mut self, address: u32) -> u32 {
        let bytes = self.read_data_bytes(address);
        match self.backing.endian {
            Endian::Little => u32::from_le_bytes(bytes),
            Endian::Big => u32::from_be_bytes(bytes),
        }
    }
    #[inline]
    pub(crate) fn write_u32(&mut self, address: u32, value: u32) {
        let bytes = match self.backing.endian {
            Endian::Little => value.to_le_bytes(),
            Endian::Big => value.to_be_bytes(),
        };
        self.write_data_bytes(address, bytes);
    }
    #[cfg(test)]
    pub(crate) fn fetch32(&mut self, address: u32) -> u32 {
        let bytes = self.fetch_bytes(address);
        match self.backing.endian {
            Endian::Little => u32::from_le_bytes(bytes),
            Endian::Big => u32::from_be_bytes(bytes),
        }
    }
    pub(crate) fn fetch_packet(&mut self, address: u32) -> [u8; 32] {
        debug_assert_eq!(address & 0x1f, 0);
        self.fetch_bytes(address)
    }
    fn fetch_bytes<const N: usize>(&mut self, address: u32) -> [u8; N] {
        if !self.hierarchy.l1p.enabled() || !self.hierarchy.l1p_cacheable(address) {
            return std::array::from_fn(|offset| {
                self.read_l2_byte(address.wrapping_add(offset as u32))
            });
        }
        if let Some(bytes) = self.hierarchy.l1p.read_bytes_direct(address) {
            return bytes;
        }
        if self.hierarchy.l1p.frozen() {
            return std::array::from_fn(|offset| {
                self.read_l2_byte(address.wrapping_add(offset as u32))
            });
        }
        let line_size = self.hierarchy.l1p.line_size();
        let base = address & !(line_size - 1);
        let from_l2_cache = self.hierarchy.l2_cacheable(base);
        let data = self.read_l2_bytes(base, line_size as usize);
        self.hierarchy.l1p_fills.push(LineFill {
            line: base,
            from_l2_cache,
        });
        self.hierarchy.l1p.insert(base, data);
        self.hierarchy.l1p.read_filled_bytes(address).unwrap()
    }
    fn read_data_bytes<const N: usize>(&mut self, address: u32) -> [u8; N] {
        let line_size = self.hierarchy.l1d.line_size();
        let within_line = (address & (line_size - 1)) + N as u32 <= line_size;
        if !within_line
            || !self.hierarchy.l1d.enabled()
            || !self.hierarchy.l1_cacheable(address)
            || self.hierarchy.l1d.frozen()
        {
            return std::array::from_fn(|offset| {
                self.read_data_byte(address.wrapping_add(offset as u32))
            });
        }
        if let Some(bytes) = self.hierarchy.l1d.read_bytes(address) {
            return bytes;
        }
        let base = address & !(line_size - 1);
        let data = self.read_l2_bytes(base, line_size as usize);
        if let Some(evicted) = self.hierarchy.l1d.insert(base, data) {
            self.hierarchy.l1d_victim_flush |= evicted.dirty;
            self.write_l1_line(evicted);
        }
        self.hierarchy.l1d.read_filled_bytes(address).unwrap()
    }
    fn write_data_bytes<const N: usize>(&mut self, address: u32, bytes: [u8; N]) {
        let line_size = self.hierarchy.l1d.line_size();
        let within_line = (address & (line_size - 1)) + N as u32 <= line_size;
        if !within_line {
            for (offset, byte) in bytes.into_iter().enumerate() {
                self.write_data_byte(address.wrapping_add(offset as u32), byte);
            }
            return;
        }
        self.backing.note_memory_write(address, N);
        if self.hierarchy.l1d.enabled()
            && self.hierarchy.l1_cacheable(address)
            && self.hierarchy.l1d.write_bytes(address, bytes)
        {
            return;
        }
        for (offset, byte) in bytes.into_iter().enumerate() {
            self.write_l2_byte(address.wrapping_add(offset as u32), byte);
        }
    }
    fn read_data_byte(&mut self, address: u32) -> u8 {
        if !self.hierarchy.l1d.enabled() || !self.hierarchy.l1_cacheable(address) {
            return self.read_l2_byte(address);
        }
        if let Some(value) = self.hierarchy.l1d.read_byte(address) {
            return value;
        }
        if self.hierarchy.l1d.frozen() {
            return self.read_l2_byte(address);
        }
        let line_size = self.hierarchy.l1d.line_size();
        let base = address & !(line_size - 1);
        let data = self.read_l2_bytes(base, line_size as usize);
        if let Some(evicted) = self.hierarchy.l1d.insert(base, data) {
            self.hierarchy.l1d_victim_flush |= evicted.dirty;
            self.write_l1_line(evicted);
        }
        self.hierarchy.l1d.read_filled_byte(address).unwrap()
    }
    fn write_data_byte(&mut self, address: u32, value: u8) {
        self.backing.note_memory_write(address, 1);
        if self.hierarchy.l1d.enabled()
            && self.hierarchy.l1_cacheable(address)
            && self.hierarchy.l1d.write_byte(address, value)
        {
            return;
        }
        self.write_l2_byte(address, value);
    }
    fn read_l2_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
        if !self.hierarchy.l2_cacheable(address) {
            self.backing
                .read_bytes(self.hierarchy.requester(), address, length)
        } else {
            (0..length)
                .map(|offset| self.read_l2_byte(address.wrapping_add(offset as u32)))
                .collect()
        }
    }
    fn read_l2_byte(&mut self, address: u32) -> u8 {
        if !self.hierarchy.l2_cacheable(address) {
            return self
                .backing
                .read_byte(self.hierarchy.requester(), address)
                .unwrap_or(0);
        }
        if let Some(value) = self.hierarchy.l2.read_byte(address) {
            return value;
        }
        let line_size = self.hierarchy.l2.line_size();
        let base = address & !(line_size - 1);
        let data = self
            .backing
            .read_bytes(self.hierarchy.requester(), base, line_size as usize);
        if let Some(evicted) = self.hierarchy.l2.insert(base, data) {
            self.write_l2_line(evicted);
        }
        self.hierarchy.l2.read_filled_byte(address).unwrap()
    }
    fn write_l2_byte(&mut self, address: u32, value: u8) {
        if !self.hierarchy.l2_cacheable(address) {
            self.backing
                .write_byte(self.hierarchy.requester(), address, value);
            return;
        }
        if !self.hierarchy.l2.write_byte(address, value) {
            let line_size = self.hierarchy.l2.line_size();
            let base = address & !(line_size - 1);
            let data =
                self.backing
                    .read_bytes(self.hierarchy.requester(), base, line_size as usize);
            if let Some(evicted) = self.hierarchy.l2.insert(base, data) {
                self.write_l2_line(evicted);
            }
            assert!(self.hierarchy.l2.write_filled_byte(address, value));
        }
    }
    pub(crate) fn cache_register_write(&mut self, address: u32, value: u32) -> bool {
        match address {
            L2CFG => self.configure_l2(value),
            L1PCFG => self.configure_l1p(value),
            L1DCFG => self.configure_l1d(value),
            L1PCC => {
                let previous = self.hierarchy.l1p.freeze(value & 1 != 0);
                self.hierarchy.l1p_cc = value & 1 | (u32::from(previous) << 16);
            }
            L1DCC => {
                let previous = self.hierarchy.l1d.freeze(value & 1 != 0);
                self.hierarchy.l1d_cc = value & 1 | (u32::from(previous) << 16);
            }
            L2WBAR => self.hierarchy.cache_bars[0] = value,
            L2WIBAR => self.hierarchy.cache_bars[1] = value,
            L2IBAR => self.hierarchy.cache_bars[2] = value,
            L1PIBAR => self.hierarchy.cache_bars[3] = value,
            L1DWIBAR => self.hierarchy.cache_bars[4] = value,
            L1DWBAR => self.hierarchy.cache_bars[5] = value,
            L1DIBAR => self.hierarchy.cache_bars[6] = value,
            L2WWC => self.l2_range_operation(0, value, false, false),
            L2WIWC => self.l2_range_operation(1, value, true, true),
            L2IWC => self.l2_range_operation(2, value, true, false),
            L1PIWC => {
                let length = value.saturating_mul(4);
                self.hierarchy
                    .l1p
                    .invalidate_range(self.hierarchy.cache_bars[3], length, false);
            }
            L1DWIWC => self.l1d_range_operation(4, value, true, true),
            L1DWWC => self.l1d_range_operation(5, value, false, true),
            L1DIWC => self.l1d_range_operation(6, value, true, false),
            L2WB if value & 1 != 0 => {
                self.flush_l1d(false);
                self.flush_l2(false);
            }
            L2WBINV if value & 1 != 0 => {
                self.flush_l1d(true);
                self.hierarchy.l1p.invalidate_all(false);
                self.flush_l2(true);
            }
            L2INV if value & 1 != 0 => {
                self.hierarchy.l1d.invalidate_all(false);
                self.hierarchy.l1p.invalidate_all(false);
                self.hierarchy.l2.invalidate_all(false);
            }
            L1PINV if value & 1 != 0 => {
                self.hierarchy.l1p.invalidate_all(false);
            }
            L1DWB if value & 1 != 0 => self.flush_l1d(false),
            L1DWBINV if value & 1 != 0 => self.flush_l1d(true),
            L1DINV if value & 1 != 0 => {
                self.hierarchy.l1d.invalidate_all(false);
            }
            L2WB | L2WBINV | L2INV | L1PINV | L1DWB | L1DWBINV | L1DINV => {}
            _ => return false,
        }
        self.hierarchy.l1p_epoch = self.hierarchy.l1p_epoch.wrapping_add(1);
        true
    }
    fn configure_l1p(&mut self, value: u32) {
        self.hierarchy.l1p.invalidate_all(false);
        self.hierarchy.l1p_cfg = value & 7;
        self.hierarchy
            .l1p
            .configure(l1_capacity(self.hierarchy.l1p_cfg));
        self.report_sram();
    }
    fn configure_l1d(&mut self, value: u32) {
        self.flush_l1d(true);
        self.hierarchy.l1d_cfg = value & 7;
        self.hierarchy
            .l1d
            .configure(l1_capacity(self.hierarchy.l1d_cfg));
        self.report_sram();
    }
    fn configure_l2(&mut self, value: u32) {
        self.flush_l1d(true);
        self.hierarchy.l1p.invalidate_all(false);
        self.flush_l2(true);
        self.hierarchy.l2_cfg = value & 7;
        self.hierarchy
            .l2
            .configure(l2_capacity(self.hierarchy.l2_cfg));
        self.report_sram();
    }
    /// Tell the storage how much of this megamodule's three windows is still SRAM, which is
    /// what a cache size change decides besides the cache's own capacity. The cache is carved
    /// out of the memory rather than added to it, so the aperture shrinks by exactly the
    /// cache's size and the addresses it took answer as nothing (SPRU862B section 2.3).
    ///
    /// The order matters and is the order silicon uses: the caches have already been written
    /// back and invalidated by the time the aperture moves, so a dirty line whose address is
    /// about to stop being SRAM reaches memory first (SPRU862B section 2.7.2 - the controller
    /// "will automatically invalidate - writeback all cache lines before initializing it with
    /// the new size"). What TI's Table 2-6 asks of software is the other half: copy anything it
    /// still needs out of the addresses that are becoming cache, because the hardware will not.
    fn report_sram(&mut self) {
        self.backing.set_core_sram(
            self.hierarchy.core,
            sram_remaining(
                self.hierarchy.map.l2_bytes(),
                l2_capacity(self.hierarchy.l2_cfg),
            ),
            sram_remaining(
                self.hierarchy.map.l1p_bytes(),
                l1_capacity(self.hierarchy.l1p_cfg),
            ),
            sram_remaining(
                self.hierarchy.map.l1d_bytes(),
                l1_capacity(self.hierarchy.l1d_cfg),
            ),
        );
    }
    fn l1d_range_operation(&mut self, bar: usize, words: u32, invalidate: bool, writeback: bool) {
        let start = self.hierarchy.cache_bars[bar];
        let length = words.saturating_mul(4);
        let lines = if invalidate {
            self.hierarchy
                .l1d
                .invalidate_range(start, length, writeback)
        } else {
            self.hierarchy.l1d.writeback_range(start, length)
        };
        self.write_l1_lines(lines);
    }
    fn l2_range_operation(&mut self, bar: usize, words: u32, invalidate: bool, writeback: bool) {
        let start = self.hierarchy.cache_bars[bar];
        let length = words.saturating_mul(4);
        let l1_lines = if invalidate {
            self.hierarchy
                .l1d
                .invalidate_range(start, length, writeback)
        } else {
            self.hierarchy.l1d.writeback_range(start, length)
        };
        self.write_l1_lines(l1_lines);
        if invalidate {
            self.hierarchy.l1p.invalidate_range(start, length, false);
        }
        let lines = if invalidate {
            self.hierarchy.l2.invalidate_range(start, length, writeback)
        } else {
            self.hierarchy.l2.writeback_range(start, length)
        };
        self.write_l2_lines(lines);
    }
    fn flush_l1d(&mut self, invalidate: bool) {
        let lines = if invalidate {
            self.hierarchy.l1d.invalidate_all(true)
        } else {
            self.hierarchy.l1d.writeback_all()
        };
        self.write_l1_lines(lines);
    }
    fn flush_l2(&mut self, invalidate: bool) {
        let lines = if invalidate {
            self.hierarchy.l2.invalidate_all(true)
        } else {
            self.hierarchy.l2.writeback_all()
        };
        self.write_l2_lines(lines);
    }
    /// An L1D victim, which SPRU871K section 4.3.8.3 gives a policy of its own rather than
    /// treating as an ordinary write: "L1D victim writebacks do not trigger line allocations in
    /// L2. L1D victims are written directly to external memory if they miss L2. L1D victim
    /// writebacks also do not update L2's LRU if they hit in L2. They do update L2's dirty
    /// status as needed."
    ///
    /// So this cannot go through `write_l2_byte`, which is the CPU's write path and allocates on
    /// a miss: a victim that misses L2 goes straight to the self.backing store, and one that hits
    /// updates the line's data and dirty bit without touching its replacement clock. Getting
    /// this wrong is invisible to a program's answer and visible in the L2 cache's contents - an
    /// allocation the hardware never makes evicts a line the hardware would have kept.
    fn write_l1_line(&mut self, line: EvictedLine) {
        if !line.dirty {
            return;
        }
        if !self.hierarchy.l2_cacheable(line.address)
            || !self.hierarchy.l2.write_victim(line.address, &line.data)
        {
            self.backing
                .write_bytes(self.hierarchy.requester(), line.address, &line.data);
        }
    }
    fn write_l1_lines(&mut self, lines: Vec<EvictedLine>) {
        for line in lines {
            self.write_l1_line(line);
        }
    }
    fn write_l2_line(&mut self, line: EvictedLine) {
        if line.dirty {
            self.backing
                .write_bytes(self.hierarchy.requester(), line.address, &line.data);
        }
    }
    fn write_l2_lines(&mut self, lines: Vec<EvictedLine>) {
        for line in lines {
            self.write_l2_line(line);
        }
    }
    /// A host write goes into memory, so every cached copy of those bytes has to go: L1P
    /// invalidated, L1D and L2 invalidated with writeback of anything dirty.
    pub(crate) fn host_invalidate(&mut self, address: u32, length: u32) {
        self.hierarchy.l1p.invalidate_range(address, length, false);
        self.hierarchy.l1p_epoch = self.hierarchy.l1p_epoch.wrapping_add(1);
        let l1_lines = self.hierarchy.l1d.invalidate_range(address, length, true);
        self.write_l1_lines(l1_lines);
        let l2_lines = self.hierarchy.l2.invalidate_range(address, length, true);
        self.write_l2_lines(l2_lines);
    }
    pub(crate) fn read_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
        (0..length)
            .map(|offset| self.read_u8(address.wrapping_add(offset as u32)))
            .collect()
    }
}

fn mar_index(address: u32) -> Option<usize> {
    let offset = address.checked_sub(MAR_BASE)?;
    (offset < 1024 && offset % 4 == 0).then_some((offset / 4) as usize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::space::C6455_MAP;

    /// The C6455's L1D window, and four external lines that index the same set of a 4 KB
    /// two-way L1D: 4096 / 64 per line / 2 ways is 32 sets, so the stride is 2048 bytes.
    const L1D_WINDOW: u32 = 0x00f0_0000;
    const EXTERNAL: [u32; 4] = [0xe000_0000, 0xe000_0800, 0xe000_1000, 0xe000_1800];

    fn hierarchy() -> (MemoryHierarchy, Backing) {
        (
            MemoryHierarchy::new(0, &C6455_MAP),
            Backing::new(&C6455_MAP, Endian::Little),
        )
    }

    /// Cache the DDR2 region and turn on a 4 KB L1D cache and a 32 KB L2 cache.
    fn cache_external(mm: &mut MemoryHierarchy, backing: &mut Backing) {
        assert!(mm.mar_write(MAR_BASE + 4 * (EXTERNAL[0] >> 24), 1));
        let mut bound = mm.bind(backing);
        bound.cache_register_write(L1DCFG, 1);
        bound.cache_register_write(L2CFG, 3);
    }

    #[test]
    fn l1d_sram_is_served_without_a_fill() {
        let (mm, _backing) = hierarchy();
        // Every cache comes out of reset disabled, so all of the L1D window is SRAM.
        assert_eq!(
            mm.access_timing(L1D_WINDOW),
            AccessTiming {
                resident: true,
                read_miss: false,
                from_l2_cache: false,
            }
        );
    }

    #[test]
    fn the_l1d_cache_carves_its_aperture_out_of_the_window() {
        let (mut mm, mut backing) = hierarchy();
        cache_external(&mut mm, &mut backing);
        // The conversion runs from the top of the window down, so the first 28 KB is still SRAM
        // and the last 4 KB is the cache itself - which the CPU may not treat as resident.
        assert!(mm.access_timing(L1D_WINDOW).resident);
        assert!(!mm.access_timing(L1D_WINDOW + 28 * 1024).resident);
    }

    #[test]
    fn an_uncacheable_address_never_reports_a_read_miss() {
        let (mut mm, mut backing) = hierarchy();
        mm.bind(&mut backing).cache_register_write(L1DCFG, 1);
        // MAR leaves the DDR2 region uncacheable, so the access reaches L2 without a line.
        assert_eq!(
            mm.access_timing(EXTERNAL[0]),
            AccessTiming {
                resident: false,
                read_miss: false,
                from_l2_cache: false,
            }
        );
    }

    #[test]
    fn an_l2_sram_read_miss_is_not_charged_as_an_l2_cache_line() {
        let (mut mm, mut backing) = hierarchy();
        cache_external(&mut mm, &mut backing);
        // L2 SRAM sits in MAR region 0, which the L2 cache never holds.
        let timing = mm.access_timing(0x0080_0100);
        assert!(timing.read_miss);
        assert!(!timing.from_l2_cache);
    }

    #[test]
    fn an_external_read_miss_is_charged_as_an_l2_cache_line() {
        let (mut mm, mut backing) = hierarchy();
        cache_external(&mut mm, &mut backing);
        assert_eq!(
            mm.access_timing(EXTERNAL[0]),
            AccessTiming {
                resident: false,
                read_miss: true,
                from_l2_cache: true,
            }
        );
        // Once the line is resident there is no fill, and an access that costs no line has no
        // source: this is the shape both consumers read, and each only asks having seen a fill.
        mm.bind(&mut backing).read_u32(EXTERNAL[0]);
        assert_eq!(
            mm.access_timing(EXTERNAL[0]),
            AccessTiming {
                resident: true,
                read_miss: false,
                from_l2_cache: false,
            }
        );
    }

    /// The invariant the CPU's timing arithmetic now relies on: the write buffer used to ask
    /// about a quad-word and the miss cost about a 64-byte line, and both now take the answer the
    /// hierarchy gave for the access's own address. Residency and cacheability are line- and
    /// region-granular, so the three addresses cannot disagree.
    #[test]
    fn a_byte_its_quad_word_and_its_line_agree() {
        let (mut mm, mut backing) = hierarchy();
        cache_external(&mut mm, &mut backing);
        for address in [EXTERNAL[0] + 0x2c, 0x0080_012c, L1D_WINDOW + 0x2c] {
            let byte = mm.access_timing(address);
            assert_eq!(byte, mm.access_timing(address & !15));
            assert_eq!(byte, mm.access_timing(address & !63));
        }
    }

    #[test]
    fn only_a_dirty_eviction_reports_a_victim_flush() {
        let (mut mm, mut backing) = hierarchy();
        cache_external(&mut mm, &mut backing);
        mm.bind(&mut backing).read_u32(EXTERNAL[0]);
        mm.bind(&mut backing).read_u32(EXTERNAL[1]);
        // Dirty the first line, then evict the second: a clean victim is no disruption.
        mm.bind(&mut backing).write_u32(EXTERNAL[0], 0xdead_beef);
        mm.bind(&mut backing).read_u32(EXTERNAL[2]);
        assert!(!mm.take_l1d_victim_flush());
        // Now evict the dirty one.
        mm.bind(&mut backing).read_u32(EXTERNAL[3]);
        assert!(mm.take_l1d_victim_flush());
        // The flag is a report of what happened since the last drain, so it does not repeat.
        assert!(!mm.take_l1d_victim_flush());
        assert_eq!(mm.bind(&mut backing).read_u32(EXTERNAL[0]), 0xdead_beef);
    }

    #[test]
    fn program_fills_report_the_line_and_its_source() {
        let (mut mm, mut backing) = hierarchy();
        cache_external(&mut mm, &mut backing);
        mm.bind(&mut backing).cache_register_write(L1PCFG, 1);
        mm.bind(&mut backing).fetch_packet(EXTERNAL[0]);
        mm.bind(&mut backing).fetch_packet(0x0080_0000);
        let fills: Vec<_> = mm.drain_l1p_fills().collect();
        assert_eq!(
            fills,
            [
                LineFill {
                    line: EXTERNAL[0],
                    from_l2_cache: true,
                },
                LineFill {
                    line: 0x0080_0000,
                    from_l2_cache: false,
                },
            ]
        );
        // Draining is what clears them: the fetch path charges each fill exactly once.
        assert_eq!(mm.drain_l1p_fills().count(), 0);
    }
}
