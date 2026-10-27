#[derive(serde::Serialize)]
pub struct CacheLineView {
    pub address: u32,
    /// The set this address indexes; only these ways ever compete for it.
    pub set: usize,
    pub dirty: bool,
    /// Replacement clock: the least recently used line in a set has the lowest value.
    pub used: u64,
}

/// Cumulative cache-local traffic since reset. Access counts are byte lookups because the
/// memory model services instruction and data transfers one byte at a time.
#[derive(Clone, Copy, Default, serde::Serialize)]
pub struct CacheStats {
    pub read_hits: u64,
    pub read_misses: u64,
    pub write_hits: u64,
    pub write_misses: u64,
    pub line_fills: u64,
    pub evictions: u64,
    pub dirty_evictions: u64,
    pub writebacks: u64,
    pub invalidations: u64,
}

#[derive(serde::Serialize)]
pub struct CacheView {
    pub name: &'static str,
    pub enabled: bool,
    pub frozen: bool,
    pub capacity_lines: usize,
    pub ways: usize,
    pub line_size: u32,
    pub lines: Vec<CacheLineView>,
    pub stats: CacheStats,
}

#[derive(Clone)]
pub struct EvictedLine {
    pub address: u32,
    pub data: Vec<u8>,
    pub dirty: bool,
    /// The line held bytes a master had written to memory since it was filled, so writing it
    /// back puts older data over the master's.
    pub stale: bool,
}

struct CacheLine {
    address: u32,
    data: Vec<u8>,
    dirty: bool,
    used: u64,
    /// One bit per byte of the line that a master has written in memory since the line was
    /// filled and that the CPU has not written since - the bytes this copy has wrong. Only the
    /// coherence check sets it (`mark_stale`); with the check off it stays zero.
    stale: u128,
}

/// The bit mask of `length` bytes from `offset` within a line of at most 128 bytes.
#[inline(always)]
fn byte_mask(offset: u32, length: u32) -> u128 {
    let bits = if length >= 128 {
        u128::MAX
    } else {
        (1u128 << length) - 1
    };
    bits << offset
}

const EMPTY_SLOT: usize = usize::MAX;

pub struct Cache {
    line_size: u32,
    line_shift: u32,
    /// Line frames per set: one for a direct-mapped cache, more for a set-associative one. A line
    /// can only ever live in the set its address indexes, so a working set that lands in too few
    /// sets thrashes however much capacity is spare - the conflict misses of SPRU862B 1.8.1.
    ways: usize,
    capacity: usize,
    /// Hardware cache geometries are powers of two, so indexing uses a mask rather than two
    /// integer divisions on every byte lookup.
    set_mask: usize,
    clock: u64,
    frozen: bool,
    lines: Vec<CacheLine>,
    set_indices: Vec<[usize; 4]>,
    stats: CacheStats,
    /// The CPU reads that hit stale bytes since the coherence check last asked, the first per
    /// line: a cycle may read several stale lines, and each is a separate report.
    stale_hits: Vec<u32>,
}

impl Cache {
    pub fn new(line_size: u32, ways: usize) -> Self {
        assert!(line_size.is_power_of_two());
        assert!(ways.is_power_of_two());
        Self {
            line_size,
            line_shift: line_size.trailing_zeros(),
            ways,
            capacity: 0,
            clock: 0,
            set_mask: 0,
            frozen: false,
            lines: Vec::new(),
            set_indices: Vec::new(),
            stats: CacheStats::default(),
            stale_hits: Vec::new(),
        }
    }

    pub fn enabled(&self) -> bool {
        self.capacity != 0
    }
    pub fn frozen(&self) -> bool {
        self.frozen
    }

    pub fn freeze(&mut self, active: bool) -> bool {
        let previous = self.frozen;
        self.frozen = active;
        previous
    }

    /// Resize, evicting whatever no longer fits. A size change re-indexes every set, so a line
    /// can be left in a set that is now over-occupied even when the total still fits; TI's own
    /// procedure for changing cache sizes (SPRU862B Table 2-6) has software write back and
    /// invalidate first for exactly that reason.
    pub fn configure(&mut self, bytes: usize) -> Vec<EvictedLine> {
        self.capacity = bytes / self.line_size as usize;
        let sets = (self.capacity / self.ways).max(1);
        assert!(sets.is_power_of_two());
        self.set_mask = sets - 1;
        self.set_indices.clear();
        let mut evicted = Vec::new();
        loop {
            let mut over = None;
            for line in &self.lines {
                let set = self.set_of(line.address);
                if self
                    .lines
                    .iter()
                    .filter(|other| self.set_of(other.address) == set)
                    .count()
                    > self.ways.min(self.capacity.max(1))
                {
                    over = Some(set);
                    break;
                }
            }
            let Some(set) = over.filter(|_| !self.lines.is_empty()) else {
                break;
            };
            let index = self.least_recently_used(set);
            let line = self.remove(index);
            self.record_eviction(&line);
            evicted.push(line);
        }
        while self.lines.len() > self.capacity {
            let set = self.set_of(self.lines[0].address);
            let index = self.least_recently_used(set);
            let line = self.remove(index);
            self.record_eviction(&line);
            evicted.push(line);
        }
        self.rebuild_set_indices();
        evicted
    }

    /// A naturally aligned instruction fetch that stays within one line. Lookup and LRU
    /// bookkeeping happen once while the byte-granular counters retain their existing meaning.
    pub fn read_bytes<const N: usize>(&mut self, address: u32) -> Option<[u8; N]> {
        let base = self.line_base(address);
        let offset = (address - base) as usize;
        debug_assert!(offset + N <= self.line_size as usize);
        let Some(index) = self.find_index(base) else {
            self.stats.read_misses = self.stats.read_misses.wrapping_add(1);
            return None;
        };
        self.stats.read_hits = self.stats.read_hits.wrapping_add(N as u64);
        if !self.frozen {
            self.clock = self.clock.wrapping_add(N as u64);
            self.lines[index].used = self.clock;
        }
        self.check_stale(index, address, offset as u32, N as u32);
        let mut bytes = [0; N];
        bytes.copy_from_slice(&self.lines[index].data[offset..offset + N]);
        Some(bytes)
    }

    /// Record a CPU read that hit bytes a master has overwritten in memory since the line was
    /// filled, for the coherence check to collect. A line nothing marked costs one test.
    #[inline(always)]
    fn check_stale(&mut self, index: usize, address: u32, offset: u32, length: u32) {
        if self.lines[index].stale != 0 && self.lines[index].stale & byte_mask(offset, length) != 0
        {
            self.record_stale_hit(address);
        }
    }

    #[cold]
    fn record_stale_hit(&mut self, address: u32) {
        let line = self.line_base(address);
        if !self
            .stale_hits
            .iter()
            .any(|&hit| self.line_base(hit) == line)
        {
            self.stale_hits.push(address);
        }
    }

    /// The CPU's own write makes those bytes the newest copy again.
    #[inline(always)]
    fn clear_stale(&mut self, index: usize, offset: u32, length: u32) {
        let line = &mut self.lines[index];
        if line.stale != 0 {
            line.stale &= !byte_mask(offset, length);
        }
    }

    /// The direct-mapped variant of `read_bytes`: the L1P cache is always one way, so the
    /// lookup is a single slot test instead of the generic way scan, which matters on the
    /// instruction fetch path.
    pub fn read_bytes_direct<const N: usize>(&mut self, address: u32) -> Option<[u8; N]> {
        debug_assert_eq!(self.ways, 1);
        let base = self.line_base(address);
        let offset = (address - base) as usize;
        debug_assert!(offset + N <= self.line_size as usize);
        let Some(index) = self.find_index_direct(base) else {
            self.stats.read_misses = self.stats.read_misses.wrapping_add(1);
            return None;
        };
        self.stats.read_hits = self.stats.read_hits.wrapping_add(N as u64);
        self.clock = self.clock.wrapping_add(N as u64);
        self.lines[index].used = self.clock;
        self.check_stale(index, address, offset as u32, N as u32);
        let mut bytes = [0; N];
        bytes.copy_from_slice(&self.lines[index].data[offset..offset + N]);
        Some(bytes)
    }

    /// Finish a multi-byte access after insertion. The first byte caused the miss and the
    /// insertion itself advanced LRU; the remaining bytes are the hits the old byte loop saw.
    /// The CPU consumes these bytes, so a stale one the fill carried in is reported here.
    pub fn read_filled_bytes<const N: usize>(&mut self, address: u32) -> Option<[u8; N]> {
        let base = self.line_base(address);
        let offset = (address - base) as usize;
        let index = self.find_index(base)?;
        let following = N.saturating_sub(1) as u64;
        self.stats.read_hits = self.stats.read_hits.wrapping_add(following);
        self.clock = self.clock.wrapping_add(following);
        self.lines[index].used = self.clock;
        self.check_stale(index, address, offset as u32, N as u32);
        let mut bytes = [0; N];
        bytes.copy_from_slice(&self.lines[index].data[offset..offset + N]);
        Some(bytes)
    }

    /// Part of a line, read to fill the level above rather than by the CPU: the bytes, and which
    /// of them a master has overwritten since this line was filled (bit 0 is `address`). Hits
    /// and replacement order are counted as `read_bytes` counts them, but nothing is reported -
    /// the stale bytes travel with the fill, and the CPU's own read of one is what reports it.
    pub fn read_span(&mut self, address: u32, length: u32) -> Option<(Vec<u8>, u128)> {
        let base = self.line_base(address);
        let Some(index) = self.find_index(base) else {
            self.stats.read_misses = self.stats.read_misses.wrapping_add(1);
            return None;
        };
        self.stats.read_hits = self.stats.read_hits.wrapping_add(u64::from(length));
        if !self.frozen {
            self.clock = self.clock.wrapping_add(u64::from(length));
            self.lines[index].used = self.clock;
        }
        Some(self.span(index, address - base, length))
    }

    /// `read_span` for the line a miss has just inserted, counted as `read_filled_bytes` is.
    pub fn read_filled_span(&mut self, address: u32, length: u32) -> Option<(Vec<u8>, u128)> {
        let base = self.line_base(address);
        let index = self.find_index(base)?;
        let following = u64::from(length.saturating_sub(1));
        self.stats.read_hits = self.stats.read_hits.wrapping_add(following);
        self.clock = self.clock.wrapping_add(following);
        self.lines[index].used = self.clock;
        Some(self.span(index, address - base, length))
    }

    fn span(&self, index: usize, offset: u32, length: u32) -> (Vec<u8>, u128) {
        let line = &self.lines[index];
        let data = line.data[offset as usize..(offset + length) as usize].to_vec();
        let stale = (line.stale & byte_mask(offset, length)) >> offset;
        (data, stale)
    }

    /// Give the line holding `address` the stale bytes the fill that brought it in carried from
    /// the level below (bit 0 is the line's first byte). A frozen cache that declined the fill
    /// holds no line, and nothing happens.
    pub fn carry_stale(&mut self, address: u32, stale: u128) {
        if stale == 0 {
            return;
        }
        if let Some(index) = self.find_index(self.line_base(address)) {
            self.lines[index].stale |= stale;
        }
    }
    pub fn read_byte(&mut self, address: u32) -> Option<u8> {
        let base = self.line_base(address);
        let Some(index) = self.find_index(base) else {
            self.stats.read_misses = self.stats.read_misses.wrapping_add(1);
            return None;
        };
        self.stats.read_hits = self.stats.read_hits.wrapping_add(1);
        // SPRU871K 3.3.4: frozen hits service data without modifying replacement order.
        if !self.frozen {
            self.clock = self.clock.wrapping_add(1);
            self.lines[index].used = self.clock;
        }
        self.check_stale(index, address, address - base, 1);
        Some(self.lines[index].data[(address - base) as usize])
    }

    pub fn write_byte(&mut self, address: u32, value: u8) -> bool {
        let base = self.line_base(address);
        let Some(index) = self.find_index(base) else {
            self.stats.write_misses = self.stats.write_misses.wrapping_add(1);
            return false;
        };
        self.stats.write_hits = self.stats.write_hits.wrapping_add(1);
        if !self.frozen {
            self.clock = self.clock.wrapping_add(1);
            self.lines[index].used = self.clock;
        }
        self.clear_stale(index, address - base, 1);
        let line = &mut self.lines[index];
        line.data[(address - base) as usize] = value;
        line.dirty = true;
        true
    }

    /// One naturally aligned multi-byte write within a line. A write miss does not allocate in
    /// L1D, so every byte records a miss; a hit updates replacement state once for the transfer.
    pub fn write_bytes<const N: usize>(&mut self, address: u32, bytes: [u8; N]) -> bool {
        let base = self.line_base(address);
        let offset = (address - base) as usize;
        debug_assert!(offset + N <= self.line_size as usize);
        let Some(index) = self.find_index(base) else {
            self.stats.write_misses = self.stats.write_misses.wrapping_add(N as u64);
            return false;
        };
        self.stats.write_hits = self.stats.write_hits.wrapping_add(N as u64);
        if !self.frozen {
            self.clock = self.clock.wrapping_add(N as u64);
            self.lines[index].used = self.clock;
        }
        self.clear_stale(index, offset as u32, N as u32);
        let line = &mut self.lines[index];
        line.data[offset..offset + N].copy_from_slice(&bytes);
        line.dirty = true;
        true
    }

    /// Complete the logical access which caused a fill without recording a second hit. Insertion
    /// already updates replacement order; these helpers only read or modify the new line. The
    /// read is the CPU's, so a stale byte the fill carried in is reported.
    pub fn read_filled_byte(&mut self, address: u32) -> Option<u8> {
        let base = self.line_base(address);
        let index = self.find_index(base)?;
        self.check_stale(index, address, address - base, 1);
        Some(self.lines[index].data[(address - base) as usize])
    }

    pub fn write_filled_byte(&mut self, address: u32, value: u8) -> bool {
        let base = self.line_base(address);
        let Some(index) = self.find_index(base) else {
            return false;
        };
        self.lines[index].data[(address - base) as usize] = value;
        self.lines[index].dirty = true;
        true
    }

    /// A victim writeback from the level above, which is not an ordinary write.
    ///
    /// SPRU871K section 4.3.8.3 gives this its own policy, and all three halves of it matter:
    /// "L1D victim writebacks do not trigger line allocations in L2. L1D victims are written
    /// directly to external memory if they miss L2. L1D victim writebacks also do not update
    /// L2's LRU if they hit in L2. They do update L2's dirty status as needed." So a miss is
    /// answered `false` and the caller takes the victim past this cache to memory, and a hit
    /// updates the data and the dirty bit while leaving the replacement clock alone - a victim
    /// arriving is not a use of the line it lands in.
    pub fn write_victim(&mut self, address: u32, bytes: &[u8]) -> bool {
        let base = self.line_base(address);
        let offset = (address - base) as usize;
        debug_assert!(offset + bytes.len() <= self.line_size as usize);
        let Some(index) = self.find_index(base) else {
            self.stats.write_misses = self.stats.write_misses.wrapping_add(bytes.len() as u64);
            return false;
        };
        self.stats.write_hits = self.stats.write_hits.wrapping_add(bytes.len() as u64);
        self.clear_stale(index, offset as u32, bytes.len() as u32);
        let line = &mut self.lines[index];
        line.data[offset..offset + bytes.len()].copy_from_slice(bytes);
        line.dirty = true;
        true
    }

    /// A DMA write the L2 controller forwards into the lines this cache is holding, over the
    /// whole transfer. It updates the data without dirtying anything - memory now has the same
    /// bytes - and leaves the replacement order alone, since no CPU access happened
    /// (SPRU862B section 2.4.2: "the dirty bit is not affected by this operation"). One pass
    /// over the lines rather than a lookup per byte, because a DMA transfer can be a whole
    /// Ethernet frame.
    pub fn snoop_write_range(&mut self, address: u32, bytes: &[u8]) {
        if self.lines.is_empty() || bytes.is_empty() {
            return;
        }
        let end = address.wrapping_add(bytes.len() as u32);
        for line in &mut self.lines {
            let line_end = line.address + self.line_size;
            if line_end <= address || line.address >= end {
                continue;
            }
            let from = address.max(line.address);
            let to = end.min(line_end);
            for byte_address in from..to {
                line.data[(byte_address - line.address) as usize] =
                    bytes[(byte_address - address) as usize];
            }
        }
    }

    /// The bytes a DMA read must be served from this cache instead of from memory: only a dirty
    /// line holds data memory does not have (SPRU862B Figure 2-4).
    pub fn snoop_read_range(&self, address: u32, data: &mut [u8]) {
        if self.lines.is_empty() || data.is_empty() {
            return;
        }
        let end = address.wrapping_add(data.len() as u32);
        for line in &self.lines {
            if !line.dirty || line.address + self.line_size <= address || line.address >= end {
                continue;
            }
            let from = address.max(line.address);
            let to = end.min(line.address + self.line_size);
            for byte_address in from..to {
                data[(byte_address - address) as usize] =
                    line.data[(byte_address - line.address) as usize];
            }
        }
    }

    /// Mark the bytes a master has just written in memory as wrong in every line here that
    /// holds them: nothing snooped them into the cache, so the CPU's copy is now older than
    /// memory's. The coherence check alone calls this. Answers whether any line held them.
    pub fn mark_stale(&mut self, address: u32, length: u32) -> bool {
        let end = u64::from(address) + u64::from(length);
        let mut marked = false;
        for line in &mut self.lines {
            let line_end = u64::from(line.address) + u64::from(self.line_size);
            if line_end <= u64::from(address) || u64::from(line.address) >= end {
                continue;
            }
            let start = u64::from(line.address);
            let from = u64::from(address).max(start);
            let to = end.min(line_end);
            line.stale |= byte_mask((from - start) as u32, (to - from) as u32);
            marked = true;
        }
        marked
    }

    /// The first dirty line here holding any of these bytes, which a master reading memory
    /// would miss because nothing snoops it.
    pub fn dirty_overlap(&self, address: u32, length: u32) -> Option<u32> {
        let end = u64::from(address) + u64::from(length);
        self.lines
            .iter()
            .filter(|line| {
                line.dirty
                    && u64::from(line.address) + u64::from(self.line_size) > u64::from(address)
                    && u64::from(line.address) < end
            })
            .map(|line| line.address)
            .min()
    }

    /// Every line a CPU read found stale since the last call, by the first address read in it.
    pub fn take_stale_hits(&mut self) -> Vec<u32> {
        std::mem::take(&mut self.stale_hits)
    }

    /// Byte lookups a caller served past this cache without asking it each time.
    pub fn count_read_misses(&mut self, bytes: u32) {
        self.stats.read_misses = self.stats.read_misses.wrapping_add(u64::from(bytes));
    }

    pub fn insert(&mut self, address: u32, data: Vec<u8>) -> Option<EvictedLine> {
        if self.capacity == 0 {
            return None;
        }
        debug_assert_eq!(data.len(), self.line_size as usize);
        self.stats.line_fills = self.stats.line_fills.wrapping_add(1);
        let base = self.line_base(address);
        if let Some(index) = self.find_index(base) {
            self.clock = self.clock.wrapping_add(1);
            self.lines[index] = CacheLine {
                address: base,
                data,
                dirty: false,
                used: self.clock,
                stale: 0,
            };
            return None;
        }
        if self.frozen {
            return None;
        }
        // Only this address's set can hold the line, and only its least recently used way is
        // available once the set is full.
        let set = self.set_of(base);
        let occupied = self.set_indices[set][..self.ways]
            .iter()
            .filter(|&&index| index != EMPTY_SLOT)
            .count();
        let evicted = if occupied == self.ways.min(self.capacity) {
            let index = self.least_recently_used(set);
            let line = self.remove(index);
            self.record_eviction(&line);
            Some(line)
        } else {
            None
        };
        self.clock = self.clock.wrapping_add(1);
        self.add_index(base, self.lines.len());
        self.lines.push(CacheLine {
            address: base,
            data,
            dirty: false,
            used: self.clock,
            stale: 0,
        });
        evicted
    }

    pub fn writeback_all(&mut self) -> Vec<EvictedLine> {
        let mut written = Vec::new();
        for line in &mut self.lines {
            if !line.dirty {
                continue;
            }
            line.dirty = false;
            written.push(EvictedLine {
                address: line.address,
                data: line.data.clone(),
                dirty: true,
                stale: std::mem::take(&mut line.stale) != 0,
            });
        }
        self.stats.writebacks = self.stats.writebacks.wrapping_add(written.len() as u64);
        written
    }

    pub fn invalidate_all(&mut self, writeback: bool) -> Vec<EvictedLine> {
        let invalidated = self.lines.len() as u64;
        let mut removed = Vec::with_capacity(self.lines.len());
        self.set_indices.fill([EMPTY_SLOT; 4]);
        for line in self.lines.drain(..) {
            if writeback && line.dirty {
                removed.push(EvictedLine {
                    address: line.address,
                    data: line.data,
                    dirty: true,
                    stale: line.stale != 0,
                });
            }
        }
        self.stats.invalidations = self.stats.invalidations.wrapping_add(invalidated);
        self.stats.writebacks = self.stats.writebacks.wrapping_add(removed.len() as u64);
        removed
    }
    pub fn writeback_range(&mut self, start: u32, length: u32) -> Vec<EvictedLine> {
        let end = start.saturating_add(length);
        let mut written = Vec::new();
        for line in &mut self.lines {
            let line_end = line.address.saturating_add(self.line_size);
            if !line.dirty || line.address >= end || line_end <= start {
                continue;
            }
            line.dirty = false;
            written.push(EvictedLine {
                address: line.address,
                data: line.data.clone(),
                dirty: true,
                stale: std::mem::take(&mut line.stale) != 0,
            });
        }
        self.stats.writebacks = self.stats.writebacks.wrapping_add(written.len() as u64);
        written
    }

    pub fn invalidate_range(
        &mut self,
        start: u32,
        length: u32,
        writeback: bool,
    ) -> Vec<EvictedLine> {
        let end = start.saturating_add(length);
        let mut removed = Vec::new();
        let mut invalidated = 0u64;
        let mut index = 0;
        while index < self.lines.len() {
            let line_end = self.lines[index].address.saturating_add(self.line_size);
            if self.lines[index].address < end && line_end > start {
                let line = self.remove(index);
                invalidated = invalidated.wrapping_add(1);
                if writeback && line.dirty {
                    removed.push(line);
                }
            } else {
                index += 1;
            }
        }
        self.stats.invalidations = self.stats.invalidations.wrapping_add(invalidated);
        self.stats.writebacks = self.stats.writebacks.wrapping_add(removed.len() as u64);
        removed
    }

    /// True when the line covering `address` is already resident.
    pub fn contains(&self, address: u32) -> bool {
        let base = self.line_base(address);
        self.find_index(base).is_some()
    }

    /// Cached bytes exactly as the CPU sees them. Inspection is passive: it does not update LRU
    /// order or the hit counters.
    pub fn line_data(&self, address: u32) -> Option<&[u8]> {
        let base = self.line_base(address);
        self.find_index(base)
            .map(|index| self.lines[index].data.as_slice())
    }

    pub fn line_size(&self) -> u32 {
        self.line_size
    }

    fn line_base(&self, address: u32) -> u32 {
        address & !(self.line_size - 1)
    }

    /// A read-only snapshot for inspection tools: what the cache is configured as, and every
    /// resident line with the set it occupies and its replacement order. Sorted by set and
    /// then by address so a display shows the geometry rather than insertion order.
    pub fn view(&self, name: &'static str) -> CacheView {
        let mut lines: Vec<CacheLineView> = self
            .lines
            .iter()
            .map(|line| CacheLineView {
                address: line.address,
                set: self.set_of(line.address),
                dirty: line.dirty,
                used: line.used,
            })
            .collect();
        lines.sort_unstable_by_key(|line| (line.set, line.address));
        CacheView {
            name,
            enabled: self.capacity != 0,
            frozen: self.frozen,
            capacity_lines: self.capacity,
            ways: self.ways,
            line_size: self.line_size,
            lines,
            stats: self.stats,
        }
    }

    /// The index of this address's set: the line number modulo the number of sets, which is the
    /// capacity in lines divided over the ways.
    #[inline(always)]
    fn set_of(&self, address: u32) -> usize {
        ((address >> self.line_shift) as usize) & self.set_mask
    }

    fn least_recently_used(&self, set: usize) -> usize {
        self.set_indices[set][..self.ways]
            .iter()
            .copied()
            .filter(|&index| index != EMPTY_SLOT)
            .min_by_key(|&index| self.lines[index].used)
            .unwrap()
    }

    fn remove(&mut self, index: usize) -> EvictedLine {
        let last = self.lines.len() - 1;
        if !self.set_indices.is_empty() {
            let set = self.set_of(self.lines[index].address);
            let slot = self.set_indices[set]
                .iter_mut()
                .find(|slot| **slot == index)
                .unwrap();
            *slot = EMPTY_SLOT;
            if index != last {
                let moved_set = self.set_of(self.lines[last].address);
                let moved_slot = self.set_indices[moved_set]
                    .iter_mut()
                    .find(|slot| **slot == last)
                    .unwrap();
                *moved_slot = index;
            }
        }
        let line = self.lines.swap_remove(index);
        EvictedLine {
            address: line.address,
            data: line.data,
            dirty: line.dirty,
            stale: line.stale != 0,
        }
    }

    #[inline(always)]
    fn find_index(&self, base: u32) -> Option<usize> {
        if self.set_indices.is_empty() {
            return None;
        }
        let indices = &self.set_indices[self.set_of(base)];
        indices[..self.ways]
            .iter()
            .find(|&&index| index != EMPTY_SLOT && self.lines[index].address == base)
            .copied()
    }

    /// One-way lookup: the single slot in the set, used by the direct-mapped L1P.
    #[inline(always)]
    fn find_index_direct(&self, base: u32) -> Option<usize> {
        let slot = self.set_indices.get(self.set_of(base)).map(|set| set[0])?;
        (slot != EMPTY_SLOT && self.lines[slot].address == base).then_some(slot)
    }

    fn add_index(&mut self, address: u32, index: usize) {
        let set = self.set_of(address);
        let slot = self.set_indices[set][..self.ways]
            .iter_mut()
            .find(|slot| **slot == EMPTY_SLOT)
            .unwrap();
        *slot = index;
    }

    fn rebuild_set_indices(&mut self) {
        if self.capacity == 0 {
            self.set_indices.clear();
            return;
        }
        let sets = (self.capacity / self.ways).max(1);
        self.set_indices = vec![[EMPTY_SLOT; 4]; sets];
        for index in 0..self.lines.len() {
            self.add_index(self.lines[index].address, index);
        }
    }

    fn record_eviction(&mut self, line: &EvictedLine) {
        self.stats.evictions = self.stats.evictions.wrapping_add(1);
        if line.dirty {
            self.stats.dirty_evictions = self.stats.dirty_evictions.wrapping_add(1);
            self.stats.writebacks = self.stats.writebacks.wrapping_add(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Cache;

    #[test]
    fn statistics_distinguish_misses_from_post_fill_accesses() {
        let mut cache = Cache::new(32, 1);
        cache.configure(32);

        assert_eq!(cache.read_byte(0), None);
        cache.insert(0, vec![0; 32]);
        assert_eq!(cache.read_filled_byte(1), Some(0));
        assert_eq!(cache.read_byte(2), Some(0));
        assert!(cache.write_byte(3, 0xaa));
        assert!(!cache.write_byte(64, 0xbb));

        let evicted = cache.insert(64, vec![0; 32]).unwrap();
        assert!(evicted.dirty);
        cache.invalidate_all(false);

        let stats = cache.view("test").stats;
        assert_eq!(stats.read_misses, 1);
        assert_eq!(stats.read_hits, 1);
        assert_eq!(stats.write_hits, 1);
        assert_eq!(stats.write_misses, 1);
        assert_eq!(stats.line_fills, 2);
        assert_eq!(stats.evictions, 1);
        assert_eq!(stats.dirty_evictions, 1);
        assert_eq!(stats.writebacks, 1);
        assert_eq!(stats.invalidations, 1);
    }

    /// SPRU871K section 4.3.8.3 gives a victim writeback its own policy, in three parts, and
    /// this checks each: "L1D victim writebacks do not trigger line allocations in L2. L1D
    /// victims are written directly to external memory if they miss L2. L1D victim writebacks
    /// also do not update L2's LRU if they hit in L2. They do update L2's dirty status as
    /// needed."
    #[test]
    fn a_victim_writeback_neither_allocates_nor_promotes_the_line_it_lands_in() {
        // Two ways and two sets, so one set holds exactly two lines and the other's occupancy
        // cannot interfere.
        let mut cache = Cache::new(64, 2);
        cache.configure(4 * 64);

        // A victim for an address the cache does not hold is refused, and nothing is allocated:
        // the caller has to take it past this cache to memory.
        assert!(!cache.write_victim(0x4000, &[0xaa; 64]));
        assert_eq!(cache.view("l2").lines.len(), 0);

        // Fill both ways of set 0. The way stride is two lines, so 0 and 0x80 share a set.
        cache.insert(0x0000, vec![0x11; 64]);
        cache.insert(0x0080, vec![0x22; 64]);
        // Line 0 is now the least recently used of the two, because 0x80 went in after it.
        // A victim landing on it updates its data and its dirty bit...
        assert!(cache.write_victim(0x0000, &[0x33; 64]));
        assert_eq!(cache.line_data(0x0000), Some(&[0x33; 64][..]));
        let line = |address: u32| {
            cache
                .view("l2")
                .lines
                .into_iter()
                .find(|line| line.address == address)
                .expect("line is resident")
        };
        assert!(line(0x0000).dirty, "a victim dirties the line it lands in");
        // ...and does not promote it. If the victim had counted as a use, line 0x80 would now be
        // the least recently used one and the next miss in this set would take it instead.
        assert!(
            line(0x0000).used < line(0x0080).used,
            "a victim writeback must not update the replacement order"
        );
        let evicted = cache
            .insert(0x0100, vec![0x44; 64])
            .expect("the set was full, so the fill evicts");
        assert_eq!(
            evicted.address, 0x0000,
            "the line the victim landed on is still the one the set replaces"
        );
        assert!(evicted.dirty, "and it leaves carrying the victim's data");
        assert_eq!(evicted.data, vec![0x33; 64]);
    }
}
