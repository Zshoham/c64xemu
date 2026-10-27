# Memory and caches

The memory hierarchy of one megamodule — its L1P, L1D and L2 caches, its MAR cacheability bits,
its cache configuration and maintenance registers — and the backing store every master shares
behind it. This document covers the address map, cache geometry and the SRAM carve-out, cache
maintenance and freeze modes, the write buffer, coherence, the memory protection units, the
bandwidth manager, and what a miss costs. The seams it sits inside are
[Architecture](architecture.md)'s; IDMA and the EDMA3 are [Peripherals](peripherals.md)'s.

## Two objects, one access

`src/memory.rs` splits what a C64x+ device splits. A cache belongs to one megamodule and is
tagged by the address that megamodule issued; the bytes behind it belong to the device, because
every core's L2 SRAM also carries a global address any other master can reach. So
`MemoryHierarchy` owns three `Cache` values, 256 MAR registers and the cache configuration, one
per megamodule, and `Backing` owns the storage, one per device. The memory hierarchy holds no
bytes at all.

The two meet at `MemoryHierarchy::bind`, which returns a `BoundHierarchy` — two borrows and
nothing else. Every operation that moves bytes is issued on that value: `read_u32`, `write_u32`,
`fetch_packet`, `cache_register_write`, `host_invalidate`. Below that seam no helper re-supplies
the store, so a read, a write, a fetch and a cache maintenance name an address and nothing else.

`Backing` holds every core's L2, L1P and L1D SRAM in one flat array of `core_stride` bytes per
core, the shared L2 RAM and ROM of a multicore device, the DDR2 window as 64 KiB pages allocated
on first touch, and a `BTreeMap` for everything sparse. A DDR2 page never touched reads as zero
without allocating, and the local L2 window has a fast path of its own — `local_l2_run` is one
subtraction and one bounds check — because that is where code, stacks and most buffers live.
`Backing` also stamps a version on each written word (`note_memory_write`), which each core's
`LinkReservation` (held by its megamodule, asked through `Fabric::load_link`, `store_link` and
`commit_link`) compares to detect an intervening store. The stamps are the device's because any
master's write moves them; the reservation is the core's because a core has exactly one. Linked
atomics on SL2 do not come here: the shared-memory controller's monitors judge them (below).

## The address map

`src/space.rs` answers one question: given the requester and an address, which physical memory
does the access land in. `Placement` names the answer — `CoreL2`, `CoreL1P`, `CoreL1D`, `Shared`,
`SharedRom`, `Dram`, `ConvertedToCache`, `Sparse` — and `Requester` is either `Core(n)` or `Dma`.

A megamodule sees its own memories through local windows that mean nothing to any other master.
A multicore device gives every megamodule's memories a second, global address, and those are the
addresses another core, an EMAC, an EDMA3 transfer controller or the host port must use
(SPRS612G Table 2-2). The aliases are deliberately not canonicalised away: two addresses for one
byte are two cache tags on silicon, and firmware that mixes them sees stale data. `Backing`
resolves the alias so both land on the same bytes and leaves the tags alone, which reproduces
that hazard rather than hiding it.

`AddressMap::placement` checks the local windows first, because they are what a core issues
almost every cycle, then the DDR2 window, then the global alias block. A `Requester::Dma`
address resolves through the local windows only on a device with no global aliases — there is
just one megamodule to mean — which is why the C6455's EMAC and EDMA3 address L2 SRAM at
0080 0000h while a C6472 DMA master asking for 0080 0000h gets `Sparse`.

### The C6455 address map

One megamodule, no global aliases, no shared memory (SPRS276M section 2.3, Table 2-2).

| Range | Bytes | Placement |
|---|---|---|
| 0080 0000h – 009F FFFFh | 2 MB | local L2 SRAM |
| 00E0 0000h – 00E0 7FFFh | 32 KB | local L1P SRAM |
| 00F0 0000h – 00F0 7FFFh | 32 KB | local L1D SRAM |
| E000 0000h – EFFF FFFFh | 256 MB | DDR2 |
| anything else | — | sparse |

### The C6472 address map

Six megamodules with 608 KB of L2 SRAM each, and a global alias block striding 16 MB per core
from 1000 0000h (SPRS612G Table 2-2 and section 5.1).

| Range | Bytes | Placement |
|---|---|---|
| 0010 0000h – 001B FFFFh | 768 KB | shared L2 ROM |
| 0020 0000h – 002B FFFFh | 768 KB | shared L2 RAM |
| 0080 0000h – 0089 7FFFh | 608 KB | the requesting core's L2 SRAM |
| 00E0 0000h – 00E0 7FFFh | 32 KB | the requesting core's L1P SRAM |
| 00F0 0000h – 00F0 7FFFh | 32 KB | the requesting core's L1D SRAM |
| 1*n*00 0000h + the local offsets | — | core *n*'s memories, for any master |
| E000 0000h – FFFF FFFFh | 512 MB | DDR2 |
| anything else | — | sparse |

The shared ROM is the one place SPRS612G contradicts itself: Table 2-2 calls
0000 0000h–001F FFFFh reserved, while section 5.1 and Table 5-1 place 768 KB of ROM at
0010 0000h — which is also where Table 2-4 starts core 0 for every ROM boot mode. `C6472_MAP`
models it where the two sections that describe it say it is.

Not every address is memory. `is_memory` excludes the configuration space,
0180 0000h–03FF FFFFh, and the three peripheral data ports outside it: the UTOPIA cell queues at
3C00 0000h and the two McBSP data buses at 3000 0000h and 3400 0000h (SPRS276M Tables 7-105,
7-57 and 7-58). A DMA master reaches the [slave port](peripherals.md) rather than memory there.

### The shared L2 and the shared-memory controller

On a C6472 every access that names SL2 RAM - through the local window at 0020 0000h or any
core's global alias of it - is a request to the shared-memory controller
([SPRUEG5D](../reference/sprueg5d.pdf); `src/smc.rs`) unless the megamodule's own caches answer
it first. `AddressMap::shared_ram_offset` turns the address into an offset into the RAM, which is
the controller's whole view of it ("ADDR IN SMC", Table 11), so the two views of one word are one
address to it. The fabric asks on every CPU load, store and program fetch
(`Fabric::observe_sl2`: two subtractions and two compares for an address that is not SL2) and on
every master transfer (`note_sl2_transfer`). What a request does:

- It wakes the page it lands in (§4.3). Page 0 is the first 512 KB and page 1 the 256 KB after
  it (§4.2); the guide never says which addresses a page covers, so this is the in-order reading.
  The page stays awake until the next SL2PDSLEEP or SL2PDWAKE write merges the cores' permissions
  again.
- A read is served by the bank its 256-bit granule selects (address bits 6-5, the choice the guide
  leaves open) in three wait states, the figure for a request that misses the prefetch buffers
  (§2.4), and the requesting megamodule's profiler files it under both. A store is not a read
  request and is not counted. A program fetch is one request per line the L1P fills; a load is
  one per access the L1D and L2 caches do not answer, so a line fill is counted as the miss that
  caused it.
- A master has no port of its own on the SMC ("all the accesses to the SMC go through the L2
  controller", §1.3), so a master's transfer is the request of the megamodule whose global alias
  of SL2 it used, one per 256-bit bank word. That attribution is the model's reading.
- LL, SL and CMTL are the monitors' (§3.1; `Fabric::load_link`, `store_link`, `commit_link`).
  LL and CMTL are read requests whatever the caches hold; SL is a write to the monitor only and
  never to memory. An ordinary store does not break a link, which is the one place SL2 and the
  version-stamp reservation of the other memories differ.
- The SMC does not judge the access: it has "no memory protection support" (§1.2), so
  `admit` and the L2 controller's protection units decide first and a refused access never
  reaches it.

## Cache geometry

`src/cache.rs` is one `Cache` type parameterised by line size and associativity, with the
geometry set once in `MemoryHierarchy::new` (SPRU862B Table 1-2 and section 1.8, SPRS276M
section 4):

| Level | Line | Ways | Sizes | Configuration register |
|---|---|---|---|---|
| L1P | 32 B | 1, direct-mapped | 0, 4, 8, 16, 32 KB | L1PCFG at 0184 0020h |
| L1D | 64 B | 2, LRU | 0, 4, 8, 16, 32 KB | L1DCFG at 0184 0040h |
| L2 | 128 B | 4, LRU | 0, 32, 64, 128, 256 KB | L2CFG at 0184 0000h |

A line can only ever live in the set its address indexes — `set_of` is
`(address >> line_shift) & set_mask` — and only that set's ways compete for it, so a working set
landing in too few sets thrashes however much capacity is spare. Lookup goes through
`set_indices`, four slots per set; L1P has `read_bytes_direct`, a single slot test rather than a
way scan, because it is on the instruction fetch path.

Every cache comes out of reset disabled (`capacity == 0`), so all of every window is SRAM.
`l1_capacity` and `l2_capacity` read the three-bit mode field: 4 KB per step for L1, 32 KB per
step for L2, and **every encoding above 4 is the maximum**. That is not a guess. TI's own kernel
writes 7 to L1PCFG and L1DCFG in `L1_cache_on`, and `L2MODE_256K_CACHE` is 0x7, set at boot by
`L2_cache_set_mode` (Linux v5.10 `arch/c6x/platforms/cache.c`); reading those encodings as
reserved left the model running Linux with no caches at all. The 256 KB L2 ceiling holds whatever
the size of the L2 SRAM — the C6472's 608 KB L2 does not extend it, it only leaves more SRAM
behind it (SPRS612G section 5.1, SPRU871K Table 4-4).

## The carve-out rule

A cache size decides two things: the cache's capacity, and how much of the window it came out of
is still addressable SRAM. The conversion runs from the top downwards. SPRU862B section 2.3 says
of L2 that "when the L2 cache size is increased, the memory is taken from the high memory
addresses", and its Figure 2-1 places a 256 KB cache in a 2048 KB L2 at 009C 0000h; SPRU871K
section 2.4 says L1P "converts memory from RAM to cache by starting at the top of the L1P memory
map and working downwards", and section 3.3 says the same of L1D. So `sram_remaining` is a
saturating subtraction, what is left as SRAM is always a prefix of the window, and a cache as
large as the window — 32 KB of L1P or L1D at maximum size — leaves none of it.

`BoundHierarchy::report_sram` pushes the three remainders into `Backing::set_core_sram` on every
configuration change, and `Backing::placement` answers `ConvertedToCache` past them. That is the
one seam every master crosses, so an address the cache has taken answers as nothing to the CPU,
to a DMA master, to the snoop path and to an image loader alike: reads give zero, writes are
dropped. No document in this set says what silicon returns there, and answering as nothing is the
only claim TI's own Note licenses — that memory "is not valid for the linker to place code or
data in" (SPRU862B section 2.3).

It also stops holding what was written to it: `set_core_sram` zeroes the range that became cache,
because the cache is using those bits for its lines and tags. SPRU871K section 2.4.3 names the
consequence of growing a cache over live data — "data loss due to portions of L1P RAM becoming
cache" — and SPRU862B Table 2-6 is the copy-out procedure that avoids it. The order is the
hardware's too: `configure_l2` writes back and invalidates before the aperture moves (SPRU862B
section 2.7.2), so a dirty line whose address is about to stop being SRAM reaches memory first.

With `--cache`, which programs 256 KB of L2 (see [Command line](../guide/command-line.md)), the
2 MB L2 window keeps SRAM up to 009B FFFFh and the cache has 009C 0000h upwards — Figure 2-1's
own number. The debugger's `write` is the debug port, so the carve is not being bypassed here:

```
$ printf 'write 0x009bfff0 aabbccdd\nmem 0x009bfff0 16\nwrite 0x009c0000 aabbccdd\nmem 0x009c0000 16\nquit\n' \
    | ./target/release/c64xemu debug --cache tests/firmware/bios_kernel/build/bios_kernel.out
c64xemu debugger - 'help' for commands
pc=0x00812140 <_c_int00> cycle=0: mvk .l2 0,b0
(c64xemu) wrote 4 bytes at 0x009bfff0
(c64xemu) 009bfff0 aa bb cc dd 00 00 00 00 00 00 00 00 00 00 00 00  ................
(c64xemu) wrote 4 bytes at 0x009c0000
(c64xemu) 009c0000 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
```

The same rule from a different top on the C6472: 608 KB minus 256 KB leaves SRAM ending at
0085 7FFFh, and `--device c6472 --cache` gives the same pair of answers there.
`system::tests::a_cache_size_moves_the_end_of_the_sram_it_came_out_of` walks all four sizes and
the maximal encoding, over all three windows and both device profiles, through a DMA master —
the only requester that can ask about L1P, since "the CPU cannot read from L1P, even if L1P is
memory-mapped" (SPRU871K section 2.3.1.2).

## MAR, and what each cache may hold

The 256 memory attribute registers sit at 0184 8000h, one per 16 MB region, and bit 0 is the
cacheability bit. `MemoryHierarchy::new` sets region 0 — the on-chip memories — and leaves the
rest clear. `mar_write` accepts bit 0 only, and only for index ≥ 16: the on-chip regions are
read-only and always cacheable (`cache:mar-reset-and-permissions`). Three predicates read it:

- `l1_cacheable` — L1D allocation. Requires `is_memory`, the region's MAR bit, and that the
  address is not this megamodule's own L1 memory.
- `l2_cacheable` — requires the L2 cache enabled, MAR index ≥ 16 and the MAR bit set. L2 caches
  external addresses only (SPRU862B section 2.2), so an access to L2 SRAM goes straight past it.
- `l1p_cacheable` — **ignores MAR entirely**. SPRU862B section 2.3: on C64x+ devices "program
  fetches to external memory addresses are always cached in L1P, regardless of the cacheability
  setting", where C64x had MAR govern all three caches. Since L1P is never snooped the difference
  is observable, and it was a fixed defect: external code with MAR clear used to be refetched
  from memory every time.

All three refuse this megamodule's own L1P and L1D windows and the aliases of them inside this
core's own global alias block. SPRU871K section 2.3.1: "L1P memory can not be cached within Level
1 data (L1D) cache, Level 1 program (L1P) cache, or Level 2 (L2) cache within the same
megamodule", and section 3.2.1 says the same of L1D memory. Another core's L1 through the global
block is ordinary memory to this one and stays cacheable. The predicate is on the hottest path in
the emulator, so `own_l1` is one shift and four compares against the windows' high bits — both
L1 windows are a power-of-two size aligned to it. The whole window is refused, cache-converted
part included: an address the L1D configuration turned into cache is not memory, so caching it
would answer a read the storage refuses.

## Cache maintenance and freeze

`cache_register_read` and `cache_register_write` own the register file. The base-address
registers read back what was written; the word-count and global operation registers read as zero,
because an operation completes within the write. Every accepted write bumps `l1p_epoch`, which
invalidates the CPU's one-packet fetch buffer.

CPU writes also pass `Fabric::check_access` before reaching this register file. SPRU871K
Tables 2-12, 3-16 and 4-23 make L1PCFG, L1DCFG, L2CFG, MARs, L1DINV and L2INV read-only
to User mode. Such writes are discarded even when all SRAM pages are unrestricted. L1PCFG
additionally records LOCAL/UW in L1PMPFSR and raises the local protection event, as section
2.9.1.2 explicitly requires; the L1D/L2 tables specify no register-fault subtype. User freeze,
block coherence, global writeback/writeback-invalidate, and L1P invalidate writes remain
permitted. These CPU checks do not restrict DMA, EMAC or HPI configuration accesses.

| Register | Address | Effect of a write |
|---|---|---|
| L1PCFG / L1DCFG | 0184 0020h / 0040h | flush the cache, resize, re-report the SRAM remainder |
| L2CFG | 0184 0000h | a new L2MODE writes back and invalidates L2 only, resizes it and re-reports the remainder; bit 3 (L2CC) freezes L2; bits 9 and 8 (IP, ID) invalidate all of L1P or L1D |
| L1PCC / L1DCC | 0184 0024h / 0044h | bit 0 freezes; the previous state reads back in bit 16 |
| L2WBAR / L2WWC | 0184 4000h / 4004h | writeback range: L1D then L2 |
| L2WIBAR / L2WIWC | 0184 4010h / 4014h | writeback-invalidate range: L1D, L1P, L2 |
| L2IBAR / L2IWC | 0184 4018h / 401Ch | invalidate range: L1D, L1P, L2 |
| L1PIBAR / L1PIWC | 0184 4020h / 4024h | invalidate an L1P range |
| L1DWIBAR / L1DWIWC | 0184 4030h / 4034h | writeback-invalidate an L1D range |
| L1DWBAR / L1DWWC | 0184 4040h / 4044h | writeback an L1D range |
| L1DIBAR / L1DIWC | 0184 4048h / 404Ch | invalidate an L1D range |
| L2WB / L2WBINV / L2INV | 0184 5000h / 5004h / 5008h | global, on bit 0 |
| L1PINV | 0184 5028h | global, on bit 0 |
| L1DWB / L1DWBINV / L1DINV | 0184 5040h / 5044h / 5048h | global, on bit 0 |

A word-count register counts 32-bit words: `l1d_range_operation` and `l2_range_operation`
multiply by four for the byte length, and a line overlapping the range at either end is operated
on. The L2 operations reach down — an L2 writeback writes L1D back first, an L2 invalidate
invalidates L1P as well — and a global operation without bit 0 does nothing. Nothing here costs
cycles: a maintenance operation is not charged.

Freezing is `Cache::freeze`, and L1PCC/L1DCC keep the returned previous value in bit 16. A frozen
cache still serves hits and still updates a line it already holds, but `insert` allocates
nothing, so a miss is served from the level below and leaves no line
(`cache:l1d-freeze-blocks-allocation`). Frozen L1D read and write hits leave replacement order
unchanged (SPRU871K section 3.3.4), so the first conflicting fill after unfreezing evicts the
same line it would have before the frozen hits. L2CFG.L2CC freezes L2 the same way: SPRU871K
section 4.3.5 sends read and write misses "directly to external memory, as if L2 cache were not
present", so a frozen L2 miss neither fills nor allocates on a write
(`system::tests::a_frozen_l2_serves_hits_and_sends_misses_to_external_memory`).

L2CFG is three registers in one (SPRU871K Table 4-11), and an L2 mode change is narrower than the
L1 ones. "Changing L2's mode does not affect the contents of either L1 cache" (section 4.3.4) and
"changing of the L2 cache size will not cause any evictions from L1P or L1D cache" (SPRU862B
section 2.7.2), so `configure_l2` writes back and invalidates the L2 cache alone. The write-only IP
and ID bits are the legacy global invalidates of L1P and L1D: each invalidates its own cache and
nothing else, ID without writing anything back
(`system::tests::an_l2_mode_change_leaves_both_l1_caches_alone`,
`l2cfg_ip_and_id_invalidate_only_the_l1_cache_they_name`).

**[INFERENCE]** The L2 flush runs only when a write changes the L2MODE field. No manual says what
an L2CFG write that leaves L2MODE as it was does to the L2 cache: section 4.3.4 describes the
writeback-invalidate as what L2 does "when a program writes a new cache mode", and Table 4-5 only
covers switching to more or less cache. The model reads "new" as "different from the field's
current value", so a write that only sets IP, ID or L2CC leaves the L2 cache's contents alone -
which `l2cfg_ip_and_id_invalidate_only_the_l1_cache_they_name` pins by checking that a dirty L2
line is still unwritten afterwards. Two consequences follow from reading the field rather than the
size: rewriting the same mode does not flush, and moving between encodings that mean the same
256 KB - 4 to 7, say - does. Silicon could equally flush on every L2MODE write; a program would see
the difference only in whether its L2 lines survive such a write - when a dirty one reaches memory,
and whether a clean one is fetched again.

## The write buffer

L1D is read-allocate: `write_data_bytes` never fills, and a store that misses a cacheable
address goes into the write buffer instead. The buffer is four 128-bit entries between L1D and
L2, held by the megamodule's `MemoryHierarchy` (`L1dTiming::write_buffer` in `src/memory.rs`),
because what it costs is cycles rather than bytes — the bytes have already been written through.

`note_store` is the store side, and answers the stall cycles a full buffer costs on the spot. An
entry is the store's quad-word, `address & !15`; drained entries are swept first, then the merge
rule, then the queue:

- Two writes merge only into the *immediately preceding* entry, and only when the line is L2 SRAM
  rather than an address the L2 cache may hold — two of the six conditions in SPRU871K section
  3.5.3 (`timing:l2-cache-addresses-never-merge-in-the-write-buffer`).
- A fifth write miss with all four entries busy stalls until the head entry frees
  (`timing:a-full-write-buffer-stalls-the-next-write-miss`).
- The drain rate is SPRU862B Table 3-2's: two cycles per entry into L2 SRAM, six into the L2
  cache (`timing:write-buffer-drains-at-two-cycles-an-entry`,
  `timing:write-buffer-drains-at-six-cycles-an-entry-into-l2-cache`).

The read side is `data_stalls`: a read miss waits for the buffer to drain completely
before its own line fill starts (`timing:read-miss-waits-for-the-write-buffer`). Two of section
3.5.3's remaining conditions are deliberately unmodelled. "The first write has not yet been
presented to the L2 controller" is the drained-entry sweep; "both writes have the same privilege
level" is not distinguished, because this model carries no privilege level per buffered write.

The rules are pinned at the hierarchy seam in `src/memory.rs` as well: merging only into the
immediately preceding SRAM quad-word is not observable in a program (merging changes the queue's
depth, and depth only changes a cycle count when the drain is slower than the fill, which is
exactly when merging is forbidden), but a test that looks at how long a read miss then waits
sees it.

## Coherence

The protocol is what SPRU862B chapter 2 grants and nothing more: **L2 SRAM is snooped against
L1D, nothing else is, and L1P is never snooped at all.** `src/system/master.rs` (master access)
routes it. `snooped_l2` asks `Backing::placement` for the DMA address and answers only for
`CoreL2`, so external memory is outside the protocol whatever the MAR says and memory the core has
converted into cache is not L2 SRAM either. It returns the owning megamodule and the local
address that core would have used, and `memory_bytes` and `store_memory_bytes` apply the snoop
under both tags, because on a multicore device the line may be tagged with either.

- A DMA read is served from L1D where that line is dirty (`snoop_read` →
  `Cache::snoop_read_range`), so a buffer the CPU has only written into its cache still goes out
  on the wire. The line stays dirty: the data is forwarded, not written back on the way past
  (SPRU862B Figure 2-4).
- A DMA write is forwarded into any L1D line holding that address (`snoop_write` →
  `Cache::snoop_write_range`), so the CPU sees it with no cache operation. It does not dirty the
  line — memory now has the same bytes — and does not move the replacement order, since no CPU
  access happened (SPRU862B section 2.4.2: "the dirty bit is not affected by this operation").

Three other paths are not the DMA path and differ on purpose. `host_read_bytes` is the debug
port, and the debug port sees what the CPU wrote, so `host_snoop_read` reads dirty lines out of
**both** L2 and L1D; without it, `printf` from a program with L1D enabled prints nothing.
`host_write_bytes` goes into memory, so `host_invalidate` invalidates L1P and invalidates L1D and
L2 with writeback of anything dirty. `load_bytes` — an image loader or a boot table — writes
straight into the storage, behind every cache.

The firmware bug the L1P rule exposes is the self-modifying-code case of SPRU862B section 2.6,
and it is a real program. `tests/programs/idma_l1p_stale_code` enables L1P as a 32 KB cache, calls
a routine in L2 SRAM so its fetch packets are cached, has IDMA1 overwrite that routine with one
returning a different constant, and calls it again: the old answer comes back — 273 twice — and
only a range invalidate through L1PIBAR and L1PIWC brings the replacement in, giving 546. A model
that snoops L1P produces a different checksum. Firmware that moves code with a DMA master and
does not invalidate L1P has this bug, and the emulator shows it rather than papering over it. The
same applies to any buffer outside L2 SRAM, which is why the NIMU driver in
`tests/firmware/ndk_tcp/nimu_emac.c` calls `Cache_inv` on a received frame before the stack reads
it — invalidate, not writeback-invalidate, or a stale line would be pushed back over the new
data. See [Ethernet](ethernet.md) for the driver side and [Peripherals](peripherals.md) for IDMA and the EDMA3,
which reach memory through this one path.

An L1D victim is not an ordinary write. SPRU871K section 4.3.8.3: "L1D victim writebacks do not
trigger line allocations in L2. L1D victims are written directly to external memory if they miss
L2. L1D victim writebacks also do not update L2's LRU if they hit in L2. They do update L2's
dirty status as needed." So `write_l1_line` goes through `Cache::write_victim`, which answers
`false` on a miss and sends the victim past L2 to the store, and on a hit updates the data and
the dirty bit while leaving the replacement clock alone
(`cache::tests::a_victim_writeback_neither_allocates_nor_promotes_the_line_it_lands_in`,
`system::tests::an_l1d_victim_that_misses_l2_does_not_allocate_a_line_there`).

## Memory protection

Each of L2, L1P and L1D has a protection unit: fault address, status and clear registers at a
base, and the per-page attribute registers 0x200 further on — L2 at 0184 A000h / 0184 A200h, L1P
at 0184 A400h / 0184 A600h, L1D at 0184 AC00h / 0184 AE00h. Page geometry is a field of the
[device profile](architecture.md), not a branch: the C6455 uses 32 pages of 64 KB over L2 and 32
pages of 1 KB over each L1; the C6472 uses 19 pages of 32 KB over its 608 KB L2 plus L2MPPA32–43
for the shared L2 in 64 KB pages, and 16 pages of 2 KB over each L1 at MPPA16–31.

An MPPA register carries LOCAL in bit 8, SR SW SX UR UW UX in bits 5–0, and the AID enables above
them. Reset is 0000 FFFFh, which grants everything, so firmware that never programs a unit sees
no page faults; `mpu_restrictive` and `mpu_dma_restrictive` are recomputed on every MPPA write so
the common page-check case is one boolean test on the access path.

`Fabric::check_access` checks a CPU access against the page of whichever local memory holds it,
consulting LOCAL plus the supervisor or user permission bit — the AID fields gate DMA masters by
requester id, and the CPU is the local requester. `check_dma_access` checks the AID bit for the
requester id (IDs 0–5 have their own bit, 6 and above share AIDX) and the permission bit for the
requester's privilege mode: IDMA and the HPI's DMA are ID 0 supervisor, the EMACs and the UTOPIA
PDMA ID 0 user (SPRS612G section 5.2 and Table 5-3); `Privilege` carries the pair, and
`Privilege::allowed_id_bit` is the one AID decode both this check and the EDMA3's use. The
EDMA3's transfers are made for whoever programmed them: every write of a PaRAM word stamps PRIV
and PRIVID into the set's OPT (bit 31, bits 27–24) from the identity the fabric last presented
to the channel controller — a CPU access presents its core index and mode through
`check_access`, a system master's register access its own through master access
(`register_word`/`store_register_word`) — and `Edma::transfer` issues the reads and writes under the stamp the
set carries when the request runs, not the CPU's privilege at that moment (SPRU966C section
2.10.2). Any new path into the EDMA3's slave port has to present its identity the same way, or
it is judged and stamped as whoever came before it. The channel controller's own register
protection judges the presented PrivID against the AID bits, so a system master's write to an
EDMA3 register is its own: an EMAC or UTOPIA PDMA write (ID 0 user) to a page attribute register,
which is supervisor-only, is refused. A refusal there records the PrivID in MPFSR's FID
(bits 12–9) beside the error bit (SPRU966C Table 4-27).
A refused access is discarded, and `record_mpu_fault` latches the address in MPFAR and the
attempted permission in MPFSR and raises the unit's system event: 124, 120, 122 for a CPU fault
on L2, L1P, L1D, and 125, 121, 123 for a requester's. That event reaches the CPU as `EXCEP`
through the exception combiner — see [Interrupts and exceptions](interrupts.md).

The EMAC's descriptor words cross the same check as its packet buffers, with PrivID 0 in user
mode (`MasterAccess::read_word`/`write_word`). A refused descriptor read returns zero, so a
transmit descriptor without UR sends nothing; the zero also reads as a descriptor the host did
not hand over, which the model reports as the owner host error. Whether silicon raises that
error for a faulted read is unverified, and no case pins it. A refused write is discarded, so
ownership stays and the fault is recorded
with UW. Protection reports a violation "to the operating environment" (SPRU871K 8.3.3.2), not
to the requestor, so the EMAC is not told its write went nowhere: it still sends or stores the
frame, advances the head pointer, and posts the descriptor in the completion pointer, raising
the channel interrupt while that descriptor still shows OWNER. Descriptors outside memory, in
the EMAC's own descriptor RAM in the peripheral aperture, are not protected.
`memory:emac-{tx,rx}-descriptor-without-user-read`, `-without-user-write` and
`-with-user-access` cover both directions.

Every system-master word - EMAC descriptor words and the HPI's and IDMA's words - is judged on
all four of its bytes by `admit`, the one judge of master access (`src/system/master.rs`),
as byte transfers are. Each descriptor word is a separate access, so a descriptor that
straddles a page boundary is judged word by word: a refused word reads as zero while its
neighbours on the other page are read normally. SPRU871K does not say how an access spanning
two pages is judged, nor whether the EMAC fetches a descriptor as one burst (SPRU975E is
silent too), so this per-word rule is the model's choice rather than a quoted one.

A unit holds its first fault and raises nothing further until software clears it (SPRU871K
8.4.2.1.3). The one exception is per-unit and quoted rather than generalised: L1P (2.9.2.3.3) and
L2 (4.6.2.4.3) let a local fault replace a remote one and signal a new exception, and L1D's own
section carries neither sentence, so `local_replaces_remote` is false for L1D alone
(`memory:a-local-fault-replaces-a-remote-one`,
`memory:l1d-holds-its-first-fault-whatever-its-source`). MPFCLR takes bit 0 and nothing else —
"Writing a 1 to any other bit of the MPFCR register has no effect" — so a write that misses the
command bit leaves the recorded fault standing (`memory:fault-clear-takes-its-command-bit-only`).
The lock sequence is modelled too, four key words then a set, unlock or start command, and a
command out of sequence is itself a local fault with SW (`memory:protection-lock-sequence`).

## The bandwidth manager

Twelve arbitration registers, with reset value and writable field mask from SPRU871K Table 6-2:
CPUARBU, IDMAARBU, SDMAARBU and UCARBU at 0184 1000h; the `*ARBD` set at 0184 1040h; and
CPUARBE, IDMAARBE, SDMAARBE and MDMAARBE at 0182 0200h. They hold their values and mask off the
reserved fields, so firmware that programs bandwidth arbitration reads back what the hardware
would (`memory:bandwidth-arbitration-registers`). The arbitration itself is not modelled: no
access's cost changes with them.

## What a miss costs

Every number here is SPRU862B's for **this device's** L2 — 2048 KB at 0080 0000h, 2-cycle
latency, 4 × 128-bit banks (SPRS276M section 4), which is the tables' 1-wait-state column.
The CPU owns none of this arithmetic. `MemoryHierarchy` holds the L1D's timing state beside the
caches it times, and the CPU's whole part is to say what it accessed and when: `note_load` and
`note_store` for each data access, `data_stalls` once the cycle's packets have executed, and
`fetch_stalls` for the program fetch. The clocks are the CPU's and cross the seam as a
`DataClock` — the execute cycle, which stall cycles do not advance and which "the cycle right
after another miss" is counted in, and the wall clock, which they do and which the write buffer
drains on — so the hierarchy never reaches back into the CPU. The stall cycles come back as
numbers and the CPU adds them to `pending_stalls`. A cycle that issues from the SPLOOP buffer
alone asks exactly as a program-memory cycle does: the buffer spares the program fetch
(SPRU862B §3.4.3.1), not the data access, and SPRU732J §7.15.2 exempts only the cross path and
AMR use stalls from it (`timing:sploop-replayed-bank-conflicts-stall-every-iteration`,
`timing:sploop-replayed-read-misses-cost-what-straight-line-ones-do`).

`MemoryHierarchy::access_timing` answers all three facts one data access tells the timing model
at once — resident, read-miss, and whether the fill's line comes from the L2 cache — for the
address the access issued, so the write buffer's merge rule and the miss cost cannot disagree
about the same line. Only the SRAM the L1D configuration has left answers as resident in one
cycle; the rest of the window is the cache itself. `data_stalls` collects the distinct 64-byte
lines this cycle missed on and charges, from SPRU862B Table 3-2:

| | out of L2 SRAM | out of the L2 cache |
|---|---|---|
| a single miss | 12.5 cycles | 14.5 cycles |
| a miss in the cycle after another | 3 | 7 |
| each further miss in the same cycle | 4 | 8 |

The half cycle is real: the access lands on one of the two phases of the CPU/2 L2 clock, only the
average is documented, so it is spent as an alternation over successive full-price misses
(`L1dTiming::miss_phase`). Reading the phase off the issue cycle's parity instead ties it to a
loop's period, and made SPRU862B §3.4.7's worked totals come out 64 and 32 cycles high. Table
3-2's fourth row is what makes the pipelined discount conditional — M consecutive *parallel* misses
cost 12.5 + 4(M−1), four for every miss after the first rather than 3 plus 4 per cycle — so the
discount holds only while misses arrive one per cycle. A victim buffer flush on the previous miss
"disrupts miss pipelining" (Table 3-2), which `MemoryHierarchy` records when a fill evicts a
dirty line and `data_stalls` honours by refusing the discount; the up-to-ten extra stalls the
same row mentions are not modelled, because the disruption needs no number and they do. A line
the L2 cache misses comes from external memory, whose latency neither table quantifies, and is
charged as an L2 cache line rather than given an invented number.

The costs are pinned two ways. At the seam, the tests in `src/memory.rs` drive `note_load`,
`note_store` and `data_stalls` with no instruction stream and assert each number in the table,
the alternation, pipelining across stall cycles, the dirty-eviction disruption, the write-buffer
rules above and the bank conflict. As executed cycles: `timing:l1d-single-read-miss` is 13 or
14, `timing:l1d-single-read-miss-out-of-l2-cache` 15 or 16, `timing:l1d-consecutive-read-misses`
25 or 26, `timing:l1d-parallel-read-misses` 17 or 18. Two of SPRU862B §3.4.7's worked examples
are reproduced exactly through the CPU: `cpu::tests::the_manuals_dotprod_read_miss_total` gets
the manual's 1600 stall cycles from 128 misses, and
`the_manuals_vecaddc_read_miss_and_write_buffer_total` gets both 800 for the read misses alone
and Table 3-4's "12.5 + ((12.5 + 1) × 63) = 863" with the write buffer in the picture.

### The line-occupancy argument

L1P is priced differently, because the fetch path pipelines. The hierarchy records the fills that
happened rather than answering a residency question before the fetch — a C64x+ execute packet
may span two fetch packets, so one packet's fetch can fill two lines — and
`MemoryHierarchy::fetch_stalls(now)` charges each exactly once, keeping the fetch path's
occupancy (`FetchTiming`) alongside the caches. It is the same kind of megamodule hardware as
the L1D rules above, so it lives in the same place; the CPU supplies only the clock.

SPRU862B Table 1-3 gives the average stalls per execute packet for a straight-line miss stream at
each packet width, over two L2 configurations: thirty-two numbers. One occupancy per line explains
all of them. With the fill of the next line overlapped against execution of the current one, a
width-*n* packet consumes a 32-byte line in 8/*n* cycles, so a line occupying the path for *k*
cycles stalls max(0, *k* − 8/*n*) per line, which is max(0, *k*·*n*/8 − 1) per packet. Solving
against the 1-wait-state column gives *k* = 14/3 for a line out of L2 SRAM and *k* = 6 for one
the L2 cache holds. The average is what the table gives, so the fraction is spent as a repeating
5, 5, 4 over successive fills (`l1p_fill_phase`) — which fill lands on which phase is not
documented. A sequential line is credited the run-ahead it already had; a branch target's fill
can only begin once the branch has redirected the fetch stream.
`timing:l1p-miss-stream-eight-wide` is 20 cycles, `timing:l1p-miss-stream-four-wide` 21 and
`timing:l1p-line-out-of-l2-cache-costs-more` 25, with `timing:l1p-as-sram-takes-no-fill` and
`timing:l1p-miss-pipelining-covers-narrow-packets` holding the two ends.

## Seeing it run

`--perf` reports each cache's geometry and its counters. SYS/BIOS programs the L1 caches itself,
so the report below is the same with and without `--cache` except for the L1P miss counts — and
L2 comes back disabled, because the firmware writes L2CFG back to zero after the `--cache`
preamble set it to 256 KB:

```
$ ./target/release/c64xemu run --cache --perf tests/firmware/bios_kernel/build/bios_kernel.out --state-out /dev/null
bios kernel consumed 64 relayed 64 outer 22 inner 22 nested 22 swi 22 woken 16 ...
...
  L1P  1024 lines, 32768 bytes, 1 ways, enabled=true
           100.0% read hit  (    16312837 hits,         3707 misses)
             0.0% write hit (           0 hits,            0 misses)
            3707 line fills,         2356 evictions,            0 writebacks,          600 invalidations

  L1D  512 lines, 32768 bytes, 2 ways, enabled=true
           100.0% read hit  (     1640823 hits,          153 misses)
            96.0% write hit (     1417334 hits,        58605 misses)
             153 line fills,           16 evictions,            7 writebacks,            4 invalidations

  L2  0 lines, 0 bytes, 4 ways, enabled=false
...
```

The 58605 write misses against 153 line fills are the read-allocate policy: those stores went
through the write buffer, not into a line. The debugger's `cache` command shows the resident lines
with the set each occupies, and inspection is passive — no counter, line or replacement clock
moves:

```
(c64xemu) continue 400000
cycle budget exhausted at pc=0x00812c06
pc=0x00812c06 <$C$L602+0x2> cycle=400000: add .l2 b5,-1,b5
(c64xemu) cache
L1P: enabled 1024 lines of 32 B in 1 ways (254 resident)
     32768 B total
     set   12  0x00808180  clean
...
L1D: enabled 512 lines of 64 B in 2 ways (73 resident)
     32768 B total
     set    0  0xe0000000  dirty
     set    1  0xe0000040  dirty
...
L2: disabled 0 lines of 128 B in 4 ways (0 resident)
     0 B total
```

Those dirty DDR2 lines are the firmware's own MAR programming at work: without MAR224's
cacheability bit nothing external would allocate at all. See
[Debugging](../guide/debugging.md) for the rest of the inspection surface and
[Running the tests](../guide/testing.md) for how the vector cases named throughout are run.

## Not modelled

- Bandwidth arbitration: the registers hold their values, the priorities change no cost.
- The extra stalls a victim buffer flush adds beyond disrupting miss pipelining.
- External memory latency as such: a line the L2 cache misses is charged as an L2 cache line.
- Cache maintenance costs no cycles, and no operation reports itself as in progress: the
  word-count registers read as zero.
- The privilege-level condition on write buffer merging.
- An L2 freeze register.
- The shared L2's prefetch buffers, per-bank arbitration and write token controller
  ([SPRUEG5D](../reference/sprueg5d.pdf) §2.3 to §2.5), and every SL2 wait state beyond the
  three a read that misses them is documented to take. Nothing prefetches, so SL2PFCPEN changes
  no access and SL2PFCCNT never counts. A sleeping SL2 page wakes on the access that finds it,
  and the access waits no longer for it.
