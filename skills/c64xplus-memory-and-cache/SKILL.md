---
name: c64xplus-memory-and-cache
description: "Model the C6455 memory map, C64x+ L1P, L1D, and L2 caches, coherence, IDMA, bandwidth management, and memory protection."
---

# C64x+ memory and cache

*The C64x+ megamodule memory system — the C6455 memory map, L1P/L1D/L2 geometry and control registers, coherence rules and what a functional emulator must model versus may ignore, IDMA, bandwidth management and memory protection.*

## Design position for a functional emulator

The user's target is **instruction-accurate, not cycle-accurate**, but with caches modelled.
Resolve that as follows:

- Model **cache state** (tags, valid, dirty, LRU) and **cache control operations** faithfully,
  because firmware issues writebacks and invalidates around DMA and *the data it sees
  depends on them*.
- Do **not** model miss penalties, bank conflicts, or the bandwidth manager as cycle costs.
- Treat the cache as a coherence oracle: every access goes through it so that a missing
  `L2WBINV` before an EMAC transmit shows up as stale data in the emulator, exactly as on
  silicon.

If you skipped cache modelling entirely (flat memory), firmware bugs of that class would
silently *work* in the emulator and fail on hardware — the opposite of the stated goal.
This is why TI's own CPU-cycle-accurate simulators use flat memory and their *device*
simulators model the cache (SPRU600I §1.4.2).

## C6455 memory map (SPRS276M Table 2-2)

| Range | Size | Contents |
|---|---|---|
| `0x0000_0000`–`0x000F_FFFF` | 1 MB | reserved |
| `0x0010_0000`–`0x0010_7FFF` | 32 KB | internal ROM (bootloader) |
| `0x0080_0000`–`0x009F_FFFF` | 2 MB | **L2 SRAM** |
| `0x00E0_0000`–`0x00E0_7FFF` | 32 KB | **L1P SRAM** |
| `0x00F0_0000`–`0x00F0_7FFF` | 32 KB | **L1D SRAM** |
| `0x0180_0000`–`0x01BF_FFFF` | 4 MB | **C64x+ megamodule registers** (INTC at `0x0180_0000`, cache/MPU at `0x0184_0000`) |
| `0x0288_0000` | 256 KB | HPI control |
| `0x028C_0000` / `0x0290_0000` | 256 KB each | McBSP0 / McBSP1 registers |
| `0x0294_0000` / `0x0298_0000` | | Timer 0 / Timer 1 registers |
| `0x029A_0000` / `0x029C_0000` | 512 B each | PLL1 (incl. reset controller) / PLL2 |
| `0x02A0_0000` | 32 KB | EDMA3 channel controller |
| `0x02A2_0000`, `0x02A2_8000`, `0x02A3_0000`, `0x02A3_8000` | 32 KB each | EDMA3 TC0–TC3 |
| `0x02A8_0000` | 256 KB | chip-level registers |
| `0x02AC_0000` | 256 KB | device state control |
| `0x02B0_0000` | 16 KB | GPIO |
| `0x02B0_4000` | 256 KB | I2C |
| `0x02B4_0000` | 512 B | UTOPIA control |
| `0x02B8_0000` / `0x02BA_0000` | 128 KB each | VCP2 / TCP2 |
| `0x02C0_0000` | 256 KB | PCI control |
| **`0x02C8_0000`** | **4 KB** | **EMAC port registers** |
| **`0x02C8_1000`** | **2 KB** | **EMAC control module registers** (EWCTL, EWINTTCNT) |
| **`0x02C8_1800`** | **2 KB** | **MDIO registers** |
| **`0x02C8_2000`–`0x02C8_3FFF`** | **8 KB** | **EMAC descriptor RAM** (512 descriptors) |
| `0x02D0_0000` | 256 KB | RapidIO control |
| `0x02E0_0000` | 16 KB | RapidIO CPPI RAM |
| `0x3000_0000` / `0x3400_0000` | 256 B each | McBSP0 / McBSP1 data |
| `0x3C00_0000` / `0x3C00_0400` | 1 KB each | UTOPIA Rx / Tx queues |
| `0x4000_0000` | 256 MB | PCI external memory |
| `0x5000_0000` / `0x5800_0000` | 128 MB each | TCP2 / VCP2 data |
| `0x7000_0000` | 128 MB | EMIFA (EMIF64) config registers |
| `0x7800_0000` | 128 MB | DDR2 controller config registers |
| `0xA000_0000` / `0xB000_0000` / `0xC000_0000` / `0xD000_0000` | 8 MB each | EMIFA CE2/CE3/CE4/CE5 |
| `0xE000_0000`–`0xFFFF_FFFF` | 512 MB | **DDR2 SDRAM (CE0)** |

EMIFA CE0/CE1 are not functional on the C6455 and are not pinned out.

For an Ethernet-only workload the essential regions are: L2 SRAM, L1P/L1D SRAM, megamodule
registers, EMAC + descriptor RAM, timers, PLL, EDMA3, and DDR2. Everything else can be a
"reads return 0, writes logged" stub — but *do* implement the stub, and log accesses to
reserved ranges. TI's simulator has a "reserved memory access detection" feature
(SPRU600I §2.2) for exactly this reason, and it catches real firmware bugs.

## Cache geometry (SPRU862B Tables 1-2, 1-4, 1-6)

| | L1P | L1D | L2 |
|---|---|---|---|
| Organization | direct-mapped | 2-way set-associative | 4-way set-associative |
| Line size | **32 B** | **64 B** | **128 B** |
| Capacity | 4 / 8 / 16 / 32 KB | 4 / 8 / 16 / 32 KB | 32 / 64 / 128 / 256 KB |
| Protocol | read-allocate | read-allocate, **write-back** | read **and write** allocate, write-back |
| Replacement | n/a | LRU | LRU |
| CPU access | 1 cycle | 1 cycle | — |
| Write buffer | — | 4 × 128-bit | — |
| External memory cacheability | always cached | configurable | configurable |

The key asymmetry: **L1D is read-allocate only**. A CPU write that misses L1D goes through
the write buffer to L2 without allocating a line. This is why, in TI's double-buffering
idiom, output buffers never need L1D writeback — they were never cached there.

Memory on the C6455 is *configurable* between SRAM and cache. L1P and L1D are **32 KB each**,
and whatever is configured as cache is carved out of that 32 KB. L2 is 2 MB of SRAM, of which
up to 256 KB can be configured as cache. The cache-size fields must therefore also shrink the
visible SRAM aperture at `0x00E0_0000` / `0x00F0_0000` / `0x0080_0000`.

## Cache control registers (SPRU871K, base `0x0184_0000`)

| Address | Register | Function |
|---|---|---|
| `0x0184_0000` | L2CFG (legacy name: CCFG) | L2 cache size and mode |
| `0x0184_0020` | L1PCFG | L1P cache size |
| `0x0184_0024` | L1PCC | L1P freeze mode (`OPER`, `POPER`) |
| `0x0184_0040` | L1DCFG | L1D cache size |
| `0x0184_0044` | L1DCC | L1D freeze mode |
| `0x0184_4000` / `4004` | L2WBAR / L2WWC | L2 block writeback: base address + word count |
| `0x0184_4010` / `4014` | L2WIBAR / L2WIWC | L2 block writeback-invalidate |
| `0x0184_4018` / `401C` | L2IBAR / L2IWC | L2 block invalidate |
| `0x0184_4020` / `4024` | L1PIBAR / L1PIWC | L1P block invalidate |
| `0x0184_4030` / `4034` | L1DWIBAR / L1DWIWC | L1D block writeback-invalidate |
| `0x0184_4040` / `4044` | L1DWBAR / L1DWWC | L1D block writeback (leaves lines valid) |
| `0x0184_4048` / `404C` | L1DIBAR / L1DIWC | L1D block invalidate (no writeback) |
| `0x0184_5000` | L2WB | global L2 writeback |
| `0x0184_5004` | L2WBINV | global L2 writeback-invalidate |
| `0x0184_5008` | L2INV | global L2 invalidate |
| `0x0184_5028` | L1PINV | global L1P invalidate |
| `0x0184_5040` | L1DWB | global L1D writeback |
| `0x0184_5044` | L1DWBINV | global L1D writeback-invalidate |
| `0x0184_5048` | L1DINV | global L1D invalidate (also reachable via the ID bit in CCFG) |
| `0x0184_C040`-`C064` | L2PDWAKE0/1, L2PDSLEEP0/1, L2PDSTAT0/1 | L2 power-down control |

(Addresses from SPRU871K Tables 2-6, 3-6 and 4-6.)

Semantics to implement:

- **Block operations**: write base address to `xBAR`, then **write the word count to `xWC`
  to trigger**. The operation runs over all lines *touched by* the range — round the range
  out to line boundaries. `xWC` reads back as 0 (or decrements) while in progress; firmware
  polls it for completion.
- **Global operations**: write 1 to the `I`/`C` bit to start; the bit reads back until done.
- A functional emulator can complete every cache operation instantly, but must still make
  the *effect* correct (dirty lines flushed to the next level, tags invalidated).
- L1P is also affected by L2 operations. An L2 writeback-invalidate invalidates the
  corresponding L1P and L1D lines.
- **Freeze mode** (`L1PCC.OPER`, `L1DCC.OPER`): the cache still services hits but does not
  allocate new lines or evict. `POPER` holds the previous value so an ISR can save/restore it.
- Cache config registers are Supervisor-writable only (SPRU871K §2.6.4).

The legacy `CSR.PCC`/`CSR.DCC` fields are **ignored on the C64x+** — firmware must use
L1PCFG/L1DCFG/L2CFG. Accept writes, ignore effect.

## Memory attribute registers (MARn) — external cacheability

**Easy to miss and consequential: on reset the entire external address space is
non-cacheable.** The L2 controller holds 256 memory attribute registers that decide whether
each 16 MB region may be held in cache at all.

| | |
|---|---|
| Address | `0x0184_8000 + 4n`, for `MAR0` … `MAR255` |
| Region covered | `MARn` covers `n << 24` … `(n << 24) + 0xFFFFFF` (16 MB) |
| Layout | bits 31-1 reserved; **bit 0 = PC (permit copies)** |
| PC = 0 | region **not** cacheable |
| PC = 1 | region cacheable |
| Reset value | **0 — the whole address space is non-cacheable by default** |
| Access | read/write by **Supervisor only** |

Fixed exceptions:

- **MAR0** covers local L2 RAM and is read-only with **PC always reading 1**.
- **MAR1-MAR15** cover internal and external configuration space; read-only with PC = 0.
- Software therefore only ever writes MAR16-MAR255.

On the C6455 the ranges that matter here are **MAR224-MAR255** (`0xE000_0000`-`0xFFFF_FFFF`,
the 512 MB of DDR2 — 32 registers) and **MAR160 / MAR176 / MAR192 / MAR208** (the 8 MB
EMIFA CE2/CE3/CE4/CE5 windows, one MAR each since each window sits at the base of its
16 MB MAR region).

How it composes with the rest of the model:

- On an L1P or L1D miss that reaches L2 for an address held in neither L2 RAM nor L2 cache,
  the L2 controller consults the MAR for that address. PC = 0 makes it a non-cacheable
  access, serviced directly from external memory with no allocation at any level.
- This interacts directly with the Ethernet path. If firmware never sets the MAR for its
  DDR2 packet-buffer range, those buffers are uncached and the coherence problem below
  simply does not arise. If it *does* set PC = 1 — which it will, for performance — then
  every DMA-visible buffer in DDR2 needs explicit writeback/invalidate. **Model the MARs, or
  you will model the wrong one of those two worlds.**
- TI's own warning is worth repeating: the C `volatile` keyword does *not* make a variable
  uncached. Firmware polling a hardware-updated location must both mark it `volatile` and
  clear the PC bit of the containing MAR.

## Coherence — the rules the emulator enforces

Hardware maintains coherence **only between the CPU and DMA for accesses to L2 SRAM**, via
snoops. Specifically (SPRU862B ch.2):

- DMA *write* to L2 SRAM: the L2 controller snoop-writes into L1D so the CPU sees fresh data.
- DMA *read* from L2 SRAM: L2 checks whether the line is cached dirty in L1D and forwards.
- **No hardware coherence for external memory (DDR2/EMIFA)** — *when it is cached at all*.
  Whether it is depends entirely on the MAR PC bit for that 16 MB region (previous section).
  With PC = 0 the region is never cached, so a DMA master and the CPU always see the same
  bytes and there is nothing to maintain. With PC = 1, a buffer in DDR2 touched by both the
  CPU and a DMA master (EDMA3, or the EMAC's own CPPI DMA) requires software to issue the
  right cache operation. This is the number-one source of "works in the emulator, fails on
  hardware" — so model both worlds and let the second one fail.

Practical rule set to implement in the memory layer:

1. Every CPU access consults L1D → L2 → external, updating tags/dirty/LRU — but only
   allocates for an external address if that region's MAR PC bit is 1.
2. Every DMA-master access (EMAC descriptor and buffer traffic, EDMA3, IDMA) enters at the
   L2 port. For addresses inside L2 SRAM, perform the snoop. For addresses outside, go
   straight to external memory — **bypassing** any stale line the CPU still holds.
3. Provide a debug mode that flags "DMA read of an address that is dirty in L1D/L2 outside
   the snooped region" — that is a firmware bug, and catching it is a feature.

## IDMA (SPRU871K ch.5)

Two channels of local DMA inside the megamodule:

- **IDMA0** — programs peripheral configuration registers in bulk.
- **IDMA1** — block transfers **between L1P, L1D and L2**, and to/from the peripheral
  configuration space. It does **not** reach external memory.

Registers are in the megamodule range; completion raises system events **13 (IDMA0)** and
**14 (IDMA1)**; invalid parameters raise event **97 (EMC_IDMAERR)**. Worth implementing:
NDK and DSP/BIOS code uses IDMA1 for fast L2↔L1 staging.

## Bandwidth management (SPRU871K ch.6)

Arbitration priorities between CPU, IDMA, and slave DMA for L1P/L1D/L2/configuration-bus
access, with a starvation counter. **Timing-only** — a functional emulator implements the
registers as plain storage and ignores the effect.

## Memory protection (SPRU871K ch.8)

Per-page attribute registers `L1PMPPAn` (`0x0184_A600`+), `L1DMPPAn` (`0x0184_AE00`+),
`L2MPPAn` (`0x0184_A200`+), 32 pages each. Each attribute register carries:

```
15 AID5 | 14 AID4 | 13 AID3 | 12 AID2 | 11 AID1 | 10 AID0 | 9 AIDX | 8 LOCAL
 5 SR | 4 SW | 3 SX | 2 UR | 1 UW | 0 UX
```

`AIDn` grants access to DMA master ID *n* (`AIDX` covers IDs ≥ 6); `LOCAL` covers CPU access
to local memories; `SR/SW/SX` and `UR/UW/UX` are supervisor/user read/write/execute.

Violations latch an address in `L1PMPFAR` / `L1DMPFAR` / `L2MPFAR` and a cause in the
matching `MPFSR`, cleared through `MPFCLR`, and raise an exception to the CPU. There are also
lock registers (`MPLK0-3`, `MPLKCMD`, `MPLKSTAT`) implementing a key sequence that prevents
further changes.

For a single-image bare-metal firmware this is usually left wide open. Implement the
registers as storage and the check as an optional mode — but do implement the fault
registers, because DSP/BIOS and Linux both read them on exception.

## Power-down (SPRU871K ch.9)

`PDCCMD` and the `CSR.PWRD` field. `IDLE` plus a PWRD mode halts the CPU until an interrupt.
Implement `IDLE` as "advance time until the next pending interrupt"; the rest of the PDC can
be register storage.

## Endianness

Selected at boot and readable in `CSR.EN`. Affects data access, instruction word extraction,
and the ABI. Make it a parameter of the memory layer, not a compile-time constant.

## Files that expand on this

**Documents**

- `docs/reference/sprs276m.pdf` (SPRS276M) §2.3, pp. 11-12 — Table 2-2, the complete
  C6455 memory map reproduced above; §2.4 the boot sequence; §7 the peripheral overviews
- `docs/reference/spru871k.pdf` (SPRU871K) — the megamodule itself
  - ch.2, pp. 23-40 — L1P: regions, cache architecture, freeze mode, coherence operations,
    the register table (2-6) and per-register bit tables, memory protection
  - ch.3, pp. 41-80 — L1D, including Table 3-6, the L1D cache-control register addresses
  - ch.4, pp. 81-124 — L2: §4.3.7 and §4.4.4-4.4.5 (pp. 90, 100-106) the memory attribute
    registers and Table 4-21's full MAR0-MAR255 address/range listing; §4.4.2 L2CFG;
    §4.4.3 the block and global cache operations; §4.6 L2 memory protection; §4.5 power-down
  - ch.5, pp. 125-142 — IDMA channels 0 and 1
  - ch.6, pp. 143-154 — bandwidth management (timing only)
  - ch.8, pp. 187-200 — memory protection: page attributes, lock registers, fault registers
  - ch.9, pp. 201-206 — the power-down controller and PDCCMD
- `docs/reference/spru862b.pdf` (SPRU862B) — the cache user's guide
  - ch.1 — Tables 1-2/1-4/1-6, the L1P/L1D/L2 parameter tables quoted above, plus allocation
    and miss behaviour
  - ch.2 — coherence: what hardware maintains, what software must, snoop-writes, and the
    L2 SRAM double-buffering idiom firmware actually uses
- `docs/reference/spru970g.pdf` and `spru971e.pdf` — the DDR2 controller and EMIFA
- `docs/reference/sprui03f.pdf` (SPRUI03F) — linker command files: how `MEMORY`/`SECTIONS` place code
  and data into the ranges above, which is what the loader has to satisfy
- `docs/reference/sprui04g.pdf` (SPRUI04G) §7.9.5 and §7.9.7, pp. 163-164 — the `DATA_ALIGN` and
  `DATA_SECTION` pragmas firmware uses to cache-line-align DMA buffers

**Source**

- [`arch/c6x/platforms/cache.c`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/platforms/cache.c) — a complete, readable implementation
  of the C64x+ cache control registers; its `IMCR_*` offsets independently confirm every
  address in the table above, and its block-operation loops show the poll-for-completion
  protocol
- [`arch/c6x/include/asm/cache.h`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/include/asm/cache.h) and [`cacheflush.h`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/include/asm/cacheflush.h) — line sizes and
  the operation set expressed as an API
- [`arch/c6x/kernel/setup.c`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/kernel/setup.c) — memory-map discovery and setup on a
  real board
