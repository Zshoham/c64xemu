# The six-core device

Running `--device c6472`: what changes from the default C6455, how cores are released and
focused, how they interrupt each other and share memory, and how six cores share one Ethernet
connection.

Register-level modelling lives in [Peripherals](../internals/peripherals.md) and
[Interrupts and exceptions](../internals/interrupts.md); this document is operational.

## What `--device c6472` changes

`src/device.rs` holds one `DeviceProfile` per device and every difference is a field of it
rather than a branch in the model.

| | C6455 | C6472 |
|---|---|---|
| Cores | 1 | 6 |
| L2 SRAM per core | 2 MB at `0080 0000h` | 608 KB at `0080 0000h` |
| L1P / L1D per core | 32 KB at `00E0 0000h` / `00F0 0000h` | same |
| Shared L2 RAM | none | 768 KB at `0020 0000h` |
| Shared ROM | none | 768 KB at `0010 0000h` |
| Global aliases | none | `1000 0000h`, stride `0100 0000h` — core *n* at `1n80 0000h` |
| Timers | 2 | 12: timers 0-5 local to megamodules 0-5, 6-11 shared |
| Ethernet | one MAC at `02C8 0000h` | two MACs, plus an interrupt-control module in front of each |
| McBSP | 2 | none (three TSIPs instead, not modelled) |
| EMIFA | yes | no — DDR2 is the only external memory |
| Device configuration | `src/devcfg.rs` | boot controller, chip configuration, power/sleep and shared-memory controllers |
| JTAG ID | `0009 202Fh` | `0009 102Fh` |

Everything else — the CPU, the caches, the memory protection units, EDMA3 — is the same
megamodule six times over.

## Local windows and global aliases

Each megamodule sees its own memories at the local addresses above. Every megamodule's memories
*also* carry a global address, `1n80 0000h` for core *n*'s L2, and the distinction is not
cosmetic:

- Another core, an EDMA3 transfer controller or an Ethernet MAC must use the global address.
  [SPRS612G](../reference/sprs612g.pdf) Table 2-2 marks the local window "DATA SPACE ON DMA", so
  a DMA master reaching for a local window does not get the memory.
- A DMA access through the global alias is snooped against the owning core's L1D exactly as a
  local one would be.
- The address map deliberately does *not* canonicalise an alias away, because two addresses for
  one byte are two cache tags on silicon. See [Memory and caches](../internals/memory-and-caches.md).

The shared L2 RAM at `0020 0000h` needs no alias: all six cores address it identically.

## Starting the cores

`--boot-mode 0` starts every core at the ELF entry point at once. The HPI, I2C, UTOPIA and MAC
modes (1, 2-3, 5-8, 9-10) hold every core until their boot source has delivered the image. A
mode with no modelled boot source - 4 (I2C slave), or 11 to 15 - loads the ELF, runs core 0 and
leaves the rest internally held in reset until core 0 releases them — which is what real silicon
does after a boot, and what the firmware in `tests/firmware/c6472_multicore/` demonstrates:

```c
#define BOOT_COMPLETE_STAT (*(volatile unsigned *)0x02AB0004)
#define DSP_BOOT_ADDR(core) (*(volatile unsigned *)(0x02AB0204 + 0x20 * (core)))
#define IPCGR(core) (*(volatile unsigned *)(0x02A80540 + 4 * (core)))

for (index = 1; index < CORES; index++) {
    DSP_BOOT_ADDR(index) = SECONDARY_ENTRY >> 10;      /* address shifted right by ten */
}
for (index = 1; index < CORES; index++) {
    BOOT_COMPLETE_STAT = 1u << index;                  /* 0 -> 1 releases core `index` */
    IPCGR(index) = (1u << 4) | IPC_GENERATE;           /* announce ourselves */
}
```

Three details worth carrying away. `DSP_BOOT_ADDRn` holds the start address **shifted right by
ten**. The release is the 0→1 transition of the set-only `BCn` bit in BOOT_COMPLETE_STAT. And a
held core's memories still answer, so core 0 (or a host) can initialise them while the core waits
— which is exactly what an Ethernet boot table does, since writing those two registers is a boot
table section like any other. See [Booting an image](booting.md).

An image learns which core it is running on from **DNUM**, control register `0x11`.

## Inter-processor interrupts

A core (or the host) raises an interrupt in core *n* by writing `IPCGRn` at `02A8 0540h + 4n`.
It arrives in that core's megamodule as **system event 84**, IPC_LOCAL, and the receiving core
clears the source bits through `IPCARn`. The upper bits are source flags the sender chooses, so
the receiver can tell who called. `NMIGRn` is the same shape for a non-maskable interrupt.

## Shared memory and power

Every access to the 768 KB SL2 goes through the shared-memory controller: it wakes the page the
access lands in, counts the read in the requesting core's profiler, and judges LL, SL and CMTL
with its atomic monitors - a second SL after a stored link breaks the link, so the commit
returns 0 and stores nothing. A User-mode write to one of its supervisor-only registers is
refused, recorded in SL2MPFSR and SL2MPFAR, and raised as event 63 in all six cores. Prefetch
and arbitration are not modelled. The power/sleep controller can stop a core's clock: stage
`MDCTLn.NEXT`, write the domain's GO bit in `PTCMD`, poll `PTSTAT.GOSTAT`, read the reached state
from `MDSTATn.STATE`.
Both are in [Peripherals](../internals/peripherals.md).

## Six cores at the front ends

Every front end drives the **focused core**: the one whose registers `info` prints, whose memory
`mem` dumps and which `stepi` advances. The fabric follows the focus, so a front end written for
one core works unchanged. In the REPL, `core` lists the cores and `core N` moves the focus.

The state report gains a `cores` array, one entry per core, alongside the focused core's own
fields:

```console
$ c64xemu run IMAGE --device c6472 --max-cycles 2000 --state-out state.json
$ jq -c '.device, .cores[]' state.json
"c6472"
{"core":0,"cycles":2000,"fault":null,"focused":true,"halted":false,"held_in_reset":false,"pc":8466072}
{"core":1,"cycles":2000,"fault":null,"focused":false,"halted":false,"held_in_reset":false,"pc":8466072}
...
{"core":5,"cycles":2000,"fault":null,"focused":false,"halted":false,"held_in_reset":false,"pc":8466072}
```

That is boot mode 0 — all six running the same image in lockstep, which is why the cycle counts
and program counters agree. With any other boot mode, cores 1-5 report `held_in_reset: true`
until something releases them.

## One Ethernet connection, six cores

Each MAC has an interrupt-control module (`src/emic.rs`) in front of it that decides, per core,
which channels that core is interrupted by. One core owns the descriptor ring, and the receive
interrupt follows whichever core enabled the channel — that is the whole mechanism by which six
cores share a single Ethernet connection. See [Ethernet](../internals/ethernet.md).

`--boot-mode 9` and `10` boot the whole device over MAC port 0 or 1 with no image loaded at all;
`tests/tools/emac_boot.py --core 0:image.out --core 1:image.out` is the host side, and each
`--core` places that image through the target core's global alias.

## Running the multicore cases

`tests/test_multicore.py` builds and runs `tests/firmware/c6472_multicore/`, which does exactly
the sequence above and rendezvouses all six cores in the shared L2:

```console
$ python3 -m pytest tests/test_multicore.py
```

The image is a build product; the suite builds it on demand.
