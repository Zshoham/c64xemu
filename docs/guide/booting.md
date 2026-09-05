# Booting an image

How memory gets filled and where execution starts: the default ELF path, boot tables and boot
configuration tables, the I2C EEPROM boot, and the Ethernet boot with its host side.

`--boot-mode` is the BOOTMODE pins. Mode 0 starts every core at once; every other mode runs core
0 and leaves it to release the rest. Modes 5, 9 and 10 additionally hand the boot to the ROM
bootloader model, which runs *in place of* core 0 while core 0 is held.

A note that applies to all three ROM modes: the ROM's own instructions are not modelled. TI ships
them in on-chip ROM this project has no copy of, and SPRUEC6G warns that their addresses change
between silicon revisions. What is modelled is everything the ROM does that software or a device
on the bus can observe — the register programming, the frames on the wire, the memory writes, the
cores released and the entry point branched to.

## The default: ELF segments

`c64xemu run IMAGE` loads the ELF's `PT_LOAD` segments into the backing store and starts at the
ELF entry point. This is the path every target program and both firmware images use, and it is
the only mode in which the ELF's *contents* place anything in memory.

`src/elf.rs` accepts ELF32 only, requires the machine type to be `EM_TI_C6000`, and refuses a
segment whose file size exceeds its memory size. **Endianness is not an option** — it is read
from `e_ident[5]` and becomes the device's, which is what strapping LENDIAN would do on a board.
A big-endian image therefore just runs; `tests/programs/big_endian` and `big_endian_rts` are the
cases that hold that down, and the byte order also decides how boot-table words are read and how
three peripherals report their registers.

Symbols are read in every mode, because `--break`, `--stop-pc` and the debugger resolve names
through them.

## Boot tables

A boot table is the format the ROM bootloader loads memory from, whether it sits in memory or
arrives over a link. `src/boot.rs` implements [SPRUEC6G](../reference/spruec6g.pdf) §6.2.2 and
§6.2.3 exactly:

```
u32   entry point
repeat:
  u32 byte count        (zero terminates the table)
  u32 destination address
  ...   `count` bytes of data
u32   0                 terminator
```

Words are read in the device's byte order, which is what the hex conversion utility's `-order`
option matches to the build.

`--boot-table FILE:ADDR` places the file at `ADDR` and loads memory from it instead of from the
ELF segments; the ELF then supplies only symbols. The destination of a section is any address a
CPU access reaches — which is the whole trick behind multicore boot, because writing
DSP_BOOT_ADDRn and BOOT_COMPLETE_STAT is a section like any other. See
[The six-core device](multicore.md).

## Boot configuration tables

A boot configuration table is a list of three-word entries applied before the boot table:

```
repeat:
  u32 address
  u32 set mask
  u32 clear mask
end: three zero words
```

The bootloader reads the address, sets the bits in the set mask, clears the bits in the clear
mask, and toggles the bits that appear in **both**. An entry whose two masks are zero names a
function the bootloader calls with the return address in B3; `src/boot.rs` reports those
addresses to its caller rather than branching itself. The table ends when all three words are
zero.

`--boot-config FILE:ADDR` applies one. It is how a real board programs a memory controller or a
PLL before the image lands.

## I2C EEPROM boot

`--boot-mode 5 --i2c-eeprom FILE` is the master I2C boot of SPRUEC6G Table 3 and §2.3.4: the way
a board with a boot EEPROM comes up. The ELF loads nothing.

The read sequence goes through the modelled I2C master's own registers — ICSAR, ICCNT, ICMDR,
ICDXR and ICDRR are driven exactly as firmware drives them, two internal address bytes as a
master transmit, then a repeated START and a master receive — and the EEPROM answers as a
board's would. `src/i2c.rs` therefore models what §2.3.4 requires of the device: two-byte
internal addressing, an auto-incrementing address counter, and slave address 0x50.

Everything on the EEPROM is blocked, at most 128 bytes per block including a four-byte header:

| Offset | Field |
|---|---|
| 0 | 16-bit block size, counting the header |
| 2 | 16-bit ones-complement checksum over the block, including the size and checksum fields |
| 4 | the block's payload |

Valid checksums are 0 and −0, and a checksum field of 0 disables the check. A failed block is
re-read once.

The boot is two stages. First a boot parameter block: the blocks begin at EEPROM address 0 and
each is 128 bytes, and the DSP picks one by CFGGP[2:0] in DEVSTAT — address = `0x80 * index`,
which `--boot-param-index N` sets. Its options field says what follows: 0 another parameter
table, 1 a boot table, 2 a boot configuration table. For a boot table the sections are streamed
block by block into memory, and core 0 is then branched to the entry point.

## Ethernet boot

`--boot-mode 9` (MAC port 0) or `--boot-mode 10` (port 1) is SPRUEC6G §4.3.5, the way a board
with no boot EEPROM comes up. Nothing at all is loaded from the ELF: every byte of memory arrives
in UDP frames.

What the ROM model does, in order:

1. Holds core 0 and leaves cores 1-5 in reset.
2. Resets and brings up the MAC port the BOOTMODE pins chose, and posts a ten-buffer receive
   ring. The reset is why the announcement matters: a frame that arrived before it is gone with
   the rest of the MAC's state.
3. Sends one BOOTP-shaped Ethernet-ready announcement, naming itself by the MAC address fused
   into the MAC ID words at `02A8 0700h` (`--mac-id` supplies the fuse value, since no public one
   exists). It is sent once and never retransmitted, and nothing in this protocol is acknowledged.
4. Accepts UDP frames on the boot port (`--boot-udp-port`, default 9), checking each as the
   manual specifies: MAC header (DIX, or 802.3 with SNAP/LLC, with or without a VLAN tag), IPv4,
   UDP ports and checksum, then the `0x544B` magic, the opcode, and a sequence number that must
   not skip.
5. Hands each payload to boot-table processing as it arrives — `BootStream` in `src/boot.rs`
   processes the records across fragment boundaries, since no record is guaranteed to fit inside
   one frame.
6. On the terminating zero count, clears the receive channel's head descriptor pointer and
   branches core 0 to the entry point the table named.

One deliberate departure from the letter of §4.3.5, forced by the device model: TI's ROM keeps
its ten 1600-byte packet buffers in L1D at `00F0 0000h`, but no DMA master in this model can
reach a megamodule's *local* L1D window — [SPRS612G](../reference/sprs612g.pdf) Table 2-2 marks
it "DATA SPACE ON DMA" — so the ring and its buffers live in the top of core 0's L2, addressed
through core 0's global alias so the MAC and the boot code name the same bytes. That is inside
the L2 region §4.1 already reserves for the boot code.

### The host side

`tests/tools/emac_boot.py` waits for the announcement, turns each ELF's load segments into a boot
table addressed to the right core, and sends it. The invocation `tests/test_netboot.py` uses:

```console
$ python3 tests/tools/emac_boot.py --socket host.sock --peer emu.sock --json \
      --core 0:image.out --core 1:image.out &
$ c64xemu run image.out --device c6472 --boot-mode 9 \
      --net-unix emu.sock --net-peer host.sock \
      --max-cycles 100000000 --state-out state.json
```

Order matters, and the test enforces it: the sender must bind first, because the announcement is
sent once a few cycles into the boot and a host that is not listening yet has missed it.

Useful sender options, all from `tests/tools/emac_boot.py`: `--entry` overrides the boot table's
entry point, `--release` sets the BOOT_COMPLETE_STAT value written as a section and
`--no-release` omits that section entirely, `--dest-port` (default 9) and `--src-port` move the
UDP ports, `--dst-mac`/`--src-mac`/`--src-ip`/`--dst-ip` set the addresses, `--payload` sets the
bytes per frame (default 1024) and `--gap` the delay between frames.

On the emulator side `--net-port` chooses which MAC receives frames from the backend; by default
it follows the port an Ethernet boot was strapped to.
