# Interrupts and exceptions

The path from a peripheral asserting a system event to the CPU vectoring into a service routine,
the exception path beside it, and the device event maps that decide which event number a
peripheral raises and which core sees it.

Two modules meet here. `src/system.rs` owns the megamodule interrupt controller — the event
flags, the masks, the combiners and the interrupt selector — and `src/control.rs` owns the CPU
side: IER, IFR/ISR/ICR, GIE, ISTP, TSR/ITSR/NTSR, IRP and NRP. `src/cpu.rs` joins them once per
cycle in `finish_cycle`. `src/events.rs` is neither: it is the device event map, a table the
fabric consults instead of hardcoding event numbers in its tick and latch methods.

## From event to interrupt, stage by stage

1. A peripheral raises a system event. The fabric looks the number up in the device event map
   and calls `Megamodule::raise_event` on every megamodule the map's `Destination` names —
   `Destination::All`, or `Destination::Core(n)` for a steered source.
2. `raise_event` does two things with one bit: it latches it in `event_flags`, where it stays
   until software clears it, and it asserts it in `event_lines` for this cycle only.
3. At the cycle boundary, `Cpu::finish_cycle` asks the megamodule for both the CPU interrupt
   lines and the exception combiner's output, through `take_pending_cpu_interrupts` and the
   exception-pending flag.
4. `Control::tick` merges those lines into IFR under IER and GIE, and reports back any interrupt
   that had to be dropped, which the megamodule records.
5. If nothing is pending and no loop buffer is draining, the cycle ends in the fast path.
   Otherwise `take_exceptions_and_interrupts` runs, and may vector.

The split between `event_flags` and `event_lines` is the load-bearing detail, and it comes from
[SPRU871K](../reference/spru871k.pdf) Figure 7-9: the interrupt selector is fed from the *raw*
system events while the event combiner is fed from the *latched* flags. The consequence, which
the comment on `take_pending_cpu_interrupts` spells out, is that a directly selected event
asserts its CPU interrupt when it is received, not for as long as its flag stays set — so an
unacknowledged flag cannot re-enter the handler, and §7.4.2 is right that a routine for a
directly selected event need neither read nor clear EVTFLAG. The four combiner outputs behave
the other way round: they are the level of their group's unmasked flags, which is why a
combined-event routine must clear the flags before returning.

## The megamodule interrupt controller

The register block sits at `0180 0000h`. `src/system.rs` names the offsets it implements:

| Register | Address | Behaviour |
|---|---|---|
| EVTFLAG0-3 | `0180 0000h` | the latched flag for each of the 128 chip-level events |
| EVTSET0-3 | `0180 0020h` | set flags from software |
| EVTCLR0-3 | `0180 0040h` | clear flags |
| EVTMASK0-3 | `0180 0080h` | the event combiner's mask; a set bit keeps the event out of its combined output |
| MEVTFLAG0-3 | `0180 00a0h` | the masked flags, which is what the combiner reduces |
| EXPMASK0-3 | `0180 00c0h` | the exception combiner's mask, same polarity as EVTMASK |
| MEXPFLAG0-3 | `0180 00e0h` | the masked flags feeding the single `EXCEP` input |
| INTMUX1-3 | `0180 0104h` | the selector byte for each of CPU interrupts INT4 to INT15 |
| INTXSTAT / INTXCLR | `0180 0180h` / `0184h` | dropped-interrupt status and its clear |
| INTDMASK | `0180 0188h` | which interrupts are exempt from dropped-interrupt reporting |

Events 0 to 3 are the interrupt combiner's own outputs. They never latch in EVTFLAG0 and
therefore cannot reach the exception combiner; `reserved_events` encodes that as a mask of `0xf`
on bank 0. `EXCEP` itself is event 96 on every C64x+ megamodule (`EXCEP_EVENT`).

`selected_cpu_interrupts` is the selector: for each of INT4 to INT15 it reads the seven-bit
event number out of the INTMUX byte, and asserts that interrupt either from the *combiner*
(`event < 4`, taken as the level of unmasked flags in that bank) or from the raw event line.
The reset value of `interrupt_mux` is `[0x0706_0504, 0x0b0a_0908, 0x0f0e_0d0c]`, so INT4 to
INT15 select events 4 to 15 before firmware programs anything.

Dropped interrupts are reported by `record_dropped_interrupt`: the first drop latches, further
drops are suppressed until the status is cleared, and an interrupt masked in INTDMASK is exempt.

## The CPU side

`Control` in `src/control.rs` holds the interrupt and exception state. The fields are narrow on
purpose — `ifr` and `ier` are `u16`, because there are sixteen CPU interrupts.

| Register | Field | Notes |
|---|---|---|
| IFR / ISR / ICR | `ifr` | encoding `0x02` reads IFR and writes ISR; a separate name table maps the write side (`written_register_name`) |
| IER | `ier` | bit 1 is NMIE, which gates every maskable interrupt and the external exception (`IER_NMIE`) |
| ISTP | `istp_base` | the interrupt service table base; each ISFP is eight words |
| IRP / NRP | `irp`, `nrp` | the maskable and non-maskable return pointers |
| TSR | `tsr` | GIE bit 0, SGIE bit 1, GEE bit 2, XEN bit 3, CXM bits 7-6, INT bit 9, EXC bit 10, SPLX bit 14 |
| ITSR / NTSR | `itsr`, `ntsr` | the saved TSR for an interrupt and for an exception; NTSR bit 16 is HWE, which distinguishes a hardware exception from `SWE`/`SWENR` |
| EFR / ECR | `efr` | SXF bit 0, IXF bit 1, EXF bit 30, NXF bit 31; encoding `0x1d` reads EFR and writes ECR |
| IERR | `ierr` | the internal exception cause |
| DIER, REP, SSR, ILC, RILC | | the remaining C64x+ control registers, each with its own field |

Every bit position above is a named constant at the top of `src/control.rs`, each carrying the
[SPRU732J](../reference/spru732j.pdf) or [SPRUFE8B](../reference/sprufe8b.pdf) table it comes
from. `restricted_in_user` is the User-mode access list: ECR, EFR, ICR, IER, IERR, IFR, ISR,
ISTP, ITSR, NTSR, REP and DIER, which is the union of SPRU732J §8.2.4.1 and Table 3-22 — an
`MVC` naming one from User mode raises an exception reporting both `IERR.RAX` and `IERR.PRX`.

## Taking an interrupt

`take_exceptions_and_interrupts` in `src/cpu.rs` decides which source wins a cycle boundary, in
priority order: NMI as an exception first when GEE is set, then a pending internal or software
exception, then the maskable `EXCEP` input, then NMI as an interrupt, then the
highest-priority maskable interrupt. It does not vector itself; every winner goes through
the one interrupt entry, `Cpu::enter`.

Three conditions defer a take rather than cancel it. A branch shadow (`branches` non-empty, or
`cycle < branch_block_until`) holds the interrupt until the delay slots finish. An interrupt
stall — `interrupt_stall`, set to 9 on every entry — holds off the next one. An active SPLOOP
holds it until the buffer has drained. In all three cases `interrupt_ready_since` records the
cycle the interrupt became ready, so the interrupt-latency statistic measures from readiness
rather than from the take.

Entry is `Cpu::enter(Entry)`, whatever the source — `Entry::Interrupt(n)`, `Entry::NmiInterrupt`
or `Entry::Exception(_)`. The source decides only the return address and what becomes of an
SPLOOP; `Control::enter` saves the context into the register pair the source returns through
(ITSR/IRP for an interrupt, NTSR/NRP for NMI and exceptions) and returns the vector. Everything
after is shared: `branches` is cleared, `user_mode` drops to Supervisor, any queued mode
change is cancelled, remaining multicycle NOPs and IDLE are annulled, and `interrupt_stall` is
set to 9. That number is the documented overhead for all CPU interrupts —
[SPRUFE8B](../reference/sprufe8b.pdf) §5.5.1, which the test
`interrupt_overhead_is_nine_cycles` in `src/cpu.rs` pins directly.

## Interrupts inside a software-pipelined loop

A loop is never interrupted mid-iteration. When an interrupt becomes takeable while the SPLOOP
buffer is active, `ready_to_drain_for_interrupt` decides whether the buffer is at a point where
it may drain, `begin_interrupt_drain` starts the pipe-down, and the take waits. When the drain
completes the interrupt is taken with `in_sploop` true, so the return address is the SPLOOP
packet itself and the loop pipes back up on return. The mechanism lives in `src/sploop.rs`; see
[Execution and timing](execution.md) for the buffer.

An *exception* does not drain. Its entry calls `sploop.abandon()`, which leaves the buffer
idle with no epilog, clears `pipe_up`, and records in NTSR.SPLX that a loop was destroyed — the
handler can then tell that it cannot resume the loop.

## Exceptions

`control::Exception` has four shapes: `Internal` (cause in IERR), `Software` (`SWE`/`SWENR`,
with a `restricted` flag that enters at REP), `Nmi` and `External`. Each flags its own EFR bit
in `Control::enter`, and a nested exception — one taken while TSR.EXC is already set — saves
into ITSR/IRP instead and vectors to reset.

An exception the firmware does not handle is still an exception the model took; `--trace-exceptions`
reports the cause of each one, which is the difference between "the image is wrong" and "the
image is deliberately handling faults". See [Debugging](../guide/debugging.md).

## The device event maps

`src/events.rs` holds one `EventMap` per device. Fixed sources are fields; sources whose number
or destination depends on an index are methods, because a multicore device steers some of them.

| Source | C6455 | C6472 |
|---|---|---|
| HPI (DSPINT) | 15 | 85 |
| I2C interrupt / EDMA3 channels | 16 / 44, 45 | 72 / 20, 21 |
| EMAC 0 | one event, 17, for receive, transmit, statistics and MDIO | receive 5, transmit 6, common 70 |
| EMAC 1 | not instantiated | receive 7, transmit 8, common 71 |
| EDMA3 global / error / protection | 24 / 79 / 80 | 69 / 57 / 58 |
| Transfer controller errors | 81-84 | 59-62 |
| EDMA3 region completion | 71-78, one per region, all cores | regions 0-5 are event 15 in *that core only*; regions 6 and 7 are 67 and 68 everywhere |
| Timers | two timers, halves at 67-70, EDMA3 channels 1, 2, 16, 17 | timers 0-5 are local: events 16 and 17 in their own core; shared timers 6-11 are events 18-29 with EDMA3 channels 8-19 |
| GPIO | pins 0-15 are events 51-66, EDMA3 channels 48-63 | pins 0-5 are event 73 in core *n*; pins 6-15 are events 74-83 everywhere; pins 0-7 drive EDMA3 channels 0-7 |
| UTOPIA | 36, EDMA3 channels 32 and 40 | UINT 56; PDMA PRINT 30 and PXINT 31 in the SINT-selected core; PXINT0-5 (SINT 0-5) are also EDMA3 channels 22-27. |
| McBSP | RINT/XINT 40-43, EDMA3 channels 12-15 | none (the C6472 has three TSIPs instead) |
| Memory protection, CPU / DMA | 124, 120, 122 / 125, 121, 123 (L2, L1P, L1D) | same |
| Inter-processor interrupt | none | 84 |
| Power/sleep controller | none | 65 |
| Shared-memory fault / profiler | none | 63 / 64 |

The C6455 numbers are cited in the source to [SPRS276M](../reference/sprs276m.pdf) Tables 7-9 to
7-11, [SPRU966C](../reference/spru966c.pdf) Tables 2-6, 2-10 and 2-11, and
[SPRU975E](../reference/spru975e.pdf) §2.16.4; the C6472 numbers to
[SPRS612G](../reference/sprs612g.pdf) Table 7-10 pp. 137-140, Table 7-4 pp. 124-126, §7.6.3 for
the GPIO split and §3.7 for the IPC.

Steering is why `Destination` exists at all. Most C6472 events are one source every megamodule's
interrupt controller can see, and software decides which core services them; the exceptions —
the first six EDMA3 region completions, the six local timers, the first six GPIO pins, and the
inter-processor interrupt — are delivered to exactly one core. That is what makes six cores
share one channel controller and one GPIO block without racing for the same interrupt. See
[The six-core device](../guide/multicore.md).
