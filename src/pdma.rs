//! The C6472 UTOPIA's peripheral DMA controller.
//!
//! SPRUEG2E chapter 4 and section 15.2: a PDMA owns a context RAM of channels - a switch address,
//! a peripheral address, a buffer size or byte count and a transfer control word each - which the
//! CPU programs through per-channel proxy registers (the PIM, section 5), and two peripheral
//! control registers, one per event group. The TX group is channels 0-7 and the RX group channels
//! 8-15, so the first receive proxy block at 0x02BC_0180 is channel 8 (SPRS612G Table 7-143: six
//! of each exist).
//!
//! Each direction services its UTOPIA queue one burst per synchronisation event, and the switch
//! side of a burst is the same in both: `PBLEN + 1` words through the channel's switch address,
//! in auto-buffering mode around a circular buffer (section 4.3.1) or in block mode until the
//! byte count runs out (section 4.3.2), with the interrupt Table 5 gives - ABU with IE raises one
//! at the end of the buffer and with IMOD also at the half-buffer point, block mode one when the
//! block completes. A receive event - UREVT with the routing unit's channel ID - reads the burst
//! out of the receive queue and writes it to memory, which is the path the ROM's UTOPIA boot
//! drives (SPRUEC6G section 4.3.4). A transmit event - UXEVT, asserted while the transmit queue
//! has room for a cell (SPRUEG2E section 3.3) - reads the burst from memory and writes it into the
//! transmit queue, which is section 4.4's setup.
//!
//! A transmit burst starts only when the queue can take all of it. Section 4.2 says a transmit
//! channel waits for data to reach the PDMA's buffer before it goes active, and Table 31 counts a
//! stall each time the PDMA has to wait for space; the model counts that stall in SR0 and leaves
//! the event pending rather than pushing half a burst at a full queue. UXEVT already means room
//! for a whole cell, so this only bites a burst longer than the space a cell leaves.
//!
//! Event control. A group takes PE = 0 (channel 0 of the group) and PE = 1, Example 1's setup:
//! the group keeps a current channel, uses it on every event whatever ID the event carries, and
//! arbitrates for another only once the current one is no longer enabled - Table 32's "when the
//! transfer count of the current channel becomes zero", since a completed block clears STRT. The
//! order of that arbitration is not documented [INFERENCE]: the model takes the next enabled
//! channel after the current one, round robin. A receive group also takes PE = 2, where the
//! routing unit's event ID picks the channel.
//!
//! Element sizes. The peripheral side is always a 32-bit word: sections 3.2, 3.4 and 4.4 say the
//! PDMA's access to the UTOPIA is always 32 bits. The switch side is 32 bits in either mode, or
//! 8 or 16 bits in block mode, where Table 38 counts the block in bytes of switch element and
//! section 4.3.2.2 asks for a 32-bit aligned base; each element is the low bits of the word
//! ("data is always right justified on the bus", Table 32) **[INFERENCE: the manual states the
//! justification for the endian conversions, and the model reads it as the rule for a
//! narrower switch element]**. The address steps by the element size and must be aligned to it.
//! An auto-buffering buffer of narrow elements is refused: Table 3 gives its base alignment for
//! 32-bit elements only.
//!
//! Not modelled, and refused as an error count in the group's SR rather than half-done, because
//! the manual does not establish what the hardware does. TI's C6472 CSL (`docs/reference/README.md`)
//! was read for each and settles none: its `TX_EVENT` and `RX_EVENT` are the event-group indices
//! 0 and 1 that pick PCR0 or PCR1, not a UXEVT event ID.
//!
//! * Receive PE = 3 and transmit PE = 2 and PE = 3. PE = 3 responds to an event "even if the PDMA
//!   channel is inactive" and counts an error when the channel buffer is full or empty, which
//!   puts a channel buffer between the peripheral and the switch; its depth and when the switch
//!   side drains it are not given. Transmit PE = 2 and 3 select a channel by event ID, and UXEVT
//!   carries none (the manual gives UREVT the routing unit's channel, and no ID for UXEVT).
//! * Unsynchronised transfers (SYNC = 0): the manual says only "no synchronization". What starts
//!   a burst, which channel of a group serves it without an event to select by, and how the
//!   bursts are paced against a queue that is empty or full (section 9 calls such an access a
//!   user error that stalls) are not given.
//! * An incrementing peripheral address (PMOD = 0): Table 18 gives the data ports two words, URQ
//!   and UXQ, and nothing for an address that walks past them.
//! * A peripheral burst length different from the switch's: with one FIFO word on the peripheral
//!   side, a different element count per side again needs the channel buffer, and the manual
//!   does not say how many elements an event moves then.
//! * A peripheral element size other than 32 bits, a direction other than the group's, and the
//!   emulation halt bits, which section 11.2 says the peripheral event groups ignore anyway.
//!
//! The peripheral address (PAR) of a channel must be its queue's port: `RECEIVE_QUEUE_PORT` or
//! `TRANSMIT_QUEUE_PORT`, the offsets Table 18 gives URQ and UXQ. That the PDMA's PAR is such an
//! offset rather than a global address is an interpretation of Table 18 and section 4.1
//! [INFERENCE]: no document available here gives the data ports a global address, and SPRS612G
//! Table 2-2 maps none.

use crate::port::{PortContext, PortWrite, SlavePort};
/// The receive queue as the PDMA's peripheral bus addresses it. SPRUEG2E Table 18 lists the UTOPIA
/// data ports by offset - URQ at 0, UXQ at 4 - and the PDMA reaches its peripheral over a bus of
/// its own (section 4.1) rather than through the switch, so PAR is that offset. Where the CPU sees
/// the same port in the global map is not documented (`utopia.rs`).
pub(crate) const RECEIVE_QUEUE_PORT: u32 = 0;
/// The transmit queue, UXQ, the same way.
pub(crate) const TRANSMIT_QUEUE_PORT: u32 = 4;

const BASE: u32 = 0x02bc_0000;
const PID: u32 = BASE;
const GCSR: u32 = BASE + 0x04;
const SR: [u32; 2] = [BASE + 0x20, BASE + 0x24];
const PCR: [u32; 2] = [BASE + 0x60, BASE + 0x64];
const TX_PROXIES: u32 = BASE + 0x100;
const RX_PROXIES: u32 = BASE + 0x180;
const PROXY_STRIDE: u32 = 0x10;
/// UTOPIA has six transmit and six receive channels (UTXCHN[5:0], URXCHN[5:0]).
const CHANNELS_PER_GROUP: usize = 6;
const SLOTS: usize = 2 * CHANNELS_PER_GROUP;
/// The event groups: transmit channels (0-7) and receive channels (8-15).
const TX_GROUP: usize = 0;
const RX_GROUP: usize = 1;

/// PID: type 1 (PDMA) in 23-16, class 4 (DMA controller) in 15-8, revision 1 (Table 29).
const PID_VALUE: u32 = (0x01 << 16) | (0x04 << 8) | 0x01;
/// GCSR: STRT in bit 31 enables the PDMA, SOFT and FREE in 1-0 are the emulation controls. It
/// comes out of reset halted with SOFT and FREE set (`CSL_PIM_GCSR_RESETVAL`, TI's C6472 CSL
/// `cslr_pim.h`); SPRUEG2E Table 30 gives STRT's reset value only.
const GCSR_STRT: u32 = 1 << 31;
const GCSR_WRITABLE: u32 = GCSR_STRT | 0b11;
const GCSR_RESET: u32 = 0b11;

/// PCR (Table 32). CLRS is a write-only strobe. Bit 24, reserved in Table 32, is `SFRM`, a
/// super-frame-sync select, in TI's C6472 CSL (`CSL_PIM_PCR_SFRM_MASK`, `cslr_pim.h`); neither
/// source says what it does, so the model keeps it reserved: it is not writable and reads zero.
const PCR_CLRS: u32 = 1 << 25;
const PCR_PE_SHIFT: u32 = 22;
const PCR_SYNC: u32 = 1 << 20;
const PCR_ABU: u32 = 1 << 17;
const PCR_DIR_PERIPHERAL_TO_SWITCH: u32 = 1 << 16;
const PCR_PBLEN_SHIFT: u32 = 12;
const PCR_PMOD_CONSTANT: u32 = 1 << 11;
const PCR_SBLEN_SHIFT: u32 = 4;
const PCR_SMOD_CONSTANT: u32 = 1 << 3;
const PCR_WRITABLE: u32 = (1 << 26)
    | (0b11 << PCR_PE_SHIFT)
    | PCR_SYNC
    | (0b11 << 18)
    | PCR_ABU
    | PCR_DIR_PERIPHERAL_TO_SWITCH
    | (0xf << PCR_PBLEN_SHIFT)
    | PCR_PMOD_CONSTANT
    | (0xf << PCR_SBLEN_SHIFT)
    | PCR_SMOD_CONSTANT;
const PE_CHANNEL_ZERO: u32 = 0;
const PE_CURRENT_CHANNEL: u32 = 1;
const PE_EVENT_ID: u32 = 2;

/// TCR (Table 39): STRT 31, IMOD 28, IE 27, SINT 26-24, PSIZ 23-22, SSIZ 21-20, FF 17, BCZ 16,
/// WRPTR 14-8 and RDPTR 6-0 are writable; the rest read zero. Table 39 leaves FF unnamed; TI's
/// C6472 CSL calls it the FIFO full flag (`CSL_PIM_TCR_FF_MASK` in `cslr_pim.h`, described in
/// `csl_pdma.h`). It and the two pointers belong to the channel buffer, which the model does not
/// have, so they hold what was written.
const TCR_STRT: u32 = 1 << 31;
const TCR_IMOD: u32 = 1 << 28;
const TCR_IE: u32 = 1 << 27;
const TCR_SINT_SHIFT: u32 = 24;
const TCR_PSIZ_SHIFT: u32 = 22;
const TCR_SSIZ_SHIFT: u32 = 20;
/// BCZ: "Block count equal zero", which the state machine sets when a block completes.
const TCR_BCZ: u32 = 1 << 16;
const TCR_WRITABLE: u32 = 0x9ff3_7f7f;
/// A 32-bit element in PSIZ and SSIZ.
const SIZE_32_BIT: u32 = 2;
/// The valid ABU buffer sizes in bytes (section 4.3.1.1), and the block count field (Table 38).
const ABU_MIN_SIZE: u32 = 0x0008;
const ABU_MAX_SIZE: u32 = 0xfffc;
const ABU_SWITCH_SIZE: u32 = 0xffff;
const BLOCK_COUNT: u32 = 0x07ff_ffff;
/// One burst is at most sixteen words (a 4-bit length in the "-1" format).
pub(crate) const MAX_BURST_WORDS: usize = 16;

/// One channel's context RAM entry, which is also the layout of a proxy block.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
struct Context {
    switch: u32,
    peripheral: u32,
    size: u32,
    control: u32,
}

/// UREVT as the PDMA sees it: the routing unit's channel ID for the cell, and a count of how
/// many times the UTOPIA has asserted the event, which tells a new assertion from the same one
/// still waiting.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ReceiveEvent {
    pub(crate) id: usize,
    pub(crate) generation: u64,
}

/// UXEVT as the PDMA sees it: which assertion it is, and how many words the transmit queue can
/// take without stalling.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TransmitEvent {
    pub(crate) generation: u64,
    pub(crate) room_words: usize,
}

/// A burst the PDMA has accepted: which channel, and how many words it moves through the queue.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Burst {
    slot: usize,
    pub(crate) words: usize,
}

/// The switch side of a burst: the global addresses it reads or writes, in order, the size in
/// bytes of the element at each, and the DSP subsystem the channel's interrupt goes to (SINT),
/// if it raised one.
#[derive(Debug, Default, PartialEq, Eq)]
pub(crate) struct SwitchBurst {
    addresses: [u32; MAX_BURST_WORDS],
    count: usize,
    pub(crate) element: u32,
    pub(crate) interrupt: Option<usize>,
}

impl SwitchBurst {
    pub(crate) fn addresses(&self) -> &[u32] {
        &self.addresses[..self.count]
    }

    fn push(&mut self, address: u32) {
        self.addresses[self.count] = address;
        self.count += 1;
    }
}

#[cfg_attr(test, derive(Clone, Debug, PartialEq))]
pub(crate) struct Pdma {
    global: u32,
    peripheral_control: [u32; 2],
    /// SR0 and SR1: the stall count in the upper half and the error count in the lower.
    statistics: [u32; 2],
    contexts: [Context; SLOTS],
    proxies: [Context; SLOTS],
    /// Per group, the assertion of its event whose refusal has already been counted, so an event
    /// that waits for its channel is one error or stall rather than one per clock.
    counted_generation: [Option<u64>; 2],
    /// Each group's current channel under PE = 1.
    current: [Option<usize>; 2],
}

impl Default for Pdma {
    fn default() -> Self {
        Self {
            global: GCSR_RESET,
            peripheral_control: [0; 2],
            statistics: [0; 2],
            contexts: [Context::default(); SLOTS],
            proxies: [Context::default(); SLOTS],
            counted_generation: [None; 2],
            current: [None; 2],
        }
    }
}

/// The proxy block an address falls in and the word within it, if it is a proxy register.
fn proxy_register(address: u32) -> Option<(usize, usize)> {
    for (base, first_slot) in [(TX_PROXIES, 0), (RX_PROXIES, CHANNELS_PER_GROUP)] {
        let offset = address.wrapping_sub(base);
        let block = (offset / PROXY_STRIDE) as usize;
        if offset < PROXY_STRIDE * CHANNELS_PER_GROUP as u32 && address & 3 == 0 {
            return Some((first_slot + block, ((offset % PROXY_STRIDE) / 4) as usize));
        }
    }
    None
}

fn context_word(context: &Context, word: usize) -> u32 {
    [
        context.switch,
        context.peripheral,
        context.size,
        context.control,
    ][word]
}

impl SlavePort for Pdma {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        if let Some((slot, word)) = proxy_register(address) {
            // A read of the switch address proxy copies the channel's context into the proxies
            // (section 5); the other three then read that snapshot.
            if word == 0 {
                self.proxies[slot] = self.contexts[slot];
            }
            return Some(context_word(&self.proxies[slot], word));
        }
        Some(match address {
            PID => PID_VALUE,
            GCSR => self.global,
            a if a == SR[0] || a == SR[1] => self.statistics[(a - SR[0]) as usize / 4],
            a if a == PCR[0] || a == PCR[1] => self.peripheral_control[(a - PCR[0]) as usize / 4],
            _ => return None,
        })
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if let Some((slot, word)) = proxy_register(address) {
            match word {
                0 => self.proxies[slot].switch = value,
                1 => self.proxies[slot].peripheral = value,
                2 => self.proxies[slot].size = value,
                _ => {
                    self.proxies[slot].control = value & TCR_WRITABLE;
                    self.commit(slot);
                }
            }
            return PortWrite::Done;
        }
        match address {
            // Read-only registers ignore writes (Tables 29 and 31).
            PID => {}
            a if a == SR[0] || a == SR[1] => {}
            GCSR => self.global = value & GCSR_WRITABLE,
            a if a == PCR[0] || a == PCR[1] => {
                let group = (a - PCR[0]) as usize / 4;
                self.peripheral_control[group] = value & PCR_WRITABLE;
                if value & PCR_CLRS != 0 {
                    self.statistics[group] = 0;
                }
            }
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl Pdma {
    /// A write to the transfer control proxy moves the proxies into the context RAM - except
    /// that the context of an enabled channel changes only to disable it (section 5).
    fn commit(&mut self, slot: usize) {
        let enabled = self.contexts[slot].control & TCR_STRT != 0;
        if !enabled || self.proxies[slot].control & TCR_STRT == 0 {
            self.contexts[slot] = self.proxies[slot];
        }
    }

    /// An event the group could not service: counted once per assertion, in the error half of
    /// the group's SR or, for a stall, the wait half (Table 31).
    fn refuse(&mut self, group: usize, generation: u64, stall: bool) -> Option<Burst> {
        if self.counted_generation[group] != Some(generation) {
            self.counted_generation[group] = Some(generation);
            let statistics = self.statistics[group];
            self.statistics[group] = if stall {
                (statistics & 0xffff) | (statistics & 0xffff_0000).wrapping_add(1 << 16)
            } else {
                (statistics & 0xffff_0000) | (statistics.wrapping_add(1) & 0xffff)
            };
        }
        None
    }

    /// The receive event the UTOPIA is presenting - UREVT with the routing unit's channel ID,
    /// and which assertion of it this is - or none when it is not asserted. Answers the burst
    /// the PDMA starts for it. An event the PDMA is not set up to service counts once in SR1's
    /// error field and stays pending in the peripheral until a channel can take it (section 5,
    /// the event server).
    pub(crate) fn receive_event(&mut self, event: Option<ReceiveEvent>) -> Option<Burst> {
        let ReceiveEvent {
            id: event_id,
            generation,
        } = event?;
        if self.global & GCSR_STRT == 0 {
            // A halted PDMA does not look at events at all.
            return None;
        }
        let control = self.peripheral_control[RX_GROUP];
        let slot = match (control >> PCR_PE_SHIFT) & 0b11 {
            PE_CHANNEL_ZERO => CHANNELS_PER_GROUP,
            PE_CURRENT_CHANNEL => match self.current_channel(RX_GROUP) {
                Some(slot) => slot,
                None => return self.refuse(RX_GROUP, generation, false),
            },
            PE_EVENT_ID if event_id < CHANNELS_PER_GROUP => CHANNELS_PER_GROUP + event_id,
            _ => return self.refuse(RX_GROUP, generation, false),
        };
        match self.burst_words(RX_GROUP, slot) {
            Some(words) => Some(Burst { slot, words }),
            None => self.refuse(RX_GROUP, generation, false),
        }
    }

    /// The transmit event the UTOPIA is presenting - UXEVT, which assertion of it this is, and
    /// the room its queue has - or none when it is not asserted. Answers the burst the PDMA
    /// starts for it: the words `advance` names are to be read from memory and written into the
    /// transmit queue. An event with no channel to serve it counts once in SR0's error field, and
    /// one whose burst the queue cannot take whole counts once as a stall; both stay pending.
    pub(crate) fn transmit_event(&mut self, event: Option<TransmitEvent>) -> Option<Burst> {
        let TransmitEvent {
            generation,
            room_words,
        } = event?;
        if self.global & GCSR_STRT == 0 {
            return None;
        }
        let control = self.peripheral_control[TX_GROUP];
        let slot = match (control >> PCR_PE_SHIFT) & 0b11 {
            PE_CHANNEL_ZERO => 0,
            PE_CURRENT_CHANNEL => match self.current_channel(TX_GROUP) {
                Some(slot) => slot,
                None => return self.refuse(TX_GROUP, generation, false),
            },
            _ => return self.refuse(TX_GROUP, generation, false),
        };
        let Some(words) = self.burst_words(TX_GROUP, slot) else {
            return self.refuse(TX_GROUP, generation, false);
        };
        if words > room_words {
            return self.refuse(TX_GROUP, generation, true);
        }
        Some(Burst { slot, words })
    }

    /// PE = 1: the group's current channel while it is enabled, otherwise the next enabled
    /// channel of the group after it, which becomes current.
    fn current_channel(&mut self, group: usize) -> Option<usize> {
        let first = group * CHANNELS_PER_GROUP;
        let enabled = |slot: usize| self.contexts[slot].control & TCR_STRT != 0;
        if let Some(slot) = self.current[group].filter(|&slot| enabled(slot)) {
            return Some(slot);
        }
        let after = self.current[group].map_or(0, |slot| slot + 1 - first);
        let next = (0..CHANNELS_PER_GROUP)
            .map(|offset| first + (after + offset) % CHANNELS_PER_GROUP)
            .find(|&slot| enabled(slot))?;
        self.current[group] = Some(next);
        Some(next)
    }

    /// The words one burst of an enabled `slot` moves, or none when the channel is disabled or
    /// set up in a way the model does not run: the group's direction, synchronised transfers of
    /// 32-bit peripheral elements, a constant peripheral address at the group's queue port,
    /// equal burst lengths, a switch element of 32 bits - or of 8 or 16 in block mode - aligned
    /// to its size, and a valid buffer. A block that has fewer elements left than a burst ends
    /// the burst there.
    fn burst_words(&self, group: usize, slot: usize) -> Option<usize> {
        let control = self.peripheral_control[group];
        let context = self.contexts[slot];
        let (to_switch, port) = if group == RX_GROUP {
            (true, RECEIVE_QUEUE_PORT)
        } else {
            (false, TRANSMIT_QUEUE_PORT)
        };
        let psiz = (context.control >> TCR_PSIZ_SHIFT) & 0b11;
        let element = Self::element_bytes(context.control, control)?;
        let pblen = (control >> PCR_PBLEN_SHIFT) & 0xf;
        let sblen = (control >> PCR_SBLEN_SHIFT) & 0xf;
        let supported = context.control & TCR_STRT != 0
            && control & PCR_SYNC != 0
            && (control & PCR_DIR_PERIPHERAL_TO_SWITCH != 0) == to_switch
            && control & PCR_PMOD_CONSTANT != 0
            && psiz == SIZE_32_BIT
            && pblen == sblen
            && context.peripheral == port
            && context.switch & (element - 1) == 0
            && Self::buffer_is_valid(control, &context, element);
        if !supported {
            return None;
        }
        let words = pblen as usize + 1;
        Some(if control & PCR_ABU == 0 {
            words.min(((context.size & BLOCK_COUNT) / element) as usize)
        } else {
            words
        })
    }

    /// The bytes in a switch element, when the model runs that size. The peripheral side is
    /// always a 32-bit word (sections 3.2, 3.4 and 4.4: "the PDMA access to the UTOPIA is
    /// always 32 bits"); the switch side may be narrower, each element then the low bits of the
    /// word (Table 32: "data is always right justified on the bus"). Table 3 gives an
    /// auto-buffering base alignment for 32-bit elements only, so a narrower element is a block
    /// mode form.
    fn element_bytes(transfer: u32, peripheral: u32) -> Option<u32> {
        match (transfer >> TCR_SSIZ_SHIFT) & 0b11 {
            0 if peripheral & PCR_ABU == 0 => Some(1),
            1 if peripheral & PCR_ABU == 0 => Some(2),
            SIZE_32_BIT => Some(4),
            _ => None,
        }
    }

    /// ABU needs a buffer of a valid size with the address pointing into it; block mode needs a
    /// whole number of `element`s still to move.
    fn buffer_is_valid(control: u32, context: &Context, element: u32) -> bool {
        if control & PCR_ABU != 0 {
            let size = context.size & ABU_SWITCH_SIZE;
            if !(ABU_MIN_SIZE..=ABU_MAX_SIZE).contains(&size) || size & 3 != 0 {
                return false;
            }
            let base = Self::buffer_base(context.switch, size);
            context.switch - base < size
        } else {
            let count = context.size & BLOCK_COUNT;
            count != 0 && count % element == 0
        }
    }

    /// An ABU buffer's base address: the address aligned down to the smallest power of two
    /// greater than the buffer size (section 4.3.1.2, Table 3).
    fn buffer_base(address: u32, size: u32) -> u32 {
        address & !((1u32 << (32 - size.leading_zeros())) - 1)
    }

    /// The switch side of an accepted burst: the addresses its words go to or come from, in
    /// order. The channel's context moves on - the address, the block count, and STRT when a
    /// block completes - and the interrupt Table 5 asks for is reported.
    pub(crate) fn advance(&mut self, burst: Burst) -> SwitchBurst {
        let control = self.peripheral_control[burst.slot / CHANNELS_PER_GROUP];
        let constant = control & PCR_SMOD_CONSTANT != 0;
        let context = &mut self.contexts[burst.slot];
        let element = Self::element_bytes(context.control, control)
            .expect("a burst is accepted only for an element size the model runs");
        let mut switch = SwitchBurst {
            element,
            ..SwitchBurst::default()
        };
        let mut half = false;
        let mut end = false;
        if control & PCR_ABU != 0 {
            let size = context.size & ABU_SWITCH_SIZE;
            let base = Self::buffer_base(context.switch, size);
            let mut offset = context.switch - base;
            for _ in 0..burst.words {
                switch.push(base + offset);
                if constant {
                    continue;
                }
                let next = offset + 4;
                half |= offset < size / 2 && next >= size / 2;
                if next >= size {
                    offset = 0;
                    end = true;
                } else {
                    offset = next;
                }
            }
            context.switch = base + offset;
        } else {
            let mut count = context.size & BLOCK_COUNT;
            for _ in 0..burst.words {
                switch.push(context.switch);
                if !constant {
                    context.switch = context.switch.wrapping_add(element);
                }
                count -= element;
            }
            context.size = (context.size & !BLOCK_COUNT) | count;
            if count == 0 {
                context.control = (context.control & !TCR_STRT) | TCR_BCZ;
                end = true;
            }
        }
        let enabled = context.control & TCR_IE != 0 && {
            let imod = context.control & TCR_IMOD != 0;
            if control & PCR_ABU != 0 {
                end || (imod && half)
            } else {
                // Block mode with IMOD set is reserved: no interrupt is defined for it.
                end && !imod
            }
        };
        if enabled {
            switch.interrupt = Some(((context.control >> TCR_SINT_SHIFT) & 0b111) as usize);
        }
        switch
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::system::Endian;

    const CONTEXT: PortContext = PortContext {
        endian: Endian::Little,
    };
    const RXC0: u32 = RX_PROXIES;
    const PCR_RECEIVE: u32 = PCR_SYNC
        | (PE_EVENT_ID << PCR_PE_SHIFT)
        | PCR_ABU
        | PCR_DIR_PERIPHERAL_TO_SWITCH
        | (13 << PCR_PBLEN_SHIFT)
        | PCR_PMOD_CONSTANT
        | (13 << PCR_SBLEN_SHIFT);
    const TCR_RECEIVE: u32 = TCR_STRT
        | TCR_IMOD
        | TCR_IE
        | (SIZE_32_BIT << TCR_PSIZ_SHIFT)
        | (SIZE_32_BIT << TCR_SSIZ_SHIFT);

    fn event(id: usize) -> Option<ReceiveEvent> {
        Some(ReceiveEvent { id, generation: 1 })
    }

    fn program(pdma: &mut Pdma, control: u32, switch: u32, size: u32, tcr: u32) {
        pdma.write(PCR[RX_GROUP], control, CONTEXT);
        pdma.write(RXC0, switch, CONTEXT);
        pdma.write(RXC0 + 4, RECEIVE_QUEUE_PORT, CONTEXT);
        pdma.write(RXC0 + 8, size, CONTEXT);
        pdma.write(RXC0 + 12, tcr, CONTEXT);
        pdma.write(GCSR, GCSR_STRT, CONTEXT);
    }

    #[test]
    fn identification_and_proxy_map_follow_the_manual() {
        let mut pdma = Pdma::default();
        assert_eq!(pdma.read(PID, CONTEXT), Some(0x0001_0401));
        pdma.write(PID, 0xffff_ffff, CONTEXT);
        assert_eq!(pdma.read(PID, CONTEXT), Some(0x0001_0401));
        // Table 27: six proxy blocks of each direction, the receive ones from 0x180.
        assert_eq!(proxy_register(TX_PROXIES + 0x5c), Some((5, 3)));
        assert_eq!(proxy_register(RX_PROXIES), Some((6, 0)));
        assert_eq!(proxy_register(RX_PROXIES + 0x5c), Some((11, 3)));
        assert_eq!(proxy_register(RX_PROXIES + 0x60), None);
        assert_eq!(pdma.read(BASE + 0x28, CONTEXT), None);
        assert_eq!(pdma.write(BASE + 0x68, 1, CONTEXT), PortWrite::Unhandled);
        // GCSR keeps STRT, SOFT and FREE only.
        pdma.write(GCSR, 0xffff_ffff, CONTEXT);
        assert_eq!(pdma.read(GCSR, CONTEXT), Some(0x8000_0003));
        // Reset leaves it halted with SOFT and FREE set.
        assert_eq!(Pdma::default().read(GCSR, CONTEXT), Some(0x0000_0003));
    }

    #[test]
    fn an_enabled_channel_context_changes_only_to_disable_it() {
        let mut pdma = Pdma::default();
        program(&mut pdma, PCR_RECEIVE, 0x10f0_0000, 0x3fe0, TCR_RECEIVE);
        // A second enabling write with a new address is ignored (section 4.2).
        pdma.write(RXC0, 0x10f0_1000, CONTEXT);
        pdma.write(RXC0 + 12, TCR_RECEIVE, CONTEXT);
        assert_eq!(pdma.read(RXC0, CONTEXT), Some(0x10f0_0000));
        assert_eq!(pdma.read(RXC0 + 12, CONTEXT), Some(TCR_RECEIVE));
        // A write with STRT clear replaces the context and disables the channel.
        pdma.write(RXC0, 0x10f0_1000, CONTEXT);
        pdma.write(RXC0 + 12, TCR_RECEIVE & !TCR_STRT, CONTEXT);
        assert_eq!(pdma.read(RXC0, CONTEXT), Some(0x10f0_1000));
        assert_eq!(pdma.read(RXC0 + 12, CONTEXT), Some(TCR_RECEIVE & !TCR_STRT));
        assert!(pdma.receive_event(event(0)).is_none());
    }

    #[test]
    fn a_disabled_channel_or_halted_pdma_takes_no_event_and_counts_it_once() {
        let mut pdma = Pdma::default();
        // Halted: not even an error.
        assert!(pdma.receive_event(event(0)).is_none());
        assert_eq!(pdma.read(SR[RX_GROUP], CONTEXT), Some(0));
        pdma.write(GCSR, GCSR_STRT, CONTEXT);
        pdma.write(PCR[RX_GROUP], PCR_RECEIVE, CONTEXT);
        for _ in 0..5 {
            assert!(pdma.receive_event(event(0)).is_none());
        }
        assert_eq!(pdma.read(SR[RX_GROUP], CONTEXT), Some(1));
        // The event dropping and returning is a new event.
        assert!(pdma.receive_event(None).is_none());
        let later = Some(ReceiveEvent {
            id: 0,
            generation: 2,
        });
        assert!(pdma.receive_event(later).is_none());
        assert_eq!(pdma.read(SR[RX_GROUP], CONTEXT), Some(2));
        pdma.write(PCR[RX_GROUP], PCR_RECEIVE | PCR_CLRS, CONTEXT);
        assert_eq!(pdma.read(SR[RX_GROUP], CONTEXT), Some(0));
        // An event ID for a channel that is not enabled is an error even though another is.
        program(&mut pdma, PCR_RECEIVE, 0x10f0_0000, 0x3fe0, TCR_RECEIVE);
        assert!(pdma.receive_event(None).is_none());
        assert!(pdma.receive_event(event(1)).is_none());
        assert!(pdma.receive_event(event(0)).is_some());
    }

    #[test]
    fn the_circular_buffer_wraps_and_interrupts_at_half_and_end() {
        // Four 14-word packets in a 224-byte buffer: the second burst reaches the half point,
        // the fourth the end, and the pointer comes back to the base (section 4.3.1).
        let base = 0x10f0_0000;
        let mut pdma = Pdma::default();
        program(&mut pdma, PCR_RECEIVE, base, 224, TCR_RECEIVE);
        let mut interrupts = Vec::new();
        for burst in 0..5 {
            let accepted = pdma
                .receive_event(event(0))
                .expect("channel 8 takes the event");
            assert_eq!(accepted.words, 14);
            let switch = pdma.advance(accepted);
            assert_eq!(switch.addresses().len(), 14);
            assert_eq!(switch.addresses()[0], base + (burst % 4) * 56);
            interrupts.push(switch.interrupt);
        }
        assert_eq!(
            interrupts,
            [None, Some(0), None, Some(0), None],
            "half and end of buffer"
        );
        assert_eq!(pdma.read(RXC0, CONTEXT), Some(base + 56));
    }

    #[test]
    fn imod_clear_interrupts_only_at_the_end_and_ie_clear_never() {
        let base = 0x10f0_0000;
        for (tcr, expected) in [
            (TCR_RECEIVE & !TCR_IMOD, [None, None, None, Some(1)]),
            (TCR_RECEIVE & !TCR_IE, [None; 4]),
        ] {
            let mut pdma = Pdma::default();
            program(
                &mut pdma,
                PCR_RECEIVE,
                base,
                224,
                tcr | (1 << TCR_SINT_SHIFT),
            );
            let got: Vec<_> = (0..4)
                .map(|_| {
                    let burst = pdma.receive_event(event(0)).unwrap();
                    pdma.advance(burst).interrupt
                })
                .collect();
            let expected: Vec<_> = expected
                .into_iter()
                .map(|x: Option<usize>| x.filter(|_| tcr & TCR_IE != 0))
                .collect();
            assert_eq!(got, expected);
        }
    }

    #[test]
    fn a_buffer_not_a_power_of_two_aligns_to_the_next_power() {
        // 292 cells of 56 bytes: 16352 bytes, whose base boundary is 16 KB (Table 3).
        let size = 292 * 56;
        assert_eq!(size, 16352);
        assert_eq!(Pdma::buffer_base(0x10f0_3fdc, size), 0x10f0_0000);
        assert_eq!(Pdma::buffer_base(0x10f0_4010, size), 0x10f0_4000);
        // A buffer of exactly 16 KB needs 32 KB alignment, per the table's next row.
        assert_eq!(Pdma::buffer_base(0x10f0_4010, 0x4000), 0x10f0_0000);
        let mut pdma = Pdma::default();
        program(
            &mut pdma,
            PCR_RECEIVE,
            0x10f0_0000 + 16352 - 56,
            size,
            TCR_RECEIVE,
        );
        let burst = pdma.receive_event(event(0)).unwrap();
        let switch = pdma.advance(burst);
        assert_eq!(switch.addresses()[13], 0x10f0_0000 + 16352 - 4);
        assert_eq!(switch.interrupt, Some(0));
        assert_eq!(pdma.read(RXC0, CONTEXT), Some(0x10f0_0000));
    }

    #[test]
    fn block_mode_counts_down_and_stops_the_channel() {
        let mut pdma = Pdma::default();
        let control = PCR_RECEIVE & !PCR_ABU;
        program(
            &mut pdma,
            control,
            0x10f0_0100,
            14 * 4 + 8,
            TCR_RECEIVE & !TCR_IMOD,
        );
        let burst = pdma.receive_event(event(0)).unwrap();
        assert_eq!(burst.words, 14);
        let first = pdma.advance(burst);
        assert_eq!(first.interrupt, None);
        assert_eq!(pdma.read(RXC0, CONTEXT), Some(0x10f0_0100 + 56));
        let running = pdma.read(RXC0 + 12, CONTEXT).unwrap();
        assert_eq!(
            running & (TCR_STRT | TCR_BCZ),
            TCR_STRT,
            "a block under way"
        );
        // Only two words are left, so the burst is cut to them.
        let burst = pdma.receive_event(event(0)).unwrap();
        assert_eq!(burst.words, 2);
        let last = pdma.advance(burst);
        assert_eq!(last.interrupt, Some(0));
        // The size and control proxies read the snapshot a read of the SAR proxy takes: the
        // count is spent, STRT is clear and BCZ says the block completed (Table 39).
        pdma.read(RXC0, CONTEXT);
        assert_eq!(pdma.read(RXC0 + 8, CONTEXT), Some(0));
        let done = pdma.read(RXC0 + 12, CONTEXT).unwrap();
        assert_eq!(done & (TCR_STRT | TCR_BCZ), TCR_BCZ);
        assert!(pdma.receive_event(event(0)).is_none());
    }

    #[test]
    fn configurations_the_model_does_not_run_are_errors_not_transfers() {
        let base = 0x10f0_0000;
        let cases: [(u32, u32, u32, u32); 5] = [
            // Unsynchronised.
            (PCR_RECEIVE & !PCR_SYNC, base, 224, TCR_RECEIVE),
            // Always-respond event control.
            (PCR_RECEIVE | (1 << PCR_PE_SHIFT), base, 224, TCR_RECEIVE),
            // Switch to peripheral.
            (
                PCR_RECEIVE & !PCR_DIR_PERIPHERAL_TO_SWITCH,
                base,
                224,
                TCR_RECEIVE,
            ),
            // 16-bit switch elements around a circular buffer: Table 3 gives 32-bit elements only.
            (
                PCR_RECEIVE,
                base,
                224,
                (TCR_RECEIVE & !(3 << TCR_SSIZ_SHIFT)) | (1 << TCR_SSIZ_SHIFT),
            ),
            // A buffer size that is not a word multiple.
            (PCR_RECEIVE, base, 226, TCR_RECEIVE),
        ];
        for (control, switch, size, tcr) in cases {
            let mut pdma = Pdma::default();
            program(&mut pdma, control, switch, size, tcr);
            assert!(
                pdma.receive_event(event(0)).is_none(),
                "{control:#x} {tcr:#x}"
            );
            assert_eq!(pdma.read(SR[RX_GROUP], CONTEXT), Some(1));
        }
        // A peripheral address that is not the receive queue.
        let mut pdma = Pdma::default();
        program(&mut pdma, PCR_RECEIVE, base, 224, TCR_RECEIVE);
        pdma.write(RXC0 + 12, TCR_RECEIVE & !TCR_STRT, CONTEXT);
        pdma.write(RXC0 + 4, RECEIVE_QUEUE_PORT + 4, CONTEXT);
        pdma.write(RXC0 + 12, TCR_RECEIVE, CONTEXT);
        assert!(pdma.receive_event(event(0)).is_none());
    }

    #[test]
    fn pe_zero_sends_every_event_to_channel_eight() {
        let mut pdma = Pdma::default();
        program(
            &mut pdma,
            (PCR_RECEIVE & !(3 << PCR_PE_SHIFT)) | (PE_CHANNEL_ZERO << PCR_PE_SHIFT),
            0x10f0_0000,
            224,
            TCR_RECEIVE,
        );
        assert!(pdma.receive_event(event(3)).is_some());
    }

    #[test]
    fn pe_one_keeps_the_current_receive_channel_until_its_block_ends() {
        // Table 32, PE = 01b for the receive group: the event ID is not consulted, the current
        // channel takes every event, and another is arbitrated for when its count reaches zero.
        let mut pdma = Pdma::default();
        let block =
            (PCR_RECEIVE & !(PCR_ABU | (3 << PCR_PE_SHIFT))) | (PE_CURRENT_CHANNEL << PCR_PE_SHIFT);
        pdma.write(PCR[RX_GROUP], block, CONTEXT);
        pdma.write(GCSR, GCSR_STRT, CONTEXT);
        for (channel, switch, count) in [(2, 0x1080_0000, 2 * 56), (4, 0x1080_1000, 56)] {
            let proxy = RX_PROXIES + channel * PROXY_STRIDE;
            pdma.write(proxy, switch, CONTEXT);
            pdma.write(proxy + 4, RECEIVE_QUEUE_PORT, CONTEXT);
            pdma.write(proxy + 8, count, CONTEXT);
            pdma.write(proxy + 12, TCR_RECEIVE & !TCR_IMOD, CONTEXT);
        }
        let serve = |pdma: &mut Pdma, id, generation| {
            let burst = pdma.receive_event(Some(ReceiveEvent { id, generation }))?;
            let switch = pdma.advance(burst);
            Some((burst.slot, switch.addresses()[0], switch.interrupt))
        };
        // Receive group channel 2 (slot 8) stays current through its two cells whatever IDs the
        // events carry, although channel 4 is enabled all along; then channel 4 (slot 10) takes
        // over.
        assert_eq!(serve(&mut pdma, 5, 0), Some((8, 0x1080_0000, None)));
        assert_eq!(serve(&mut pdma, 0, 1), Some((8, 0x1080_0038, Some(0))));
        assert_eq!(serve(&mut pdma, 3, 2), Some((10, 0x1080_1000, Some(0))));
        // Both blocks are done: the event finds no channel, and is one error however long it
        // waits.
        assert_eq!(serve(&mut pdma, 0, 3), None);
        assert_eq!(serve(&mut pdma, 0, 3), None);
        assert_eq!(pdma.read(SR[RX_GROUP], CONTEXT), Some(1));
    }

    #[test]
    fn block_mode_moves_narrow_switch_elements_one_per_peripheral_word() {
        // A block of N bytes is N one-byte elements (Table 38), each a 32-bit peripheral word
        // right justified into the element, and the address steps by the element size.
        let base = 0x10f0_0000;
        for (ssiz, element, count) in [(0, 1u32, 5), (1, 2, 10)] {
            let mut pdma = Pdma::default();
            let tcr =
                (TCR_RECEIVE & !(TCR_IMOD | (3 << TCR_SSIZ_SHIFT))) | (ssiz << TCR_SSIZ_SHIFT);
            program(&mut pdma, PCR_RECEIVE & !PCR_ABU, base, count, tcr);
            let burst = pdma.receive_event(event(0)).unwrap();
            // Five elements are left, fewer than the burst length, so the burst is cut to them.
            assert_eq!(burst.words, (count / element) as usize);
            let switch = pdma.advance(burst);
            assert_eq!(switch.element, element);
            assert_eq!(
                switch.addresses(),
                (0..count / element)
                    .map(|i| base + i * element)
                    .collect::<Vec<_>>()
            );
            assert_eq!(switch.interrupt, Some(0));
            pdma.read(RXC0, CONTEXT);
            assert_eq!(pdma.read(RXC0 + 8, CONTEXT), Some(0));
            assert_eq!(
                pdma.read(RXC0 + 12, CONTEXT).unwrap() & (TCR_STRT | TCR_BCZ),
                TCR_BCZ
            );
        }
    }

    #[test]
    fn a_narrow_element_misaligned_or_miscounted_or_circular_is_an_error() {
        let block = PCR_RECEIVE & !PCR_ABU;
        let narrow = |ssiz: u32| {
            (TCR_RECEIVE & !(TCR_IMOD | (3 << TCR_SSIZ_SHIFT))) | (ssiz << TCR_SSIZ_SHIFT)
        };
        for (control, switch, size, tcr) in [
            // A halfword at an odd address, and a halfword count that is odd.
            (block, 0x10f0_0001, 4, narrow(1)),
            (block, 0x10f0_0000, 5, narrow(1)),
            // The reserved element size.
            (block, 0x10f0_0000, 4, narrow(3)),
            // Auto-buffering.
            (PCR_RECEIVE, 0x10f0_0000, 224, narrow(0)),
        ] {
            let mut pdma = Pdma::default();
            program(&mut pdma, control, switch, size, tcr);
            assert!(pdma.receive_event(event(0)).is_none(), "{switch:#x} {size}");
            assert_eq!(pdma.read(SR[RX_GROUP], CONTEXT), Some(1));
        }
    }

    /// Section 4.4's transmit group: PE = 1, block mode, switch to peripheral, a constant
    /// peripheral address and an incrementing switch address, 14-word bursts.
    const PCR_TRANSMIT: u32 = PCR_SYNC
        | (PE_CURRENT_CHANNEL << PCR_PE_SHIFT)
        | (13 << PCR_PBLEN_SHIFT)
        | PCR_PMOD_CONSTANT
        | (13 << PCR_SBLEN_SHIFT);
    const TCR_TRANSMIT: u32 =
        TCR_STRT | TCR_IE | (SIZE_32_BIT << TCR_PSIZ_SHIFT) | (SIZE_32_BIT << TCR_SSIZ_SHIFT);

    fn transmit(generation: u64, room_words: usize) -> Option<TransmitEvent> {
        Some(TransmitEvent {
            generation,
            room_words,
        })
    }

    fn program_transmit(pdma: &mut Pdma, channel: u32, switch: u32, count: u32) {
        let proxy = TX_PROXIES + channel * PROXY_STRIDE;
        pdma.write(proxy, switch, CONTEXT);
        pdma.write(proxy + 4, TRANSMIT_QUEUE_PORT, CONTEXT);
        pdma.write(proxy + 8, count, CONTEXT);
        pdma.write(
            proxy + 12,
            TCR_TRANSMIT | (channel << TCR_SINT_SHIFT),
            CONTEXT,
        );
    }

    #[test]
    fn pe_one_keeps_the_current_transmit_channel_until_its_block_ends() {
        // Table 32, PE = 01b: "Use the current PDMA channel on transfer events", and arbitrate
        // for another when its transfer count reaches zero.
        let mut pdma = Pdma::default();
        pdma.write(PCR[TX_GROUP], PCR_TRANSMIT, CONTEXT);
        pdma.write(GCSR, GCSR_STRT, CONTEXT);
        program_transmit(&mut pdma, 2, 0x1080_0000, 2 * 56);
        program_transmit(&mut pdma, 4, 0x1080_1000, 56);
        let serve = |pdma: &mut Pdma, generation| {
            let burst = pdma.transmit_event(transmit(generation, 28))?;
            let switch = pdma.advance(burst);
            Some((burst.slot, switch.addresses()[0], switch.interrupt))
        };
        // Channel 2 is the first enabled channel and stays current through its two cells,
        // although channel 4 is enabled all along; then channel 4 takes over.
        assert_eq!(serve(&mut pdma, 0), Some((2, 0x1080_0000, None)));
        assert_eq!(serve(&mut pdma, 1), Some((2, 0x1080_0038, Some(2))));
        assert_eq!(serve(&mut pdma, 2), Some((4, 0x1080_1000, Some(4))));
        // Both blocks are done: the event finds no channel, which is one error however long it
        // waits, until a channel is enabled again.
        assert_eq!(serve(&mut pdma, 3), None);
        assert_eq!(serve(&mut pdma, 3), None);
        assert_eq!(pdma.read(SR[TX_GROUP], CONTEXT), Some(1));
        program_transmit(&mut pdma, 2, 0x1080_2000, 56);
        assert_eq!(serve(&mut pdma, 3), Some((2, 0x1080_2000, Some(2))));
    }

    #[test]
    fn a_transmit_burst_waits_for_room_and_counts_one_stall() {
        // Table 31: a stall is the PDMA waiting for space to service the event.
        let mut pdma = Pdma::default();
        pdma.write(PCR[TX_GROUP], PCR_TRANSMIT, CONTEXT);
        pdma.write(GCSR, GCSR_STRT, CONTEXT);
        program_transmit(&mut pdma, 0, 0x1080_0000, 56);
        assert!(pdma.transmit_event(transmit(7, 13)).is_none());
        assert!(pdma.transmit_event(transmit(7, 13)).is_none());
        assert_eq!(pdma.read(SR[TX_GROUP], CONTEXT), Some(1 << 16));
        let burst = pdma.transmit_event(transmit(7, 14)).unwrap();
        assert_eq!(burst.words, 14);
        // The context did not move while it waited.
        assert_eq!(pdma.advance(burst).addresses()[0], 0x1080_0000);
    }

    #[test]
    fn a_transmit_channel_set_up_for_the_other_direction_or_port_is_an_error() {
        for (control, port) in [
            (
                PCR_TRANSMIT | PCR_DIR_PERIPHERAL_TO_SWITCH,
                TRANSMIT_QUEUE_PORT,
            ),
            (PCR_TRANSMIT, RECEIVE_QUEUE_PORT),
            // PE = 2 needs an event ID, which UXEVT does not carry.
            (
                (PCR_TRANSMIT & !(3 << PCR_PE_SHIFT)) | (PE_EVENT_ID << PCR_PE_SHIFT),
                TRANSMIT_QUEUE_PORT,
            ),
        ] {
            let mut pdma = Pdma::default();
            pdma.write(PCR[TX_GROUP], control, CONTEXT);
            pdma.write(GCSR, GCSR_STRT, CONTEXT);
            program_transmit(&mut pdma, 0, 0x1080_0000, 56);
            pdma.write(TX_PROXIES + 12, 0, CONTEXT);
            pdma.write(TX_PROXIES + 4, port, CONTEXT);
            pdma.write(TX_PROXIES + 12, TCR_TRANSMIT, CONTEXT);
            assert!(
                pdma.transmit_event(transmit(0, 28)).is_none(),
                "{control:#x}"
            );
            assert_eq!(pdma.read(SR[TX_GROUP], CONTEXT), Some(1));
        }
    }
}
