//! The C6472 UTOPIA's peripheral DMA controller, receive direction.
//!
//! SPRUEG2E chapter 4 and section 15.2: a PDMA owns a context RAM of channels - a switch address,
//! a peripheral address, a buffer size or byte count and a transfer control word each - which the
//! CPU programs through per-channel proxy registers (the PIM, section 5), and two peripheral
//! control registers, one per event group. The TX group is channels 0-7 and the RX group channels
//! 8-15, so the first receive proxy block at 0x02BC_0180 is channel 8 (SPRS612G Table 7-143: six
//! of each exist).
//!
//! Only the receive path is modelled, because it is the one the ROM's UTOPIA boot drives
//! (SPRUEC6G section 4.3.4). A receive event - UREVT with the routing unit's channel ID - starts
//! one burst: the peripheral side reads `PBLEN + 1` words from the receive queue and the switch
//! side writes them out, in auto-buffering mode through a circular buffer (section 4.3.1) or in
//! block mode until the byte count runs out (section 4.3.2). The interrupt rules are Table 5: ABU
//! with IE raises one at the end of the buffer, and with IMOD also at the half-buffer point; block
//! mode raises one when the block completes. The transmit group's contexts are programmable but no
//! UXEVT is serviced.
//!
//! Not modelled, and refused as an error count in SR1 rather than half-done: event control other
//! than PE = 0 (channel 0 of the group) and PE = 2 (the event ID picks an active channel);
//! unsynchronised transfers (SYNC = 0); peripheral or switch element sizes other than 32 bits;
//! incrementing peripheral addressing (the receive queue is one FIFO word); a peripheral burst
//! length different from the switch's; and the emulation halt bits, which section 11.2 says the
//! peripheral event groups ignore anyway.
//!
//! The peripheral address (PAR) of a receive channel must be `RECEIVE_QUEUE_PORT`, the offset
//! Table 18 gives URQ. That the PDMA's PAR is such an offset rather than a global address is an
//! interpretation of Table 18 and section 4.1 [INFERENCE]: no document available here gives the
//! data port a global address, and SPRS612G Table 2-2 maps none.

use crate::port::{PortContext, PortWrite, SlavePort};
/// The receive queue as the PDMA's peripheral bus addresses it. SPRUEG2E Table 18 lists the UTOPIA
/// data ports by offset - URQ at 0, UXQ at 4 - and the PDMA reaches its peripheral over a bus of
/// its own (section 4.1) rather than through the switch, so PAR is that offset. Where the CPU sees
/// the same port in the global map is not documented (`utopia.rs`).
pub(crate) const RECEIVE_QUEUE_PORT: u32 = 0;

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
/// The event group the receive channels (8-15) belong to; the transmit group is 0.
const RX_GROUP: usize = 1;

/// PID: type 1 (PDMA) in 23-16, class 4 (DMA controller) in 15-8, revision 1 (Table 29).
const PID_VALUE: u32 = (0x01 << 16) | (0x04 << 8) | 0x01;
/// GCSR: STRT in bit 31 enables the PDMA, SOFT and FREE in 1-0 are the emulation controls.
const GCSR_STRT: u32 = 1 << 31;
const GCSR_WRITABLE: u32 = GCSR_STRT | 0b11;

/// PCR (Table 32). CLRS is a write-only strobe.
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
const PE_EVENT_ID: u32 = 2;

/// TCR (Table 39): STRT 31, IMOD 28, IE 27, SINT 26-24, PSIZ 23-22, SSIZ 21-20, FF 17, BCZ 16,
/// WRPTR 14-8 and RDPTR 6-0 are writable; the rest read zero.
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

/// A receive burst the PDMA has accepted: which channel, and how many words to take from the
/// receive queue.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ReceiveBurst {
    slot: usize,
    pub(crate) words: usize,
}

/// What a finished burst owes the rest of the device.
#[derive(Debug, Default, PartialEq, Eq)]
pub(crate) struct Delivery {
    stores: [(u32, u32); MAX_BURST_WORDS],
    count: usize,
    /// The DSP subsystem the channel's interrupt goes to (SINT), if it raised one.
    pub(crate) interrupt: Option<usize>,
}

impl Delivery {
    /// Words to write into switch memory, as (global address, word), in order.
    pub(crate) fn stores(&self) -> &[(u32, u32)] {
        &self.stores[..self.count]
    }

    fn push(&mut self, address: u32, word: u32) {
        self.stores[self.count] = (address, word);
        self.count += 1;
    }
}

#[derive(Default)]
#[cfg_attr(test, derive(Clone, Debug, PartialEq))]
pub(crate) struct Pdma {
    global: u32,
    peripheral_control: [u32; 2],
    /// SR0 and SR1: the stall count in the upper half and the error count in the lower.
    statistics: [u32; 2],
    contexts: [Context; SLOTS],
    proxies: [Context; SLOTS],
    /// The assertion of UREVT whose refusal has already been counted as an error, so a cell
    /// that waits for its channel is one error rather than one per clock.
    error_generation: Option<u64>,
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

    fn error(&mut self, group: usize) {
        self.statistics[group] = (self.statistics[group] & 0xffff_0000)
            | (self.statistics[group].wrapping_add(1) & 0xffff);
    }

    fn receive_error(&mut self, generation: u64) -> Option<ReceiveBurst> {
        if self.error_generation != Some(generation) {
            self.error_generation = Some(generation);
            self.error(RX_GROUP);
        }
        None
    }

    /// The receive event the UTOPIA is presenting - UREVT with the routing unit's channel ID,
    /// and which assertion of it this is - or none when it is not asserted. Answers the burst
    /// the PDMA starts for it. An event the PDMA is not set up to service counts once in SR1's
    /// error field and stays pending in the peripheral until a channel can take it (section 5,
    /// the event server).
    pub(crate) fn receive_event(&mut self, event: Option<ReceiveEvent>) -> Option<ReceiveBurst> {
        let ReceiveEvent {
            id: event_id,
            generation,
        } = event?;
        if self.global & GCSR_STRT == 0 {
            // A halted PDMA does not look at events at all.
            return None;
        }
        let control = self.peripheral_control[RX_GROUP];
        if control & PCR_SYNC == 0 {
            return self.receive_error(generation);
        }
        let in_group = match (control >> PCR_PE_SHIFT) & 0b11 {
            PE_CHANNEL_ZERO => 0,
            PE_EVENT_ID => event_id,
            _ => return self.receive_error(generation),
        };
        if in_group >= CHANNELS_PER_GROUP {
            return self.receive_error(generation);
        }
        let slot = CHANNELS_PER_GROUP + in_group;
        let context = self.contexts[slot];
        if context.control & TCR_STRT == 0 {
            return self.receive_error(generation);
        }
        let psiz = (context.control >> TCR_PSIZ_SHIFT) & 0b11;
        let ssiz = (context.control >> TCR_SSIZ_SHIFT) & 0b11;
        let pblen = (control >> PCR_PBLEN_SHIFT) & 0xf;
        let sblen = (control >> PCR_SBLEN_SHIFT) & 0xf;
        let supported = control & PCR_DIR_PERIPHERAL_TO_SWITCH != 0
            && control & PCR_PMOD_CONSTANT != 0
            && psiz == SIZE_32_BIT
            && ssiz == SIZE_32_BIT
            && pblen == sblen
            && context.peripheral == RECEIVE_QUEUE_PORT
            && context.switch & 3 == 0
            && self.buffer_is_valid(control, &context);
        if !supported {
            return self.receive_error(generation);
        }
        let mut words = pblen as usize + 1;
        if control & PCR_ABU == 0 {
            words = words.min((context.size & BLOCK_COUNT) as usize / 4);
        }
        Some(ReceiveBurst { slot, words })
    }

    /// ABU needs a buffer of a valid size with the address pointing into it; block mode needs a
    /// whole number of words still to move.
    fn buffer_is_valid(&self, control: u32, context: &Context) -> bool {
        if control & PCR_ABU != 0 {
            let size = context.size & ABU_SWITCH_SIZE;
            if !(ABU_MIN_SIZE..=ABU_MAX_SIZE).contains(&size) || size & 3 != 0 {
                return false;
            }
            let base = Self::buffer_base(context.switch, size);
            context.switch - base < size
        } else {
            let count = context.size & BLOCK_COUNT;
            count != 0 && count & 3 == 0
        }
    }

    /// An ABU buffer's base address: the address aligned down to the smallest power of two
    /// greater than the buffer size (section 4.3.1.2, Table 3).
    fn buffer_base(address: u32, size: u32) -> u32 {
        address & !((1u32 << (32 - size.leading_zeros())) - 1)
    }

    /// The words the burst read from the receive queue, placed in the switch buffer. The channel's
    /// context moves on - the address, the block count, and STRT when a block completes - and
    /// the interrupt Table 5 asks for is reported.
    pub(crate) fn deliver_receive(&mut self, burst: ReceiveBurst, words: &[u32]) -> Delivery {
        let control = self.peripheral_control[RX_GROUP];
        let constant = control & PCR_SMOD_CONSTANT != 0;
        let context = &mut self.contexts[burst.slot];
        let mut delivery = Delivery::default();
        let mut half = false;
        let mut end = false;
        if control & PCR_ABU != 0 {
            let size = context.size & ABU_SWITCH_SIZE;
            let base = Self::buffer_base(context.switch, size);
            let mut offset = context.switch - base;
            for &word in words {
                delivery.push(base + offset, word);
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
            for &word in words {
                delivery.push(context.switch, word);
                if !constant {
                    context.switch = context.switch.wrapping_add(4);
                }
                count -= 4;
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
            delivery.interrupt = Some(((context.control >> TCR_SINT_SHIFT) & 0b111) as usize);
        }
        delivery
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
        let words = [0xa5a5_0000u32; 14];
        let mut interrupts = Vec::new();
        for burst in 0..5 {
            let accepted = pdma
                .receive_event(event(0))
                .expect("channel 8 takes the event");
            assert_eq!(accepted.words, 14);
            let delivery = pdma.deliver_receive(accepted, &words);
            assert_eq!(delivery.stores().len(), 14);
            assert_eq!(delivery.stores()[0].0, base + (burst % 4) * 56);
            interrupts.push(delivery.interrupt);
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
                    pdma.deliver_receive(burst, &[0; 14]).interrupt
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
        let delivery = pdma.deliver_receive(burst, &[0; 14]);
        assert_eq!(delivery.stores()[13].0, 0x10f0_0000 + 16352 - 4);
        assert_eq!(delivery.interrupt, Some(0));
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
        let first = pdma.deliver_receive(burst, &[1; 14]);
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
        let last = pdma.deliver_receive(burst, &[2; 2]);
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
            // 16-bit switch elements.
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
}
