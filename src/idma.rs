//! C64x+ megamodule internal DMA channels (SPRU871K chapter 5).
use crate::port::{PortContext, PortWrite, SlavePort};

const IDMA0_STAT: u32 = 0x0182_0000;
const IDMA0_MASK: u32 = 0x0182_0004;
const IDMA0_SOURCE: u32 = 0x0182_0008;
const IDMA0_DEST: u32 = 0x0182_000c;
const IDMA0_COUNT: u32 = 0x0182_0010;
const IDMA1_STAT: u32 = 0x0182_0100;
const IDMA1_SOURCE: u32 = 0x0182_0108;
const IDMA1_DEST: u32 = 0x0182_010c;
const IDMA1_COUNT: u32 = 0x0182_0110;

const IDMA0_EVENT: usize = 13;
const IDMA1_EVENT: usize = 14;
const IDMA_ERROR_EVENT: usize = 97;

const L2_SRAM: core::ops::Range<u32> = 0x0080_0000..0x00a0_0000;
const L1P_SRAM: core::ops::Range<u32> = 0x00e0_0000..0x00e0_8000;
const L1D_SRAM: core::ops::Range<u32> = 0x00f0_0000..0x00f0_8000;
const CFG: core::ops::Range<u32> = 0x0200_0000..0x0400_0000;

#[derive(Clone, Copy)]
pub(crate) enum Source {
    Address(u32),
    Fill(u32),
}

pub(crate) struct Step {
    pub source: Option<Source>,
    pub destination: u32,
    pub completion_event: Option<usize>,
}

#[derive(Clone, Copy)]
struct Transfer {
    source: u32,
    destination: u32,
    words: u32,
    position: u32,
    mask: u32,
    fill: bool,
    interrupt: bool,
}

#[derive(Default)]
struct Channel {
    active: Option<Transfer>,
    pending: Option<Transfer>,
}

impl Channel {
    fn submit(&mut self, transfer: Transfer) {
        if self.active.is_some() {
            self.pending = Some(transfer);
        } else {
            self.active = Some(transfer);
        }
    }

    fn status(&self) -> u32 {
        u32::from(self.active.is_some()) | (u32::from(self.pending.is_some()) << 1)
    }

    fn step(&mut self, completion_event: usize) -> Option<Step> {
        let transfer = self.active.as_mut()?;
        while transfer.position < transfer.words {
            let position = transfer.position;
            transfer.position += 1;
            if transfer.mask & (1 << (position % 32)) != 0 {
                continue;
            }
            let source = if transfer.fill {
                Source::Fill(transfer.source)
            } else {
                Source::Address(transfer.source.wrapping_add(position * 4))
            };
            let destination = transfer.destination.wrapping_add(position * 4);
            let complete = transfer.position == transfer.words;
            let interrupt = complete && transfer.interrupt;
            if complete {
                self.active = self.pending.take();
            }
            return Some(Step {
                source: Some(source),
                destination,
                completion_event: interrupt.then_some(completion_event),
            });
        }
        let interrupt = transfer.interrupt;
        self.active = self.pending.take();
        Some(Step {
            source: None,
            destination: 0,
            completion_event: interrupt.then_some(completion_event),
        })
    }
}

#[derive(Default)]
pub(crate) struct Idma {
    channel0: Channel,
    channel1: Channel,
    mask0: u32,
    source0: u32,
    destination0: u32,
    count0: u32,
    source1: u32,
    destination1: u32,
    count1: u32,
}

impl SlavePort for Idma {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address {
            IDMA0_STAT => self.channel0.status(),
            IDMA0_MASK => self.mask0,
            IDMA0_SOURCE => self.source0 & !0x1f,
            IDMA0_DEST => self.destination0 & !0x1f,
            IDMA0_COUNT => self.count0,
            IDMA1_STAT => self.channel1.status(),
            IDMA1_SOURCE => self.source1,
            IDMA1_DEST => self.destination1 & !3,
            IDMA1_COUNT => self.count1,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            IDMA0_STAT | IDMA1_STAT => PortWrite::Done,
            IDMA0_MASK => {
                self.mask0 = value;
                PortWrite::Done
            }
            IDMA0_SOURCE => {
                self.source0 = value & !0x1f;
                PortWrite::Done
            }
            IDMA0_DEST => {
                self.destination0 = value & !0x1f;
                PortWrite::Done
            }
            IDMA0_COUNT => {
                self.count0 = value & 0x1000_000f;
                let words = ((self.count0 & 0xf) + 1) * 32;
                let bytes = words * 4;
                let source_local = local_range(self.source0, bytes).is_some();
                let destination_local = local_range(self.destination0, bytes).is_some();
                let source_cfg = cfg_range(self.source0, bytes);
                let destination_cfg = cfg_range(self.destination0, bytes);
                if !(source_local && destination_cfg || source_cfg && destination_local) {
                    return PortWrite::Event(IDMA_ERROR_EVENT);
                }
                self.channel0.submit(Transfer {
                    source: self.source0,
                    destination: self.destination0,
                    words,
                    position: 0,
                    mask: self.mask0,
                    fill: false,
                    interrupt: self.count0 & (1 << 28) != 0,
                });
                PortWrite::Done
            }
            IDMA1_SOURCE => {
                self.source1 = value;
                PortWrite::Done
            }
            IDMA1_DEST => {
                self.destination1 = value & !3;
                PortWrite::Done
            }
            IDMA1_COUNT => {
                self.count1 = value & 0xf001_ffff;
                let bytes = self.count1 & 0xffff;
                let fill = self.count1 & (1 << 16) != 0;
                let source_valid = fill || local_range(self.source1 & !3, bytes).is_some();
                let destination_valid = local_range(self.destination1, bytes).is_some();
                if bytes & 3 != 0 || !source_valid || !destination_valid {
                    return PortWrite::Event(IDMA_ERROR_EVENT);
                }
                self.channel1.submit(Transfer {
                    source: if fill {
                        self.source1
                    } else {
                        self.source1 & !3
                    },
                    destination: self.destination1,
                    words: bytes / 4,
                    position: 0,
                    mask: 0,
                    fill,
                    interrupt: self.count1 & (1 << 28) != 0,
                });
                PortWrite::Done
            }
            _ => PortWrite::Unhandled,
        }
    }
}

impl Idma {
    pub fn active(&self) -> bool {
        self.channel0.active.is_some() || self.channel1.active.is_some()
    }

    pub fn tick(&mut self) -> [Option<Step>; 2] {
        [
            self.channel0.step(IDMA0_EVENT),
            self.channel1.step(IDMA1_EVENT),
        ]
    }
}

fn local_range(address: u32, length: u32) -> Option<()> {
    let end = address.checked_add(length)?;
    [L2_SRAM, L1P_SRAM, L1D_SRAM]
        .into_iter()
        .any(|range| range.contains(&address) && end <= range.end)
        .then_some(())
}

fn cfg_range(address: u32, length: u32) -> bool {
    address
        .checked_add(length)
        .is_some_and(|end| CFG.contains(&address) && end <= CFG.end)
}
