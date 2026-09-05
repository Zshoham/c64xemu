//! The device address map: where each memory lives, and how a core's local aliases resolve.
//!
//! A C64x+ megamodule sees its own L2, L1P and L1D SRAM through a local window - L2 at
//! 0080 0000h, L1P at 00E0 0000h, L1D at 00F0 0000h - which means nothing to any other master.
//! A single-core device can stop there, and the C6455 does. A multicore device cannot: the
//! C6472 gives every megamodule's memories a second, global address (SPRS612G Table 2-2), and
//! those are the addresses another core, the EMAC, an EDMA3 transfer controller or the host
//! port must use. This module owns that translation and nothing else: given the requester and
//! an address it says which physical memory the access lands in, so the memory hierarchy can
//! keep the caches (which are tagged by the address the requester issued) separate from the
//! storage behind them (which is one device-wide array per memory).
//!
//! The local aliases are deliberately *not* canonicalised away. Two addresses for the same
//! byte are two cache tags on real silicon, and firmware that mixes them sees stale data; that
//! hazard is the reason TI documents "use the global address from any other master". Modelling
//! the translation at the storage layer and leaving the tags alone reproduces it.

/// Which memory an access lands in, once the requester's local aliases are resolved.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Placement {
    /// A megamodule's L2 SRAM: the core that owns it, and the byte offset into it.
    CoreL2 { core: usize, offset: u32 },
    /// A megamodule's L1P SRAM.
    CoreL1P { core: usize, offset: u32 },
    /// A megamodule's L1D SRAM.
    CoreL1D { core: usize, offset: u32 },
    /// The shared L2 RAM every core reaches, through its local window or any core's alias.
    Shared { offset: u32 },
    /// The shared L2 ROM the boot controller starts core 0 in.
    SharedRom { offset: u32 },
    /// The DDR2 window, served by dense pages allocated on first touch.
    Dram { offset: u32 },
    /// Local SRAM the cache configuration has converted into cache. The conversion runs from
    /// the top of each window downwards - SPRU862B section 2.3 says of L2 that "when the L2
    /// cache size is increased, the memory is taken from the high memory addresses", and
    /// SPRU871K section 2.4 says L1P "converts memory from RAM to cache by starting at the top
    /// of the L1P memory map and working downwards" - so what is left as SRAM is always a
    /// prefix of the window. TI's note is that this memory "is not valid for the linker to
    /// place code or data in" (SPRU862B section 2.3), and no document in this set says what
    /// silicon returns for an access to it, so the model answers as nothing: reads give zero
    /// and writes are dropped. That is the one thing the manual does license - the bytes are
    /// not there - and it is what makes a section linked over the carve observably gone.
    ConvertedToCache,
    /// Anything else: external asynchronous memory and addresses no memory answers, served
    /// sparsely so an emulated device does not allocate four gigabytes.
    Sparse,
}

/// Who is making an access. The local aliases exist only for a megamodule; a DMA master's
/// address is always global (SPRS612G Table 2-2 marks the local window "DATA SPACE ON DMA",
/// i.e. it does not resolve to any megamodule's memory from the switch fabric).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Requester {
    Core(usize),
    Dma,
}

/// One memory's local window.
#[derive(Clone, Copy)]
struct Window {
    base: u32,
    bytes: u32,
}

impl Window {
    const fn none() -> Self {
        Self { base: 0, bytes: 0 }
    }

    #[inline]
    fn offset(&self, address: u32) -> Option<u32> {
        let offset = address.wrapping_sub(self.base);
        (self.bytes != 0 && offset < self.bytes).then_some(offset)
    }
}

/// The memory map of one device.
pub(crate) struct AddressMap {
    /// How many megamodules the device has.
    pub(crate) cores: usize,
    /// The local L2, L1P and L1D SRAM windows, identical for every core.
    l2: Window,
    l1p: Window,
    l1d: Window,
    /// The shared L2 RAM and ROM local windows, absent on a single-core device.
    shared_ram: Window,
    shared_rom: Window,
    /// The global alias block: core `n`'s memories appear at `global + n * stride`, carrying
    /// the same offsets the local windows use. Zero when the device has no global aliases.
    global_base: u32,
    global_stride: u32,
    /// The DDR2 window.
    dram: Window,
}

/// The C6455: one megamodule, 2 MB of L2 SRAM, no global aliases and no shared memory
/// (SPRS276M section 2.3, Table 2-2).
pub(crate) const C6455_MAP: AddressMap = AddressMap {
    cores: 1,
    l2: Window {
        base: 0x0080_0000,
        bytes: 2 * 1024 * 1024,
    },
    l1p: Window {
        base: 0x00e0_0000,
        bytes: 32 * 1024,
    },
    l1d: Window {
        base: 0x00f0_0000,
        bytes: 32 * 1024,
    },
    shared_ram: Window::none(),
    shared_rom: Window::none(),
    global_base: 0,
    global_stride: 0,
    dram: Window {
        base: 0xe000_0000,
        bytes: 0x1000_0000,
    },
};

/// The C6472: six megamodules with 608 KB of L2 SRAM each, 768 KB of shared L2 RAM and 768 KB
/// of shared L2 ROM, and a global alias per core every 16 MB from 1000 0000h (SPRS612G
/// Table 2-2 and section 5.1).
///
/// The shared ROM range is the one place Table 2-2 and section 5.1 disagree: the memory map
/// calls 0000 0000h-001F FFFFh reserved, while section 5.1 and Table 5-1 place 768 KB of ROM
/// at 0010 0000h - which is also where Table 2-4 starts core 0 for every ROM boot mode. The
/// ROM is modelled where the two sections that describe it say it is.
pub(crate) const C6472_MAP: AddressMap = AddressMap {
    cores: 6,
    l2: Window {
        base: 0x0080_0000,
        bytes: 608 * 1024,
    },
    l1p: Window {
        base: 0x00e0_0000,
        bytes: 32 * 1024,
    },
    l1d: Window {
        base: 0x00f0_0000,
        bytes: 32 * 1024,
    },
    shared_ram: Window {
        base: 0x0020_0000,
        bytes: 768 * 1024,
    },
    shared_rom: Window {
        base: 0x0010_0000,
        bytes: 768 * 1024,
    },
    global_base: 0x1000_0000,
    global_stride: 0x0100_0000,
    dram: Window {
        base: 0xe000_0000,
        bytes: 0x2000_0000,
    },
};

impl AddressMap {
    pub(crate) const fn l2_bytes(&self) -> u32 {
        self.l2.bytes
    }

    pub(crate) const fn l1p_bytes(&self) -> u32 {
        self.l1p.bytes
    }

    pub(crate) const fn l1d_bytes(&self) -> u32 {
        self.l1d.bytes
    }

    pub(crate) const fn shared_ram_bytes(&self) -> u32 {
        self.shared_ram.bytes
    }

    pub(crate) const fn shared_rom_bytes(&self) -> u32 {
        self.shared_rom.bytes
    }

    pub(crate) const fn dram_bytes(&self) -> u32 {
        self.dram.bytes
    }

    /// The local L2 SRAM window, which is where a core's own code and data live and the only
    /// memory the coherence protocol snoops (SPRU862B chapter 2).
    pub(crate) const fn local_l2(&self) -> (u32, u32) {
        (self.l2.base, self.l2.bytes)
    }

    /// The two local L1 SRAM windows, which no cache inside the megamodule that owns them may
    /// hold (SPRU871K sections 2.3.1 and 3.2.1).
    pub(crate) const fn local_l1(&self) -> [(u32, u32); 2] {
        [
            (self.l1p.base, self.l1p.bytes),
            (self.l1d.base, self.l1d.bytes),
        ]
    }

    /// Where one core's memories appear in the global alias block, if the device has one: the
    /// same offsets the local windows use, at a base of the core's own.
    pub(crate) const fn global_alias_base(&self, core: usize) -> Option<u32> {
        if self.global_stride == 0 {
            return None;
        }
        Some(self.global_base + core as u32 * self.global_stride)
    }

    /// Whether this device aliases each core's memories into a global block.
    pub(crate) const fn has_global_aliases(&self) -> bool {
        self.global_stride != 0
    }

    /// The local address a global one names, for the core that owns it: the inverse of
    /// `global_of_local`, used by the snoop path to find a line a core cached locally.
    pub(crate) fn local_of_global(&self, address: u32) -> Option<(usize, u32)> {
        if !self.has_global_aliases() {
            return None;
        }
        let offset = address.checked_sub(self.global_base)?;
        let core = (offset / self.global_stride) as usize;
        if core >= self.cores {
            return None;
        }
        Some((core, offset % self.global_stride))
    }

    /// Where an access lands. The order matters: the local windows are checked first because
    /// they are what a core issues almost every cycle, and the global block is a single
    /// subtraction away when it is not one of them.
    #[inline]
    pub(crate) fn placement(&self, requester: Requester, address: u32) -> Placement {
        // A device with no global aliases has only one megamodule, so its local window *is* the
        // address every master uses: the C6455's EMAC and EDMA3 both address L2 SRAM at
        // 0080 0000h (SPRS276M Table 2-2).
        let local_view = match requester {
            Requester::Core(core) => Some(core),
            Requester::Dma => (!self.has_global_aliases()).then_some(0),
        };
        if let Some(core) = local_view {
            if let Some(offset) = self.l2.offset(address) {
                return Placement::CoreL2 { core, offset };
            }
            if let Some(offset) = self.shared_ram.offset(address) {
                return Placement::Shared { offset };
            }
            if let Some(offset) = self.l1d.offset(address) {
                return Placement::CoreL1D { core, offset };
            }
            if let Some(offset) = self.l1p.offset(address) {
                return Placement::CoreL1P { core, offset };
            }
            if let Some(offset) = self.shared_rom.offset(address) {
                return Placement::SharedRom { offset };
            }
        }
        if let Some(offset) = self.dram.offset(address) {
            return Placement::Dram { offset };
        }
        if self.has_global_aliases() {
            if let Some((core, local)) = self.local_of_global(address) {
                if let Some(offset) = self.l2.offset(local) {
                    return Placement::CoreL2 { core, offset };
                }
                if let Some(offset) = self.shared_ram.offset(local) {
                    return Placement::Shared { offset };
                }
                if let Some(offset) = self.l1d.offset(local) {
                    return Placement::CoreL1D { core, offset };
                }
                if let Some(offset) = self.l1p.offset(local) {
                    return Placement::CoreL1P { core, offset };
                }
                return Placement::Sparse;
            }
        }
        Placement::Sparse
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn c6455_local_l2_is_core_zero() {
        assert_eq!(
            C6455_MAP.placement(Requester::Core(0), 0x0080_0100),
            Placement::CoreL2 {
                core: 0,
                offset: 0x100
            }
        );
        // A single-core device has no alias block, so the C6472 global range is ordinary
        // unbacked memory on it.
        assert_eq!(
            C6455_MAP.placement(Requester::Core(0), 0x1080_0000),
            Placement::Sparse
        );
    }

    #[test]
    fn c6455_dma_reaches_l2_through_the_local_window() {
        // The C6455's EMAC and EDMA3 address L2 SRAM at 0080 0000h; there is no other name
        // for it on that device (SPRS276M Table 2-2).
        assert_eq!(
            C6455_MAP.placement(Requester::Dma, 0x0080_0004),
            Placement::CoreL2 { core: 0, offset: 4 }
        );
    }

    #[test]
    fn c6472_local_window_follows_the_requesting_core() {
        for core in 0..6 {
            assert_eq!(
                C6472_MAP.placement(Requester::Core(core), 0x0080_0040),
                Placement::CoreL2 { core, offset: 0x40 }
            );
        }
    }

    #[test]
    fn c6472_global_aliases_name_every_core() {
        for core in 0..6 {
            let base = 0x1000_0000 + 0x0100_0000 * core as u32;
            assert_eq!(
                C6472_MAP.placement(Requester::Dma, base + 0x0080_0008),
                Placement::CoreL2 { core, offset: 8 }
            );
            assert_eq!(
                C6472_MAP.placement(Requester::Dma, base + 0x00e0_0000),
                Placement::CoreL1P { core, offset: 0 }
            );
            assert_eq!(
                C6472_MAP.placement(Requester::Dma, base + 0x00f0_0010),
                Placement::CoreL1D { core, offset: 0x10 }
            );
            // Every core's shared-RAM alias is the same 768 KB (SPRUEG5D section 2.2).
            assert_eq!(
                C6472_MAP.placement(Requester::Dma, base + 0x0020_0000),
                Placement::Shared { offset: 0 }
            );
        }
    }

    #[test]
    fn c6472_local_alias_does_not_resolve_for_a_dma_master() {
        assert_eq!(
            C6472_MAP.placement(Requester::Dma, 0x0080_0000),
            Placement::Sparse
        );
    }

    #[test]
    fn c6472_l2_sram_ends_at_608k() {
        assert_eq!(
            C6472_MAP.placement(Requester::Core(3), 0x0089_7fff),
            Placement::CoreL2 {
                core: 3,
                offset: 608 * 1024 - 1
            }
        );
        assert_eq!(
            C6472_MAP.placement(Requester::Core(3), 0x0089_8000),
            Placement::Sparse
        );
    }

    #[test]
    fn c6472_shared_rom_is_where_core_zero_boots() {
        // Table 2-4 starts core 0 at 0010 0000h for every ROM boot mode, and section 5.1
        // places 768 KB of shared ROM there.
        assert_eq!(
            C6472_MAP.placement(Requester::Core(0), 0x0010_0000),
            Placement::SharedRom { offset: 0 }
        );
    }
}
