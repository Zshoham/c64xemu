//! The SYSCLK3 schedule: the peripherals clocked by SYSCLK3 - the timers, the GPIO edge
//! detector, the UTOPIA port and the device configuration block - step only on the edges where
//! they do something the fabric can see.
//!
//! SPRS276M section 7.6 fixes SYSCLK3 at the CPU clock divided by six, so the device tick meets
//! an edge every sixth cycle, and on nearly every one of them these blocks do nothing but count:
//! a timer walks towards its period, a prescaler towards its match, a pulse towards its end, a
//! clock-detect counter towards its limit. None of that reaches the fabric until it arrives - an
//! event raised, an EDMA3 channel triggered, the UTOPIA level asserted - and until then the only
//! way to see it is to read the block's registers.
//!
//! So each block answers two questions. `quiet_edges` says how many of the coming edges are
//! certain to be quiet: to raise no event, trigger no channel and add nothing to the tick's
//! `moved`. `skip_quiet` advances over that many quiet edges at once, leaving the block exactly
//! where the same number of `tick_sysclk3` calls would. The fabric keeps the earliest edge any
//! block is *due* on and skips the calls into them before it; a block is caught up through the
//! quiet edges it missed whenever something is about to look at it - a register access from any
//! master - and on its due edge, where it then takes that edge's real step.
//!
//! A quiet-edge count is allowed to be short - the block is then stepped for real on an edge that
//! turns out to do nothing, which is what the device tick did on every edge before - but never
//! long.

/// A block the SYSCLK3 schedule can skip. `tick_sysclk3` itself stays an inherent method, because
/// what an edge produces differs per block.
pub(crate) trait Sysclk3Block {
    /// How many of the coming SYSCLK3 edges are certain to be quiet, `u64::MAX` for all of them.
    fn quiet_edges(&self) -> u64;

    /// Advance over `edges` edges that `quiet_edges` promised quiet.
    fn skip_quiet(&mut self, edges: u64);
}

/// A SYSCLK3 block with its place in the schedule: the edge its state reflects, and the edge it
/// must next be stepped on for real.
pub(crate) struct Clocked<T> {
    block: T,
    synced: u64,
    due: u64,
}

impl<T: Sysclk3Block> Clocked<T> {
    /// A block starts synced to edge 0 and is stepped for real on the first edge, which plans it.
    pub(crate) fn new(block: T) -> Self {
        Self {
            block,
            synced: 0,
            due: 1,
        }
    }

    /// The edge the block must next be stepped on.
    #[inline(always)]
    pub(crate) fn due(&self) -> u64 {
        self.due
    }

    /// The block as it stands after `edge` edges, for an access to look at or change. A block
    /// whose clock is gated saw its edges pass without counting.
    #[inline(always)]
    pub(crate) fn at(&mut self, edge: u64, clocked: bool) -> &mut T {
        self.sync(edge, clocked);
        &mut self.block
    }

    /// Bring the block through the edges since it was last synced, every one of them quiet by
    /// construction: the schedule never lets a block's due edge pass without stepping it.
    #[inline(always)]
    pub(crate) fn sync(&mut self, edge: u64, clocked: bool) {
        if edge > self.synced {
            if clocked {
                self.block.skip_quiet(edge - self.synced);
            }
            self.synced = edge;
        }
    }

    /// Step the block through `edge` for real, which must be the edge after the ones already
    /// synced or skipped, and plan its next due edge.
    #[inline(always)]
    pub(crate) fn step<R: Default>(
        &mut self,
        edge: u64,
        clocked: bool,
        tick: impl FnOnce(&mut T) -> R,
    ) -> R {
        self.sync(edge - 1, clocked);
        let produced = if clocked {
            tick(&mut self.block)
        } else {
            R::default()
        };
        self.synced = edge;
        self.plan(clocked);
        produced
    }

    /// Recompute the due edge from the synced state: after a real step, and after an access that
    /// may have changed what the block will do. A gated block does nothing until its clock comes
    /// back, which re-plans it.
    #[inline(always)]
    pub(crate) fn plan(&mut self, clocked: bool) {
        self.due = if clocked {
            self.synced
                .saturating_add(self.block.quiet_edges())
                .saturating_add(1)
        } else {
            u64::MAX
        };
    }
}

/// Assert the two promises a block makes to the schedule from `block`'s state: none of its first
/// `quiet_edges` edges - up to `limit` of them, stepped one at a time with `tick` answering whether
/// the edge was quiet - is anything but quiet, and skipping any number of them leaves the block
/// exactly where stepping them does.
#[cfg(test)]
pub(crate) fn assert_skip_matches_steps<T>(
    block: &T,
    limit: u64,
    rng: &mut TestRng,
    tick: impl Fn(&mut T) -> bool,
) where
    T: Sysclk3Block + Clone + PartialEq + std::fmt::Debug,
{
    let quiet = block.quiet_edges().min(limit);
    // Every prefix length is checked for short runs; longer ones sample a few, always including
    // the whole run.
    let mut lengths: Vec<u64> = if quiet <= 64 {
        (0..=quiet).collect()
    } else {
        (0..6).map(|_| rng.below(quiet + 1)).collect()
    };
    lengths.push(quiet);
    let mut stepped = block.clone();
    let mut taken = 0;
    lengths.sort_unstable();
    for length in lengths {
        while taken < length {
            assert!(
                tick(&mut stepped),
                "edge {} of {} promised quiet was not: {block:?}",
                taken + 1,
                block.quiet_edges()
            );
            taken += 1;
        }
        let mut skipped = block.clone();
        skipped.skip_quiet(length);
        assert_eq!(skipped, stepped, "skipping {length} edges from {block:?}");
    }
}

/// A fixed-seed generator for the property tests, so they stay deterministic without a crate.
#[cfg(test)]
pub(crate) struct TestRng(u64);

#[cfg(test)]
impl TestRng {
    pub(crate) fn new(seed: u64) -> Self {
        Self(seed)
    }

    /// splitmix64.
    pub(crate) fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    pub(crate) fn below(&mut self, bound: u64) -> u64 {
        self.next() % bound
    }

    pub(crate) fn u32(&mut self) -> u32 {
        self.next() as u32
    }
}
