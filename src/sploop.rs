//! The C64x+ SPLOOP hardware loop buffer.
//!
//! The buffer holds one scheduled iteration of a software-pipelined loop and overlays
//! time-shifted copies of it, so the prolog, kernel and epilog are not written anywhere - they
//! emerge from which stages are valid at a given cycle offset (SPRU732J chapter 7).
//!
//! What this module owns is the state machine: the iteration interval, the buffered body, the
//! iteration counters, whether the loop is loading, draining or waiting to reload, and whether
//! the drain is for an interrupt. The CPU owns the cycle engine and every register, and the two
//! meet at the operations below. That split is the point: the state used to sit on the CPU as an
//! eighteen-field value that the cycle step, both execute paths, the interrupt pipe-down, the
//! exception teardown and the trace all reached into directly, so every one of them had to know
//! what "loading and not yet reloading" meant.
//!
//! Deliberately not modelled as the manual's four counters. The manual describes LBC, a loading
//! counter, a draining counter and a reloading counter; this keeps the loop's start cycle and
//! derives the offset from the current one, which answers the same questions with less state
//! because nothing here has to retrace load order - the body is a list in load order already.

use crate::view::{Bank, SploopDump, SploopEntry};

/// One execute packet of the loop body, with the cycle offset from the loop start at which it
/// was loaded. The offset is what places it in the pipeline: a packet loaded at offset n runs
/// again at n + ii, n + 2*ii, and so on.
pub(crate) struct BufferedPacket<F> {
    pub(crate) offset: u32,
    pub(crate) instructions: Vec<F>,
}

/// The stages active in one cycle, packed into one allocation.
///
/// Stage ends retain packet boundaries because validation and execution still work a packet at
/// a time. The CPU keeps one of these and lends it out per cycle, because a loop that replays
/// for a million cycles must not allocate a million times.
pub(crate) struct ReplayPackets<F> {
    pub(crate) instructions: Vec<F>,
    pub(crate) ends: Vec<usize>,
}

impl<F> Default for ReplayPackets<F> {
    fn default() -> Self {
        Self {
            instructions: Vec::new(),
            ends: Vec::new(),
        }
    }
}

impl<F> ReplayPackets<F> {
    pub(crate) fn packets(&self) -> impl Iterator<Item = &[F]> {
        let mut start = 0;
        self.ends.iter().map(move |&end| {
            let packet = &self.instructions[start..end];
            start = end;
            packet
        })
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.ends.is_empty()
    }

    pub(crate) fn len(&self) -> usize {
        self.ends.len()
    }

    pub(crate) fn clear(&mut self) {
        self.instructions.clear();
        self.ends.clear();
    }
}

/// A register a loop test reads: which bank, which index, and whether the test passes when the
/// register is zero.
pub(crate) type TestRegister = (Bank, usize, bool);

/// What an `SPLOOP`, `SPLOOPD` or `SPLOOPW` instruction asks the buffer to start.
pub(crate) struct LoopStart {
    /// The iteration interval, in cycles.
    pub(crate) ii: u32,
    pub(crate) start_cycle: u64,
    /// The address of the execute packet holding the loop-start instruction. An interrupt taken
    /// during the loop returns here so the loop pipes back up (SPRUFE8B 7.13.4).
    pub(crate) start_address: u32,
    /// `SPLOOPD` and `SPLOOPW` allow ILC to be written in parallel with the loop start, so the
    /// internal down-counter is loaded four cycles later - the ILC load latency.
    pub(crate) delayed: bool,
    /// `SPLOOPW` is a do-while: its predicate is the loop-exit test and ILC is unused. The
    /// register is re-read at every stage boundary; when the test fails the loop exits abruptly
    /// and iterations already started simply do not finish.
    pub(crate) while_test: Option<TestRegister>,
    /// A predicated `SPLOOP(D)` marks a nested loop, and the register its would-be predicate
    /// field names is the reload selector: `SPMASKR`/`SPKERNELR` reloads ILC from RILC and
    /// replays the buffer only while that register says the outer loop has another pass to run
    /// (binutils `nfu_uspldr`: "use op field as a predicate address register selector").
    pub(crate) reload_test: Option<TestRegister>,
    pub(crate) nested: bool,
}

/// What a stage boundary asks the CPU to do.
#[derive(Default)]
pub(crate) struct StageOutcome {
    /// ILC's new value, when the boundary decremented it.
    pub(crate) set_ilc: Option<u32>,
}

/// What completing a cycle asks the CPU to do.
#[derive(Default)]
pub(crate) struct DrainOutcome {
    /// ILC reloaded from RILC, when a nested loop's reload took effect.
    pub(crate) set_ilc: Option<u32>,
    /// The loop finished draining for an interrupt: resume at this address so it pipes back up.
    pub(crate) restart: Option<u32>,
}

struct Active<F> {
    ii: u32,
    start_cycle: u64,
    loading: bool,
    body: Vec<BufferedPacket<F>>,
    delayed: bool,
    while_test: Option<TestRegister>,
    reload_test: Option<TestRegister>,
    nested: bool,
    reload_pending: bool,
    /// Cycles for which the program-memory side still issues alongside a reloaded buffer:
    /// program fetch is disabled again at once, but the packets that were already in flight -
    /// the SPMASKed complement of the buffer's first stages - still execute.
    reload_overlap: u32,
    /// A reloaded invocation has no loading pass from program memory, so the buffer supplies the
    /// first iteration as well.
    replays_first_iteration: bool,
    /// A drained nested loop keeps its body in the buffer while the outer loop runs, waiting for
    /// the reload point.
    awaiting_reload: bool,
    counter: Option<u32>,
    started: u32,
    total: Option<u32>,
    start_address: u32,
    /// Set when the loop is draining because an interrupt is pending: no new iterations start and
    /// the interrupt is taken once the drain finishes.
    draining_for_interrupt: bool,
}

impl<F> Active<F> {
    /// The length of one iteration of the body in cycles, counting from the cycle after the
    /// loop-start instruction. The manual calls this dynlen.
    fn dynlen(&self) -> u32 {
        self.body
            .last()
            .map_or(0, |packet| packet.offset + 1)
            .max(1)
    }

    /// The offset at which the last stage of the last iteration finishes.
    fn final_offset(&self) -> Option<u32> {
        let last = self.body.last().map(|packet| packet.offset).unwrap_or(0);
        self.total
            .map(|total| last.saturating_add(total.saturating_sub(1).saturating_mul(self.ii)))
    }

    /// Restart the iteration counters for a reloaded invocation.
    ///
    /// A reloaded invocation takes ILC from RILC at once: the four-cycle delayed testing belongs
    /// to the `SPLOOPD` instruction itself, which is why the compiler writes the full iteration
    /// count to RILC but ILC minus one.
    fn restart(&mut self, cycle: u64, overlap: u32) {
        self.start_cycle = cycle;
        self.counter = None;
        self.started = 0;
        self.total = None;
        self.reload_overlap = overlap;
        self.replays_first_iteration = true;
        self.delayed = false;
    }

    /// Stop starting iterations so the loop drains.
    fn begin_drain(&mut self) {
        if self.total.is_none() {
            self.total = Some(self.started);
        }
    }
}

/// One core's loop buffer, idle or active.
pub(crate) struct LoopBuffer<F> {
    state: Option<Active<F>>,
}

impl<F> Default for LoopBuffer<F> {
    fn default() -> Self {
        Self { state: None }
    }
}

impl<F: Copy> LoopBuffer<F> {
    /// True while the buffer is active, which is what `TSR.SPLX` mirrors.
    #[inline(always)]
    pub(crate) fn active(&self) -> bool {
        self.state.is_some()
    }

    /// The iteration interval of the active loop, for the trace.
    pub(crate) fn interval(&self) -> Option<u32> {
        self.state.as_ref().map(|state| state.ii)
    }

    /// The buffer is issuing on its own: the loading pass is over, no reloaded invocation is
    /// still overlapping the program-memory side, and the loop is not parked waiting to reload.
    ///
    /// This is the condition for a replay-only cycle, and it is one place because it used to be
    /// three copies of the same three-field test - the run loop's fast path, the cycle step, and
    /// the interrupt-drain readiness check.
    #[inline(always)]
    pub(crate) fn replaying_alone(&self) -> bool {
        self.state.as_ref().is_some_and(|state| {
            !state.loading && state.reload_overlap == 0 && !state.awaiting_reload
        })
    }

    /// How many cycles into the loop body this cycle is.
    #[inline(always)]
    pub(crate) fn offset(&self, cycle: u64) -> Option<u32> {
        self.state
            .as_ref()
            .map(|state| (cycle - state.start_cycle - 1) as u32)
    }

    /// Whether the buffer is still being filled from program memory.
    pub(crate) fn loading(&self) -> bool {
        self.state.as_ref().is_some_and(|state| state.loading)
    }

    /// Whether the active loop is a nested one, which a taken branch does not evict.
    pub(crate) fn nested(&self) -> bool {
        self.state.as_ref().is_some_and(|state| state.nested)
    }

    pub(crate) fn while_register(&self) -> Option<TestRegister> {
        self.state.as_ref()?.while_test
    }

    pub(crate) fn reload_register(&self) -> Option<TestRegister> {
        self.state.as_ref()?.reload_test
    }

    /// Start a loop, unless one that is not merely waiting for its reload point already holds
    /// the buffer. A nested loop parked at its reload point is replaced by a fresh `SPLOOP`.
    pub(crate) fn start(&mut self, start: LoopStart) {
        if self
            .state
            .as_ref()
            .is_some_and(|state| !state.awaiting_reload)
        {
            return;
        }
        self.state = Some(Active {
            ii: start.ii,
            start_cycle: start.start_cycle,
            loading: true,
            body: Vec::with_capacity(14),
            delayed: start.delayed,
            while_test: start.while_test,
            reload_test: start.reload_test,
            nested: start.nested,
            reload_pending: false,
            reload_overlap: 0,
            replays_first_iteration: false,
            awaiting_reload: false,
            counter: None,
            started: 0,
            total: None,
            start_address: start.start_address,
            draining_for_interrupt: false,
        });
    }

    /// `SPKERNEL` marks the end of the body: loading ends and program fetch stops.
    pub(crate) fn end_loading(&mut self) {
        if let Some(state) = &mut self.state {
            state.loading = false;
        }
    }

    /// Store one execute packet of the body, which only happens while loading.
    pub(crate) fn buffer(&mut self, offset: u32, instructions: Vec<F>) {
        if let Some(state) = &mut self.state {
            if state.loading {
                state.body.push(BufferedPacket {
                    offset,
                    instructions,
                });
            }
        }
    }

    /// Fill `into` with the stages active at `offset`, dropping instructions on units the
    /// cycle's `SPMASK` names.
    pub(crate) fn replay(
        &self,
        offset: u32,
        into: &mut ReplayPackets<F>,
        mut inhibited: impl FnMut(&F) -> bool,
    ) {
        let Some(state) = &self.state else {
            return;
        };
        for packet in &state.body {
            let Some(elapsed) = offset.checked_sub(packet.offset) else {
                continue;
            };
            if elapsed % state.ii != 0 || (elapsed == 0 && !state.replays_first_iteration) {
                continue;
            }
            let iteration = elapsed / state.ii;
            if state.total.is_some_and(|total| iteration >= total) {
                continue;
            }
            into.instructions.extend(
                packet
                    .instructions
                    .iter()
                    .copied()
                    .filter(|fetched| !inhibited(fetched)),
            );
            into.ends.push(into.instructions.len());
        }
    }

    /// Stage-boundary bookkeeping, evaluated before the cycle's packets execute so the iteration
    /// that would run past the end of the loop is never started.
    ///
    /// `while_value` is the do-while register's value, which the caller reads because the
    /// register file is the CPU's; `ilc` is the counter's current value for the same reason.
    pub(crate) fn begin_stage(
        &mut self,
        offset: u32,
        ilc: u32,
        while_value: Option<u32>,
    ) -> StageOutcome {
        let Some(state) = &mut self.state else {
            return StageOutcome::default();
        };
        if offset % state.ii != 0 {
            return StageOutcome::default();
        }
        if let Some((_, _, zero)) = state.while_test {
            if (while_value.unwrap_or(0) == 0) == zero {
                state.started += 1;
            } else {
                // The do-while condition failed: no epilog, the buffer stops immediately.
                self.state = None;
            }
            return StageOutcome::default();
        }
        // Once the loop is draining - either because ILC ran out or because an interrupt is
        // pending - no further iteration starts and ILC keeps the count that is left.
        if state.total.is_some() {
            return StageOutcome::default();
        }
        if state.counter.is_none() && (!state.delayed || offset >= 4) {
            state.counter = Some(ilc);
        }
        match state.counter {
            Some(0) => {
                state.begin_drain();
                StageOutcome::default()
            }
            Some(remaining) => {
                state.counter = Some(remaining - 1);
                state.started += 1;
                StageOutcome {
                    set_ilc: Some(remaining - 1),
                }
            }
            None => {
                state.started += 1;
                StageOutcome::default()
            }
        }
    }

    /// `SPMASKR`/`SPKERNELR` marks the reload point of a nested loop: when the current
    /// invocation finishes draining, ILC is reloaded from RILC and the buffer replays while
    /// program memory carries on with the outer loop (SPRU732J 7.6).
    ///
    /// `armed` is the reload selector's test, which the caller evaluates. The compiler pairs
    /// that register with the outer loop's own branch - `SPMASKR || [B0] BNOP outer` is one
    /// execute packet - so the buffer replays exactly when the outer loop takes another pass,
    /// and the last pass lets it go idle instead of running one inner loop too many.
    pub(crate) fn arm_reload(&mut self, cycle: u64, rilc: u32, armed: bool) -> DrainOutcome {
        let Some(state) = &mut self.state else {
            return DrainOutcome::default();
        };
        if !armed {
            // No further pass: an invocation that has already drained has nothing left to do,
            // and one still running ends the way an unnested loop does.
            if state.awaiting_reload {
                self.state = None;
            } else {
                state.nested = false;
            }
            return DrainOutcome::default();
        }
        if state.awaiting_reload {
            state.awaiting_reload = false;
            // Program fetch is disabled again the moment the buffer takes over, but the packets
            // already in flight still issue: the reload point sits in the same execute packet as
            // the outer loop's branch, so the five delay slots of that branch - the SPMASKed
            // complement of the buffer's first stages, and where the outer body resets its
            // accumulators and stores its results - run alongside the replay before the branch
            // redirects the frozen fetch stream.
            state.restart(cycle, 6);
            DrainOutcome {
                set_ilc: Some(rilc),
                restart: None,
            }
        } else {
            state.reload_pending = true;
            DrainOutcome::default()
        }
    }

    /// End-of-cycle bookkeeping: if the loop has finished draining, either reload it, park it
    /// waiting for its reload point, or let the buffer go idle.
    pub(crate) fn complete_stage(&mut self, offset: u32, cycle: u64, rilc: u32) -> DrainOutcome {
        let Some(state) = &mut self.state else {
            return DrainOutcome::default();
        };
        if state.loading || !state.final_offset().is_some_and(|final_| offset >= final_) {
            return DrainOutcome::default();
        }
        if state.reload_pending {
            state.reload_pending = false;
            state.restart(cycle, state.ii + 1);
            DrainOutcome {
                set_ilc: Some(rilc),
                restart: None,
            }
        } else if state.nested {
            state.awaiting_reload = true;
            state.reload_overlap = 0;
            DrainOutcome::default()
        } else {
            let restart = state.draining_for_interrupt.then_some(state.start_address);
            self.state = None;
            DrainOutcome {
                set_ilc: None,
                restart,
            }
        }
    }

    /// The conditions in SPRUFE8B 7.13.1 for a loop to start draining because an interrupt is
    /// pending: on a stage boundary, termination condition false, not loading, not reloading or
    /// waiting to reload, and at least four cycles into an `SPLOOPD` or `SPLOOPW`.
    pub(crate) fn interruptible(&self, cycle: u64, ilc: u32) -> bool {
        let Some(state) = &self.state else {
            return false;
        };
        if state.draining_for_interrupt {
            return false;
        }
        // A loop cannot begin draining on the cycle it starts: no stage boundary has been reached
        // yet, so an interrupt pending exactly then waits (SPRUFE8B 7.13.1). Without the guard
        // the offset arithmetic underflows.
        let Some(offset) = cycle.checked_sub(state.start_cycle + 1) else {
            return false;
        };
        let offset = offset as u32;
        let boundary = offset % state.ii == 0;
        let settled = self.replaying_alone() && !state.reload_pending && state.total.is_none();
        let past_start = !state.delayed || offset >= 3;
        // SPRUFE8B 7.13.1: for SPLOOP and SPLOOPD the remaining ILC must cover the loading
        // stages, or piping the loop back up would early-exit. The number of loading stages is
        // ceil(dynlen / ii).
        let enough_iterations =
            state.while_test.is_some() || ilc >= state.dynlen().div_ceil(state.ii);
        boundary && settled && past_start && enough_iterations
    }

    /// Stop starting iterations so the loop drains, and remember that the drain is for an
    /// interrupt. Program fetch is already frozen while the buffer runs.
    pub(crate) fn begin_interrupt_drain(&mut self) {
        if let Some(state) = &mut self.state {
            state.begin_drain();
            state.draining_for_interrupt = true;
        }
    }

    /// An exception leaves the buffer idle at once, without running an epilog, which is why an
    /// SPLOOP interrupted by an exception can never be resumed (SPRU732J 7.13.6). Answers
    /// whether a loop was destroyed, which the handler learns from `NTSR.SPLX`.
    pub(crate) fn abandon(&mut self) -> bool {
        self.state.take().is_some()
    }

    /// A taken branch makes the buffer idle after its last delay slot, so a loop is never left
    /// in front of an `SPLOOP` that must then never run. A nested loop is exempt: the branch that
    /// closes its outer loop body fires while the buffer holds the inner loop and only redirects
    /// the frozen fetch stream.
    pub(crate) fn end_on_branch(&mut self) {
        if !self.nested() {
            self.state = None;
        }
    }

    /// Count down the cycles a reloaded invocation still shares with the program-memory side.
    pub(crate) fn tick_reload_overlap(&mut self) {
        if let Some(state) = &mut self.state {
            state.reload_overlap = state.reload_overlap.saturating_sub(1);
        }
    }

    /// The interval, whether the buffer is still filling, and how many iterations have started
    /// out of the total once that is known. `dump` carries the same facts plus the body, so this
    /// is for a caller that wants the counters without rendering fourteen packets.
    #[cfg(test)]
    pub(crate) fn summary(&self) -> Option<(u32, bool, u32, Option<u32>)> {
        let state = self.state.as_ref()?;
        Some((state.ii, state.loading, state.started, state.total))
    }

    /// Full buffer contents with pipeline offsets, rendered by the caller because the
    /// instruction view belongs to the instruction, not to the buffer.
    pub(crate) fn dump(&self, mut text: impl FnMut(&F) -> String) -> Option<SploopDump> {
        let state = self.state.as_ref()?;
        Some(SploopDump {
            ii: state.ii,
            filling: state.loading,
            started: state.started,
            total: state.total,
            entries: state
                .body
                .iter()
                .map(|packet| SploopEntry {
                    offset: packet.offset,
                    instructions: packet.instructions.iter().map(&mut text).collect(),
                })
                .collect(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A stand-in for the CPU's fetch slot: the buffer never looks inside what it stores, so a
    /// test can hand it anything Copy and identify instructions by value.
    type Slot = u32;

    fn started(ii: u32, cycle: u64) -> LoopBuffer<Slot> {
        let mut buffer = LoopBuffer::default();
        buffer.start(LoopStart {
            ii,
            start_cycle: cycle,
            start_address: 0x100,
            delayed: false,
            while_test: None,
            reload_test: None,
            nested: false,
        });
        buffer
    }

    /// A body of `stages` packets, one per cycle offset, each holding a single instruction
    /// numbered by its offset.
    fn load_body(buffer: &mut LoopBuffer<Slot>, stages: u32) {
        for offset in 0..stages {
            buffer.buffer(offset, vec![offset]);
        }
        buffer.end_loading();
    }

    fn replayed(buffer: &LoopBuffer<Slot>, offset: u32) -> Vec<Vec<Slot>> {
        let mut packets = ReplayPackets::default();
        buffer.replay(offset, &mut packets, |_| false);
        packets.packets().map(<[Slot]>::to_vec).collect()
    }

    #[test]
    fn a_fresh_buffer_is_idle() {
        let buffer: LoopBuffer<Slot> = LoopBuffer::default();
        assert!(!buffer.active());
        assert!(!buffer.replaying_alone());
        assert_eq!(buffer.offset(10), None);
        assert_eq!(buffer.summary(), None);
    }

    /// The buffer is only issuing on its own once loading has ended. While it fills, the
    /// program-memory side is still supplying the same instructions.
    #[test]
    fn the_buffer_issues_alone_only_after_loading_ends() {
        let mut buffer = started(2, 0);
        assert!(buffer.active());
        assert!(buffer.loading());
        assert!(!buffer.replaying_alone());
        load_body(&mut buffer, 4);
        assert!(!buffer.loading());
        assert!(buffer.replaying_alone());
    }

    /// Only the loading pass fills the buffer: a packet offered after `SPKERNEL` is not stored,
    /// which is what stops a reloaded invocation from growing its own body.
    #[test]
    fn a_packet_offered_after_loading_ends_is_not_stored() {
        let mut buffer = started(2, 0);
        load_body(&mut buffer, 2);
        buffer.buffer(9, vec![99]);
        let dump = buffer.dump(|slot| slot.to_string()).unwrap();
        assert_eq!(dump.entries.len(), 2);
    }

    /// The pipeline emerges from the offsets: at offset n the buffer replays every packet whose
    /// own offset is n - ii, n - 2*ii, and so on. Nothing writes down a prolog or a kernel; they
    /// are which stages happen to be old enough to have come round again.
    ///
    /// A stage's *first* pass comes from program memory, not the buffer - that is what the
    /// loading pass is - so the packet at the current offset is never replayed. Only a reloaded
    /// invocation, which has no loading pass, replays its first iteration too.
    #[test]
    fn the_stages_active_at_an_offset_are_the_pipeline() {
        let mut buffer = started(2, 0);
        load_body(&mut buffer, 6);
        // Offset 0: nothing is old enough yet.
        assert_eq!(replayed(&buffer, 0), Vec::<Vec<Slot>>::new());
        // Offset 2: stage 0 has come round once. Stage 2 itself is the program-memory pass.
        assert_eq!(replayed(&buffer, 2), vec![vec![0]]);
        // Offset 4: stages 0 and 2 overlap - the pipeline filling up.
        assert_eq!(replayed(&buffer, 4), vec![vec![0], vec![2]]);
        // Offset 8, past the body: every stage is replaying, which is the kernel.
        assert_eq!(
            replayed(&buffer, 8),
            vec![vec![0], vec![2], vec![4]],
            "the even stages, each an iteration apart"
        );
    }

    /// An `SPMASK` naming a unit inhibits the buffered instruction on it, which the caller
    /// decides because units belong to the instruction view.
    #[test]
    fn a_masked_unit_is_dropped_from_the_replay() {
        let mut buffer = started(2, 0);
        load_body(&mut buffer, 6);
        let mut packets = ReplayPackets::default();
        buffer.replay(8, &mut packets, |slot| *slot == 2);
        let stages: Vec<Vec<Slot>> = packets.packets().map(<[Slot]>::to_vec).collect();
        // The stage is still a packet, now an empty one: the mask inhibits the instruction, it
        // does not remove the stage from the pipeline.
        assert_eq!(stages, vec![vec![0], vec![], vec![4]]);
    }

    /// ILC counts iterations down at every stage boundary, and the boundary that finds it zero
    /// starts no further iteration - it fixes the total instead, which is what makes the epilog
    /// drain exactly the iterations that were started.
    #[test]
    fn ilc_counts_iterations_down_and_the_last_boundary_starts_none() {
        let mut buffer = started(2, 0);
        load_body(&mut buffer, 4);
        // Offsets that are not stage boundaries do nothing.
        assert_eq!(buffer.begin_stage(1, 3, None).set_ilc, None);
        assert_eq!(buffer.begin_stage(0, 3, None).set_ilc, Some(2));
        assert_eq!(buffer.begin_stage(2, 2, None).set_ilc, Some(1));
        assert_eq!(buffer.begin_stage(4, 1, None).set_ilc, Some(0));
        assert_eq!(buffer.summary().unwrap().2, 3, "three iterations started");
        // ILC is exhausted: no fourth iteration, and the total is fixed at three.
        assert_eq!(buffer.begin_stage(6, 0, None).set_ilc, None);
        assert_eq!(buffer.summary().unwrap().3, Some(3));
    }

    /// A do-while loop tests its register at every stage boundary and exits abruptly when the
    /// test fails: no epilog, iterations already started simply do not finish.
    #[test]
    fn a_do_while_loop_exits_at_the_boundary_without_an_epilog() {
        let mut buffer = LoopBuffer::<Slot>::default();
        buffer.start(LoopStart {
            ii: 2,
            start_cycle: 0,
            start_address: 0x100,
            delayed: false,
            while_test: Some((Bank::B, 0, false)),
            reload_test: None,
            nested: false,
        });
        load_body(&mut buffer, 4);
        // The test passes while the register is non-zero, and ILC is not consulted at all.
        buffer.begin_stage(0, 0, Some(1));
        assert!(buffer.active());
        assert_eq!(buffer.summary().unwrap().2, 1);
        buffer.begin_stage(2, 0, Some(1));
        assert_eq!(buffer.summary().unwrap().2, 2);
        // It fails: the buffer goes idle at once rather than draining.
        buffer.begin_stage(4, 0, Some(0));
        assert!(!buffer.active());
    }

    /// The interrupt conditions of SPRUFE8B 7.13.1, one at a time.
    #[test]
    fn a_loop_is_interruptible_only_on_a_settled_stage_boundary() {
        let mut buffer = started(2, 0);
        // While loading, no.
        assert!(!buffer.interruptible(1, 8));
        load_body(&mut buffer, 4);
        // On a boundary with ILC covering the loading stages, yes. dynlen is 4 and ii is 2, so
        // two loading stages.
        assert!(buffer.interruptible(1, 8));
        // Off a boundary, no.
        assert!(!buffer.interruptible(2, 8));
        // On a boundary but with too few iterations left to pipe back up, no.
        assert!(!buffer.interruptible(1, 1));
        // Already draining for an interrupt, no: the drain is not restarted.
        buffer.begin_interrupt_drain();
        assert!(!buffer.interruptible(1, 8));
    }

    /// A loop cannot begin draining on the cycle it starts: no stage boundary has been reached,
    /// and the offset arithmetic would underflow.
    #[test]
    fn a_loop_is_not_interruptible_on_its_own_start_cycle() {
        let mut buffer = started(2, 10);
        load_body(&mut buffer, 4);
        assert!(!buffer.interruptible(10, 8));
        assert!(buffer.interruptible(11, 8));
    }

    /// A delayed loop start - `SPLOOPD` or `SPLOOPW`, where ILC may be written in parallel - is
    /// not interruptible for its first three cycles, because ILC is not usable yet.
    #[test]
    fn a_delayed_start_is_not_interruptible_for_three_cycles() {
        let mut buffer = LoopBuffer::<Slot>::default();
        buffer.start(LoopStart {
            ii: 1,
            start_cycle: 0,
            start_address: 0x100,
            delayed: true,
            while_test: None,
            reload_test: None,
            nested: false,
        });
        load_body(&mut buffer, 2);
        for cycle in 1..4 {
            assert!(!buffer.interruptible(cycle, 8), "cycle {cycle}");
        }
        assert!(buffer.interruptible(4, 8));
    }

    /// Draining for an interrupt finishes at the offset the last started iteration completes,
    /// and hands back the loop's own start address so execution resumes there and the loop pipes
    /// back up (SPRUFE8B 7.13.4).
    #[test]
    fn an_interrupt_drain_resumes_at_the_loop_start() {
        let mut buffer = started(2, 0);
        load_body(&mut buffer, 4);
        buffer.begin_stage(0, 4, None);
        buffer.begin_interrupt_drain();
        // One iteration started, so the last stage of it finishes at the body's last offset.
        assert_eq!(buffer.complete_stage(2, 9, 0).restart, None);
        let outcome = buffer.complete_stage(3, 9, 0);
        assert_eq!(outcome.restart, Some(0x100));
        assert!(!buffer.active());
    }

    /// An ordinary loop that finishes draining just goes idle, with nothing to resume.
    #[test]
    fn a_loop_that_drains_normally_goes_idle() {
        let mut buffer = started(2, 0);
        load_body(&mut buffer, 4);
        buffer.begin_stage(0, 1, None);
        buffer.begin_stage(2, 0, None);
        let outcome = buffer.complete_stage(3, 9, 0);
        assert_eq!(outcome.restart, None);
        assert!(!buffer.active());
    }

    /// A nested loop parks at its reload point instead of going idle, keeps its body, and
    /// reloads ILC from RILC when the outer loop says there is another pass.
    #[test]
    fn a_nested_loop_parks_and_reloads_from_rilc() {
        let mut buffer = LoopBuffer::<Slot>::default();
        buffer.start(LoopStart {
            ii: 2,
            start_cycle: 0,
            start_address: 0x100,
            delayed: false,
            while_test: None,
            reload_test: Some((Bank::B, 0, false)),
            nested: true,
        });
        load_body(&mut buffer, 4);
        buffer.begin_stage(0, 1, None);
        buffer.begin_stage(2, 0, None);
        // Draining ends, but a nested loop waits rather than going idle.
        assert_eq!(buffer.complete_stage(3, 9, 7).set_ilc, None);
        assert!(buffer.active());
        assert!(!buffer.replaying_alone(), "parked, not issuing");

        // The outer loop has another pass: ILC is reloaded from RILC and the buffer restarts.
        let outcome = buffer.arm_reload(20, 7, true);
        assert_eq!(outcome.set_ilc, Some(7));
        assert_eq!(buffer.offset(21), Some(0));
        assert_eq!(buffer.summary().unwrap().2, 0, "counters restarted");
        // A reloaded invocation has no loading pass, so it replays its first iteration too.
        assert_eq!(replayed(&buffer, 0), vec![vec![0]]);
    }

    /// The last pass lets a parked nested loop go idle rather than running one inner loop too
    /// many.
    #[test]
    fn a_parked_nested_loop_whose_selector_is_false_goes_idle() {
        let mut buffer = LoopBuffer::<Slot>::default();
        buffer.start(LoopStart {
            ii: 2,
            start_cycle: 0,
            start_address: 0x100,
            delayed: false,
            while_test: None,
            reload_test: Some((Bank::B, 0, false)),
            nested: true,
        });
        load_body(&mut buffer, 4);
        buffer.begin_stage(0, 1, None);
        buffer.begin_stage(2, 0, None);
        buffer.complete_stage(3, 9, 7);
        assert!(buffer.active());
        buffer.arm_reload(20, 7, false);
        assert!(!buffer.active());
    }

    /// An exception leaves the buffer idle at once, without an epilog, which is why an SPLOOP
    /// interrupted by an exception can never be resumed.
    #[test]
    fn an_exception_abandons_the_loop_without_draining() {
        let mut buffer = started(2, 0);
        load_body(&mut buffer, 4);
        assert!(buffer.abandon(), "reports that a loop was destroyed");
        assert!(!buffer.active());
        assert!(!buffer.abandon(), "and nothing to destroy twice");
    }

    /// A taken branch evicts an ordinary loop after its last delay slot, but not a nested one:
    /// the branch closing the outer body fires while the buffer holds the inner loop.
    #[test]
    fn a_taken_branch_evicts_an_ordinary_loop_but_not_a_nested_one() {
        let mut ordinary = started(2, 0);
        load_body(&mut ordinary, 4);
        ordinary.end_on_branch();
        assert!(!ordinary.active());

        let mut nested = LoopBuffer::<Slot>::default();
        nested.start(LoopStart {
            ii: 2,
            start_cycle: 0,
            start_address: 0x100,
            delayed: false,
            while_test: None,
            reload_test: Some((Bank::B, 0, false)),
            nested: true,
        });
        load_body(&mut nested, 4);
        nested.end_on_branch();
        assert!(nested.active());
    }

    /// A fresh loop start replaces a loop parked at its reload point, but not one still running.
    #[test]
    fn a_fresh_start_replaces_only_a_parked_loop() {
        let mut running = started(2, 0);
        load_body(&mut running, 4);
        running.start(LoopStart {
            ii: 7,
            start_cycle: 50,
            start_address: 0x200,
            delayed: false,
            while_test: None,
            reload_test: None,
            nested: false,
        });
        assert_eq!(
            running.interval(),
            Some(2),
            "the running loop kept the buffer"
        );

        let mut parked = LoopBuffer::<Slot>::default();
        parked.start(LoopStart {
            ii: 2,
            start_cycle: 0,
            start_address: 0x100,
            delayed: false,
            while_test: None,
            reload_test: Some((Bank::B, 0, false)),
            nested: true,
        });
        load_body(&mut parked, 4);
        parked.begin_stage(0, 1, None);
        parked.begin_stage(2, 0, None);
        parked.complete_stage(3, 9, 7);
        parked.start(LoopStart {
            ii: 7,
            start_cycle: 50,
            start_address: 0x200,
            delayed: false,
            while_test: None,
            reload_test: None,
            nested: false,
        });
        assert_eq!(parked.interval(), Some(7), "the parked loop was replaced");
    }
}
