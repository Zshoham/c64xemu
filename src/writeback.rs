//! The write-back queue: every effect an instruction makes after the cycle it issues.
//!
//! A C64x+ instruction does not write its result when it executes. A register destination lands
//! at the end of the instruction's last delay slot, a store reaches memory at the
//! end of its execute cycle, an `MVC` reaches the control register file at the end of its cycle -
//! four cycles later for ILC and RILC, the SPLOOP counters - and a functional unit's saturation
//! flag appears one cycle after its result (SPRUFE8B Table 2-22).
//! This module holds all four kinds of effect between the cycle they are scheduled and the cycle
//! they are due, and hands the due ones back in the order the CPU commits them.
//!
//! The CPU keeps every architectural action: writing the register, checking a store against
//! memory protection, narrowing a control-register write, setting SSR, and tracing each of them.
//! What lives here is the bookkeeping - where an effect waits, which one wins when two collide,
//! and the order in which the due ones come back. That order is observable (trace events, two
//! stores to one address, two control writes to one register), so each queue documents why its
//! order is the one the CPU has always committed in.
//!
//! Register writes wait in a ring of `SLOTS` slots indexed by due cycle, so scheduling and
//! draining are an index rather than a search. `occupied` summarises which slots, and which of
//! the other three queues, hold anything, so the cycle with nothing due - most stall cycles, and
//! every cycle whose packet scheduled nothing that lands now - costs one mask test.

use crate::view::Bank;

/// The ring length. Every delay must be shorter than this, so a write scheduled this cycle
/// never shares a slot with the cycle currently being committed. The longest C64x+ delay is
/// four slots (loads); sixteen keeps the slot index a mask.
pub(crate) const SLOTS: usize = 16;
const SLOT_MASK: u64 = SLOTS as u64 - 1;

/// `occupied` bits past the register slots: one per queue that is not indexed by due cycle.
const STORES: u32 = 1 << SLOTS;
const CONTROLS: u32 = 1 << (SLOTS + 1);
const SATURATIONS: u32 = 1 << (SLOTS + 2);
const UNSLOTTED: u32 = STORES | CONTROLS | SATURATIONS;

/// A register destination waiting for the end of its last delay slot.
#[derive(Clone, Copy)]
pub(crate) struct RegisterWrite {
    pub(crate) due: u64,
    pub(crate) value: u32,
    pub(crate) bank: Bank,
    pub(crate) register: u8,
    /// Load destinations never cause a cross path stall (SPRUFE8B §3.8.4).
    pub(crate) from_load: bool,
    /// How far past its issue cycle the write lands. Two writes with the same due cycle issued
    /// on the same cycle exactly when their delays match, and the issue cycle is what decides a
    /// collision between them.
    delay: u8,
}

impl RegisterWrite {
    pub(crate) fn new(
        issued: u64,
        delay: u64,
        bank: Bank,
        register: usize,
        value: u32,
        from_load: bool,
    ) -> Self {
        debug_assert!(delay < SLOTS as u64);
        debug_assert!(register < 32);
        Self {
            due: issued + delay,
            value,
            bank,
            register: register as u8,
            from_load,
            delay: delay as u8,
        }
    }

    /// This write's bit in a slot's register summary: side A in the low word, B in the high.
    fn register_bit(&self) -> u64 {
        let side = match self.bank {
            Bank::A => 0,
            Bank::B => 32,
        };
        1 << (side + u32::from(self.register))
    }
}

/// A store waiting for the end of the cycle it executed on.
#[derive(Clone, Copy)]
pub(crate) struct StoreWrite {
    pub(crate) due: u64,
    pub(crate) address: u32,
    pub(crate) size: u8,
    pub(crate) value: u32,
}

/// An `MVC` to a control register waiting for its due cycle.
#[derive(Clone, Copy)]
pub(crate) struct ControlWrite {
    pub(crate) due: u64,
    pub(crate) register: u8,
    pub(crate) value: u32,
    /// The mode the `MVC` issued in: a User-mode write only reaches the bits User mode owns.
    pub(crate) user: bool,
}

/// Two instructions issued on the same cycle write one register on the same cycle: the
/// "easily detectable" conflict of SPRUFE8B §3.8.8. The later write was not queued.
pub(crate) struct SameCycleDuplicate;

pub(crate) struct WritebackQueue {
    registers: [Vec<RegisterWrite>; SLOTS],
    /// Per slot, every register some write in that slot targets. It may name a register whose
    /// write was replaced, never miss one that is present, so a clear bit proves there is no
    /// collision without searching the slot. Cleared when the slot drains.
    targets: [u64; SLOTS],
    /// Bit `n` set while register slot `n` holds a write; `STORES`, `CONTROLS` and
    /// `SATURATIONS` while those queues are not empty.
    occupied: u32,
    stores: Vec<StoreWrite>,
    controls: Vec<ControlWrite>,
    /// (due cycle, SSR bit) pairs.
    saturations: Vec<(u64, u8)>,
}

impl Default for WritebackQueue {
    fn default() -> Self {
        Self {
            registers: std::array::from_fn(|_| Vec::with_capacity(2)),
            targets: [0; SLOTS],
            occupied: 0,
            stores: Vec::with_capacity(4),
            controls: Vec::with_capacity(2),
            saturations: Vec::new(),
        }
    }
}

fn slot(cycle: u64) -> usize {
    (cycle & SLOT_MASK) as usize
}

impl WritebackQueue {
    /// Queue a register write. A write already waiting for the same register on the same cycle
    /// is either a same-cycle duplicate, which is refused, or one issued earlier, which the new
    /// write replaces.
    pub(crate) fn schedule_register(
        &mut self,
        write: RegisterWrite,
    ) -> Result<(), SameCycleDuplicate> {
        let slot = slot(write.due);
        let bit = write.register_bit();
        let bucket = &mut self.registers[slot];
        if self.targets[slot] & bit != 0 {
            if let Some(index) = bucket.iter().position(|queued| {
                queued.due == write.due
                    && queued.bank == write.bank
                    && queued.register == write.register
            }) {
                if bucket[index].delay == write.delay {
                    return Err(SameCycleDuplicate);
                }
                // Across packets SPRUFE8B §3.8.8 leaves the result undefined and raises
                // nothing. The later-issued instruction wins, which is the resolution TI's own
                // code depends on: the compiler issues a speculative load whose result is dead
                // and then overwrites the register with a live value exactly four cycles later.
                // `swap_remove` then `push` is the layout the drain order below was always
                // taken from.
                bucket.swap_remove(index);
            }
        } else {
            self.targets[slot] |= bit;
        }
        bucket.push(write);
        self.occupied |= 1 << slot;
        Ok(())
    }

    /// Queue a store. A store lands at the end of the cycle it executes on, so `due` is always
    /// the current cycle and every store is drained by the write-back that ends it.
    pub(crate) fn schedule_store(&mut self, store: StoreWrite) {
        self.stores.push(store);
        self.occupied |= STORES;
    }

    pub(crate) fn schedule_control(&mut self, write: ControlWrite) {
        self.controls.push(write);
        self.occupied |= CONTROLS;
    }

    /// Queue a saturation of the unit whose SSR bit is `bit`, visible from cycle `due`.
    pub(crate) fn schedule_saturation(&mut self, due: u64, bit: u8) {
        self.saturations.push((due, bit));
        self.occupied |= SATURATIONS;
    }

    /// Whether the write-back of `cycle` has nothing to do.
    #[inline(always)]
    pub(crate) fn nothing_due(&self, cycle: u64) -> bool {
        self.occupied & ((1 << slot(cycle)) | UNSLOTTED) == 0
    }

    /// The next register write the write-back of `cycle` lands. The slot is drained from its
    /// end, so writes come back in reverse of the slot's order. Every cycle ends with a
    /// write-back, so nothing in the slot is from an earlier lap of the ring.
    pub(crate) fn pop_register(&mut self, cycle: u64) -> Option<RegisterWrite> {
        let slot = slot(cycle);
        let write = self.registers[slot].pop()?;
        debug_assert!(write.due <= cycle);
        if self.registers[slot].is_empty() {
            self.occupied &= !(1 << slot);
            self.targets[slot] = 0;
        }
        Some(write)
    }

    /// The next store the write-back of `cycle` commits. Every queued store is due (see
    /// `schedule_store`), so this is the front of the queue, swapped with the back as it
    /// leaves: the order is first, last, second to last, ..., second. That is the order the
    /// CPU's original due-scan with `swap_remove` produced whenever everything was due.
    pub(crate) fn pop_store(&mut self, cycle: u64) -> Option<StoreWrite> {
        if self.stores.is_empty() {
            return None;
        }
        let store = self.stores.swap_remove(0);
        debug_assert!(store.due <= cycle);
        if self.stores.is_empty() {
            self.occupied &= !STORES;
        }
        Some(store)
    }

    /// The next control write due exactly on `cycle`, continuing a scan that starts with
    /// `cursor` at zero. Control writes are rare and not every queued one is due, and the
    /// processing order of two due together depends on where the undue ones sit, so this keeps
    /// the scan-and-`swap_remove` the CPU has always used rather than re-deriving its order. A
    /// control write whose due cycle passed without a write-back is never committed.
    pub(crate) fn next_control(&mut self, cycle: u64, cursor: &mut usize) -> Option<ControlWrite> {
        while *cursor < self.controls.len() {
            if self.controls[*cursor].due == cycle {
                let write = self.controls.swap_remove(*cursor);
                if self.controls.is_empty() {
                    self.occupied &= !CONTROLS;
                }
                return Some(write);
            }
            *cursor += 1;
        }
        None
    }

    /// Remove every saturation due by `cycle` and return their SSR bits as a mask. Recording a
    /// saturation only sets its unit's bit and CSR.SAT, so the order they are recorded in
    /// cannot matter and a mask carries all of it.
    pub(crate) fn take_saturations(&mut self, cycle: u64) -> u8 {
        let mut bits = 0;
        self.saturations.retain(|&(due, bit)| {
            let is_due = due <= cycle;
            if is_due {
                bits |= 1 << bit;
            }
            !is_due
        });
        if self.saturations.is_empty() {
            self.occupied &= !SATURATIONS;
        }
        bits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bank_index(bank: Bank) -> u8 {
        match bank {
            Bank::A => 0,
            Bank::B => 1,
        }
    }

    type RegisterKey = (u64, u32, u8, u8, bool);

    fn register_key(write: &RegisterWrite) -> RegisterKey {
        (
            write.due,
            write.value,
            bank_index(write.bank),
            write.register,
            write.from_load,
        )
    }

    fn drain_registers(queue: &mut WritebackQueue, cycle: u64) -> Vec<RegisterKey> {
        std::iter::from_fn(|| queue.pop_register(cycle))
            .map(|write| register_key(&write))
            .collect()
    }

    fn write(issued: u64, delay: u64, register: usize, value: u32) -> RegisterWrite {
        RegisterWrite::new(issued, delay, Bank::A, register, value, false)
    }

    #[test]
    fn a_register_write_lands_only_on_its_due_cycle() {
        let mut queue = WritebackQueue::default();
        assert!(queue.schedule_register(write(10, 4, 1, 0xaa)).is_ok());
        assert!(queue.schedule_register(write(10, 0, 2, 0xbb)).is_ok());
        assert_eq!(
            drain_registers(&mut queue, 10),
            vec![(10, 0xbb, 0, 2, false)]
        );
        for cycle in 11..14 {
            assert!(queue.nothing_due(cycle));
            assert!(drain_registers(&mut queue, cycle).is_empty());
        }
        assert!(!queue.nothing_due(14));
        assert_eq!(
            drain_registers(&mut queue, 14),
            vec![(14, 0xaa, 0, 1, false)]
        );
        assert!(queue.nothing_due(30));
    }

    #[test]
    fn the_longest_delay_does_not_alias_an_earlier_write_back() {
        let mut queue = WritebackQueue::default();
        let delay = SLOTS as u64 - 1;
        assert!(queue.schedule_register(write(7, delay, 3, 1)).is_ok());
        for cycle in 7..7 + delay {
            assert!(queue.nothing_due(cycle), "cycle {cycle}");
        }
        assert_eq!(
            drain_registers(&mut queue, 7 + delay),
            vec![(7 + delay, 1, 0, 3, false)]
        );
    }

    #[test]
    fn two_writes_to_one_register_issued_together_are_refused() {
        let mut queue = WritebackQueue::default();
        assert!(queue.schedule_register(write(5, 1, 4, 1)).is_ok());
        assert!(queue.schedule_register(write(5, 1, 4, 2)).is_err());
        // The same register on the other side, or landing on another cycle, is no collision.
        assert!(queue
            .schedule_register(RegisterWrite::new(5, 1, Bank::B, 4, 3, false))
            .is_ok());
        assert!(queue.schedule_register(write(5, 2, 4, 4)).is_ok());
        assert_eq!(
            drain_registers(&mut queue, 6),
            vec![(6, 3, 1, 4, false), (6, 1, 0, 4, false)]
        );
    }

    #[test]
    fn a_later_issued_write_to_the_same_register_and_cycle_wins() {
        let mut queue = WritebackQueue::default();
        assert!(queue
            .schedule_register(RegisterWrite::new(0, 4, Bank::A, 6, 0xdead, true))
            .is_ok());
        assert!(queue.schedule_register(write(4, 0, 6, 0x1234)).is_ok());
        assert_eq!(
            drain_registers(&mut queue, 4),
            vec![(4, 0x1234, 0, 6, false)]
        );
    }

    /// A seeded xorshift, so the property test below is deterministic.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        fn below(&mut self, bound: u64) -> u64 {
            self.next() % bound
        }
    }

    /// The four queues exactly as the CPU kept them before this module: a vector per register
    /// bucket searched linearly, and three vectors scanned with `swap_remove`.
    #[derive(Default)]
    struct Oracle {
        writes: [Vec<(RegisterKey, u64)>; SLOTS],
        stores: Vec<StoreWrite>,
        controls: Vec<ControlWrite>,
        saturations: Vec<(u64, u8)>,
    }

    impl Oracle {
        fn schedule(&mut self, cycle: u64, write: RegisterKey) -> bool {
            let bucket = &mut self.writes[slot(write.0)];
            if let Some(index) = bucket.iter().position(|(queued, _)| {
                queued.0 == write.0 && queued.2 == write.2 && queued.3 == write.3
            }) {
                if bucket[index].1 == cycle {
                    return false;
                }
                bucket.swap_remove(index);
            }
            bucket.push((write, cycle));
            true
        }

        fn commit(&mut self, cycle: u64) -> (Vec<RegisterKey>, Vec<u32>, Vec<u32>, u8) {
            let mut registers = Vec::new();
            while let Some((write, _)) = self.writes[slot(cycle)].pop() {
                registers.push(write);
            }
            let mut stores = Vec::new();
            let mut index = 0;
            while index < self.stores.len() {
                if self.stores[index].due <= cycle {
                    stores.push(self.stores.swap_remove(index).value);
                } else {
                    index += 1;
                }
            }
            let mut controls = Vec::new();
            let mut index = 0;
            while index < self.controls.len() {
                if self.controls[index].due == cycle {
                    controls.push(self.controls.swap_remove(index).value);
                } else {
                    index += 1;
                }
            }
            let mut saturations = 0;
            let mut index = 0;
            while index < self.saturations.len() {
                if self.saturations[index].0 <= cycle {
                    saturations |= 1 << self.saturations.swap_remove(index).1;
                } else {
                    index += 1;
                }
            }
            (registers, stores, controls, saturations)
        }
    }

    fn commit(
        queue: &mut WritebackQueue,
        cycle: u64,
    ) -> (Vec<RegisterKey>, Vec<u32>, Vec<u32>, u8) {
        if queue.nothing_due(cycle) {
            return (Vec::new(), Vec::new(), Vec::new(), 0);
        }
        let registers = drain_registers(queue, cycle);
        let stores = std::iter::from_fn(|| queue.pop_store(cycle))
            .map(|store| store.value)
            .collect();
        let mut cursor = 0;
        let controls = std::iter::from_fn(|| queue.next_control(cycle, &mut cursor))
            .map(|write| write.value)
            .collect();
        (registers, stores, controls, queue.take_saturations(cycle))
    }

    /// Random schedules of every kind the CPU makes - register writes with every delay, few
    /// enough registers that same-cycle duplicates and later-issued replacements are common,
    /// stores due on their issue cycle, `MVC`s with and without ILC's four delay slots,
    /// saturations due a cycle later - drained by a write-back every cycle. Everything drained,
    /// and every duplicate refused, must match the old queues item for item and in order.
    #[test]
    fn the_queue_commits_in_exactly_the_order_of_the_original_scans() {
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        let mut queue = WritebackQueue::default();
        let mut oracle = Oracle::default();
        let mut value = 0u32;
        let mut committed = 0usize;
        for cycle in 0..40_000u64 {
            for _ in 0..rng.below(7) {
                value += 1;
                let delay = match rng.below(4) {
                    0 => rng.below(SLOTS as u64),
                    1 => 4,
                    _ => rng.below(2),
                };
                let bank = if rng.below(2) == 0 { Bank::A } else { Bank::B };
                let register = rng.below(3) as usize;
                let from_load = rng.below(2) == 0;
                let write = RegisterWrite::new(cycle, delay, bank, register, value, from_load);
                let expected = oracle.schedule(cycle, register_key(&write));
                assert_eq!(queue.schedule_register(write).is_ok(), expected);
            }
            for _ in 0..rng.below(4) {
                value += 1;
                let store = StoreWrite {
                    due: cycle,
                    address: 0,
                    size: 4,
                    value,
                };
                queue.schedule_store(store);
                oracle.stores.push(store);
            }
            if rng.below(3) == 0 {
                value += 1;
                let write = ControlWrite {
                    due: cycle + 4 * rng.below(2),
                    register: 0,
                    value,
                    user: false,
                };
                queue.schedule_control(write);
                oracle.controls.push(write);
            }
            for _ in 0..rng.below(3) {
                let bit = rng.below(6) as u8;
                queue.schedule_saturation(cycle + 1, bit);
                oracle.saturations.push((cycle + 1, bit));
            }
            let expected = oracle.commit(cycle);
            committed += expected.0.len() + expected.1.len() + expected.2.len();
            assert_eq!(commit(&mut queue, cycle), expected, "cycle {cycle}");
        }
        assert!(committed > 100_000, "{committed}");
    }
}
