//! A passive observer of one core's stack window, for `--stack-window`.
//!
//! The observer watches two things the core commits and nothing else: guest writes to B15, and
//! CPU stores that reach memory. It reads no guest memory, writes none, and changes no cache or
//! pipeline state, so a run with it configured retires the same instructions on the same cycles
//! as one without.
//!
//! The window has two endpoints with deliberately different meanings:
//!
//! - B15 may equal `end`. A descending, empty stack points one past its highest word, so a
//!   linker-defined stack `[start, end)` legitimately loads B15 with `end` before the first push.
//!   B15 is in range when `start <= B15 <= end`.
//! - A stored byte is in the window when `start <= address < end`, the half-open interval of
//!   the bytes the stack owns.
//!
//! **Arming.** Startup code builds the stack pointer in several instructions (`MVK` then `MVKH`),
//! and the half-built values are nowhere near the stack. B15 writes are therefore only counted
//! as stack activity once B15 has first committed a value inside `[start, end]`; that write arms
//! the observer and seeds the minimum. Before it, B15 writes only set `sp_written`, which tells
//! "this core never wrote B15" apart from "it did, but never into this window". Once armed,
//! every committed B15 value counts: `min_sp` follows the actual low-water mark even below
//! `start`, and any value outside `[start, end]` sets `out_of_bounds_sp`.
//! B15 set by the host (the debugger, GDB) is not guest execution and is not observed.
//!
//! **Stores.** A store instruction is judged on its whole access, so a doubleword that memory
//! takes as two words is still one access here. An access that overlaps the window qualifies;
//! for a qualifying access, each byte that actually commits is classified: inside the window it
//! competes for `lowest_write`, outside it sets `out_of_bounds_write`. Bytes the MPU refuses never
//! commit, so a doubleword whose outside half is denied writes nothing out of bounds, while one
//! whose inside half is denied still reports its outside half. An access entirely outside the
//! window is ignored, whatever it overwrites: this is an overlap check, not a guard on every
//! address below the stack. Addresses wrap at 32 bits exactly as memory does.
//!
//! **Clock.** Every cycle reported is the core's own pipeline cycle at commit - the counter
//! instruction tracing stamps events with - which advances on every cycle the pipeline moves
//! (a packet, a multicycle NOP, IDLE, draining into an interrupt) but not on cycles the core is
//! frozen on memory stalls. It is neither the device clock nor the elapsed count `Cpu::cycles`
//! reports, and cores do not share it: a core held in reset does not advance.

/// What the observer has seen of one core's stack window, serialised into the state report.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub struct StackMemoryObservation {
    /// First byte of the window.
    pub start: u32,
    /// One past the window's last byte; also the highest B15 value still in range.
    pub end: u32,
    /// The core committed a guest write to B15 since the observer was configured.
    pub sp_written: bool,
    /// B15 has committed a value inside `[start, end]`; SP tracking runs from that write.
    pub armed: bool,
    /// The lowest B15 value committed since arming, including values below `start`.
    pub min_sp: Option<u32>,
    /// The pipeline cycle `min_sp` was first committed on.
    pub min_sp_cycle: Option<u64>,
    /// The lowest in-window byte a qualifying store committed.
    pub lowest_write: Option<u32>,
    /// The pipeline cycle `lowest_write` was first committed on.
    pub lowest_write_cycle: Option<u64>,
    /// After arming, B15 committed a value outside `[start, end]`.
    pub out_of_bounds_sp: bool,
    /// A store overlapping the window committed a byte outside it.
    pub out_of_bounds_write: bool,
}

impl StackMemoryObservation {
    /// A fresh observer for `[start, end)`; both endpoints must be word aligned and the window
    /// non-empty.
    pub fn new(start: u32, end: u32) -> Result<Self, String> {
        if start >= end || start & 3 != 0 || end & 3 != 0 {
            return Err("stack observation window must be non-empty and word aligned".into());
        }
        Ok(Self {
            start,
            end,
            sp_written: false,
            armed: false,
            min_sp: None,
            min_sp_cycle: None,
            lowest_write: None,
            lowest_write_cycle: None,
            out_of_bounds_sp: false,
            out_of_bounds_write: false,
        })
    }

    fn holds_byte(&self, address: u32) -> bool {
        self.start <= address && address < self.end
    }

    /// A guest write of `value` to B15 committed on `cycle`.
    pub(crate) fn observe_sp(&mut self, value: u32, cycle: u64) {
        self.sp_written = true;
        let in_range = self.start <= value && value <= self.end;
        if !self.armed {
            if in_range {
                self.armed = true;
                self.min_sp = Some(value);
                self.min_sp_cycle = Some(cycle);
            }
            return;
        }
        if !in_range {
            self.out_of_bounds_sp = true;
        }
        // Strictly lower, so a later return to the same depth keeps the first cycle.
        if self.min_sp.map_or(true, |minimum| value < minimum) {
            self.min_sp = Some(value);
            self.min_sp_cycle = Some(cycle);
        }
    }

    /// `size` bytes committed at `address` on `cycle`, as part of an access that starts
    /// `extent_offset` bytes below `address` and spans `extent_size` bytes.
    pub(crate) fn observe_store(
        &mut self,
        address: u32,
        size: u8,
        extent_offset: u8,
        extent_size: u8,
        cycle: u64,
    ) {
        let extent = address.wrapping_sub(u32::from(extent_offset));
        // At most eight bytes, and only with the observer configured: a byte walk keeps the
        // 32-bit wrap exact without interval arithmetic that has to special-case it.
        let overlaps =
            (0..u32::from(extent_size)).any(|byte| self.holds_byte(extent.wrapping_add(byte)));
        if !overlaps {
            return;
        }
        for byte in 0..u32::from(size) {
            let byte = address.wrapping_add(byte);
            if !self.holds_byte(byte) {
                self.out_of_bounds_write = true;
            } else if self.lowest_write.map_or(true, |lowest| byte < lowest) {
                self.lowest_write = Some(byte);
                self.lowest_write_cycle = Some(cycle);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    //! Guest instructions driving the observer through the real pipeline. Every encoding was
    //! produced by TI CGT 8.3.15 `cl6x -mv6400+` and checked with `dis6x`; each constant carries
    //! the disassembly it came from. Programs sit at address 0 in serial execute packets.
    use super::StackMemoryObservation;
    use crate::cpu::Cpu;
    use crate::system::Endian;

    const NOP_3: u32 = 0x0000_4000; // NOP 3
    const NOP: u32 = 0x0000_0000; // NOP
    const MVK_B15_2000: u32 = 0x0790_002a; // MVK.S2 0x2000,B15
    const MVK_B15_1FF0: u32 = 0x078f_f82a; // MVK.S2 0x1ff0,B15
    const MVK_B15_0FFC: u32 = 0x0787_fe2a; // MVK.S2 0x0ffc,B15
    const MVK_B15_MINUS_4: u32 = 0x07ff_fe2a; // MVK.S2 0xfffffffc,B15
    const MVKH_B15_800000: u32 = 0x0780_406a; // MVKH.S2 0x800000,B15
    const ADDK_B15_MINUS_4100: u32 = 0x07f7_fe52; // ADDK.S2 -4100,B15
    const ADDK_B15_16: u32 = 0x0780_0852; // ADDK.S2 16,B15
    const ADDK_B15_MINUS_16: u32 = 0x07ff_f852; // ADDK.S2 -16,B15
    const LDW_A4_B15: u32 = 0x0790_0266; // LDW.D1T2 *+A4[0],B15
    const STW_B0_B15_POSTDEC_2: u32 = 0x003c_54f6; // STW.D2T2 B0,*B15--[2]
    const STW_B0_B15_POSTDEC_1: u32 = 0x003c_34f6; // STW.D2T2 B0,*B15--[1]
    const LDW_B15_PREINC_1_B1: u32 = 0x00bc_32e6; // LDW.D2T2 *++B15[1],B1
    const STW: u32 = 0x0010_0274; // STW.D1T1 A0,*+A4[0]
    const STNW: u32 = 0x0010_0354; // STNW.D1T1 A0,*+A4[0]
    const STNDW: u32 = 0x0090_0374; // STNDW.D1T1 A1:A0,*+A4[0]
    const STDW: u32 = 0x0010_0344; // STDW.D1T1 A1:A0,*+A4[0]
    const STB: u32 = 0x0010_0234; // STB.D1T1 A0,*+A4[0]
    const STH: u32 = 0x0010_0254; // STH.D1T1 A0,*+A4[0]
    const STW_IF_B0: u32 = 0x2010_0274; // [ B0] STW.D1T1 A0,*+A4[0]
    const STW_IF_NOT_B0: u32 = 0x3010_0274; // [!B0] STW.D1T1 A0,*+A4[0]

    /// The bytes the store tests write, in little-endian memory order: A0 = 0x44332211 then
    /// A1 = 0x88776655, so a doubleword lands as 11 22 .. 88.
    const STORED: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

    fn observed(program: &[u32], start: u32, end: u32) -> Cpu {
        let mut cpu = Cpu::new(0);
        cpu.write_words(0, program);
        cpu.configure_stack_observation(start, end).unwrap();
        cpu
    }

    fn report(cpu: &Cpu) -> StackMemoryObservation {
        cpu.stack_observation().unwrap()
    }

    fn sp(report: &StackMemoryObservation) -> (Option<u32>, Option<u64>) {
        (report.min_sp, report.min_sp_cycle)
    }

    #[test]
    fn rejects_empty_and_misaligned_windows() {
        for (start, end) in [
            (0x1000, 0x1000),
            (0x2000, 0x1000),
            (0x1002, 0x2000),
            (0x1000, 0x2001),
        ] {
            assert!(
                StackMemoryObservation::new(start, end).is_err(),
                "{start:#x}:{end:#x}"
            );
            assert!(Cpu::new(0).configure_stack_observation(start, end).is_err());
        }
        assert!(StackMemoryObservation::new(0, 4).is_ok());
    }

    /// B15 = END is a valid empty stack. The write that first puts it there arms the observer
    /// and is timestamped with its own commit cycle, not the configuration-time zero.
    #[test]
    fn first_in_range_sp_arms_the_observer_and_carries_its_cycle() {
        let mut cpu = observed(&[NOP_3, MVK_B15_2000], 0x1000, 0x2000);
        cpu.run(3);
        let before = report(&cpu);
        assert!(!before.sp_written && !before.armed);
        assert_eq!(sp(&before), (None, None));
        cpu.run(1);
        assert_eq!(cpu.register('B', 15), 0x2000);
        let after = report(&cpu);
        assert!(after.sp_written && after.armed && !after.out_of_bounds_sp);
        assert_eq!(sp(&after), (Some(0x2000), Some(3)));
    }

    /// Once armed, the minimum is the real low-water mark, wherever it lands.
    #[test]
    fn armed_minimum_follows_sp_below_the_window() {
        let mut cpu = observed(&[MVK_B15_2000, ADDK_B15_MINUS_4100], 0x1000, 0x2000);
        cpu.run(2);
        assert_eq!(cpu.register('B', 15), 0x0ffc);
        assert_eq!(cpu.fault(), None);
        let report = report(&cpu);
        assert!(report.out_of_bounds_sp);
        assert_eq!(sp(&report), (Some(0x0ffc), Some(1)));
    }

    /// Leaving the window upwards is out of bounds too, but is no new minimum.
    #[test]
    fn sp_above_end_flags_without_moving_the_minimum() {
        let mut cpu = observed(&[MVK_B15_1FF0, MVK_B15_MINUS_4], 0x1000, 0x2000);
        cpu.run(2);
        assert_eq!(cpu.register('B', 15), 0xffff_fffc);
        let report = report(&cpu);
        assert!(report.out_of_bounds_sp);
        assert_eq!(sp(&report), (Some(0x1ff0), Some(0)));
    }

    /// The crt0 shape: MVK leaves B15 = 0x00002000 for one cycle, nowhere near the stack, before
    /// MVKH completes it at END. That half-built value is not a stack excursion.
    #[test]
    fn sp_built_in_two_instructions_arms_without_a_violation() {
        let mut cpu = observed(&[MVK_B15_2000, MVKH_B15_800000], 0x0080_1000, 0x0080_2000);
        cpu.run(1);
        let partial = report(&cpu);
        assert_eq!(cpu.register('B', 15), 0x2000);
        assert!(partial.sp_written && !partial.armed && !partial.out_of_bounds_sp);
        cpu.run(1);
        assert_eq!(cpu.register('B', 15), 0x0080_2000);
        let built = report(&cpu);
        assert!(built.armed && !built.out_of_bounds_sp);
        assert_eq!(sp(&built), (Some(0x0080_2000), Some(1)));
    }

    /// A core that writes B15 but never into the window says so: it is not the same as a core
    /// that never wrote B15, and its false SP flag proves nothing.
    #[test]
    fn sp_that_never_enters_the_window_is_written_but_unarmed() {
        let mut cpu = observed(&[MVK_B15_0FFC, MVK_B15_MINUS_4], 0x1000, 0x2000);
        cpu.run(2);
        let report = report(&cpu);
        assert!(report.sp_written && !report.armed && !report.out_of_bounds_sp);
        assert_eq!(sp(&report), (None, None));
    }

    /// Returning to an earlier depth keeps the cycle that depth was first reached on.
    #[test]
    fn equal_minimum_keeps_its_first_cycle() {
        let program = [
            NOP_3,
            MVK_B15_1FF0,
            ADDK_B15_16,
            ADDK_B15_MINUS_16,
            MVK_B15_1FF0,
        ];
        let mut cpu = observed(&program, 0x1000, 0x2000);
        cpu.run(7);
        assert_eq!(cpu.register('B', 15), 0x1ff0);
        assert_eq!(sp(&report(&cpu)), (Some(0x1ff0), Some(3)));
    }

    /// Debugger and GDB register writes are the host's, not the program's.
    #[test]
    fn host_register_writes_are_not_observed() {
        let mut cpu = observed(&[NOP], 0x1000, 0x2000);
        cpu.set_register("B15", 0x1800).unwrap();
        assert_eq!(cpu.register('B', 15), 0x1800);
        let report = report(&cpu);
        assert!(!report.sp_written && !report.armed);
    }

    /// A load into B15 changes nothing until its data lands, four delay slots later.
    #[test]
    fn loaded_sp_is_observed_at_writeback() {
        let mut cpu = observed(&[LDW_A4_B15, NOP, NOP, NOP, NOP], 0x1000, 0x2000);
        cpu.set_register("A4", 0x3000).unwrap();
        cpu.write_words(0x3000, &[0x1ff0]);
        cpu.run(4);
        assert_eq!(cpu.register('B', 15), 0);
        assert!(!report(&cpu).sp_written);
        cpu.run(1);
        assert_eq!(cpu.register('B', 15), 0x1ff0);
        assert_eq!(sp(&report(&cpu)), (Some(0x1ff0), Some(4)));
    }

    /// Pushes through `*B15--` move B15 in the cycle they issue and store at the old value; a
    /// `*++B15` pop moves it back. The first push stores at END, wholly outside the window,
    /// which the overlap check deliberately does not report.
    #[test]
    fn addressing_mode_updates_of_sp_and_their_stores() {
        let program = [
            MVK_B15_2000,
            STW_B0_B15_POSTDEC_2,
            STW_B0_B15_POSTDEC_1,
            LDW_B15_PREINC_1_B1,
            NOP,
            NOP,
            NOP,
            NOP,
        ];
        let mut cpu = observed(&program, 0x1000, 0x2000);
        cpu.set_register("B0", 0xdead_beef).unwrap();
        cpu.run(3);
        assert_eq!(cpu.register('B', 15), 0x1ff4);
        let pushed = report(&cpu);
        assert_eq!(sp(&pushed), (Some(0x1ff4), Some(2)));
        assert_eq!(
            (pushed.lowest_write, pushed.lowest_write_cycle),
            (Some(0x1ff8), Some(2))
        );
        assert!(!pushed.out_of_bounds_write && !pushed.out_of_bounds_sp);
        // Both pushes reached memory, including the one the observer ignores.
        assert_eq!(cpu.host_read_bytes(0x2000, 4), [0xef, 0xbe, 0xad, 0xde]);
        assert_eq!(cpu.host_read_bytes(0x1ff8, 4), [0xef, 0xbe, 0xad, 0xde]);
        cpu.run(5);
        assert_eq!(cpu.register('B', 15), 0x1ff8);
        assert_eq!(cpu.register('B', 1), 0xdead_beef);
        assert_eq!(sp(&report(&cpu)), (Some(0x1ff4), Some(2)));
    }

    /// A store case: instruction name and encoding, address, window, the lowest in-window byte
    /// expected and whether a byte is expected out of the window.
    type StoreCase = (&'static str, u32, u32, (u32, u32), Option<u32>, bool);

    /// One store instruction at `address` with A0:A1 holding `STORED`, against `[start, end)`.
    fn store(opcode: u32, address: u32, start: u32, end: u32) -> Cpu {
        let mut cpu = observed(&[opcode], start, end);
        cpu.set_register("A4", address).unwrap();
        cpu.set_register("A0", 0x4433_2211).unwrap();
        cpu.set_register("A1", 0x8877_6655).unwrap();
        cpu.run(1);
        cpu
    }

    /// Every store width against both window edges, judged on the bytes the instruction
    /// actually committed: (instruction, address, window, lowest in-window byte, any byte out).
    #[test]
    fn committed_stores_against_both_window_edges() {
        let window = (0x1000, 0x2000);
        let cases: &[StoreCase] = &[
            ("STNDW", STNDW, 0x1ffc, window, Some(0x1ffc), true),
            ("STNDW", STNDW, 0x0ffc, window, Some(0x1000), true),
            ("STNDW", STNDW, 0x1ff8, window, Some(0x1ff8), false),
            ("STNDW", STNDW, 0x0ff8, window, None, false),
            ("STNDW", STNDW, 0x2000, window, None, false),
            // END need only be word aligned, so an aligned doubleword can still straddle it.
            ("STDW", STDW, 0x1000, (0x0ff0, 0x1004), Some(0x1000), true),
            ("STDW", STDW, 0x0ff8, (0x0ffc, 0x2000), Some(0x0ffc), true),
            ("STDW", STDW, 0x1ff8, window, Some(0x1ff8), false),
            ("STDW", STDW, 0x0ff8, window, None, false),
            ("STNW", STNW, 0x0ffd, window, Some(0x1000), true),
            ("STNW", STNW, 0x1ffd, window, Some(0x1ffd), true),
            ("STNW", STNW, 0x1000, window, Some(0x1000), false),
            ("STNW", STNW, 0x0ffc, window, None, false),
            ("STNW", STNW, 0x2000, window, None, false),
            ("STW", STW, 0x1ffc, window, Some(0x1ffc), false),
            ("STW", STW, 0x2000, window, None, false),
            ("STH", STH, 0x0ffe, window, None, false),
            ("STH", STH, 0x1ffe, window, Some(0x1ffe), false),
            ("STB", STB, 0x1fff, window, Some(0x1fff), false),
            ("STB", STB, 0x0fff, window, None, false),
        ];
        for &(name, opcode, address, (start, end), lowest, outside) in cases {
            let cpu = store(opcode, address, start, end);
            let label = format!("{name} at {address:#x} in [{start:#x},{end:#x})");
            assert_eq!(cpu.fault(), None, "{label}");
            let width = match name {
                "STNDW" | "STDW" => 8,
                "STNW" | "STW" => 4,
                "STH" => 2,
                _ => 1,
            };
            assert_eq!(
                cpu.host_read_bytes(address, width),
                STORED[..width],
                "{label}"
            );
            let report = report(&cpu);
            assert_eq!(report.lowest_write, lowest, "{label}");
            assert_eq!(report.lowest_write_cycle, lowest.map(|_| 0), "{label}");
            assert_eq!(report.out_of_bounds_write, outside, "{label}");
        }
    }

    /// Guest addresses wrap at 32 bits, and so does the overlap: a store that runs off the top
    /// of the address space lands at zero, in a window there.
    #[test]
    fn stores_wrapping_the_address_space_follow_memory() {
        let cases: &[StoreCase] = &[
            ("STNW", STNW, 0xffff_fffe, (0, 4), Some(0), true),
            ("STNDW", STNDW, 0xffff_fffc, (0, 4), Some(0), true),
            ("STNDW", STNDW, 0xffff_fffa, (0, 0x1000), Some(0), true),
            ("STNW", STNW, 0xffff_fffe, (0x1000, 0x2000), None, false),
            (
                "STNDW",
                STNDW,
                0xffff_fffc,
                (0xffff_fff8, 0xffff_fffc),
                None,
                false,
            ),
        ];
        for &(name, opcode, address, (start, end), lowest, outside) in cases {
            let cpu = store(opcode, address, start, end);
            let label = format!("{name} at {address:#x} in [{start:#x},{end:#x})");
            assert_eq!(cpu.fault(), None, "{label}");
            let width = if name == "STNDW" { 8 } else { 4 };
            let top = (0u32.wrapping_sub(address)) as usize;
            assert_eq!(cpu.host_read_bytes(address, top), STORED[..top], "{label}");
            assert_eq!(
                cpu.host_read_bytes(0, width - top),
                STORED[top..width],
                "{label}"
            );
            let report = report(&cpu);
            assert_eq!(report.lowest_write, lowest, "{label}");
            assert_eq!(report.out_of_bounds_write, outside, "{label}");
        }
    }

    #[test]
    fn predicated_off_stores_are_not_observed() {
        let skipped = store(STW_IF_B0, 0x1000, 0x1000, 0x2000);
        assert_eq!(skipped.host_read_bytes(0x1000, 4), [0; 4]);
        assert_eq!(report(&skipped).lowest_write, None);
        let taken = store(STW_IF_NOT_B0, 0x1000, 0x1000, 0x2000);
        assert_eq!(taken.host_read_bytes(0x1000, 4), STORED[..4]);
        assert_eq!(report(&taken).lowest_write, Some(0x1000));
    }

    /// L1DMPPAn guards 1 KB page n of the C6455's L1D SRAM; 0x128 grants Supervisor read and
    /// execute but not write. A refused store commits nothing - L1DMPFAR records it instead - and
    /// the observer only reports what committed: per half, for a doubleword split across two pages.
    #[test]
    fn mpu_refused_stores_and_halves_are_not_observed_as_written() {
        const L1DMPPA0: u32 = 0x0184_ae00;
        const L1DMPPA1: u32 = 0x0184_ae04;
        const NO_WRITE: u32 = 0x128;
        let page = 0x00f0_0000;
        let window = (page, page + 0x400);
        let run = |opcode: u32, address: u32, denied: u32| {
            let mut cpu = observed(&[opcode], window.0, window.1);
            cpu.bus_mut().write32(denied, NO_WRITE);
            cpu.set_register("A4", address).unwrap();
            cpu.set_register("A0", 0x4433_2211).unwrap();
            cpu.set_register("A1", 0x8877_6655).unwrap();
            cpu.run(1);
            cpu
        };
        // L1DMPFAR latches the address of the first refused access.
        let refused_at = |cpu: &mut Cpu| cpu.bus_mut().read32(0x0184_ac00);

        let mut refused = run(STW, page + 0x10, L1DMPPA0);
        assert_eq!(refused_at(&mut refused), page + 0x10);
        assert_eq!(refused.host_read_bytes(page + 0x10, 4), [0; 4]);
        assert_eq!(report(&refused).lowest_write, None);
        assert!(!report(&refused).out_of_bounds_write);

        // Outside half refused: only in-window bytes committed, so nothing went out of bounds.
        let straddle = page + 0x3fc;
        let mut inner = run(STNDW, straddle, L1DMPPA1);
        assert_eq!(refused_at(&mut inner), page + 0x400);
        assert_eq!(
            inner.host_read_bytes(straddle, 8),
            [0x11, 0x22, 0x33, 0x44, 0, 0, 0, 0]
        );
        assert_eq!(report(&inner).lowest_write, Some(straddle));
        assert!(!report(&inner).out_of_bounds_write);

        // Inside half refused: the access still overlapped the window, and its outside half
        // committed, which is exactly the out-of-bounds write the flag exists for.
        let mut outer = run(STNDW, straddle, L1DMPPA0);
        assert_eq!(refused_at(&mut outer), straddle);
        assert_eq!(
            outer.host_read_bytes(straddle, 8),
            [0, 0, 0, 0, 0x55, 0x66, 0x77, 0x88]
        );
        assert_eq!(report(&outer).lowest_write, None);
        assert!(report(&outer).out_of_bounds_write);
    }

    /// The observer is passive: with L1D caching a stack in L2 SRAM, the same program in either
    /// endianness leaves identical registers, statistics, cycle counts and memory with and
    /// without it - while the observed run did see the stack move.
    #[test]
    fn observation_is_passive_with_cached_stores_in_both_endiannesses() {
        const STDW_B15: u32 = 0x003c_03c4; // STDW.D2T1 A1:A0,*+B15[0]
        const STW_A0_B15_MINUS_1: u32 = 0x003c_20f4; // STW.D2T1 A0,*-B15[1]
        const ADDK_B15_MINUS_8: u32 = 0x07ff_fc52; // ADDK.S2 -8,B15
        let program = [
            MVK_B15_2000,
            MVKH_B15_800000,
            STW_B0_B15_POSTDEC_2,
            STW_B0_B15_POSTDEC_1,
            ADDK_B15_MINUS_8,
            STDW_B15,
            STW_A0_B15_MINUS_1,
            STNDW,
            STNW,
            LDW_B15_PREINC_1_B1,
            NOP,
            NOP,
            NOP,
            NOP,
            LDW_A4_B15,
            NOP,
            NOP,
            NOP,
            NOP,
            ADDK_B15_MINUS_16,
        ];
        for (endian, name) in [(Endian::Little, "little"), (Endian::Big, "big")] {
            let mut runs = [
                Cpu::new_with_endian(0, endian),
                Cpu::new_with_endian(0, endian),
            ];
            for cpu in &mut runs {
                cpu.bus_mut().write32(0x0184_0040, 4); // L1DCFG: all 32 KB of L1D as cache.
                cpu.write_words(0, &program);
                cpu.set_register("A4", 0x0080_1ffd).unwrap();
                cpu.set_register("A0", 0x0080_1f00).unwrap();
                cpu.set_register("A1", 0x8877_6655).unwrap();
                cpu.set_register("B0", 0xdead_beef).unwrap();
                // Allocate the stack's lines before the stores so they hit the cache.
                let _ = cpu.bus_mut().read32(0x0080_1fe0);
                let _ = cpu.bus_mut().read32(0x0080_1ff0);
            }
            runs[1]
                .configure_stack_observation(0x0080_1000, 0x0080_2000)
                .unwrap();
            for cpu in &mut runs {
                cpu.run(40);
                assert_eq!(cpu.fault(), None, "{name}");
            }
            let [plain, observed] = &mut runs;
            assert_eq!(
                serde_json::to_value(plain.snapshot()).unwrap(),
                serde_json::to_value(observed.snapshot()).unwrap(),
                "{name}"
            );
            assert_eq!(
                serde_json::to_value(plain.stats().counters()).unwrap(),
                serde_json::to_value(observed.stats().counters()).unwrap(),
                "{name}"
            );
            assert_eq!(plain.cycles(), observed.cycles(), "{name}");
            assert_eq!(
                plain.host_read_bytes(0x0080_1f00, 0x200),
                observed.host_read_bytes(0x0080_1f00, 0x200),
                "{name}"
            );
            // Through the cache, where dirty lines still hold the stores.
            for address in (0x0080_1fe0..0x0080_2000).step_by(4) {
                assert_eq!(
                    plain.bus_mut().read32(address),
                    observed.bus_mut().read32(address),
                    "{name} {address:#x}"
                );
            }
            let report = report(observed);
            assert!(report.armed, "{name}");
            assert!(report.lowest_write.is_some(), "{name}");
            assert!(report.min_sp.is_some_and(|sp| sp < 0x0080_2000), "{name}");
            assert_eq!(plain.stack_observation(), None);
        }
    }
}
