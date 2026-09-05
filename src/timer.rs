//! The C6455's two 64-bit timers.
//!
//! SPRU968: each timer is a pair of 32-bit counter and period registers that work as one
//! 64-bit up-counter, as two independent 32-bit timers - one of them behind a four-bit
//! prescaler - or as a watchdog. Reaching the period raises the timer's interrupt to the CPU
//! and its event to the EDMA3 controller, and drives the timer output as a pulse or a
//! square wave. The timers are clocked by SYSCLK3, which SPRS276M section 7.6 fixes at the
//! CPU clock divided by six.
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const EMUMGT: u32 = 0x04;
const CNTLO: u32 = 0x10;
const CNTHI: u32 = 0x14;
const PRDLO: u32 = 0x18;
const PRDHI: u32 = 0x1c;
const TCR: u32 = 0x20;
const TGCR: u32 = 0x24;
const WDTCR: u32 = 0x28;
const SPAN: u32 = 0x2c;

/// The CLKDIV field reports the ratio of the CPU clock to the timer input clock, which the
/// device fixes at six (SPRS276M section 7.6: divider D3 is fixed at /6).
const CLOCK_DIVIDER: u32 = 6;

/// TGCR.TIMMODE (SPRU968 Table 13).
const MODE_64BIT: u32 = 0;
const MODE_UNCHAINED: u32 = 1;
const MODE_WATCHDOG: u32 = 2;
const MODE_CHAINED: u32 = 3;

/// WDTCR fields (SPRU968 Table 14). The service key is a sequence: A5C6h then DA7Eh.
const WDTCR_WDFLAG: u32 = 1 << 15;
const WDTCR_WDEN: u32 = 1 << 14;
const WDKEY_FIRST: u32 = 0xA5C6;
const WDKEY_SECOND: u32 = 0xDA7E;

/// Where the watchdog is in the state machine of SPRU968 Figure 12.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Watchdog {
    /// The counter runs but the output is disconnected; the timer is a general-purpose one.
    Disabled,
    /// WDEN is set and the first key has been seen; the second key activates it.
    PreActive,
    /// Servicing is required, and any other key value times out immediately.
    Active,
    /// Timed out. The watchdog cannot be enabled again until a hardware reset.
    Timeout,
}

pub struct Timer {
    base: u32,
    /// SOFT and FREE; CLKDIV is read-only.
    clkspd: u32,
    cnt: [u32; 2],
    prd: [u32; 2],
    tcr: u32,
    tgcr: u32,
    wdtcr: u32,
    #[cfg(test)]
    divide: u32,
    /// Timer clocks left in each half's output pulse.
    pulse: [u32; 2],
    /// Set when a one-shot half has reached its period and stopped counting.
    stopped: [bool; 2],
    /// The watchdog's state, and whether the first half of its service key has been seen.
    watchdog: Watchdog,
    watchdog_keyed: bool,
    /// A timeout the service key triggered, which the next tick reports as the timer's event.
    watchdog_forced: bool,
}

impl Timer {
    pub fn new(base: u32) -> Self {
        Self {
            base,
            clkspd: 0,
            cnt: [0; 2],
            prd: [0; 2],
            tcr: 0,
            tgcr: 0,
            wdtcr: 0,
            #[cfg(test)]
            divide: 0,
            pulse: [0; 2],
            stopped: [false; 2],
            watchdog: Watchdog::Disabled,
            watchdog_keyed: false,
            watchdog_forced: false,
        }
    }
}

impl SlavePort for Timer {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let offset = self.offset(address)?;
        let value = match offset {
            EMUMGT => (CLOCK_DIVIDER << 16) | (self.clkspd & 3),
            CNTLO => self.cnt[0],
            CNTHI => self.cnt[1],
            PRDLO => self.prd[0],
            PRDHI => self.prd[1],
            TCR => self.tcr,
            TGCR => self.tgcr,
            WDTCR => self.wdtcr,
            _ => 0,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        let Some(offset) = self.offset(address) else {
            return PortWrite::Unhandled;
        };
        // Once the watchdog has left the disabled state, only WDKEY writes have any effect
        // (SPRU968 section 4.4).
        let protected = matches!(self.watchdog, Watchdog::PreActive | Watchdog::Active);
        if protected && offset != WDTCR {
            return PortWrite::Done;
        }
        match offset {
            EMUMGT => self.clkspd = value & 3,
            CNTLO => self.cnt[0] = value,
            CNTHI => self.cnt[1] = value,
            PRDLO => self.prd[0] = value,
            PRDHI => self.prd[1] = value,
            TCR => {
                // TSTAT is read-only, and writing an enable mode re-arms a one-shot half.
                let status = self.tcr & (STATUS_LOW | STATUS_HIGH);
                self.tcr = (value & !(STATUS_LOW | STATUS_HIGH)) | status;
                for half in 0..2 {
                    if self.enable_mode(half) != 0 {
                        self.stopped[half] = false;
                    }
                }
            }
            TGCR => {
                self.tgcr = value & 0x0000_ffff;
                // A half held in reset clears its output status and keeps its counter
                // (SPRU968 Table 13).
                for half in 0..2 {
                    if self.tgcr & (1 << half) == 0 {
                        self.clear_status(half);
                        self.stopped[half] = false;
                    }
                }
            }
            WDTCR => self.write_wdtcr(value),
            _ => {}
        }
        PortWrite::Done
    }
}

impl Timer {
    #[cfg(test)]
    pub fn tick(&mut self) -> [bool; 2] {
        self.divide += 1;
        if self.divide == CLOCK_DIVIDER {
            self.divide = 0;
            self.tick_sysclk3()
        } else {
            [false; 2]
        }
    }

    /// One SYSCLK3 edge. System shares the /6 divider across peripherals instead of paying
    /// four separate divider checks on every CPU cycle.
    pub(crate) fn tick_sysclk3(&mut self) -> [bool; 2] {
        let mode = (self.tgcr >> 2) & 3;
        let mut events = [false; 2];
        if self.watchdog_forced {
            self.watchdog_forced = false;
            return [true, false];
        }
        match mode {
            // The watchdog counts as one 64-bit up-counter, exactly like the general-purpose
            // 64-bit timer; what differs is what reaching the period does.
            MODE_64BIT | MODE_WATCHDOG => {
                // Both halves have to be out of reset for the 64-bit counter to run.
                if self.tgcr & 3 == 3 {
                    events[0] = self.advance(0, true);
                }
                // Before the active state the output is never asserted and the flag never set:
                // reaching the period is only an interrupt (SPRU968 section 4.3's note).
                if events[0] && self.watchdog == Watchdog::Active {
                    self.watchdog_timeout();
                }
            }
            MODE_UNCHAINED => {
                if self.tgcr & 1 != 0 {
                    events[0] = self.advance(0, false);
                }
                if self.tgcr & 2 != 0 && self.prescale() {
                    events[1] = self.advance(1, false);
                }
            }
            // Chained mode: TIMHI is a 32-bit prescaler for TIMLO. One cycle after CNTHI
            // matches PRDHI a clock reaches TIMLO and CNTHI resets to 0 (SPRU968 section 2.2.1).
            MODE_CHAINED if self.tgcr & 3 == 3 && self.chain_prescale() => {
                events[0] = self.advance(0, false);
            }
            _ => {}
        }
        for (half, raised) in events.into_iter().enumerate() {
            if raised {
                self.raise_output(half);
            } else if self.pulse[half] > 0 {
                self.pulse[half] -= 1;
                if self.pulse[half] == 0 {
                    self.clear_status(half);
                }
            }
        }
        events
    }

    /// The four-bit prescaler in TGCR clocks the high half in unchained mode: TDDRHI counts
    /// timer clocks and the counter steps on the cycle after it matches PSCHI (SPRU968 2.2.2.1).
    fn prescale(&mut self) -> bool {
        if self.enable_mode(1) == 0 || self.stopped[1] {
            return false;
        }
        let counter = (self.tgcr >> 12) & 0xf;
        let period = (self.tgcr >> 8) & 0xf;
        if counter == period {
            self.tgcr &= !0xf000;
            return true;
        }
        self.tgcr = (self.tgcr & !0xf000) | ((counter + 1) << 12);
        false
    }

    /// One counter step for a half. Reaching the period raises the event; the following clock
    /// restarts a continuous timer and stops a one-shot one (SPRU968 Table 12).
    fn advance(&mut self, half: usize, wide: bool) -> bool {
        let enable = self.enable_mode(half);
        if enable == 0 || self.stopped[half] {
            return false;
        }
        let (mut counter, period) = if wide {
            (
                (u64::from(self.cnt[1]) << 32) | u64::from(self.cnt[0]),
                (u64::from(self.prd[1]) << 32) | u64::from(self.prd[0]),
            )
        } else {
            (u64::from(self.cnt[half]), u64::from(self.prd[half]))
        };
        let matched = if counter == period {
            if enable == 2 {
                counter = 0;
            } else {
                self.stopped[half] = true;
            }
            false
        } else {
            counter = counter.wrapping_add(1);
            counter == period
        };
        if wide {
            self.cnt[0] = counter as u32;
            self.cnt[1] = (counter >> 32) as u32;
        } else {
            self.cnt[half] = counter as u32;
        }
        matched
    }

    /// The 32-bit prescaler of chained mode, gated by TIMLO's enable because the two halves
    /// are one timer here (SPRU968 section 2.2.1).
    fn chain_prescale(&mut self) -> bool {
        if self.enable_mode(0) == 0 || self.stopped[0] {
            return false;
        }
        if self.cnt[1] == self.prd[1] {
            self.cnt[1] = 0;
            return true;
        }
        self.cnt[1] = self.cnt[1].wrapping_add(1);
        false
    }

    /// WDTCR, which is where the watchdog's state machine lives (SPRU968 Figure 12). WDFLAG is
    /// write-one-to-clear; WDEN moves a disabled watchdog to pre-active once the first key
    /// arrives, and in the active state anything but the right key sequence times out at once.
    fn write_wdtcr(&mut self, value: u32) {
        let key = value >> 16;
        let enable = value & WDTCR_WDEN != 0;
        match self.watchdog {
            Watchdog::Disabled => {
                self.wdtcr = (self.wdtcr & WDTCR_WDFLAG) | (value & !WDTCR_WDFLAG);
                if value & WDTCR_WDFLAG != 0 {
                    self.wdtcr &= !WDTCR_WDFLAG;
                }
                if enable && key == WDKEY_FIRST {
                    self.watchdog = Watchdog::PreActive;
                    self.watchdog_keyed = true;
                }
            }
            Watchdog::PreActive => {
                // Only a keyed write is allowed, and the second key activates the watchdog with
                // its counter cleared and its flag clear.
                if key == WDKEY_SECOND && self.wdtcr & WDTCR_WDEN != 0 {
                    self.watchdog = Watchdog::Active;
                    self.watchdog_keyed = false;
                    self.cnt = [0; 2];
                    self.wdtcr &= !WDTCR_WDFLAG;
                } else if key == WDKEY_FIRST {
                    self.watchdog_keyed = true;
                }
            }
            Watchdog::Active => {
                if key == WDKEY_FIRST {
                    self.watchdog_keyed = true;
                } else if key == WDKEY_SECOND && self.watchdog_keyed {
                    // A serviced watchdog restarts its count.
                    self.watchdog_keyed = false;
                    self.cnt = [0; 2];
                } else {
                    // Any other key times out at once, and its interrupt is reported on the
                    // next tick because that is where the timer's events are collected.
                    self.watchdog_timeout();
                    self.watchdog_forced = true;
                    self.raise_output(0);
                }
            }
            // A timed-out watchdog ignores WDEN until a hardware reset.
            Watchdog::Timeout => {}
        }
    }

    /// The timeout event's state changes: WDFLAG is set from TSTAT_LO, the counter is reset and
    /// the watchdog is disabled until a hardware reset (SPRU968 section 4.3). The output and the
    /// interrupt are the caller's business, because a timeout the counter reaches is reported
    /// through the tick's events while one a bad key triggers is not.
    fn watchdog_timeout(&mut self) {
        self.wdtcr |= WDTCR_WDFLAG;
        self.wdtcr &= !WDTCR_WDEN;
        self.cnt = [0; 2];
        self.watchdog = Watchdog::Timeout;
        self.watchdog_keyed = false;
    }

    /// Pulse mode holds the output for PWID+1 timer clocks; clock mode toggles it, giving the
    /// documented 50% duty cycle (SPRU968 Table 12).
    fn raise_output(&mut self, half: usize) {
        let shift = if half == 0 { 0 } else { 16 };
        let clock_mode = self.tcr & (1 << (shift + 3)) != 0;
        let width = ((self.tcr >> (shift + 4)) & 3) + 1;
        let status = if half == 0 { STATUS_LOW } else { STATUS_HIGH };
        if clock_mode {
            self.tcr ^= status;
            self.pulse[half] = 0;
        } else {
            self.tcr |= status;
            self.pulse[half] = width;
        }
    }

    fn clear_status(&mut self, half: usize) {
        let status = if half == 0 { STATUS_LOW } else { STATUS_HIGH };
        self.tcr &= !status;
        self.pulse[half] = 0;
    }

    fn enable_mode(&self, half: usize) -> u32 {
        if half == 0 {
            (self.tcr >> 6) & 3
        } else {
            (self.tcr >> 22) & 3
        }
    }

    fn offset(&self, address: u32) -> Option<u32> {
        if address % 4 != 0 {
            return None;
        }
        let offset = address.checked_sub(self.base)?;
        (offset < SPAN).then_some(offset)
    }
}

const STATUS_LOW: u32 = 1 << 0;
const STATUS_HIGH: u32 = 1 << 16;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    #[test]
    fn the_watchdog_follows_its_key_sequence() {
        let mut timer = Timer::new(0);
        timer.write(PRDLO, 4, TEST_CONTEXT);
        timer.write(TGCR, (MODE_WATCHDOG << 2) | 3, TEST_CONTEXT);
        timer.write(TCR, 2 << 6, TEST_CONTEXT);
        // WDEN with the first key moves it to pre-active, where nothing but the key is written.
        timer.write(WDTCR, (WDKEY_FIRST << 16) | WDTCR_WDEN, TEST_CONTEXT);
        timer.write(PRDLO, 99, TEST_CONTEXT);
        assert_eq!(timer.read(PRDLO, TEST_CONTEXT), Some(4));
        // The second key activates it, and servicing restarts the count.
        timer.write(WDTCR, (WDKEY_SECOND << 16) | WDTCR_WDEN, TEST_CONTEXT);
        for _ in 0..CLOCK_DIVIDER * 2 {
            timer.tick();
        }
        assert_eq!(timer.read(CNTLO, TEST_CONTEXT), Some(2));
        timer.write(WDTCR, WDKEY_FIRST << 16, TEST_CONTEXT);
        timer.write(WDTCR, WDKEY_SECOND << 16, TEST_CONTEXT);
        assert_eq!(timer.read(CNTLO, TEST_CONTEXT), Some(0));
        // A wrong key times out at once: the flag is set, the enable is cleared, and the enable
        // cannot come back without a hardware reset.
        timer.write(WDTCR, 0x1234 << 16, TEST_CONTEXT);
        let wdtcr = timer.read(WDTCR, TEST_CONTEXT).unwrap();
        assert_eq!(wdtcr & WDTCR_WDFLAG, WDTCR_WDFLAG);
        assert_eq!(wdtcr & WDTCR_WDEN, 0);
        timer.write(WDTCR, (WDKEY_FIRST << 16) | WDTCR_WDEN, TEST_CONTEXT);
        assert_eq!(timer.read(WDTCR, TEST_CONTEXT).unwrap() & WDTCR_WDEN, 0);
    }

    fn run(timer: &mut Timer, cycles: usize) -> [usize; 2] {
        let mut counts = [0usize; 2];
        for _ in 0..cycles {
            let events = timer.tick();
            for half in 0..2 {
                if events[half] {
                    counts[half] += 1;
                }
            }
        }
        counts
    }

    #[test]
    fn the_input_clock_is_the_cpu_clock_divided_by_six() {
        // SPRS276M section 7.6 fixes divider D3, which feeds the timers, at /6, and the CLKDIV
        // field reports that ratio.
        let mut timer = Timer::new(0);
        assert_eq!(timer.read(EMUMGT, TEST_CONTEXT), Some(6 << 16));
        timer.write(PRDLO, 3, TEST_CONTEXT);
        // 64-bit continuous mode, both halves out of reset.
        timer.write(TGCR, 3, TEST_CONTEXT);
        timer.write(TCR, 2 << 6, TEST_CONTEXT);
        assert_eq!(run(&mut timer, 17), [0, 0]);
        assert_eq!(timer.read(CNTLO, TEST_CONTEXT), Some(2));
        // The eighteenth cycle is the third timer clock, which reaches the period.
        assert_eq!(run(&mut timer, 1), [1, 0]);
        assert_eq!(timer.read(CNTLO, TEST_CONTEXT), Some(3));
        assert_eq!(
            timer.read(TCR, TEST_CONTEXT).unwrap() & STATUS_LOW,
            STATUS_LOW
        );
        // The counter restarts one timer clock later and the one-clock pulse ends with it.
        assert_eq!(run(&mut timer, 6), [0, 0]);
        assert_eq!(timer.read(CNTLO, TEST_CONTEXT), Some(0));
        assert_eq!(timer.read(TCR, TEST_CONTEXT).unwrap() & STATUS_LOW, 0);
        // Continuous mode keeps going: every four timer clocks from here.
        assert_eq!(run(&mut timer, 24), [1, 0]);
    }

    #[test]
    fn a_one_shot_half_stops_at_its_period() {
        // ENAMODE = 01b runs the timer once (SPRU968 Table 12).
        let mut timer = Timer::new(0);
        timer.write(PRDLO, 2, TEST_CONTEXT);
        timer.write(TGCR, 3, TEST_CONTEXT);
        timer.write(TCR, 1 << 6, TEST_CONTEXT);
        assert_eq!(run(&mut timer, 120), [1, 0]);
        assert_eq!(timer.read(CNTLO, TEST_CONTEXT), Some(2));
        // Re-arming the enable mode starts it again.
        timer.write(CNTLO, 0, TEST_CONTEXT);
        timer.write(TCR, 1 << 6, TEST_CONTEXT);
        assert_eq!(run(&mut timer, 120), [1, 0]);
    }

    #[test]
    fn the_unchained_high_half_runs_behind_its_prescaler() {
        // SPRU968 2.2.2.1: TDDRHI counts timer clocks and CNTHI steps when it matches PSCHI,
        // so the high half needs (PSCHI + 1) x (PRDHI + 1) timer clocks per event while the
        // low half is a plain 32-bit timer.
        let mut timer = Timer::new(0);
        timer.write(PRDLO, 1, TEST_CONTEXT);
        timer.write(PRDHI, 1, TEST_CONTEXT);
        // Unchained mode, both halves out of reset, prescale period 1.
        timer.write(TGCR, (1 << 8) | (MODE_UNCHAINED << 2) | 3, TEST_CONTEXT);
        timer.write(TCR, (2 << 22) | (2 << 6), TEST_CONTEXT);
        // Twelve timer clocks: six low events, three high events.
        assert_eq!(run(&mut timer, 12 * 6), [6, 3]);
    }
}
