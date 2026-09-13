/* Every 64-bit timer of the C6472, programmed identically and compared against each other.
 *
 * The device has twelve of them on 64 KB boundaries from 025E 0000h: "six localized timers
 * (Timer 0 - Timer 5) and six shared timers (Timer 6 - Timer 11)", and the localized ones are
 * "clocked with an internal clock with a CPU/6 frequency" (SPRS612G section 7.16). Each timer
 * has its own register block - CNTLO/CNTHI, PRDLO/PRDHI, TCR, TGCR - and nothing in the timer
 * user's guide makes one instance different from another: the input clock is the internal clock,
 * "a divided-down version of the CPU clock" whose ratio EMUMGT.CLKDIV reports (SPRU818B sections
 * 1 and 5.1), the 64-bit general-purpose mode has no prescaler at all (SPRU818B section 3.9.2 and
 * Table 1), and running it needs TIMLORS and TIMHIRS both set plus ENAMODE_LO (SPRU818B Table 13
 * and Table 3). So instances brought up the same way have to count at the same rate.
 *
 * The one thing that could differ between them in practice is *who* programs them: five of these
 * timers belong to megamodules whose cores are not the one running this code, so their writes
 * cross the switch fabric. Core 0 brings up all twelve, spins on its time-stamp counter, and
 * leaves every counter's delta in the shared L2 at a fixed address for the harness to dump; it
 * returns the worst deviation in thousandths of a percent, so the answer is 0 only when every
 * instance counted alike.
 *
 * The one documented asymmetry is not programmable from here: "C64x+ Megamodule local timers are
 * clock gated as well when the corresponding C64x+ Megamodule is clock gated" (SPRUEG3B
 * section 4.1), so a local timer stops when its own megamodule's clock is gated - which is why
 * this program leaves every megamodule alone and only moves the timers.
 */

#include <c6x.h>

#define TIMER_COUNT  12u
#define TIMER_STRIDE 0x00010000u
#define TIMER_BASE(index) (0x025E0000u + (index) * TIMER_STRIDE)

/* Register offsets from a timer's base (SPRU818B section 5; the same layout the C645x guide
 * tabulates: CNTLO 10h, CNTHI 14h, PRDLO 18h, PRDHI 1Ch, TCR 20h, TGCR 24h). */
#define TIMER_CNTLO 0x10u
#define TIMER_CNTHI 0x14u
#define TIMER_PRDLO 0x18u
#define TIMER_PRDHI 0x1cu
#define TIMER_TCR   0x20u
#define TIMER_TGCR  0x24u

/* TGCR: TIMMODE 00b is the 64-bit general-purpose timer, and both halves have to be out of reset
 * for it to run (SPRU818B Table 13). TCR: ENAMODE_LO 10b counts continuously, reloading at the
 * match (SPRU818B Table 12). */
#define TGCR_64BIT_BOTH_RUNNING   0x3u
#define TCR_ENAMODE_LO_CONTINUOUS 0x80u

/* Long enough for a million timer clocks at the C6472's CPU/6 and short enough to stay inside the
 * harness's cycle budget. */
#define SPIN_CYCLES 24000000u

/* The shared L2 every core reaches at the same address (SPRS612G Table 2-2): per timer, the
 * counter's delta and its TGCR sample, then the spin the deltas are measured against, and after
 * those the verdict - the worst deviation in thousandths - with the numbers it came from, so that
 * a failing run says which pair disagreed and by how much.
 *
 * The verdict is a word in memory rather than the return value because `c64xemu run`'s exit code
 * is not the program's: the C runtime leaves A4 reading 1 for a plain `return 0` from main on
 * this entry point, and the harness reads the state it dumped instead. */
#define SLOT_DELTA(index) (index)
#define SLOT_TGCR(index)  (TIMER_COUNT + (index))
#define SLOT_SPIN         (2u * TIMER_COUNT)
#define SLOT_WORST        (SLOT_SPIN + 1u)
#define SLOT_BASE         (SLOT_SPIN + 2u)
#define SLOT_SLACK        (SLOT_SPIN + 3u)
#define SLOT_SPREAD       (SLOT_SPIN + 4u)

#pragma DATA_SECTION(shared, ".shared_fixed")
volatile unsigned shared[32];

static void timer_write(unsigned base, unsigned offset, unsigned value)
{
    *(volatile unsigned *)(base + offset) = value;
}

static unsigned timer_read(unsigned base, unsigned offset)
{
    return *(volatile unsigned *)(base + offset);
}

/* Bring one timer up the way the sysclock driver does: hold both halves in reset, zero the
 * counter, put the period out of reach, then release the halves and count.
 *
 * PRD is written high-half-first so that no intermediate value can match the counter: a match
 * resets the counter, and a reset mid-interval would read as a slow clock.
 */
static void timer_start(unsigned index)
{
    unsigned base = TIMER_BASE(index);

    timer_write(base, TIMER_TGCR, 0u);
    timer_write(base, TIMER_TCR, 0u);
    timer_write(base, TIMER_CNTLO, 0u);
    timer_write(base, TIMER_CNTHI, 0u);
    timer_write(base, TIMER_PRDHI, 0xFFFFFFFFu);
    timer_write(base, TIMER_PRDLO, 0xFFFFFFFFu);
    timer_write(base, TIMER_PRDHI, 0xFFFFFFFFu);
    timer_write(base, TIMER_TGCR, TGCR_64BIT_BOTH_RUNNING);
    timer_write(base, TIMER_TCR, TCR_ENAMODE_LO_CONTINUOUS);
}

/* The low half of a counter that will not wrap inside a run this short, read on its own so that
 * the comparison needs no 64-bit arithmetic beyond a subtraction. */
static unsigned timer_low(unsigned index)
{
    return timer_read(TIMER_BASE(index), TIMER_CNTLO);
}

int main(void)
{
    unsigned first[TIMER_COUNT];
    unsigned index, start, spin, worst = 0;

    /* A write to TSCL enables the time-stamp counter and the value is discarded (SPRUFE8B 2.9.3). */
    TSCL = 0u;

    for (index = 0; index < 32u; index++) {
        shared[index] = 0u;
    }

    for (index = 0; index < TIMER_COUNT; index++) {
        timer_start(index);
        first[index] = timer_low(index);
    }

    start = TSCL;
    do {
        spin = TSCL - start;
    } while (spin < SPIN_CYCLES);

    for (index = 0; index < TIMER_COUNT; index++) {
        shared[SLOT_DELTA(index)] = timer_low(index) - first[index];
        shared[SLOT_TGCR(index)] = timer_read(TIMER_BASE(index), TIMER_TGCR);
    }
    shared[SLOT_SPIN] = spin;

    shared[SLOT_BASE] = shared[SLOT_DELTA(0)];

    for (index = 1; index < TIMER_COUNT; index++) {
        unsigned base = shared[SLOT_DELTA(0)];
        unsigned other = shared[SLOT_DELTA(index)];
        unsigned high = other > base ? other : base;
        unsigned low = other > base ? base : other;
        /* A timer brought up a moment before another starts a few counts earlier, so a thousandth
         * of the interval is start offset; what is left over is a rate difference. */
        unsigned slack = base / 1000u;
        unsigned deviation;

        if (high - low > shared[SLOT_SPREAD]) {
            shared[SLOT_SPREAD] = high - low;
        }
        shared[SLOT_SLACK] = slack;
        if (high - low <= slack) {
            continue;
        }
        deviation = ((high - low - slack) * 1000u) / base;
        if (deviation > worst) {
            worst = deviation;
        }
    }

    shared[SLOT_WORST] = worst;

    return (int)worst;
}
