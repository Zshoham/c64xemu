/* C6472 host-boot result probe: every released core publishes a result of its own.
 *
 * The same image is loaded into each selected core's L2. A core learns who it is from DNUM,
 * computes 1 + 2 + ... + (10 + DNUM), and stores it in its slot of a mailbox in shared L2 -
 * 55, 66, 78, 91, 105, 120 for cores 0-5. Nothing is stored for a core that never ran.
 *
 * The emulator's run ends at the first core's C$$EXIT, so exactly one core may reach it, and
 * only after everyone released has published. BOOT_COMPLETE_STAT is set-only and was written
 * as one mask before any core left reset, so reading it tells each core the same set of
 * released cores; the lowest of them waits for the rest and returns the mask that answered.
 * The others spin until the run ends.
 *
 * An HPI boot leaves L1D a 32 KB cache (SPRUEC6G Table 21), and shared L2 is cacheable with no
 * coherence between cores: a result left dirty in the writer's L1D would never reach the
 * leader, and the leader would poll a stale line. Each core therefore turns L1D into SRAM
 * before touching the mailbox; the mode change writes back and invalidates whatever the
 * startup code had cached (SPRU871K section 3.3.3), and the mailbox lines were never among it.
 *
 * `guard` tells a core that started at the wrong address from one that started at its entry.
 * When the test links the entry above the base of L2, it puts `guard` at that base, the default
 * DSP_BOOT_ADDRn: a core the boot started there instead of at its programmed address publishes
 * GUARD, which no core computes, rather than sliding through zeroed memory into `_start`.
 */

#include <c6x.h>

#define BOOT_COMPLETE_STAT (*(volatile unsigned *)0x02AB0004)
#define L1DCFG (*(volatile unsigned *)0x01840040)
#define CORES 6
/* Bounded so a core that never starts fails the test instead of hanging it. */
#define SPIN_LIMIT 200000
/* What `guard` publishes: no core computes it. */
#define GUARD 0xDEADu

#pragma DATA_SECTION(results, ".shared")
volatile unsigned results[CORES];

#pragma CODE_SECTION(guard, ".guard")
#pragma RETAIN(guard)
void guard(void)
{
    results[DNUM] = GUARD;
    for (;;) {
    }
}

int main(void)
{
    unsigned core = DNUM;
    unsigned released = BOOT_COMPLETE_STAT & ((1u << CORES) - 1);
    unsigned answered = 0;
    unsigned sum = 0;
    unsigned leader = 0;
    unsigned index;
    long spin;

    L1DCFG = 0;
    (void)L1DCFG; /* the write has taken effect once it reads back */

    for (index = 1; index <= 10 + core; index++) {
        sum += index;
    }
    results[core] = sum;

    while (leader < CORES && !((released >> leader) & 1u)) {
        leader++;
    }
    if (core != leader) {
        for (;;) {
            /* The leader's exit ends the run. */
        }
    }

    for (spin = 0; spin < SPIN_LIMIT && answered != released; spin++) {
        answered = 0;
        for (index = 0; index < CORES; index++) {
            if (((released >> index) & 1u) && results[index] != 0) {
                answered |= 1u << index;
            }
        }
    }
    return (int)answered;
}
