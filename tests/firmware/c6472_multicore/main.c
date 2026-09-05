/* C6472 multicore rendezvous: core 0 releases the other five and they meet in shared memory.
 *
 * One image runs on all six megamodules. Each core learns which one it is from DNUM
 * (SPRU871K section 2.9.4), and the boot controller decides which cores are running at all:
 * in a ROM boot mode only core 0 comes out of reset, and it releases the others by giving
 * each a start address in DSP_BOOT_ADDRn and setting its bit in BOOT_COMPLETE_STAT
 * (SPRS612G sections 2.4.1 and 3.9). Every released core writes a signature into the shared
 * L2 at 0x0020_0000, and core 0 waits for all six before reporting the total.
 *
 * The caches are left as reset leaves them - disabled - so the shared buffer needs no
 * maintenance; the C6472's shared L2 has no hardware coherence between cores (SPRUEG5D
 * section 2.6), and a firmware that enabled caching here would have to write back before
 * publishing and invalidate before reading.
 */

#include <c6x.h>

/* Boot controller (SPRS612G Table 3-14). DSP_BOOT_ADDRn holds the start address shifted right
 * by ten; the per-core registers are 0x20 apart. */
#define BOOT_COMPLETE_STAT (*(volatile unsigned *)0x02AB0004)
#define DSP_BOOT_ADDR(core) (*(volatile unsigned *)(0x02AB0204 + 0x20 * (core)))

/* The chip-level inter-processor interrupt registers (SPRS612G section 3.7.2): the source
 * flags a sender sets, and the write-one pulse that raises event 84 in the destination. */
#define IPCGR(core) (*(volatile unsigned *)(0x02A80540 + 4 * (core)))
#define IPCAR(core) (*(volatile unsigned *)(0x02A80580 + 4 * (core)))
#define IPC_GENERATE 1u

/* Where the other cores start. DSP_BOOT_ADDRn holds the address shifted right by ten, so the
 * entry has to be 1 KB aligned; the linker command file gives `.secondary` that window. */
#define SECONDARY_ENTRY 0x00810000u

#define CORES 6
#define SIGNATURE(core) (0xC0DE0000u | (unsigned)(core))
/* Bounded so a broken device fails fast instead of hanging the suite: long enough for five
 * cores to come out of reset and report, short enough that a device that never releases them
 * still reaches the tally and says so. */
#define SPIN_LIMIT 100000

/* The rendezvous buffer, in the shared L2 window every core reaches at the same address. */
#pragma DATA_SECTION(rendezvous, ".shared")
volatile unsigned rendezvous[CORES];
#pragma DATA_SECTION(acknowledged, ".shared")
volatile unsigned acknowledged[CORES];

/* What core 0 leaves behind: the sum of every core's signature, and how many answered. */
volatile unsigned total;
volatile unsigned answered;

/* DNUM, control register 0x11, is the megamodule number this code is running on; the TI
 * compiler exposes it as a control register in c6x.h. */
static unsigned core_number(void)
{
    return DNUM;
}

/* What a released core runs. It has no C runtime behind it - the boot controller drops it
 * here with nothing initialised - so it touches only the shared buffer and its own registers.
 */
#pragma CODE_SECTION(secondary, ".secondary")
void secondary(void)
{
    unsigned core = core_number();

    rendezvous[core] = SIGNATURE(core);
    /* Which source flag core 0 set when it announced itself, then acknowledge it. */
    acknowledged[core] = IPCGR(core) >> 4;
    IPCAR(core) = IPCGR(core);
    for (;;) {
        /* Nothing more to do; the run ends when core 0 reaches its exit. */
    }
}

int main(void)
{
    unsigned core = core_number();
    unsigned index;
    long spin;

    if (core != 0) {
        /* Immediate boot starts every core in main; a ROM boot mode sends them to `secondary`
         * instead. Either way a core that is not core 0 does the same thing. */
        secondary();
    }

    for (index = 0; index < CORES; index++) {
        rendezvous[index] = 0;
        acknowledged[index] = 0;
    }
    rendezvous[0] = SIGNATURE(0);

    /* Start every other core at this image's entry point and release it. Each core fetches
     * from its own L2, which already holds the same image. */
    for (index = 1; index < CORES; index++) {
        DSP_BOOT_ADDR(index) = SECONDARY_ENTRY >> 10;
    }
    for (index = 1; index < CORES; index++) {
        BOOT_COMPLETE_STAT = 1u << index;
        /* Announce ourselves to the core we just released: source flag 0 plus the pulse. */
        IPCGR(index) = (1u << 4) | IPC_GENERATE;
    }

    for (spin = 0; spin < SPIN_LIMIT; spin++) {
        unsigned seen = 0;
        for (index = 0; index < CORES; index++) {
            if (rendezvous[index] == SIGNATURE(index)) {
                seen++;
            }
        }
        if (seen == CORES) {
            break;
        }
    }

    total = 0;
    answered = 0;
    for (index = 0; index < CORES; index++) {
        if (rendezvous[index] == SIGNATURE(index)) {
            answered++;
            total += index;
        }
        if (acknowledged[index] == 1u) {
            total += 0x100u;
        }
    }
    /* 15 is 0+1+2+3+4+5, and five cores each saw source flag 0 in their IPCGR: 5 * 0x100. */
    return (int)total;
}
