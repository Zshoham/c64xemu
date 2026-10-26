/* C6472 shared-memory controller: the atomic-access monitors and the protection exception, run as
 * firmware on the emulated device.
 *
 * Core 0 runs two experiments on a word of the shared L2 with every cache off, so the word is
 * read and written where the controller sees it (SPRUEG5D section 2.8), and SL2PFCPEN is left at
 * its reset value, so the word is in non-prefetchable memory, where atomic accesses belong
 * (section 3):
 *
 *   1. LL, SL(A), SL(B), CMTL. Section 3.1.2 Table 3 says the second SL meets LinkdtV already
 *      set and "Invalidate the Link (LinkV = 0)", and section 3.1.3 Table 4 says a commit against
 *      LinkV = 0 is "Discard the request and return 0". So the CMTL answers 0, the word is
 *      unchanged, and the monitor's LinkData is still A. The control is the same sequence with
 *      one SL, which commits and stores.
 *   2. A write to SL2PFCPEN made in User mode. Section 5.1: the SMC "generates an exception back
 *      to that C64x+ megamodule as well as to other C64x+ megamodules connected to it", and
 *      section 5.3 says it records the accessing megamodule id and mode in SL2MPFSR and the
 *      address in SL2MPFAR. SPRS612G Table 7-10 gives the exception as system event 63, so core 0
 *      releases the other five cores and each reports whether it saw event 63 latched in its own
 *      EVTFLAG1. A write of the same register made in Supervisor mode, first, must raise nothing.
 *
 * Core 0 leaves the answers in the shared L2, in the order of `struct results` at 0x00200000.
 */

#include <c6x.h>

/* Boot controller (SPRS612G Table 3-14); see c6472_multicore. */
#define BOOT_COMPLETE_STAT (*(volatile unsigned *)0x02AB0004)
#define DSP_BOOT_ADDR(core) (*(volatile unsigned *)(0x02AB0204 + 0x20 * (core)))
#define SECONDARY_ENTRY 0x00810000u

/* The interrupt controller's event flags, events 32-63 (SPRU871K section 7.2): the megamodule's
 * own registers, so each core reads its own. */
#define EVTFLAG1 (*(volatile unsigned *)0x01800004)
#define SMC_EXCEPTION_EVENT 63u

/* The SMC control registers (SPRUEG5D Table 5, SPRS612G Table 2-2). */
#define SMC_BASE 0x02AA8000u
#define SL2PFCPEN (*(volatile unsigned *)(SMC_BASE + 0x000))
#define SL2MPFSR (*(volatile unsigned *)(SMC_BASE + 0x008))
#define SL2MPFAR (*(volatile unsigned *)(SMC_BASE + 0x00C))
#define SL2AMLSTAT(bank) (*(volatile unsigned *)(SMC_BASE + 0x100 + 4 * (bank)))
#define SL2AMLDATA(bank) (*(volatile unsigned *)(SMC_BASE + 0x120 + 4 * (bank)))

#define CORES 6
#define SPIN_LIMIT 200000

extern unsigned smc_ll(volatile unsigned *address);
extern void smc_sl(volatile unsigned *address, unsigned value);
extern unsigned smc_cmtl(volatile unsigned *address);
extern void smc_user_store(volatile unsigned *address, unsigned value);

/* Everything the run leaves for the test, and the word the experiments use, in the shared L2. */
struct results {
    unsigned word;            /* the contended word, 0x00200000 */
    unsigned loaded;          /* what LL read */
    unsigned contended_cmtl;  /* CMTL after LL, SL(A), SL(B): 0 */
    unsigned memory_after;    /* the word after it: unchanged */
    unsigned link_data;       /* SL2AMLDATA of the word's bank: A */
    unsigned link_status;     /* SL2AMLSTAT of the word's bank: LinkV clear, LinkdtV set */
    unsigned control_cmtl;    /* CMTL after LL, SL(C): 1 */
    unsigned control_memory;  /* the word after it: C */
    unsigned supervisor_flag; /* event 63 after a Supervisor write to SL2PFCPEN: 0 */
    unsigned prefetch_enable; /* SL2PFCPEN after the User write: still 0 */
    unsigned fault_status;    /* SL2MPFSR */
    unsigned fault_address;   /* SL2MPFAR */
    unsigned seen[CORES];     /* each core: 1 when event 63 was latched in its EVTFLAG1 */
    unsigned done[CORES];     /* each core: 1 once it has reported */
};

#pragma DATA_SECTION(results, ".shared")
volatile struct results results;

#define VALUE_INITIAL 0x11111111u
#define VALUE_A 0xAAAA0001u
#define VALUE_B 0xBBBB0002u
#define VALUE_C 0xC0DE0003u

static unsigned core_number(void)
{
    return DNUM;
}

static unsigned event_latched(void)
{
    return (EVTFLAG1 >> (SMC_EXCEPTION_EVENT - 32)) & 1u;
}

/* What a released core runs: no C runtime, so a leaf that touches only the shared results and
 * its own event flags. */
#pragma CODE_SECTION(secondary, ".secondary")
void secondary(void)
{
    unsigned core = core_number();
    long spin;

    for (spin = 0; spin < SPIN_LIMIT && !event_latched(); spin++) {
    }
    results.seen[core] = event_latched();
    results.done[core] = 1;
    for (;;) {
    }
}

int main(void)
{
    unsigned core = core_number();
    unsigned bank = (((unsigned)&results.word - 0x00200000u) >> 5) & 3u;
    unsigned index;
    long spin;

    if (core != 0) {
        secondary();
    }

    for (index = 0; index < CORES; index++) {
        results.seen[index] = 0;
        results.done[index] = 0;
    }

    /* Experiment 1: two SLs, one CMTL. */
    results.word = VALUE_INITIAL;
    results.loaded = smc_ll(&results.word);
    smc_sl(&results.word, VALUE_A);
    smc_sl(&results.word, VALUE_B);
    results.contended_cmtl = smc_cmtl(&results.word);
    results.memory_after = results.word;
    results.link_data = SL2AMLDATA(bank);
    results.link_status = SL2AMLSTAT(bank);

    /* The control: one SL, and the commit goes through. */
    smc_ll(&results.word);
    smc_sl(&results.word, VALUE_C);
    results.control_cmtl = smc_cmtl(&results.word);
    results.control_memory = results.word;

    /* Experiment 2. A Supervisor write is allowed and raises nothing. */
    SL2PFCPEN = 0;
    results.supervisor_flag = event_latched();

    for (index = 1; index < CORES; index++) {
        DSP_BOOT_ADDR(index) = SECONDARY_ENTRY >> 10;
    }
    for (index = 1; index < CORES; index++) {
        BOOT_COMPLETE_STAT = 1u << index;
    }

    smc_user_store(&SL2PFCPEN, 0xFFFFFFFFu);
    results.prefetch_enable = SL2PFCPEN;
    results.fault_status = SL2MPFSR;
    results.fault_address = SL2MPFAR;
    results.seen[0] = event_latched();
    results.done[0] = 1;

    for (spin = 0; spin < SPIN_LIMIT; spin++) {
        unsigned reported = 0;
        for (index = 0; index < CORES; index++) {
            reported += results.done[index];
        }
        if (reported == CORES) {
            break;
        }
    }
    return 0;
}
