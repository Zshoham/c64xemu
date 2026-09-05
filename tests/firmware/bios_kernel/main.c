/* A SYS/BIOS kernel exerciser for the emulated C6455.
 *
 * Nothing here is a peripheral test. What runs is TI's own scheduler and TI's own interrupt
 * dispatcher, in the paths a real application leans on:
 *
 *   - three tasks at three priorities handing work along two semaphores, so every hand-off is a
 *     context switch through ti_sysbios_family_c62_TaskSupport_swap,
 *   - a hardware interrupt whose service routine posts a Swi and readies a task,
 *   - a second hardware interrupt raised from inside the first one's routine, so the dispatcher
 *     has to nest: the outer routine spins until the inner one has run, which makes the nesting
 *     an observable fact rather than a hope.
 *
 * Every count is driven by the program, never by a timer, so the totals are exact: the rounds
 * loop raises the outer event a fixed number of times and waits for the whole chain each time.
 * The checksum folds all of them, so a lost switch, a dropped Swi or a dispatcher that refuses to
 * nest changes it.
 */

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <stdio.h>

#define REG(address) (*(volatile unsigned int *)(address))

/* Megamodule interrupt controller (SPRU871K chapter 7). */
#define EVTFLAG0 0x01800000u
#define EVTSET0  0x01800020u
#define EVTCLR0  0x01800040u

/* Two system events nothing in this program's device model drives, raised by software. */
#define EVENT_OUTER 20u
#define EVENT_INNER 21u
#define CPU_INT_OUTER 5
#define CPU_INT_INNER 7

#define ROUNDS 16u
#define EVENTS 4u
#define TICKS  4u
#define SLEEPS 2u
#define LETTERS 16u
#define ITEMS  64u
#define SPIN_LIMIT 100000u

static Semaphore_Handle work;
static Semaphore_Handle relay;
static Semaphore_Handle finished;
static Semaphore_Handle from_interrupt;
static Semaphore_Handle ticker;
static Swi_Handle deferred;
static Clock_Handle heartbeat;
static Mailbox_Handle letterbox;
static Event_Handle both_halves;

static volatile unsigned int consumed;
static volatile unsigned int relayed;
static volatile unsigned int outer_taken;
static volatile unsigned int inner_taken;
static volatile unsigned int nested_observed;
static volatile unsigned int swi_ran;
static volatile unsigned int woken_by_interrupt;
static volatile unsigned int nesting_failures;
static volatile unsigned int clock_ticks;
static volatile unsigned int slept;
static volatile unsigned int letters;
static volatile unsigned int masked_delivery;
static volatile unsigned int events_taken;
static volatile unsigned int event_phase;
/* The nested-disable section: whether the interrupt stayed away while an inner Hwi_restore had
 * been done but the outer disable was still in force, and whether it arrived once that was
 * undone too. */
static volatile unsigned int nested_disable_held;
static volatile unsigned int nested_disable_released;

static unsigned int fold(unsigned int accumulator, unsigned int value)
{
    accumulator = (accumulator << 5) | (accumulator >> 27);
    return accumulator ^ value;
}

/* The inner interrupt: higher priority than the outer one, so it preempts it. */
static void inner_isr(UArg argument)
{
    (void)argument;
    REG(EVTCLR0) = 1u << EVENT_INNER;
    inner_taken = inner_taken + 1u;
}

/* The outer interrupt: it posts a Swi and a semaphore, then raises the inner event and waits for
 * it, which can only complete if the dispatcher re-enabled interrupts around this routine. */
static void outer_isr(UArg argument)
{
    unsigned int before = inner_taken;
    unsigned int spins;

    (void)argument;
    REG(EVTCLR0) = 1u << EVENT_OUTER;
    outer_taken = outer_taken + 1u;
    Swi_post(deferred);
    Semaphore_post(from_interrupt);
    if (event_phase != 0u) {
        /* Event_post from interrupt context: the other half of what the event task waits for. */
        Event_post(both_halves, Event_Id_01);
    }

    REG(EVTSET0) = 1u << EVENT_INNER;
    for (spins = 0; spins < SPIN_LIMIT; spins++) {
        if (inner_taken != before) {
            nested_observed = nested_observed + 1u;
            return;
        }
    }
    nesting_failures = nesting_failures + 1u;
}

static void deferred_swi(UArg a, UArg b)
{
    (void)a;
    (void)b;
    swi_ran = swi_ran + 1u;
}

/* A periodic Clock function, so timer 0's interrupt drives the kernel's own tick and the module
 * that hangs off it. It posts a semaphore, which is what makes the count deterministic. */
static void heartbeat_clock(UArg argument)
{
    (void)argument;
    clock_ticks = clock_ticks + 1u;
    Semaphore_post(ticker);
}

/* One task pulls letters out of a mailbox another puts in, so the kernel's own queues and their
 * copies are in the loop as well. */
static void mailbox_task(UArg a, UArg b)
{
    unsigned int letter;

    (void)a;
    (void)b;
    while (letters < LETTERS) {
        if (Mailbox_pend(letterbox, &letter, BIOS_WAIT_FOREVER)) {
            if (letter == letters * 7u + 1u) {
                letters = letters + 1u;
            }
        }
    }
}

/* A task that only runs when both halves of an event have arrived - one from a task, one from
 * interrupt context - which is Event's and-mask contract (SYS/BIOS Event module). */
static void event_task(UArg a, UArg b)
{
    unsigned int index;

    (void)a;
    (void)b;
    for (index = 0; index < EVENTS; index++) {
        Event_pend(both_halves, Event_Id_00 + Event_Id_01, 0, BIOS_WAIT_FOREVER);
        events_taken = events_taken + 1u;
    }
}

/* The middle of the chain: one item at a time, handed on to the lowest priority task. */
static void consumer_task(UArg a, UArg b)
{
    unsigned int index;

    (void)a;
    (void)b;
    for (index = 0; index < ITEMS; index++) {
        Semaphore_pend(work, BIOS_WAIT_FOREVER);
        consumed = consumed + 1u;
        Semaphore_post(relay);
    }
}

static void relay_task(UArg a, UArg b)
{
    unsigned int index;

    (void)a;
    (void)b;
    for (index = 0; index < ITEMS; index++) {
        Semaphore_pend(relay, BIOS_WAIT_FOREVER);
        relayed = relayed + 1u;
    }
    Semaphore_post(finished);
}

/* A task woken only from interrupt context, to prove the dispatcher's task hand-off. */
static void interrupt_task(UArg a, UArg b)
{
    unsigned int index;

    (void)a;
    (void)b;
    for (index = 0; index < ROUNDS; index++) {
        Semaphore_pend(from_interrupt, BIOS_WAIT_FOREVER);
        woken_by_interrupt = woken_by_interrupt + 1u;
    }
}

static void producer_task(UArg a, UArg b)
{
    unsigned int index;
    unsigned int checksum = 0;
    Error_Block error_block;

    (void)a;
    (void)b;
    Error_init(&error_block);

    /* Hand ITEMS items down the chain, then wait for the far end to finish. */
    for (index = 0; index < ITEMS; index++) {
        Semaphore_post(work);
    }
    Semaphore_pend(finished, BIOS_WAIT_FOREVER);

    /* Then run the interrupt chain a fixed number of times, waiting for the Swi each round. */
    for (index = 0; index < ROUNDS; index++) {
        unsigned int before = swi_ran;
        unsigned int spins;

        REG(EVTSET0) = 1u << EVENT_OUTER;
        for (spins = 0; spins < SPIN_LIMIT && swi_ran == before; spins++) {
            Task_yield();
        }
    }
    /* An interrupt raised while its enable bit is clear still latches, and is taken as soon as
     * the bit comes back: the flag register is edge-triggered by the routed line, and the enable
     * only gates delivery (SPRU871K section 7.4.2). */
    {
        unsigned int before = outer_taken;
        unsigned int spins;

        Hwi_disableInterrupt(CPU_INT_OUTER);
        REG(EVTSET0) = 1u << EVENT_OUTER;
        for (spins = 0; spins < 2000u; spins++) {
        }
        if (outer_taken == before) {
            Hwi_enableInterrupt(CPU_INT_OUTER);
            for (spins = 0; spins < SPIN_LIMIT && outer_taken == before; spins++) {
                Task_yield();
            }
            if (outer_taken != before) {
                masked_delivery = 1u;
            }
        } else {
            Hwi_enableInterrupt(CPU_INT_OUTER);
        }
    }

    /* Both halves of an event, one posted here and one from interrupt context, with the task
     * waiting on an and-mask so neither half alone can release it. */
    event_phase = 1u;
    for (index = 0; index < EVENTS; index++) {
        unsigned int before = events_taken;
        unsigned int spins;

        Event_post(both_halves, Event_Id_00);
        REG(EVTSET0) = 1u << EVENT_OUTER;
        for (spins = 0; spins < SPIN_LIMIT && events_taken == before; spins++) {
            Task_yield();
        }
    }
    event_phase = 0u;

    /* Nested Hwi_disable: the inner restore must not let interrupts back in, because the outer
     * disable is still in force; only undoing that one delivers the pending interrupt. */
    {
        unsigned int before = outer_taken;
        unsigned int spins;
        UInt outer_key;
        UInt inner_key;

        outer_key = Hwi_disable();
        inner_key = Hwi_disable();
        REG(EVTSET0) = 1u << EVENT_OUTER;
        for (spins = 0; spins < 2000u; spins++) {
        }
        Hwi_restore(inner_key);
        for (spins = 0; spins < 2000u; spins++) {
        }
        if (outer_taken == before) {
            nested_disable_held = 1u;
        }
        Hwi_restore(outer_key);
        for (spins = 0; spins < SPIN_LIMIT && outer_taken == before; spins++) {
        }
        if (outer_taken != before) {
            nested_disable_released = 1u;
        }
    }

    /* The kernel's own tick, and a sleep that only the tick can end. */
    {
        Clock_Params clock_params;

        Clock_Params_init(&clock_params);
        clock_params.period = 2;      /* periodic, not one-shot */
        clock_params.startFlag = TRUE;
        heartbeat = Clock_create(heartbeat_clock, 2, &clock_params, &error_block);
    }
    for (index = 0; index < TICKS; index++) {
        Semaphore_pend(ticker, BIOS_WAIT_FOREVER);
    }
    Clock_stop(heartbeat);
    for (index = 0; index < SLEEPS; index++) {
        Task_sleep(2);
        slept = slept + 1u;
    }

    /* Letters through a mailbox, which the reader checks in order. */
    for (index = 0; index < LETTERS; index++) {
        unsigned int letter = index * 7u + 1u;

        Mailbox_post(letterbox, &letter, BIOS_WAIT_FOREVER);
    }
    for (index = 0; index < SPIN_LIMIT && letters < LETTERS; index++) {
        Task_yield();
    }

    /* The interrupt-woken task has to have caught up before the counts are read. */
    for (index = 0; index < SPIN_LIMIT && woken_by_interrupt < ROUNDS; index++) {
        Task_yield();
    }

    checksum = fold(checksum, consumed);
    checksum = fold(checksum, relayed);
    checksum = fold(checksum, outer_taken);
    checksum = fold(checksum, inner_taken);
    checksum = fold(checksum, nested_observed);
    checksum = fold(checksum, swi_ran);
    checksum = fold(checksum, woken_by_interrupt);
    checksum = fold(checksum, nesting_failures);
    checksum = fold(checksum, clock_ticks);
    checksum = fold(checksum, slept);
    checksum = fold(checksum, letters);
    checksum = fold(checksum, masked_delivery);
    checksum = fold(checksum, events_taken);
    checksum = fold(checksum, nested_disable_held);
    checksum = fold(checksum, nested_disable_released);
    checksum = fold(checksum, REG(EVTFLAG0) & ((1u << EVENT_OUTER) | (1u << EVENT_INNER)));

    printf("bios kernel consumed %u relayed %u outer %u inner %u nested %u swi %u woken %u "
           "failures %u ticks %u slept %u letters %u masked %u events %u held %u released %u "
           "checksum %u\n",
           consumed, relayed, outer_taken, inner_taken, nested_observed, swi_ran,
           woken_by_interrupt, nesting_failures, clock_ticks, slept, letters, masked_delivery,
           events_taken, nested_disable_held, nested_disable_released, checksum);
    fflush(stdout);
    BIOS_exit(0);
}

int main(void)
{
    Task_Params task_params;
    Hwi_Params hwi_params;
    Error_Block error;

    Error_init(&error);

    work = Semaphore_create(0, NULL, &error);
    relay = Semaphore_create(0, NULL, &error);
    finished = Semaphore_create(0, NULL, &error);
    from_interrupt = Semaphore_create(0, NULL, &error);
    ticker = Semaphore_create(0, NULL, &error);
    deferred = Swi_create(deferred_swi, NULL, &error);

    Hwi_Params_init(&hwi_params);
    hwi_params.eventId = EVENT_OUTER;
    hwi_params.enableInt = TRUE;
    Hwi_create(CPU_INT_OUTER, outer_isr, &hwi_params, &error);

    Hwi_Params_init(&hwi_params);
    hwi_params.eventId = EVENT_INNER;
    hwi_params.enableInt = TRUE;
    Hwi_create(CPU_INT_INNER, inner_isr, &hwi_params, &error);

    letterbox = Mailbox_create(sizeof(unsigned int), LETTERS, NULL, &error);
    both_halves = Event_create(NULL, &error);

    Task_Params_init(&task_params);
    task_params.priority = 1;
    Task_create(relay_task, &task_params, &error);
    task_params.priority = 2;
    Task_create(consumer_task, &task_params, &error);
    task_params.priority = 4;
    Task_create(producer_task, &task_params, &error);
    /* Above the producer, so a semaphore posted from interrupt context preempts it: the point is
     * to see the dispatcher hand control to a task, not to poll for it later. */
    task_params.priority = 5;
    Task_create(interrupt_task, &task_params, &error);
    task_params.priority = 6;
    Task_create(mailbox_task, &task_params, &error);
    /* Above the producer as well, so an event completed from interrupt context preempts it. */
    task_params.priority = 7;
    Task_create(event_task, &task_params, &error);

    BIOS_start();
    return 0;
}
