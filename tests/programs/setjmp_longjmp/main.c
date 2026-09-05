#include <stdio.h>
#include <setjmp.h>
/* setjmp and longjmp: the RTS saves and restores the callee-saved registers. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static jmp_buf env;

static int deep(int depth, int accumulator)
{
    int local = accumulator + depth * 3;
    if (depth == 0) {
        longjmp(env, local | 1);
    }
    local += deep(depth - 1, local ^ (int)rng_next() >> 12);
    return local;
}

static int compute(void)
{
    int total = 0;
    int i;

    for (i = 1; i <= 6; i++) {
        int landed = setjmp(env);
        if (landed == 0) {
            total += deep(i, i * 7);
        } else {
            total ^= landed;
        }
        total = (total << 1) ^ (total >> 31);
    }
    return total;
}

int main(void)
{
    printf("%s=%d\n", "setjmp_longjmp", compute());
    return 0;
}
