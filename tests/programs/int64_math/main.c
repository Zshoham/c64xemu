#include <stdio.h>
/* 64-bit integer arithmetic: exercises the RTS long-long helpers. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int compute(void)
{
    long long acc = 0;
    unsigned long long uacc = 1u;
    int i;

    for (i = 0; i < 24; i++) {
        long long a = (long long)(int)rng_next();
        unsigned long long b = ((unsigned long long)rng_next() << 21) | rng_next();

        acc += a * (long long)(i + 1);
        acc ^= (long long)(b >> 13);
        acc -= (long long)(b % 1000003ull);
        uacc = uacc * 6364136223846793005ull + (unsigned long long)a;
        acc += (long long)(uacc >> 40);
        if (a != 0) {
            acc += (long long)(b / (unsigned long long)(a < 0 ? -a : a));
        }
    }
    return (int)(acc ^ (acc >> 32));
}

int main(void)
{
    printf("%s=%d\n", "int64_math", compute());
    return 0;
}
