#include <stdio.h>
/* Signed and unsigned division and modulo across 32-bit and 64-bit widths. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int compute(void)
{
    int total = 0;
    int i;

    for (i = 0; i < 40; i++) {
        int a = (int)rng_next();
        int b = (int)(rng_next() % 65535u) + 1;
        unsigned int ua = rng_next();
        unsigned int ub = (rng_next() % 4093u) + 1u;
        long long la = ((long long)(int)rng_next() << 20) | (long long)rng_next();
        long long lb = (long long)((int)(rng_next() % 999983u) + 1);
        unsigned long long ula = ((unsigned long long)rng_next() << 24) | rng_next();
        unsigned long long ulb = (unsigned long long)((rng_next() % 65521u) + 1u);

        total += a / b;
        total ^= a % b;
        total += (int)(ua / ub);
        total ^= (int)(ua % ub);
        total += (int)(la / lb);
        total ^= (int)(la % lb);
        total += (int)(ula / ulb);
        total ^= (int)(ula % ulb);
        total += (int)(la / -7LL) + (int)(-la % 11LL);
        total = (total << 1) ^ (total >> 31);
    }
    return total;
}

int main(void)
{
    printf("%s=%d\n", "int_divmod", compute());
    return 0;
}
