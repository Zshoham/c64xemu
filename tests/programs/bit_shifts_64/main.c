#include <stdio.h>
/* 64-bit shifts, rotates and bit counting, which use the RTS long-long helpers. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static unsigned int count_bits(unsigned long long value)
{
    unsigned int count = 0;
    while (value != 0ull) {
        count += (unsigned int)(value & 1ull);
        value >>= 1;
    }
    return count;
}

static int compute(void)
{
    int total = 0;
    int i;

    for (i = 0; i < 32; i++) {
        unsigned long long value = ((unsigned long long)rng_next() << 32) | rng_next();
        long long signed_value = (long long)value;
        unsigned int shift = (unsigned int)(rng_next() % 63u) + 1u;

        total += (int)(value >> shift);
        total ^= (int)(value << shift);
        total += (int)(signed_value >> shift);
        total ^= (int)((value >> shift) | (value << (64u - shift)));
        total += (int)count_bits(value);
        total ^= (int)(signed_value < 0 ? -signed_value : signed_value) >> 3;
        total = (total << 1) ^ (total >> 31);
    }
    return total;
}

int main(void)
{
    printf("%s=%d\n", "bit_shifts_64", compute());
    return 0;
}
