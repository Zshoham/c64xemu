/* Bit manipulation: shifts, rotates, masks, byte swaps, bit counting. */

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static unsigned int rotate_left(unsigned int v, int n)
{
    return (v << n) | (v >> (32 - n));
}

static unsigned int byte_swap(unsigned int v)
{
    return ((v & 0x000000ffu) << 24) | ((v & 0x0000ff00u) << 8) |
           ((v & 0x00ff0000u) >> 8) | ((v & 0xff000000u) >> 24);
}

static int popcount(unsigned int v)
{
    int n = 0;

    while (v != 0u) {
        v &= v - 1u;
        n++;
    }
    return n;
}

int main(void)
{
    unsigned int acc = 0u;
    int i;

    for (i = 0; i < 48; i++) {
        unsigned int v = rng_next();

        acc ^= rotate_left(v, 1 + (i & 15));
        acc += byte_swap(v);
        acc -= (unsigned int)popcount(v);
        acc ^= (v & ~acc);
    }
    return (int)acc;
}
