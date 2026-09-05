/* DSPLIB maximum-index search and 32-bit vector add. */
extern int DSP_maxidx_cn(const short *x, int nx);
extern void DSP_add32_cn(const int *x, const int *y, int *r, int nx);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short s[128];
static int a[64];
static int b[64];
static int r[64];

int main(void)
{
    int i, total = 0;

    for (i = 0; i < 128; i++) {
        s[i] = (short)((int)(rng_next() >> 17) - 16384);
    }
    for (i = 0; i < 64; i++) {
        a[i] = (int)rng_next() >> 2;
        b[i] = (int)rng_next() >> 3;
    }
    total = DSP_maxidx_cn(s, 128);
    DSP_add32_cn(a, b, r, 64);
    for (i = 0; i < 64; i++) {
        total += r[i] >> (i & 7);
        total ^= r[i];
    }
    return total;
}
