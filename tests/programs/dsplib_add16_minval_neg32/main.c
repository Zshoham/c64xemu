/* DSPLIB 16-bit vector add, minimum search and 32-bit negate. */
extern void DSP_add16_cn(const short *x, const short *y, short *r, int nx);
extern short DSP_minval_cn(const short *x, int nx);
extern void DSP_neg32_cn(const int *x, int *r, int nx);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[128];
static short y[128];
static short sum[128];
static int wide[64];
static int negated[64];

int main(void)
{
    int i, total = 0;

    for (i = 0; i < 128; i++) {
        x[i] = (short)((int)(rng_next() >> 18) - 8192);
        y[i] = (short)((int)(rng_next() >> 18) - 8192);
    }
    for (i = 0; i < 64; i++) {
        wide[i] = (int)rng_next() >> 2;
    }
    DSP_add16_cn(x, y, sum, 128);
    DSP_neg32_cn(wide, negated, 64);
    total = (int)DSP_minval_cn(x, 128);
    for (i = 0; i < 128; i++) {
        total += (int)sum[i] >> (i & 3);
        total ^= (int)sum[i];
    }
    for (i = 0; i < 64; i++) {
        total += negated[i] >> (i & 7);
        total ^= negated[i];
    }
    return total;
}
