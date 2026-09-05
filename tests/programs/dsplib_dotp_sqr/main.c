/* DSPLIB dot product and sum of squares in one pass. */
extern int DSP_dotp_sqr_cn(int G, const short *x, const short *y, int *restrict d, int nx);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[96];
static short y[96];

int main(void)
{
    int i, accumulator = 0, total = 0;

    for (i = 0; i < 96; i++) {
        x[i] = (short)((int)(rng_next() >> 22) - 512);
        y[i] = (short)((int)(rng_next() >> 22) - 512);
    }
    for (i = 0; i < 4; i++) {
        total += DSP_dotp_sqr_cn(total, x + i * 8, y + i * 8, &accumulator, 64);
        total ^= accumulator >> (i + 1);
    }
    return total;
}
