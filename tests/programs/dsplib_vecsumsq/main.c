/* DSPLIB sum-of-squares: 16x16 multiplies accumulated in 32 bits. */
/* Prototype declared locally: some vendor headers misspell the symbol. */
extern int DSP_vecsumsq_cn(const short *x, int nx);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[64];

int main(void)
{
    int i;

    for (i = 0; i < 64; i++) {
        x[i] = (short)((int)(rng_next() >> 20) - 2048);
    }
    return DSP_vecsumsq_cn(x, 64);
}
