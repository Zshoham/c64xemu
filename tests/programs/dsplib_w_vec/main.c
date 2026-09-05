/* DSPLIB weighted vector sum with Q15 rounding and saturation. */
/* Prototype declared locally: some vendor headers misspell the symbol. */
extern void DSP_w_vec_cn(const short *x, const short *y, short m, short *r, int nr);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[64];
static short y[64];
static short r[64];

int main(void)
{
    int i;
    unsigned int sum = 0u;

    for (i = 0; i < 64; i++) {
        x[i] = (short)((int)(rng_next() >> 17) - 16384);
        y[i] = (short)((int)(rng_next() >> 17) - 16384);
        r[i] = 0;
    }
    DSP_w_vec_cn(x, y, (short)9182, r, 64);
    for (i = 0; i < 64; i++) {
        sum = sum * 41u + (unsigned int)(unsigned short)r[i];
    }
    return (int)sum;
}
