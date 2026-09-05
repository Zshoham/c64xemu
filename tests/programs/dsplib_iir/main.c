/* DSPLIB direct-form IIR filter over 16-bit data. */
extern void DSP_iir_cn(short *r1, const short *x, short *r2, const short *h2,
                       const short *h1, int nr);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[80];
static short r1[80];
static short r2[80];
static short h2[5];
static short h1[5];

int main(void)
{
    int i, total = 0;

    for (i = 0; i < 80; i++) {
        x[i] = (short)((int)(rng_next() >> 23) - 256);
    }
    for (i = 0; i < 5; i++) {
        h2[i] = (short)((int)(rng_next() >> 25) - 64);
    }
    /* The kernel indexes h1[1] through h1[4], so the array holds five entries. */
    for (i = 1; i < 5; i++) {
        h1[i] = (short)((int)(rng_next() >> 26) - 32);
    }
    DSP_iir_cn(r1, x, r2, h2, h1, 64);
    for (i = 0; i < 64; i++) {
        total += (int)r1[i] * (i + 1);
        total ^= (int)r2[i] << (i & 3);
    }
    return total;
}
