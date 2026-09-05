/* DSPLIB natural-C reference kernel executed on the emulated CPU. */
/* Prototype declared locally: some vendor headers misspell the symbol. */
extern short DSP_maxval_cn(const short *x, int nx);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[96];

int main(void)
{
    int i;
    int acc = 0;

    for (i = 0; i < 96; i++) {
        x[i] = (short)(rng_next() >> 16);
    }
    for (i = 8; i <= 96; i += 8) {
        acc = acc * 3 + (int)DSP_maxval_cn(x, i);
    }
    return acc;
}
