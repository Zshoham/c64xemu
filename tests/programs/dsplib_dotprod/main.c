/* DSPLIB dot product over two 16-bit vectors. */
/* Prototype declared locally: some vendor headers misspell the symbol. */
extern int DSP_dotprod_cn(const short *x, const short *y, int nx);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[64];
static short y[64];

int main(void)
{
    int i;

    for (i = 0; i < 64; i++) {
        x[i] = (short)((int)(rng_next() >> 21) - 1024);
        y[i] = (short)((int)(rng_next() >> 21) - 1024);
    }
    return DSP_dotprod_cn(x, y, 64);
}
