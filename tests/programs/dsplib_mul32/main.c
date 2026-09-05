/* DSPLIB 32x32 multiply keeping the upper 32 bits of the 64-bit product. */
/* Prototype declared locally: some vendor headers misspell the symbol. */
extern void DSP_mul32_cn(const int *x, const int *y, int *r, int nx);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int x[32];
static int y[32];
static int r[32];

int main(void)
{
    int i;
    unsigned int sum = 0u;

    for (i = 0; i < 32; i++) {
        x[i] = (int)rng_next();
        y[i] = (int)rng_next();
        r[i] = 0;
    }
    DSP_mul32_cn(x, y, r, 32);
    for (i = 0; i < 32; i++) {
        sum = sum * 33u + (unsigned int)r[i];
    }
    return (int)sum;
}
