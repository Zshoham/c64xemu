/* DSPLIB lattice IIR filter. */
extern void DSP_iir_lat_cn(short *x, int nx, const short *k, int nk, int *b, short *r);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short x[96];
static short k[8];
static int b[9];
static short r[96];

int main(void)
{
    int i, total = 0;

    for (i = 0; i < 96; i++) {
        x[i] = (short)((int)(rng_next() >> 22) - 512);
    }
    for (i = 0; i < 8; i++) {
        k[i] = (short)((int)(rng_next() >> 24) - 128);
    }
    for (i = 0; i < 9; i++) {
        b[i] = 0;
    }
    DSP_iir_lat_cn(x, 96, k, 8, b, r);
    for (i = 0; i < 96; i++) {
        total += (int)r[i] * (i + 1);
        total ^= (int)r[i] << (i & 3);
    }
    for (i = 0; i < 9; i++) {
        total ^= b[i];
    }
    return total;
}
