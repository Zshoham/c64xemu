/* DSPLIB LMS adaptive filter update. */
extern int DSP_firlms2_cn(short *h, const short *x, short b, int nh);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short h[32];
static short x[64];

int main(void)
{
    int i, total = 0;

    for (i = 0; i < 32; i++) {
        h[i] = (short)((int)(rng_next() >> 24) - 128);
    }
    for (i = 0; i < 64; i++) {
        x[i] = (short)((int)(rng_next() >> 23) - 256);
    }
    for (i = 0; i < 8; i++) {
        total += DSP_firlms2_cn(h, x + i, (short)(total & 0x7f), 32);
        total ^= (int)h[i] << (i & 3);
    }
    return total;
}
