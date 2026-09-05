/* DSPLIB generic FIR filter: nested multiply-accumulate loops with saturation. */
/* Prototype declared locally: some vendor headers misspell the symbol. */
extern void DSP_fir_gen_cn(const short *x, const short *h, short *r, int nh, int nr);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

#define NH 8
#define NR 32

static short x[NR + NH - 1];
static short h[NH];
static short r[NR];

int main(void)
{
    int i;
    unsigned int sum = 0u;

    for (i = 0; i < NR + NH - 1; i++) {
        x[i] = (short)((int)(rng_next() >> 21) - 1024);
    }
    for (i = 0; i < NH; i++) {
        h[i] = (short)((int)(rng_next() >> 24) - 128);
    }
    for (i = 0; i < NR; i++) {
        r[i] = 0;
    }
    DSP_fir_gen_cn(x, h, r, NH, NR);
    for (i = 0; i < NR; i++) {
        sum = sum * 37u + (unsigned int)(unsigned short)r[i];
    }
    return (int)sum;
}
