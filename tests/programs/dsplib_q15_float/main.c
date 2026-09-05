#include <stdio.h>
/* DSPLIB Q15 to float and back: exercises the soft-float conversions. */
extern void DSP_q15tofl_cn(const short *q15, float *flt, int nx);
extern void DSP_fltoq15_cn(const float *flt, short *q15, short nx);

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static short q[64];
static float f[64];
static short back[64];

static int compute(void)
{
    int i, total = 0;

    for (i = 0; i < 64; i++) {
        q[i] = (short)((int)(rng_next() >> 17) - 16384);
    }
    DSP_q15tofl_cn(q, f, 64);
    DSP_fltoq15_cn(f, back, 64);
    for (i = 0; i < 64; i++) {
        total += (int)back[i] * (i + 1);
        total ^= (int)(f[i] * 32768.0f) << (i & 3);
    }
    return total;
}

int main(void)
{
    printf("%s=%d\n", "dsplib_q15_float", compute());
    return 0;
}
