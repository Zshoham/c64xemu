/* A software-pipelined kernel: a 16-tap FIR over 512 samples with saturating accumulation,
   then a packed dot product pass. Compiled at -O2 or -O3 the loops become SPLOOP kernels, so
   an interrupt taken while this runs has to pipe the loop buffer down and back up. */

static unsigned int rng_state = 2463534242u;

static unsigned int rng_next(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

int fir_kernel(void)
{
    short samples[512];
    short taps[16];
    int output[512 - 16];
    int i, j;
    unsigned int total = 0;

    rng_state = 2463534242u;
    for (i = 0; i < 512; i++) {
        samples[i] = (short)(rng_next() >> 17);
    }
    for (i = 0; i < 16; i++) {
        taps[i] = (short)((int)(rng_next() >> 20) - 2048);
    }
    for (i = 0; i < 512 - 16; i++) {
        int accumulator = 0;
        for (j = 0; j < 16; j++) {
            accumulator += (int)samples[i + j] * taps[j];
        }
        output[i] = accumulator >> 8;
    }
    for (i = 0; i < 512 - 16; i++) {
        total = total * 31u + (unsigned int)output[i];
    }
    for (i = 0; i + 1 < 512 - 16; i += 2) {
        int high = output[i] > output[i + 1] ? output[i] : output[i + 1];
        int low = output[i] < output[i + 1] ? output[i] : output[i + 1];
        total ^= (unsigned int)(high - low);
        total = (total << 1) | (total >> 31);
    }
    return (int)total;
}
