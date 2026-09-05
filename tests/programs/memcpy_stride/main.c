/* Memory movement: strided copies, reversal and byte access over .bss buffers. */

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static unsigned char src[256];
static unsigned char dst[256];

int main(void)
{
    unsigned int sum = 0u;
    int i;

    for (i = 0; i < 256; i++) {
        src[i] = (unsigned char)(rng_next() >> 19);
        dst[i] = 0u;
    }
    for (i = 0; i < 256; i += 2) {
        dst[i] = src[255 - i];
    }
    for (i = 1; i < 256; i += 2) {
        dst[i] = (unsigned char)(src[i] ^ src[i - 1]);
    }
    for (i = 0; i < 256; i++) {
        sum = sum * 31u + dst[i];
    }
    return (int)sum;
}
