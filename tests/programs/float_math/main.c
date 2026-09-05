#include <stdio.h>
/* IEEE-754 float and double arithmetic through the RTS soft-float helpers. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int bits_of(double value)
{
    union { double d; unsigned int w[2]; } u;
    u.d = value;
    return (int)(u.w[0] ^ u.w[1]);
}

static int compute(void)
{
    double acc = 1.0;
    float facc = 0.5f;
    int mixed = 0;
    int i;

    for (i = 0; i < 20; i++) {
        int raw = (int)(rng_next() >> 8);
        double x = (double)raw / 65536.0;
        float y = (float)((int)(rng_next() >> 12)) * 0.125f;

        acc = acc * 1.000244140625 + x;
        acc = acc - (double)(int)acc / 3.0;
        facc = facc + y / 7.0f;
        if (acc > 1.0e12) {
            acc = acc / 1.0e6;
        }
        if (facc > 1.0e9f) {
            facc = facc / 1.0e3f;
        }
        mixed += (int)(x * 3.0) - (int)y;
    }
    return bits_of(acc) ^ bits_of((double)facc) ^ mixed;
}

int main(void)
{
    printf("%s=%d\n", "float_math", compute());
    return 0;
}
