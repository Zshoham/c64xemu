/* Integer multiplies: exercises the M unit (MPY family) and 32-bit products. */

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

int main(void)
{
    int acc = 0;
    int i;

    for (i = 0; i < 32; i++) {
        short a = (short)(rng_next() >> 16);
        short b = (short)(rng_next() >> 16);
        int wide = (int)(rng_next() >> 4);

        acc += (int)a * (int)b;
        acc ^= wide * 3;
        acc += (wide >> 5) * (int)a;
    }
    return acc;
}
