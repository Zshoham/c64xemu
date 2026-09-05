/* Dense switch: the compiler emits a jump table, a register branch and loads. */

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int classify(int v)
{
    switch (v & 7) {
    case 0: return v + 1;
    case 1: return v - 2;
    case 2: return v * 3;
    case 3: return v ^ 0x5a5a;
    case 4: return v >> 2;
    case 5: return v << 1;
    case 6: return -v;
    default: return v & 0xff;
    }
}

int main(void)
{
    int acc = 0;
    int i;

    for (i = 0; i < 64; i++) {
        acc += classify((int)(rng_next() >> 8));
    }
    return acc;
}
