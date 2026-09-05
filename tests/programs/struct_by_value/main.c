/* Structures passed and returned by value, including ones larger than a register pair. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

struct small {
    int a;
    int b;
};

struct large {
    int values[6];
    short tag;
};

static struct small make_small(int seed)
{
    struct small s;
    s.a = seed * 3 + 1;
    s.b = seed ^ 0x5a5a;
    return s;
}

static struct large make_large(int seed)
{
    struct large l;
    int i;
    for (i = 0; i < 6; i++) {
        l.values[i] = seed * (i + 1) - (i << 3);
    }
    l.tag = (short)(seed & 0x7fff);
    return l;
}

static int fold_small(struct small s, int extra)
{
    return s.a * 2 + s.b - extra;
}

static int fold_large(struct large l, struct small s, int extra)
{
    int i, total = s.a - s.b + extra + (int)l.tag;
    for (i = 0; i < 6; i++) {
        total += l.values[i] >> (i & 3);
        total ^= l.values[i];
    }
    return total;
}

static int compute(void)
{
    int total = 0;
    int i;

    for (i = 0; i < 12; i++) {
        int seed = (int)(rng_next() >> 11);
        struct small s = make_small(seed);
        struct large l = make_large(seed ^ i);
        total += fold_small(s, i);
        total ^= fold_large(l, s, i * 5);
        total = (total << 1) ^ (total >> 31);
    }
    return total;
}

int main(void)
{
    return compute();
}
