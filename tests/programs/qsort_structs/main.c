#include <stdio.h>
#include <stdlib.h>
/* qsort over structures with a comparator called through a function pointer. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

struct record {
    int key;
    short tag;
    short flags;
};

static struct record records[64];

static int by_key(const void *left, const void *right)
{
    const struct record *a = (const struct record *)left;
    const struct record *b = (const struct record *)right;
    if (a->key != b->key) {
        return a->key < b->key ? -1 : 1;
    }
    return (int)a->tag - (int)b->tag;
}

static int by_tag_then_key(const void *left, const void *right)
{
    const struct record *a = (const struct record *)left;
    const struct record *b = (const struct record *)right;
    if (a->tag != b->tag) {
        return (int)a->tag - (int)b->tag;
    }
    return a->key < b->key ? -1 : (a->key > b->key ? 1 : 0);
}

static int compute(void)
{
    int total = 0;
    int i;

    for (i = 0; i < 64; i++) {
        records[i].key = (int)(rng_next() >> 12) - 262144;
        records[i].tag = (short)(rng_next() % 7u);
        records[i].flags = (short)(i & 3);
    }
    qsort(records, 64, sizeof records[0], by_key);
    for (i = 0; i < 64; i++) {
        total += records[i].key >> (i & 7);
        total ^= (int)records[i].tag << (i & 5);
    }
    qsort(records, 64, sizeof records[0], by_tag_then_key);
    for (i = 0; i < 64; i++) {
        total += records[i].key ^ (i * 3);
        total = (total << 1) ^ (total >> 31);
    }
    return total;
}

int main(void)
{
    printf("%s=%d\n", "qsort_structs", compute());
    return 0;
}
