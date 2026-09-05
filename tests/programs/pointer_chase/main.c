/* Linked list built in a static arena: insert, reverse, sort and walk. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

struct node {
    struct node *next;
    int key;
    int payload;
};

static struct node arena[96];

static int compute(void)
{
    struct node *head = 0;
    struct node *walk;
    struct node *previous;
    int i, total = 0;

    for (i = 0; i < 96; i++) {
        arena[i].key = (int)(rng_next() >> 17) - 8192;
        arena[i].payload = (int)(rng_next() >> 13);
        arena[i].next = head;
        head = &arena[i];
    }
    /* Reverse in place. */
    previous = 0;
    walk = head;
    while (walk != 0) {
        struct node *next = walk->next;
        walk->next = previous;
        previous = walk;
        walk = next;
    }
    head = previous;
    /* Insertion sort by key, relinking nodes rather than moving them. */
    {
        struct node *sorted = 0;
        walk = head;
        while (walk != 0) {
            struct node *next = walk->next;
            struct node **slot = &sorted;
            while (*slot != 0 && (*slot)->key < walk->key) {
                slot = &(*slot)->next;
            }
            walk->next = *slot;
            *slot = walk;
            walk = next;
        }
        head = sorted;
    }
    i = 0;
    for (walk = head; walk != 0; walk = walk->next) {
        total += walk->key * (i + 1);
        total ^= walk->payload >> (i & 7);
        total = (total << 1) ^ (total >> 31);
        i++;
    }
    return total ^ i;
}

int main(void)
{
    return compute();
}
