#include <stdio.h>
/* Library string handling plus qsort with a comparison callback. */
#include <stdlib.h>
#include <string.h>

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int compare_ints(const void *left, const void *right)
{
    int a = *(const int *)left;
    int b = *(const int *)right;

    return (a > b) - (a < b);
}

static int values[48];
static char text[64];
static char copy[64];

static int compute(void)
{
    int i;
    unsigned int hash = 0u;

    for (i = 0; i < 48; i++) {
        values[i] = (int)(rng_next() >> 9) - 4194304;
    }
    qsort(values, 48, sizeof(values[0]), compare_ints);
    for (i = 0; i < 48; i++) {
        hash = hash * 31u + (unsigned int)values[i];
    }
    memset(text, 0, sizeof(text));
    for (i = 0; i < 40; i++) {
        text[i] = (char)('a' + (int)(rng_next() % 26u));
    }
    memcpy(copy, text, sizeof(copy));
    hash += (unsigned int)strlen(text) * 7u;
    hash ^= (unsigned int)(strcmp(copy, text) == 0 ? 5u : 9u);
    copy[20] = 'Z';
    hash += (unsigned int)(strcmp(copy, text) < 0 ? 11u : 22u);
    hash ^= (unsigned int)(strncmp(copy, text, 20u) == 0 ? 13u : 17u);
    hash += (unsigned int)(strchr(text, 'q') != 0 ? 3u : 6u);
    return (int)hash;
}

int main(void)
{
    printf("%s=%d\n", "string_qsort", compute());
    return 0;
}
