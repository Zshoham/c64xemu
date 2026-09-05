#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* RTS heap: malloc, realloc, free and qsort over the allocated blocks. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int compare_int(const void *left, const void *right)
{
    int a = *(const int *)left;
    int b = *(const int *)right;
    return (a > b) - (a < b);
}

static int compute(void)
{
    int *blocks[16];
    int sizes[16];
    int i, j, total = 0;

    for (i = 0; i < 16; i++) {
        sizes[i] = 1 + (int)(rng_next() % 24u);
        blocks[i] = (int *)malloc((size_t)sizes[i] * sizeof(int));
        if (blocks[i] == NULL) {
            return -1;
        }
        for (j = 0; j < sizes[i]; j++) {
            blocks[i][j] = (int)rng_next() >> 8;
        }
    }
    for (i = 0; i < 16; i += 2) {
        free(blocks[i]);
        blocks[i] = NULL;
    }
    for (i = 1; i < 16; i += 2) {
        int grown = sizes[i] + 8;
        int *moved = (int *)realloc(blocks[i], (size_t)grown * sizeof(int));
        if (moved == NULL) {
            return -2;
        }
        for (j = sizes[i]; j < grown; j++) {
            moved[j] = (int)rng_next() >> 9;
        }
        blocks[i] = moved;
        sizes[i] = grown;
        qsort(moved, (size_t)grown, sizeof(int), compare_int);
        for (j = 0; j < grown; j++) {
            total += moved[j] ^ (j * 7);
            total = (total << 1) ^ (total >> 31);
        }
    }
    for (i = 1; i < 16; i += 2) {
        char *text = (char *)malloc(32);
        if (text == NULL) {
            return -3;
        }
        memset(text, 'a' + i, 31);
        text[31] = 0;
        total += (int)strlen(text) * (int)(unsigned char)text[0];
        free(text);
        free(blocks[i]);
    }
    return total;
}

int main(void)
{
    printf("%s=%d\n", "heap_alloc", compute());
    return 0;
}
