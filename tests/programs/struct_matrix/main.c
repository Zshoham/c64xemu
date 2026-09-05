#include <stdio.h>
/* Structure assignment, arrays of structs and a small matrix product. */
struct vec3 {
    int x;
    int y;
    int z;
};

static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static struct vec3 rotate(struct vec3 value, int shift)
{
    struct vec3 result;

    result.x = value.y + (value.z >> shift);
    result.y = value.z - (value.x >> shift);
    result.z = value.x ^ (value.y << shift);
    return result;
}

static int matrix[8][8];
static int other[8][8];
static int product[8][8];

static int compute(void)
{
    struct vec3 state;
    int i;
    int j;
    int k;
    int acc = 0;

    state.x = 1;
    state.y = 2;
    state.z = 3;
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
            matrix[i][j] = (int)(rng_next() >> 20) - 2048;
            other[i][j] = (int)(rng_next() >> 22) - 512;
        }
    }
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
            int sum = 0;

            for (k = 0; k < 8; k++) {
                sum += matrix[i][k] * other[k][j];
            }
            product[i][j] = sum;
        }
    }
    for (i = 0; i < 8; i++) {
        state = rotate(state, 1 + (i & 3));
        acc += product[i][(i * 3) & 7] + state.x - state.z;
    }
    return acc;
}

int main(void)
{
    printf("%s=%d\n", "struct_matrix", compute());
    return 0;
}
