#include <stdio.h>
#include <stdarg.h>
/* Variadic calls mixing integer, long long and double arguments. */
static unsigned int rng_state = 12345u;

static unsigned int rng_next(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int sum_ints(int count, ...)
{
    va_list ap;
    int i, total = 0;
    va_start(ap, count);
    for (i = 0; i < count; i++) {
        total += va_arg(ap, int);
    }
    va_end(ap);
    return total;
}

static long long sum_mixed(int count, ...)
{
    va_list ap;
    int i;
    long long total = 0;
    va_start(ap, count);
    for (i = 0; i < count; i++) {
        total += va_arg(ap, long long);
        total ^= (long long)(va_arg(ap, double) * 4.0);
        total += va_arg(ap, int);
    }
    va_end(ap);
    return total;
}

static int compute(void)
{
    int total = 0;
    int i;

    /* Each argument is hoisted into a local first: the order in which a compiler
       evaluates call arguments is unspecified. */
    for (i = 0; i < 8; i++) {
        int first = (int)(rng_next() >> 8);
        int second = (int)(rng_next() >> 9);
        int third = (int)rng_next() >> 4;
        total += sum_ints(5, first, second, i, -i, 7);
        total ^= sum_ints(1, third);
    }
    for (i = 0; i < 4; i++) {
        long long wide = (long long)(int)rng_next();
        int narrow = (int)(rng_next() >> 12);
        long long value = sum_mixed(2, wide, 0.25, narrow, (long long)i, 2.5, i * 3);
        total += (int)(value >> 7) ^ (int)value;
    }
    return total;
}

int main(void)
{
    printf("%s=%d\n", "varargs_calls", compute());
    return 0;
}
