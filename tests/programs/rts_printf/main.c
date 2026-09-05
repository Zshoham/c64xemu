/* Full TI RTS program: _c_int00 startup, auto-initialisation and CIO printf. */
#include <stdio.h>

int main(void)
{
    int i;
    int sum = 0;

    for (i = 1; i <= 4; i++) {
        sum += i * i;
    }
    printf("hello from c64x+ sum=%d\n", sum);
    return 0;
}
