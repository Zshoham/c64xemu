/* Simple counted loop: exercises compiler loop codegen and returns 55. */
int main(void)
{
    int sum = 0;
    int i;

    for (i = 1; i <= 10; i++) {
        sum += i;
    }
    return sum;
}
