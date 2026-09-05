/* Reads the L2 word that only a boot configuration table can have written.

   The linker never places anything at 0x00980000, and the boot table copies no section there,
   so the value this returns can only come from the read/modify/write entries the bootloader
   applied before the application started (SPRUEC6G section 6.2.3). */
int main(void)
{
    volatile unsigned int *poked = (volatile unsigned int *)0x00980000u;

    return (int)*poked;
}
