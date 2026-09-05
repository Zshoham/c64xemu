 ;* Interrupt service table for the C64x+: sixteen entries, each exactly eight words, so
 ;* every entry must be 32-byte aligned and assembled without compact instructions.
        .global ist_table
        .global intc_setup
        .global intc_enable
        .ref    idma_isr

        .sect ".vecs"
        .nocmp
        .align 1024
ist_table:
        .align 32
vec0:
        B   .S2     vec0
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec1:
        B   .S2     vec1
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec2:
        B   .S2     vec2
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec3:
        B   .S2     vec3
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec4:
        B   .S2     vec4
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec5:
        B   .S2     vec5
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec6:
        B   .S2     vec6
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec7:
        B   .S2     idma_isr
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec8:
        B   .S2     vec8
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec9:
        B   .S2     vec9
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec10:
        B   .S2     vec10
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec11:
        B   .S2     vec11
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec12:
        B   .S2     vec12
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec13:
        B   .S2     vec13
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec14:
        B   .S2     vec14
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP
        .align 32
vec15:
        B   .S2     vec15
        NOP 5
        NOP
        NOP
        NOP
        NOP
        NOP

        .sect ".text"
        .nocmp
 ;* void intc_setup(unsigned int istp, unsigned int ier): A4 is the table base, B4 the mask.
intc_setup:
        MV  .L2X    A4,B5
        MVC .S2     B5,ISTP
        MVC .S2     B4,IER
        BNOP .S2    B3,5

 ;* void intc_enable(void): set GIE so maskable interrupts are taken.
intc_enable:
        MVC .S2     CSR,B4
        OR  .L2     1,B4,B4
        MVC .S2     B4,CSR
        BNOP .S2    B3,5
