;* Interrupt service table for the C64x+: sixteen entries of eight words each, 32-byte
;* aligned and assembled without compact instructions. INT5 is the software event this test
;* re-arms from inside its own service routine.
        .global ist_table
        .global intc_setup
        .global intc_enable
        .global clear_ifr
        .ref    event_isr
        .ref    _c_int00

        .sect ".vecs"
        .nocmp
        .align 1024
ist_table:
        .align 32
vec0:
        B   .S2     _c_int00
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
        B   .S2     event_isr
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
        B   .S2     vec7
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
;* void intc_setup(unsigned istp, unsigned ier): point ISTP at the table and set IER.
        .align 32
intc_setup:
        MVC     .S2X    A4,ISTP
        MVC     .S2X    B4,IER
        B       .S2     B3
        NOP     5

;* void intc_enable(void): set GIE.
        .align 32
intc_enable:
        MVC     .S2     CSR,B0
        NOP
        OR      .L2     1,B0,B0
        MVC     .S2     B0,CSR
        B       .S2     B3
        NOP     5

;* void clear_ifr(unsigned mask): clear the named CPU interrupt flags through ICR.
        .align 32
clear_ifr:
        MVC     .S2X    A4,ICR
        B       .S2     B3
        NOP     5
