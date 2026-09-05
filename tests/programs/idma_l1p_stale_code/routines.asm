* Two routines of exactly eight words each, differing only in the constant they return, so one
* is a plain block copy over the other. Each is 32-byte aligned, which is one L1P line, so the
* range invalidate in main.c names exactly the line that holds the routine being replaced.
        .nocmp
        .global first
        .global second
        .sect ".text"

        .align  32
first:
        MVK     .S1     273,A4
        B       .S2     B3
        NOP     5
        NOP
        NOP
        NOP
        NOP
        NOP

        .align  32
second:
        MVK     .S1     546,A4
        B       .S2     B3
        NOP     5
        NOP
        NOP
        NOP
        NOP
        NOP
