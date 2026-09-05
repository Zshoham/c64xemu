* The cross path sharing rules (SPRUFE8B section 2.7 "up to two units per data path, per
* execute packet, can read a source operand from the opposite register file provided that each
* unit is reading the same operand"), which no compiled program can violate: cl6x refuses to
* assemble an invalid execute packet, so the two illegal packets here are hand-assembled words.
*
* Three packets are executed, each verified against TI's own disassembler:
*
*   MV.S1X B15,A0 || ADDAW.D1X B15,32,A1 || LDDW.D2T2 *+B15[4],B29:B28
*           - legal: two A-side units read the same B register over 1X. This is TI's own
*             packet, from ti_sysbios_family_c64p_Hwi_dispatchAlways. ADDAW's long form
*             encodes `s` as the functional unit rather than the data path (adda_long in the
*             binutils opcode table encodes ENC(s, fu) where the long loads and stores encode
*             ENC(s, data_fu)), so it runs on .D1 and reads its B15 base across the cross path.
*   MV.S1X B14,A0 || ADDAW.D1X B15,32,A1
*           - illegal: the 1X path carries two different B registers in one packet.
*   MV.S1X B15,A0 || ADDAW.D1X B15,32,A1 || MV.L1X B15,A2
*           - illegal: three functional units read the same 1X operand, which the manual's
*             worked example calls out separately from the two-different-operands case.
*
* Each illegal packet must raise an internal exception that reports IERR.RCX with EFR.IXF and
* an NTSR carrying HWE, and the handler resumes at the packet after the offending one. The
* checksum folds the legal packet's three results - the base copy, the word-scaled long ADDA
* and the loaded doubleword - and then every exception observation, so a missing exception, a
* wrong cause or a wrongly rejected legal packet all change it.
        .nocmp
        .global main
        .sect ".text"

FOLD    .macro  src
        SHL     .S1     A4,5,A6
        SHRU    .S1     A4,27,A7
        OR      .L1     A6,A7,A4
        XOR     .L1X    A4,src,A4
        .endm

* The same fold for an observation that already sits in the A file.
FOLDA   .macro  src
        SHL     .S1     A4,5,A6
        SHRU    .S1     A4,27,A7
        OR      .L1     A6,A7,A4
        XOR     .L1     A4,src,A4
        .endm
main:
        MV      .L2     B3,B10                  ; caller's return address
        MV      .L2     B15,B11                 ; caller's stack pointer

        MVKL    .S2     ist,B4
        MVKH    .S2     ist,B4
        MVC     .S2     B4,ISTP

        MVC     .S2     TSR,B4
        NOP     1
        SET     .S2     B4,2,2,B4               ; TSR.GEE
        MVC     .S2     B4,TSR
        NOP     4

        MVK     .L1     0,A4                    ; checksum
        MVK     .L1     0,A5                    ; exceptions taken

* Known data 32 bytes into the buffer, which is where the legal packet's LDDW reads.
        MVKL    .S2     buffer,B15
        MVKH    .S2     buffer,B15
        MVKL    .S2     0x01234567,B4
        MVKH    .S2     0x01234567,B4
        MVKL    .S2     0x89abcdef,B5
        MVKH    .S2     0x89abcdef,B5
        STW     .D2T2   B4,*+B15[8]
        STW     .D2T2   B5,*+B15[9]

* The legal packet, then its three results.
        .word   0x003c16a1              ; MV.S1X    B15,A0
        .word   0x108020fd              ; ADDAW.D1X B15,32,A1
        .word   0x0e3c83e6              ; LDDW.D2T2 *+B15[4],B29:B28
        NOP     4
        SUB     .L1X    A0,B15,A2               ; the base copy: zero
        SUB     .L1     A1,A0,A3                ; the long ADDA: 32 words, so 128 bytes
        FOLDA   A2
        FOLDA   A3
        FOLD    B28
        FOLD    B29

* Two different B registers on 1X.
        .word   0x003816a1              ; MV.S1X    B14,A0
        .word   0x108020fc              ; ADDAW.D1X B15,32,A1
after1:
* Three units reading the same 1X operand.
        .word   0x003c16a1              ; MV.S1X    B15,A0
        .word   0x108020fd              ; ADDAW.D1X B15,32,A1
        .word   0x013c1fd8              ; MV.L1X    B15,A2
after2:
        MVC     .S2     EFR,B4
        MVC     .S2     IERR,B5
        MVC     .S2     TSR,B6
        NOP     1
        FOLD    B4
        FOLD    B5
        FOLD    B6
        MV      .L2X    A5,B4
        FOLD    B4
        MV      .L2     B11,B15                 ; restore the caller's stack pointer
        B       .S2     B10
        NOP     5

* The exception vector. It folds the cause, counts the exception and resumes at the packet
* after the offending one: NRP names the offending packet itself, and stepping over a packet
* of hand-assembled words means naming its end rather than adding a fixed four bytes.
exception_entry:
        MVC     .S2     EFR,B4
        MVC     .S2     NTSR,B5
        MVC     .S2     IERR,B6
        NOP     1
        FOLD    B4
        FOLD    B5
        FOLD    B6
        ADDK    .S1     1,A5
        CMPEQ   .L2X    A5,1,B0
        NOP     1
 [B0]   MVKL    .S2     after1,B7
 [B0]   MVKH    .S2     after1,B7
 [!B0]  MVKL    .S2     after2,B7
 [!B0]  MVKH    .S2     after2,B7
        MVC     .S2     B7,NRP
        MVC     .S2     B4,ECR                  ; acknowledge exactly the flags seen
        MVK     .L2     0,B7
        MVC     .S2     B7,IERR
        NOP     1
        B       .S2     NRP
        NOP     5

* Nothing here should nest, so landing on the table's RESET entry poisons the result.
reset_entry:
        MVKL    .S1     0xdead0002,A4
        MVKH    .S1     0xdead0002,A4
        MV      .L2     B11,B15
        B       .S2     B10
        NOP     5

        .align  1024
ist:
        B       .S1     reset_entry
        NOP     5
        .align  32
        B       .S1     exception_entry
        NOP     5
        .align  32

        .bss    buffer,128,8
