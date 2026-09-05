* Exception delivery on the C64x+ (SPRU732J chapter 6), which no compiled program can reach:
* cl6x never emits SWE or SWENR and never installs an interrupt service table.
*
* Three sources are exercised with exceptions globally enabled by TSR.GEE:
*
*   SWE     - a supervisor service request. Sets EFR.SXF, saves TSR in NTSR with HWE clear,
*             enters at the NMI/exception vector (ISTP + 0x20) and leaves NRP naming the
*             execute packet after the SWE, so `B NRP` resumes there.
*   SWENR   - the same, except that the entry point is REP, so User code cannot choose where
*             it lands in the supervisor (SPRU732J 8.4).
*   illegal - an internal exception. Sets EFR.IXF and IERR.OPX, saves TSR in NTSR with HWE
*     opcode  set, and leaves NRP naming the offending packet itself, which the handler must
*             step over before returning. The word used is 0x56454314, the illegal opcode
*             Linux/c6x installs as its breakpoint (arch/c6x/include/uapi/asm/ptrace.h).
*
* On entry to any of them TSR becomes GEE alone plus EXC: GIE, SGIE, XEN, INT and SPLX are
* cleared and CXM returns to Supervisor (Table 6-3). EFR flags are sticky and are cleared by
* writing ones to ECR; IERR is a plain read/write register, which is how a handler walks its
* causes and writes back the ones it has not dealt with.
*
* The checksum folds every observed EFR, NTSR and IERR, and finally the number of resumptions,
* so a wrong vector, a wrong NRP or a flag left standing all change the result.
        .nocmp
        .global main
        .sect ".text"

* Fold one observation into the checksum. A rotate rather than a shift, so that nothing
* observed early is shifted out before the end: with a plain shift the first exception the
* program takes would not be checked at all.
FOLD    .macro  src
        SHL     .S1     A4,5,A6
        SHRU    .S1     A4,27,A7
        OR      .L1     A6,A7,A4
        XOR     .L1X    A4,src,A4
        .endm

main:
        MV      .L2     B3,B10                  ; caller's return address

        MVKL    .S2     ist,B4
        MVKH    .S2     ist,B4
        MVC     .S2     B4,ISTP

        MVKL    .S2     rep_handler,B4
        MVKH    .S2     rep_handler,B4
        MVC     .S2     B4,REP

        MVC     .S2     TSR,B4
        NOP     1
        SET     .S2     B4,2,2,B4               ; TSR.GEE
        MVC     .S2     B4,TSR
        NOP     4

        MVK     .L1     0,A4                    ; checksum
        MVK     .L1     0,A5                    ; resumptions

* A supervisor service request: the vector is ISTP + 0x20 and the return lands on the packet
* after the SWE, so the ADDK runs exactly once.
        SWE
        ADDK    .S1     1,A5

* The same request through the restricted entry point.
        SWENR
        ADDK    .S1     1,A5

* An illegal opcode: NRP names this packet, so the handler steps past it.
        .word   0x56454314
        ADDK    .S1     1,A5

* Everything must be clean again: no flags, no cause, GEE still set and EXC gone.
        MVC     .S2     EFR,B4
        MVC     .S2     IERR,B5
        MVC     .S2     TSR,B6
        NOP     1
        FOLD    B4
        FOLD    B5
        FOLD    B6
        MV      .L2X    A5,B4
        FOLD    B4
        B       .S2     B10
        NOP     5

* The exception vector shared by NMI and every exception. It folds what it sees, steps NRP
* over an offending packet when the cause was internal, acknowledges the flags and returns.
exception_entry:
        MVC     .S2     EFR,B4
        MVC     .S2     NTSR,B5
        MVC     .S2     IERR,B6
        NOP     1
        FOLD    B4
        FOLD    B5
        FOLD    B6
        AND     .L2     B4,2,B0                 ; EFR.IXF
        NOP     1
 [B0]   MVC     .S2     NRP,B7
        NOP     1
 [B0]   ADDK    .S2     4,B7
        NOP     1
 [B0]   MVC     .S2     B7,NRP
        MVC     .S2     B4,ECR                  ; acknowledge exactly the flags seen
        MVK     .L2     0,B7
        MVC     .S2     B7,IERR
        NOP     1
        B       .S2     NRP
        NOP     5

* SWENR lands here through REP. The tag distinguishes this path from the vector above.
rep_handler:
        MVC     .S2     EFR,B4
        MVC     .S2     NTSR,B5
        NOP     1
        FOLD    B4
        FOLD    B5
        XOR     .L1     15,A4,A4
        MVC     .S2     B4,ECR
        NOP     1
        B       .S2     NRP
        NOP     5

* A nested exception vectors through the service table's RESET entry, which nothing here
* should reach: landing on it poisons the result rather than running away.
reset_entry:
        MVKL    .S1     0xdead0001,A4
        MVKH    .S1     0xdead0001,A4
        B       .S2     B10
        NOP     5

* The interrupt service table. Each fetch packet is 32 bytes and the table is 1 KB aligned,
* so ISTP can name it; the entries beyond the exception vector are never taken here.
        .align 1024
ist:
        B       .S1     reset_entry
        NOP     5
        .align 32
        B       .S1     exception_entry
        NOP     5
        .align 32
