* The C6472 shared-memory controller's three atomic instructions, and a User-mode store, as
* C-callable functions: cl6x has no intrinsic for LL, SL or CMTL (SPRUEG5D section 3.1), and it
* never leaves Supervisor mode (SPRU732J chapter 8), which the protection case needs it to.
*
* EABI: arguments arrive in A4, B4, A6; the result goes back in A4; B3 is the return address.
* Only A0-A9 and B0-B9 are touched, which the convention leaves to the callee.
        .nocmp
        .global smc_ll
        .global smc_sl
        .global smc_cmtl
        .global smc_user_store
        .sect ".text"

* unsigned smc_ll(volatile unsigned *address): load-linked. The word comes back, and the monitor
* of the bank it lives in logs this megamodule, the address and LinkV (SPRUEG5D section 3.1.1).
smc_ll:
        MV      .L2X    A4,B4
        LL      .D2T2   *B4,B5
        NOP     4
        B       .S2     B3
        MV      .L1X    B5,A4
        NOP     4

* void smc_sl(volatile unsigned *address, unsigned value): store-linked. The datum goes to the
* monitor, never to memory (SPRUEG5D section 3.1.2).
smc_sl:
        MV      .L2X    A4,B5
        SL      .D2T2   B4,*B5
        NOP     4
        B       .S2     B3
        NOP     5

* unsigned smc_cmtl(volatile unsigned *address): commit-linked; 1 when the monitor wrote its
* LinkData to the word, 0 when it refused (SPRUEG5D section 3.1.3, Table 4).
smc_cmtl:
        MV      .L2X    A4,B4
        CMTL    .D2T2   *B4,B5
        NOP     4
        B       .S2     B3
        MV      .L1X    B5,A4
        NOP     4

* void smc_user_store(volatile unsigned *address, unsigned value): one word store made in User
* mode, and then back to Supervisor mode.
*
* User mode is entered the only way the architecture allows - by returning from a context whose
* TSR.CXM is set - and left through SWENR, which enters at REP; that handler clears CXM in the
* context it returns through. This is the sequence tests/programs/privilege checks in detail.
smc_user_store:
        MV      .L1     A4,A8
        MV      .L1X    B4,A9
        MVKL    .S2     rep_handler,B5
        MVKH    .S2     rep_handler,B5
        MVC     .S2     B5,REP
        MVC     .S2     TSR,B5
        NOP     1
        SET     .S2     B5,2,2,B5               ; TSR.GEE
        MVC     .S2     B5,TSR
        NOP     4
        MVK     .S2     0x44,B5                 ; GEE, and CXM: the context returns to User mode
        MVC     .S2     B5,NTSR
        MVKL    .S2     user_store,B5
        MVKH    .S2     user_store,B5
        MVC     .S2     B5,NRP
        NOP     2
        B       .S2     NRP
        NOP     5

user_store:
        STW     .D1     A9,*A8
        NOP     5
        SWENR
        NOP     5

supervisor_again:
        B       .S2     B3
        NOP     5

rep_handler:
        MVC     .S2     NTSR,B5
        NOP     1
        CLR     .S2     B5,6,6,B5               ; CXM back to Supervisor
        MVC     .S2     B5,NTSR
        MVKL    .S2     supervisor_again,B5
        MVKH    .S2     supervisor_again,B5
        MVC     .S2     B5,NRP
        NOP     2
        B       .S2     NRP
        NOP     5
