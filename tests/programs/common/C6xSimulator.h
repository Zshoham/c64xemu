/* Build shim: the DSPLIB reference headers include this file for the prototypes their
   hand-optimised siblings need. The natural-C `_cn` implementations this suite compiles use no
   intrinsics; the only TI-specific construct they carry is `_nassert`, an optimiser hint with
   no effect on results, so a host build can discard it and still compile identical source. */
#ifndef C6XSIMULATOR_H
#define C6XSIMULATOR_H

#if !defined(__TI_COMPILER_VERSION__)
#define _nassert(condition) ((void)0)
/* The TI compiler defines _LITTLE_ENDIAN for a little-endian target and some DSPLIB reference
   kernels select their twiddle-factor convention on it, so the host build has to define it too
   or the two builds compute different transforms. */
#if !defined(_LITTLE_ENDIAN)
#define _LITTLE_ENDIAN 1
#endif
#endif

#endif
