# LMS6002 #

This Rust library provides a high-level interface to Lime Micro's
[LMS6002](https://limemicro.com/technology/lms6002d/). Note, this
library only provides a configuration API to the LMS6002, and does not
help with shuffling
[IQ](https://en.wikipedia.org/wiki/In-phase_and_quadrature_components)
samples to/from the LMS6002. Sample IO is over a different bus, and
the implementation of it is hardware-dependent.
