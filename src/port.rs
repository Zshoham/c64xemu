//! The shared-peripheral slave-port seam.
//!
//! The router is the one point both masters, the CPU and the DMA engines, cross to reach a
//! peripheral. Every port answers the same shape here, so the router's dispatch chain is a list
//! of ports rather than twenty signatures that happen to agree by convention.

use crate::system::Endian;

/// What a port needs from the device but does not own. Only the endianness today: three ports
/// report their registers byte-swapped when the device is strapped big-endian, and it is
/// settable at runtime (`Fabric::set_endian`), so a port cannot cache it.
#[derive(Clone, Copy)]
pub(crate) struct PortContext {
    pub endian: Endian,
}

/// What a write left for the router to do after a port handled it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum PortWrite {
    Unhandled,
    Done,
    /// The port raised a completion event the router must latch (IDMA's channels).
    Event(usize),
}

/// The interface a peripheral presents to the router: one 32-bit slave port that answers
/// `None` or `PortWrite::Unhandled` for an address it does not own.
pub(crate) trait SlavePort {
    fn read(&mut self, address: u32, context: PortContext) -> Option<u32>;
    fn write(&mut self, address: u32, value: u32, context: PortContext) -> PortWrite;
}
