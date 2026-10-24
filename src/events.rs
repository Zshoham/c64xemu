//! The device event map: which peripheral source raises which system event, to which
//! megamodule, and which EDMA3 synchronisation channel it drives.
//!
//! SPRS276M Tables 7-9 to 7-11 name every chip-level event on the C6455; SPRS612G Table 7-10
//! and sections 7.5.1 and 7.6.3 do the same for the C6472. The peripheral fabric consults this
//! map rather than hardcoding the numbers in its tick and latch methods, so a device variant
//! stops editing interrupt wiring in the middle of the dispatch.
//!
//! A multicore device makes the *destination* part of the map. Most C6472 events are one
//! source every megamodule's interrupt controller can see, and software decides which core
//! services them; a few are steered - the EDMA3 channel controller's first six completion
//! interrupts go to one core each, the six local timers and the first six GPIO pins belong to
//! one core each, and an inter-processor interrupt is delivered only to the core it names
//! (SPRS612G Table 7-10 footnotes, sections 7.6.3 and 3.7). Every event this map reports
//! therefore carries a `Destination`.

/// Which megamodule an event reaches.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Destination {
    /// One core: a megamodule-local source, or a shared source the device steers.
    Core(usize),
    /// Every core's interrupt controller sees the same source.
    All,
}

/// The device family a map describes, which is what the steering rules differ on.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Family {
    C6455,
    C6472,
}

/// One EMAC's events: the per-channel receive and transmit interrupts, if the device separates
/// them, and the common interrupt that carries the statistics, host-error and MDIO sources.
#[derive(Clone, Copy)]
pub(crate) struct EmacEvents {
    pub(crate) receive: Option<usize>,
    pub(crate) transmit: Option<usize>,
    pub(crate) common: usize,
}

pub(crate) struct EventMap {
    family: Family,
    /// The host-to-CPU interrupt the HPI raises when the host sets DSPINT, and the EDMA3
    /// channel that interrupt also triggers where the device wires one.
    pub(crate) hpi: usize,
    pub(crate) hpi_channel: Option<usize>,
    /// The I2C module's CPU interrupt and its receive/transmit EDMA3 channels.
    pub(crate) i2c: usize,
    pub(crate) i2c_channels: (usize, usize),
    /// The Ethernet MACs, in port order.
    pub(crate) emac: [EmacEvents; 2],
    /// The EDMA3 channel controller's global completion, error and memory-protection events.
    pub(crate) edma_global: usize,
    pub(crate) edma_error: usize,
    pub(crate) edma_mp: usize,
    /// The first of the four transfer controllers' error interrupts.
    tc_error_base: usize,
    /// The UTOPIA's UINT, and its receive/transmit EDMA3 channels where the device documents a
    /// single pair.
    pub(crate) utopia: usize,
    pub(crate) utopia_channels: Option<(usize, usize)>,
    /// The UTOPIA PDMA's receive and transmit interrupts (PRINT, PXINT), on a device whose
    /// UTOPIA has a PDMA. Which megamodule they reach is the channel's SINT field, so they carry
    /// no destination here.
    pub(crate) utopia_pdma: Option<(usize, usize)>,
    /// The memory protection unit events: a CPU access fault and its DMA-master counterpart,
    /// per unit in L2, L1P, L1D order (SPRU871K chapter 8, events from `linux-c6x` traps.c).
    pub(crate) mpu_cpu: [usize; 3],
    pub(crate) mpu_dma: [usize; 3],
    /// The first McBSP's receive interrupt, on a device that has McBSPs at all.
    pub(crate) mcbsp: Option<usize>,
    /// The inter-processor interrupt a core raises in another core (SPRS612G section 3.7).
    pub(crate) ipc: Option<usize>,
    /// The power/sleep controller's single interrupt.
    pub(crate) psc: Option<usize>,
    /// The shared-memory controller's protection fault and profiler event.
    pub(crate) smc_fault: Option<usize>,
    pub(crate) smc_event: Option<usize>,
}

/// The C6455 map (SPRS276M Tables 7-9 to 7-11, SPRU966C Tables 2-6, 2-10 and 2-11, SPRU975E
/// section 2.16.4).
pub(crate) const C6455_EVENTS: EventMap = EventMap {
    family: Family::C6455,
    hpi: 15,
    // DSP_EVT: every host DSPINT, the one ending an HPI boot included, is synchronisation
    // event 0 (SPRS276M section 7.4.1 and Table 7-3 note 2).
    hpi_channel: Some(0),
    i2c: 16,
    i2c_channels: (44, 45),
    emac: [
        // Receive, transmit, statistics and the MDIO all funnel into one event on this device.
        EmacEvents {
            receive: None,
            transmit: None,
            common: 17,
        },
        // The C6455 has one EMAC; the second slot is never instantiated.
        EmacEvents {
            receive: None,
            transmit: None,
            common: 17,
        },
    ],
    edma_global: 24,
    edma_error: 79,
    edma_mp: 80,
    tc_error_base: 81,
    utopia: 36,
    utopia_channels: Some((32, 40)),
    utopia_pdma: None,
    mpu_cpu: [124, 120, 122],
    mpu_dma: [125, 121, 123],
    mcbsp: Some(40),
    ipc: None,
    psc: None,
    smc_fault: None,
    smc_event: None,
};

/// The C6472 map (SPRS612G Table 7-10 pp. 137-140, Table 7-4 pp. 124-126 for the EDMA3
/// synchronisation channels, section 7.6.3 for the GPIO split, section 3.7 for the IPC).
pub(crate) const C6472_EVENTS: EventMap = EventMap {
    family: Family::C6472,
    hpi: 85,
    // Table 7-4 gives the HPI no synchronisation event; channel 0 is GPINT0.
    hpi_channel: None,
    i2c: 72,
    i2c_channels: (20, 21),
    emac: [
        EmacEvents {
            receive: Some(5),
            transmit: Some(6),
            common: 70,
        },
        EmacEvents {
            receive: Some(7),
            transmit: Some(8),
            common: 71,
        },
    ],
    // EDMA3CC_GINT is event 69; the error interrupt is 57 and the memory protection interrupt
    // 58 (SPRS612G Table 7-10).
    edma_global: 69,
    edma_error: 57,
    edma_mp: 58,
    tc_error_base: 59,
    utopia: 56,
    // The C6472's UTOPIA moves its cells through a six-channel PDMA rather than the single
    // EDMA3 receive/transmit pair the C6455 documents. PRINT, the PDMA receive interrupt, is
    // event 30 and PXINT, the transmit one, event 31 (SPRS612G Table 7-10).
    utopia_channels: None,
    utopia_pdma: Some((30, 31)),
    mpu_cpu: [124, 120, 122],
    mpu_dma: [125, 121, 123],
    // The C6472 has three TSIPs instead of the C6455's two McBSPs.
    mcbsp: None,
    ipc: Some(84),
    psc: Some(65),
    smc_fault: Some(63),
    smc_event: Some(64),
};

impl EventMap {
    /// The transfer controllers' error interrupts: events 81-84 on the C6455 (SPRU966C
    /// Table 2-11), 59-62 on the C6472 (SPRS612G Table 7-10).
    pub(crate) fn transfer_controller_error(&self, index: usize) -> (Destination, usize) {
        (Destination::All, self.tc_error_base + index)
    }

    /// An EDMA3 shadow-region completion interrupt.
    ///
    /// The C6455 gives all eight regions their own event, 71 to 78 (SPRU966C Table 2-10). The
    /// C6472 steers the first six: region *n*'s completion is event 15 in core *n*'s interrupt
    /// controller and nowhere else, which is how six cores share one channel controller. Its
    /// remaining two regions are events 67 and 68 in every core (SPRS612G Table 7-10, the
    /// EDMA3CCINT_LOCAL and EDMA3CC_INT6/INT7 rows).
    pub(crate) fn edma_region_completion(&self, region: usize) -> Option<(Destination, usize)> {
        match self.family {
            Family::C6455 => (region < 8).then(|| (Destination::All, 71 + region)),
            Family::C6472 => match region {
                0..=5 => Some((Destination::Core(region), 15)),
                6 => Some((Destination::All, 67)),
                7 => Some((Destination::All, 68)),
                _ => None,
            },
        }
    }

    /// One half of one timer: where its interrupt goes, and the EDMA3 channel it triggers.
    ///
    /// The C6455 has two timers, whose halves are events 67 to 70 and EDMA3 channels 1, 2, 16
    /// and 17 (SPRS276M Tables 7-9 and 7-10). The C6472 has twelve: timers 0 to 5 are one per
    /// megamodule and raise events 16 (low half) and 17 (high half) in their own core only,
    /// while the shared timers 6 to 11 raise events 18 to 29 in every core and trigger EDMA3
    /// channels 8 to 19 (SPRS612G Table 7-10 and Table 7-4).
    pub(crate) fn timer(
        &self,
        index: usize,
        half: usize,
    ) -> Option<(Destination, usize, Option<usize>)> {
        match self.family {
            Family::C6455 => {
                let channels = if index == 0 { [1, 2] } else { [16, 17] };
                (index < 2).then(|| {
                    (
                        Destination::All,
                        67 + index * 2 + half,
                        Some(channels[half]),
                    )
                })
            }
            Family::C6472 => match index {
                0..=5 => Some((Destination::Core(index), 16 + half, None)),
                6..=11 => {
                    let shared = index - 6;
                    Some((
                        Destination::All,
                        18 + shared * 2 + half,
                        Some(8 + shared * 2 + half),
                    ))
                }
                _ => None,
            },
        }
    }

    /// One GPIO pin's interrupt and EDMA3 channel.
    ///
    /// GPINT0 to GPINT15 are events 51 to 66 and EDMA3 channels 48 to 63 on the C6455
    /// (SPRS276M Table 7-10, SPRU966C Table 2-6). On the C6472 the first six pins belong to
    /// one core each - pin *n* is GPINT_LOCAL, event 73, in core *n* - and pins 6 to 15 are
    /// events 74 to 83 in every core; the first eight pins trigger EDMA3 channels 0 to 7
    /// (SPRS612G section 7.6.3 and Table 7-4).
    pub(crate) fn gpio(&self, pin: usize) -> Option<(Destination, usize, Option<usize>)> {
        match self.family {
            Family::C6455 => (pin < 16).then(|| (Destination::All, 51 + pin, Some(48 + pin))),
            Family::C6472 => {
                let channel = (pin < 8).then_some(pin);
                match pin {
                    0..=5 => Some((Destination::Core(pin), 73, channel)),
                    6..=15 => Some((Destination::All, 74 + pin - 6, channel)),
                    _ => None,
                }
            }
        }
    }

    /// The EDMA3 channel a UTOPIA PDMA transmit interrupt output drives. The transmit group has
    /// six interrupt outputs, one per DSP subsystem, and SINT *n* asserts PXINT*n*, which is the
    /// synchronisation event of EDMA3 channel 22 + *n* (SPRUEG2E section 6.2, SPRS612G
    /// Table 7-4). SINT 6 and 7 select outputs the device does not connect, and the receive
    /// group's PRINT outputs have no channel at all. The C6455 has no PDMA.
    pub(crate) fn pdma_transmit_channel(&self, sint: usize) -> Option<usize> {
        match self.family {
            Family::C6455 => None,
            Family::C6472 => (sint < 6).then_some(22 + sint),
        }
    }

    /// XEVT and REVT are EDMA3 events 12 to 15 and RINT/XINT are system events 40 to 43 for the
    /// two serial ports (SPRS276M Tables 7-9 and 7-11). Returns `[(event, channel); 2]` for the
    /// receive and transmit directions.
    pub(crate) fn mcbsp(&self, index: usize) -> Option<[(usize, usize); 2]> {
        let base = self.mcbsp?;
        Some([
            (base + 2 * index, 13 + 2 * index),
            (base + 1 + 2 * index, 12 + 2 * index),
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn c6455_keeps_its_documented_numbers() {
        assert_eq!(C6455_EVENTS.emac[0].common, 17);
        assert_eq!(
            C6455_EVENTS.timer(1, 1),
            Some((Destination::All, 70, Some(17)))
        );
        assert_eq!(
            C6455_EVENTS.gpio(15),
            Some((Destination::All, 66, Some(63)))
        );
        assert_eq!(
            C6455_EVENTS.edma_region_completion(3),
            Some((Destination::All, 74))
        );
        assert_eq!(
            C6455_EVENTS.transfer_controller_error(2),
            (Destination::All, 83)
        );
    }

    /// The C6472 steers a shadow region's completion into one core: that is what lets six
    /// cores share one channel controller without seeing each other's completions.
    #[test]
    fn c6472_steers_the_first_six_edma_regions_and_broadcasts_the_rest() {
        for core in 0..6 {
            assert_eq!(
                C6472_EVENTS.edma_region_completion(core),
                Some((Destination::Core(core), 15))
            );
        }
        assert_eq!(
            C6472_EVENTS.edma_region_completion(6),
            Some((Destination::All, 67))
        );
        assert_eq!(
            C6472_EVENTS.edma_region_completion(7),
            Some((Destination::All, 68))
        );
    }

    #[test]
    fn c6472_local_timers_belong_to_one_core_and_shared_timers_to_all() {
        for core in 0..6 {
            assert_eq!(
                C6472_EVENTS.timer(core, 0),
                Some((Destination::Core(core), 16, None))
            );
            assert_eq!(
                C6472_EVENTS.timer(core, 1),
                Some((Destination::Core(core), 17, None))
            );
        }
        assert_eq!(
            C6472_EVENTS.timer(6, 0),
            Some((Destination::All, 18, Some(8)))
        );
        assert_eq!(
            C6472_EVENTS.timer(11, 1),
            Some((Destination::All, 29, Some(19)))
        );
        assert_eq!(C6472_EVENTS.timer(12, 0), None);
    }

    #[test]
    fn c6472_splits_gpio_between_local_and_shared_pins() {
        assert_eq!(
            C6472_EVENTS.gpio(2),
            Some((Destination::Core(2), 73, Some(2)))
        );
        assert_eq!(C6472_EVENTS.gpio(6), Some((Destination::All, 74, Some(6))));
        assert_eq!(C6472_EVENTS.gpio(8), Some((Destination::All, 76, None)));
    }

    /// PXINT0-5 are EDMA3 channels 22-27 on the C6472, SPRS612G Table 7-4; nothing else is
    /// connected, and the C6455 has no PDMA to drive any.
    #[test]
    fn pdma_transmit_outputs_are_channels_22_to_27() {
        for sint in 0..6 {
            assert_eq!(C6472_EVENTS.pdma_transmit_channel(sint), Some(22 + sint));
        }
        assert_eq!(C6472_EVENTS.pdma_transmit_channel(6), None);
        assert_eq!(C6472_EVENTS.pdma_transmit_channel(7), None);
        assert_eq!(C6455_EVENTS.pdma_transmit_channel(0), None);
    }

    #[test]
    fn c6472_gives_each_emac_its_own_events() {
        assert_eq!(C6472_EVENTS.emac[0].receive, Some(5));
        assert_eq!(C6472_EVENTS.emac[0].transmit, Some(6));
        assert_eq!(C6472_EVENTS.emac[0].common, 70);
        assert_eq!(C6472_EVENTS.emac[1].receive, Some(7));
        assert_eq!(C6472_EVENTS.emac[1].transmit, Some(8));
        assert_eq!(C6472_EVENTS.emac[1].common, 71);
    }

    #[test]
    fn c6472_has_no_mcbsp() {
        assert!(C6472_EVENTS.mcbsp(0).is_none());
        assert!(C6455_EVENTS.mcbsp(1).is_some());
    }
}
