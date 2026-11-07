//! The host-side connections attached to the fabric: the Ethernet backend, the HPI host - a live
//! socket or a replayed file - and the UTOPIA's ATM master.
//!
//! Each is something outside the device that the emulated clock has to be reconciled with. The
//! machine says what the run clock reads and lends the fabric; this module answers what is due
//! (`service`), how long the device may run before something is (`next_due`), and keeps the
//! datagram protocols the two external masters speak out of the machine. Polling happens on a
//! cycle boundary rather than from a thread, because the run loop owns the device.
//!
//! The CIO handshake is deliberately not here: it is answered on whichever core reached it, so
//! it belongs to the core and device, not to a connection attached to the fabric.

use crate::hpi::HostPort;
use crate::hpi_replay::HostFile;
use crate::netio::{Ethernet, EthernetFiles, HostLink};
use crate::system::Fabric;
use crate::utopia::{CELL_BYTES, MAX_CELL_BYTES, NULL_PHY};
use serde_json::Value;
use std::path::Path;

/// A datagram socket a host process talks to the device through, with the interval its run
/// clock polls it at.
pub(crate) struct Endpoint<'a> {
    pub socket: &'a Path,
    /// Where replies go; without one, to the sender of the latest request.
    pub peer: Option<&'a Path>,
    pub poll: u64,
}

/// Where the HPI host's accesses come from: a host process on a socket, or a timed file
/// (`--hpi-replay`).
pub(crate) enum HpiWiring<'a> {
    Socket(Endpoint<'a>),
    File(&'a Path),
}

/// What a machine's command line attaches to the device.
pub(crate) struct Wiring<'a> {
    /// How often the Ethernet backend looks at its socket, in run-clock cycles.
    pub net_poll: u64,
    /// The MAC the wire is attached to.
    pub net_port: usize,
    pub net: EthernetFiles<'a>,
    pub hpi: Option<HpiWiring<'a>>,
    pub utopia: Option<Endpoint<'a>>,
}

/// The connections a device is attached to, serviced in one order: Ethernet, HPI, UTOPIA.
pub(crate) struct HostConnections {
    ethernet: Ethernet,
    hpi: Option<HpiHost>,
    utopia: Option<AtmMaster>,
}

impl HostConnections {
    /// Binds the sockets, reads the pcap input and parses the HPI replay file the wiring names.
    /// A replay file is only validated here: none of its accesses run until the run loop
    /// reaches their ticks.
    pub(crate) fn open(wiring: Wiring) -> Result<Self, String> {
        Ok(Self {
            ethernet: Ethernet::open(wiring.net_poll, wiring.net_port, wiring.net)?,
            hpi: wiring
                .hpi
                .map(|hpi| match hpi {
                    HpiWiring::Socket(endpoint) => {
                        HostLink::bind(endpoint.socket, endpoint.peer, endpoint.poll)
                            .map(|link| HpiHost::Socket(HpiSocket { link }))
                    }
                    HpiWiring::File(path) => HostFile::load(path).map(HpiHost::File),
                })
                .transpose()?,
            utopia: wiring
                .utopia
                .map(|endpoint| HostLink::bind(endpoint.socket, None, endpoint.poll))
                .transpose()?
                .map(|link| AtmMaster { link }),
        })
    }

    /// Nothing attached: nothing is preloaded, polled or written out.
    #[cfg(test)]
    pub(crate) fn offline() -> Self {
        Self {
            ethernet: Ethernet::offline(),
            hpi: None,
            utopia: None,
        }
    }

    /// An HPI replay file and nothing else attached, for tests that drive it through a machine.
    #[cfg(test)]
    pub(crate) fn replaying(text: &str) -> Self {
        Self {
            ethernet: Ethernet::offline(),
            hpi: Some(HpiHost::File(
                HostFile::parse(Path::new("host.csv"), text).unwrap(),
            )),
            utopia: None,
        }
    }

    /// A device has just been (re)built: every connection is polled on its first cycle, the
    /// pcap input is delivered to the new MACs, and an HPI replay file starts again from its
    /// first row. Host backlog and the TX archive are retained.
    pub(crate) fn install(&mut self, fabric: &mut Fabric, cycle: u64) {
        self.ethernet.reset_poll();
        for link in self.links_mut() {
            link.reset_poll();
        }
        if let Some(HpiHost::File(file)) = self.hpi.as_mut() {
            file.rewind();
        }
        self.ethernet.preload(fabric, cycle);
    }

    /// The old device is about to be rebuilt: the frames it transmitted belong in the TX archive,
    /// and the inspection history starts over.
    pub(crate) fn retire(&mut self, fabric: &mut Fabric, cycle: u64) {
        self.ethernet.collect(fabric, cycle);
        self.ethernet.reset_history();
    }

    /// Serves whatever is due at `cycle` against the fabric: frames in and out of the Ethernet
    /// backend, then the HPI host's accesses, then the ATM master's requests. A connection that
    /// is not yet due is not looked at. Once an HPI replay file has failed nothing after it is
    /// served: the run stops at this boundary (`stopped`).
    pub(crate) fn service(&mut self, fabric: &mut Fabric, cycle: u64) {
        self.ethernet.service(fabric, cycle);
        match self.hpi.as_mut() {
            Some(HpiHost::Socket(host)) => host.service(fabric, cycle),
            Some(HpiHost::File(file)) => file.service(fabric, cycle),
            None => {}
        }
        if self.stopped() {
            return;
        }
        if let Some(master) = self.utopia.as_mut() {
            master.service(fabric, cycle);
        }
    }

    /// Whether the HPI replay file has failed, which ends the run before anything else - the
    /// CIO handshake, the next cycle - happens at this boundary.
    pub(crate) fn stopped(&self) -> bool {
        self.replay().is_some_and(HostFile::failed)
    }

    /// The HPI replay file, when the HPI host is one.
    pub(crate) fn replay(&self) -> Option<&HostFile> {
        match self.hpi.as_ref() {
            Some(HpiHost::File(file)) => Some(file),
            _ => None,
        }
    }

    /// How many cycles the device may run from `cycle` before a connection is due, never zero;
    /// `None` when nothing attached polls at all. The machine clips its batches to this without
    /// inspecting sockets or queues.
    pub(crate) fn next_due(&self, cycle: u64) -> Option<u64> {
        [
            self.ethernet.poll_delay(cycle),
            self.hpi.as_ref().and_then(|host| match host {
                HpiHost::Socket(host) => Some(host.link.poll_delay(cycle)),
                HpiHost::File(file) => file.next_due(cycle),
            }),
            self.utopia
                .as_ref()
                .map(|master| master.link.poll_delay(cycle)),
        ]
        .into_iter()
        .flatten()
        .min()
    }

    fn links_mut(&mut self) -> impl Iterator<Item = &mut HostLink> {
        [
            self.hpi.as_mut().and_then(|host| match host {
                HpiHost::Socket(host) => Some(&mut host.link),
                HpiHost::File(_) => None,
            }),
            self.utopia.as_mut().map(|master| &mut master.link),
        ]
        .into_iter()
        .flatten()
    }

    /// Whether the Ethernet inspection history is recording.
    pub(crate) fn history_enabled(&self) -> bool {
        self.ethernet.history_enabled()
    }

    /// With inspection on, a cycle that has run drains the MAC so each frame is recorded at the
    /// cycle it was sent rather than at the next poll.
    pub(crate) fn after_cycle(&mut self, fabric: &mut Fabric, cycle: u64) {
        if self.ethernet.history_enabled() {
            self.ethernet.collect(fabric, cycle);
        }
    }

    pub(crate) fn enable_history(&mut self, cycle: u64) {
        self.ethernet.enable_history(cycle);
    }

    pub(crate) fn history_json(&mut self, fabric: &mut Fabric, cycle: u64) -> Value {
        self.ethernet.history_json(fabric, cycle)
    }

    /// Every frame the program has transmitted this session, in order.
    pub(crate) fn transmitted(&mut self, fabric: &mut Fabric, cycle: u64) -> &[Vec<u8>] {
        self.ethernet.transmitted(fabric, cycle)
    }

    /// Frames the live peer never took, because it fell too far behind.
    pub(crate) fn dropped(&self) -> u64 {
        self.ethernet.dropped()
    }

    /// Hands the remaining frames to the peer and writes the capture file.
    pub(crate) fn finish(&mut self, fabric: &mut Fabric, cycle: u64) -> Result<(), String> {
        self.ethernet.finish(fabric, cycle)
    }
}

/// The external host on the HPI pins: a host process on a socket, or a replayed file. Both
/// reach the device through the same `Fabric::hpi_host_read` and `hpi_host_write`.
enum HpiHost {
    Socket(HpiSocket),
    File(HostFile),
}

/// The HPI host as a process on a datagram socket. Each datagram is one access: a direction byte (`r` or
/// `w`), the HCNTL encoding of the port it addresses, two bytes of padding and a little-endian
/// value. A read is answered with the value, or with an empty datagram if the port cannot be
/// read; a write is not answered. Datagrams shorter than eight bytes, and ports the HCNTL
/// encoding does not name, are dropped without a reply.
///
/// The host port is the device's, not any core's: it attaches to the fabric like the Ethernet
/// backend and the ATM master.
struct HpiSocket {
    link: HostLink,
}

impl HpiSocket {
    const REQUEST_BYTES: usize = 8;

    fn service(&mut self, fabric: &mut Fabric, cycle: u64) {
        if !self.link.take_due(cycle) {
            return;
        }
        // Longer datagrams are truncated to the request; only its first eight bytes mean anything.
        let mut bytes = [0; 16];
        while let Some(length) = self.link.receive_datagram(&mut bytes) {
            if length < Self::REQUEST_BYTES {
                continue;
            }
            let Some(port) = host_port(bytes[1]) else {
                continue;
            };
            if bytes[0] == b'r' {
                match fabric.hpi_host_read(port) {
                    Some(value) => self.link.reply(&value.to_le_bytes()),
                    None => self.link.reply(&[]),
                }
            } else {
                let value = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
                fabric.hpi_host_write(port, value);
            }
        }
    }
}

/// The HCNTL encoding a host process sends, as SPRU969B Table 4 numbers it.
fn host_port(code: u8) -> Option<HostPort> {
    match code {
        0 => Some(HostPort::Control),
        1 => Some(HostPort::DataAutoIncrement),
        2 => Some(HostPort::Address),
        3 => Some(HostPort::Data),
        _ => None,
    }
}

/// The ATM master on the UTOPIA pins. Its requests are one datagram each: the opcode, the
/// five-bit PHY address it drives, and for a write the cell's wire bytes. The replies are the
/// address phase's answer - URCLAV/UXCLAV, or nothing for a high-impedance pin - and the
/// transfer's.
///
/// | request | meaning | reply |
/// |---|---|---|
/// | `p`, phy | poll URCLAV | `[room]`, empty if the pin is high-impedance |
/// | `w`, phy, cell | send a cell | `[accepted]` |
/// | `x`, phy | poll UXCLAV | `[waiting]`, empty if the pin is high-impedance |
/// | `r`, phy | take a cell | the cell's wire bytes, `[0]` if none was waiting |
///
/// Anything else - a short datagram, a PHY above the null PHY, an unknown opcode, a poll or read
/// with a payload, a cell of the wrong length - is answered `[0xff]` and changes nothing.
struct AtmMaster {
    link: HostLink,
}

impl AtmMaster {
    const REQUEST_HEADER: usize = 2;
    const INVALID: u8 = 0xff;

    fn service(&mut self, fabric: &mut Fabric, cycle: u64) {
        if !self.link.take_due(cycle) {
            return;
        }
        // One byte past the longest request, so an oversized cell is seen even when recv_from
        // truncates the datagram.
        let mut bytes = [0; Self::REQUEST_HEADER + MAX_CELL_BYTES + 1];
        let shortest_write = Self::REQUEST_HEADER + CELL_BYTES;
        let longest_write = Self::REQUEST_HEADER + MAX_CELL_BYTES;
        while let Some(length) = self.link.receive_datagram(&mut bytes) {
            if length < Self::REQUEST_HEADER || bytes[1] > NULL_PHY {
                self.link.reply(&[Self::INVALID]);
                continue;
            }
            let (opcode, phy) = (bytes[0], bytes[1]);
            match (opcode, length) {
                (b'p', Self::REQUEST_HEADER) => match fabric.utopia_master_poll_receive(phy) {
                    Some(room) => self.link.reply(&[u8::from(room)]),
                    None => self.link.reply(&[]),
                },
                (b'w', _) if (shortest_write..=longest_write).contains(&length) => {
                    let accepted =
                        fabric.utopia_master_send_cell(phy, &bytes[Self::REQUEST_HEADER..length]);
                    self.link.reply(&[u8::from(accepted)]);
                }
                (b'x', Self::REQUEST_HEADER) => match fabric.utopia_master_poll_transmit(phy) {
                    Some(waiting) => self.link.reply(&[u8::from(waiting)]),
                    None => self.link.reply(&[]),
                },
                (b'r', Self::REQUEST_HEADER) => match fabric.utopia_master_take_cell(phy) {
                    Some(cell) => self.link.reply(cell.bytes()),
                    None => self.link.reply(&[0]),
                },
                _ => self.link.reply(&[Self::INVALID]),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::system::{MasterAccess, Privilege};
    use std::os::unix::net::UnixDatagram;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};

    /// A scratch directory holding the sockets of one test.
    struct Sockets(PathBuf);

    impl Sockets {
        fn new() -> Self {
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let path = std::env::temp_dir().join(format!(
                "c64xemu-host-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir(&path).unwrap();
            Self(path)
        }

        fn path(&self, name: &str) -> PathBuf {
            self.0.join(name)
        }

        /// A host process: a socket the device replies to.
        fn host(&self, name: &str) -> UnixDatagram {
            let socket = UnixDatagram::bind(self.path(name)).unwrap();
            socket.set_nonblocking(true).unwrap();
            socket
        }
    }

    impl Drop for Sockets {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// Connections bound at `(socket, poll interval)` for the HPI host and the ATM master, and
    /// an Ethernet backend with no host files.
    fn connect(hpi: Option<(&Path, u64)>, utopia: Option<(&Path, u64)>) -> HostConnections {
        let endpoint = |(socket, poll)| Endpoint {
            socket,
            peer: None,
            poll,
        };
        HostConnections::open(Wiring {
            net_poll: 1,
            net_port: 0,
            net: EthernetFiles::default(),
            hpi: hpi.map(endpoint).map(HpiWiring::Socket),
            utopia: utopia.map(endpoint),
        })
        .unwrap()
    }

    fn reply(host: &UnixDatagram) -> Option<Vec<u8>> {
        let mut bytes = [0; 128];
        host.recv(&mut bytes)
            .ok()
            .map(|length| bytes[..length].to_vec())
    }

    fn request(host: &UnixDatagram, to: &Path, bytes: &[u8]) {
        host.send_to(bytes, to).unwrap();
    }

    fn hpi_access(direction: u8, port: u8, value: u32) -> Vec<u8> {
        let mut bytes = vec![direction, port, 0, 0];
        bytes.extend_from_slice(&value.to_le_bytes());
        bytes
    }

    /// The CPU writing HPIC to zero, which releases the software reset that holds the host off
    /// (SPRU969B section 7.2).
    fn release_hpi_reset(fabric: &mut Fabric) {
        fabric.write32(0, 0x0288_0030, 0);
    }

    /// The UTOPIA at 0x02b4_0000 enabled as slave 3 in multi-PHY mode, receiver and transmitter
    /// (SPRUE48 section 3), the way the PDMA tests bring it up.
    fn utopia_slave(fabric: &mut Fabric) {
        fabric.write_word(
            Privilege::ID0_SUPERVISOR,
            0x02b4_0000,
            (3 << 24) | (1 << 16) | (1 << 14) | 1,
        );
    }

    fn cell(fill: u8) -> Vec<u8> {
        vec![fill; CELL_BYTES]
    }

    #[test]
    fn hpi_accesses_reach_the_host_port_and_reads_are_answered() {
        let sockets = Sockets::new();
        let emulator = sockets.path("hpi");
        let host = sockets.host("host");
        let mut connections = connect(Some((&emulator, 10)), None);
        let mut fabric = Fabric::default();

        // HPIRST is set out of reset, so the host cannot read: an empty reply says so.
        request(&host, &emulator, &hpi_access(b'r', 2, 0));
        connections.service(&mut fabric, 0);
        assert_eq!(reply(&host), Some(vec![]));

        release_hpi_reset(&mut fabric);
        request(&host, &emulator, &hpi_access(b'w', 2, 0x0020_0800));
        request(&host, &emulator, &hpi_access(b'r', 2, 0));
        connections.service(&mut fabric, 10);
        assert_eq!(
            reply(&host),
            Some(0x0020_0800u32.to_le_bytes().to_vec()),
            "HPIA reads back what the host wrote"
        );
        assert_eq!(reply(&host), None, "a write is not answered");
        assert_eq!(fabric.hpi_host_read(HostPort::Address), Some(0x0020_0800));
    }

    #[test]
    fn hpi_requests_that_name_no_access_are_dropped_without_a_reply() {
        let sockets = Sockets::new();
        let emulator = sockets.path("hpi");
        let host = sockets.host("host");
        let mut connections = connect(Some((&emulator, 10)), None);
        let mut fabric = Fabric::default();
        release_hpi_reset(&mut fabric);
        fabric.hpi_host_write(HostPort::Address, 0x1111);

        // An HCNTL code outside Table 4, and a datagram too short to carry a value.
        request(&host, &emulator, &hpi_access(b'w', 4, 0x2222));
        request(&host, &emulator, &hpi_access(b'r', 7, 0));
        request(&host, &emulator, &[b'w', 2, 0, 0, 0x33]);
        connections.service(&mut fabric, 0);
        assert_eq!(reply(&host), None);
        assert_eq!(fabric.hpi_host_read(HostPort::Address), Some(0x1111));
    }

    #[test]
    fn utopia_poll_write_and_read_requests_are_answered_from_the_port() {
        let sockets = Sockets::new();
        let emulator = sockets.path("atm");
        let host = sockets.host("host");
        let mut connections = connect(None, Some((&emulator, 10)));
        let mut fabric = Fabric::default();
        utopia_slave(&mut fabric);
        let mut exchange = |connections: &mut HostConnections, cycle, bytes: &[u8]| {
            request(&host, &emulator, bytes);
            connections.service(&mut fabric, cycle);
            reply(&host)
        };

        assert_eq!(exchange(&mut connections, 0, b"p\x03"), Some(vec![1]));
        assert_eq!(
            exchange(&mut connections, 10, b"p\x04"),
            Some(vec![]),
            "no PHY answers address 4: the pin is high-impedance"
        );
        let mut write = b"w\x03".to_vec();
        write.extend(cell(0xa5));
        // The receive queue holds two cells; a third finds no room.
        assert_eq!(exchange(&mut connections, 20, &write), Some(vec![1]));
        assert_eq!(exchange(&mut connections, 30, &write), Some(vec![1]));
        assert_eq!(exchange(&mut connections, 40, b"p\x03"), Some(vec![0]));
        assert_eq!(exchange(&mut connections, 50, &write), Some(vec![0]));
        // Nothing has been transmitted, so the transmit side has no cell to give.
        assert_eq!(exchange(&mut connections, 60, b"x\x03"), Some(vec![0]));
        assert_eq!(exchange(&mut connections, 70, b"r\x03"), Some(vec![0]));
        assert_eq!(exchange(&mut connections, 80, b"x\x04"), Some(vec![]));
    }

    #[test]
    fn malformed_utopia_requests_are_answered_invalid_and_change_nothing() {
        let sockets = Sockets::new();
        let emulator = sockets.path("atm");
        let host = sockets.host("host");
        let mut connections = connect(None, Some((&emulator, 10)));
        let mut fabric = Fabric::default();
        utopia_slave(&mut fabric);

        let mut short_cell = b"w\x03".to_vec();
        short_cell.extend(vec![0; CELL_BYTES - 1]);
        let mut long_cell = b"w\x03".to_vec();
        long_cell.extend(vec![0; MAX_CELL_BYTES + 1]);
        let mut null_phy = b"w".to_vec();
        null_phy.push(NULL_PHY + 1);
        null_phy.extend(cell(0));
        let requests = [
            b"p".as_slice(),
            b"",
            b"p\x03\x00",
            b"x\x03\x00",
            b"r\x03\x00",
            b"z\x03",
            b"p\x20",
            short_cell.as_slice(),
            long_cell.as_slice(),
            null_phy.as_slice(),
        ];
        for malformed in requests {
            request(&host, &emulator, malformed);
        }
        connections.service(&mut fabric, 0);
        for malformed in requests {
            assert_eq!(reply(&host), Some(vec![0xff]), "{malformed:02x?}");
        }
        assert_eq!(reply(&host), None);
        assert_eq!(fabric.utopia_master_poll_receive(3), Some(true));
        assert!(fabric.utopia_master_send_cell(3, &cell(1)));
        assert!(
            fabric.utopia_master_send_cell(3, &cell(2)),
            "no request used a queue slot"
        );
    }

    #[test]
    fn a_connection_is_not_looked_at_before_it_is_due() {
        let sockets = Sockets::new();
        let emulator = sockets.path("hpi");
        let host = sockets.host("host");
        let mut connections = connect(Some((&emulator, 100)), None);
        let mut fabric = Fabric::default();

        connections.service(&mut fabric, 0);
        request(&host, &emulator, &hpi_access(b'r', 2, 0));
        connections.service(&mut fabric, 99);
        assert_eq!(reply(&host), None);
        connections.service(&mut fabric, 100);
        assert!(reply(&host).is_some());
    }

    #[test]
    fn next_due_is_the_earliest_connection_and_never_zero() {
        let sockets = Sockets::new();
        let (hpi, atm) = (sockets.path("hpi"), sockets.path("atm"));
        let mut fabric = Fabric::default();

        let mut none = HostConnections::offline();
        assert_eq!(none.next_due(0), None, "nothing attached polls");
        none.service(&mut fabric, 0);
        assert_eq!(none.next_due(5), None);

        let mut both = connect(Some((&hpi, 100)), Some((&atm, 40)));
        assert_eq!(
            both.next_due(0),
            Some(1),
            "a fresh device is polled at once"
        );
        both.service(&mut fabric, 0);
        assert_eq!(both.next_due(0), Some(40));
        assert_eq!(both.next_due(30), Some(10));
        // Overdue is not zero: the batch still advances.
        assert_eq!(both.next_due(500), Some(1));
        // The ATM master is polled again at 80, the HPI host not before 100.
        both.service(&mut fabric, 40);
        assert_eq!(both.next_due(40), Some(40));
        both.service(&mut fabric, 80);
        assert_eq!(both.next_due(80), Some(20));
    }

    #[test]
    fn a_live_ethernet_socket_bounds_next_due_too() {
        let sockets = Sockets::new();
        let hpi = sockets.path("hpi");
        let mut connections = HostConnections::open(Wiring {
            net_poll: 16,
            net_port: 0,
            net: EthernetFiles {
                socket: Some(&sockets.path("net")),
                ..EthernetFiles::default()
            },
            hpi: Some(HpiWiring::Socket(Endpoint {
                socket: &hpi,
                peer: None,
                poll: 100,
            })),
            utopia: None,
        })
        .unwrap();
        let mut fabric = Fabric::default();
        connections.service(&mut fabric, 0);
        assert_eq!(connections.next_due(0), Some(16));
        assert_eq!(connections.next_due(10), Some(6));
    }
}
