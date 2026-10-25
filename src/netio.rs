//! Host-side Ethernet frame lifetimes and concrete Unix datagram adapters.
//!
//! `Ethernet` owns live receive admission, polling deadlines, the complete transmit capture
//! and bounded inspection history. The machine supplies its emulated clock, and all frame
//! admission and draining goes through the fabric's real MAC operations. Pcap input is
//! preloaded separately from the paced live socket.
//!
//! `Bridge` carries one raw Ethernet frame per datagram, retaining its socket retry/drop
//! policy. `tests/tools/tap_bridge.py` can relay these frames to a TAP without emulator
//! privileges. `HostLink` is the separate datagram adapter for the HPI host and the ATM master.

use std::io::ErrorKind;
use std::os::unix::net::{SocketAddr, UnixDatagram};
use std::path::{Path, PathBuf};

use crate::pcap;
use crate::system::Fabric;
use serde_json::{json, Value};
use std::collections::VecDeque;

const ETHERNET_HISTORY_LIMIT: usize = 256;

/// When a host backend next looks at its socket, on the machine's run clock. Polling happens
/// on a cycle boundary rather than from a thread, because the run loop owns the device.
pub(crate) struct PollSchedule {
    interval: u64,
    next: u64,
}

impl PollSchedule {
    pub(crate) fn new(interval: u64) -> Self {
        Self { interval, next: 0 }
    }

    /// A reinstalled device is polled on its first cycle.
    pub(crate) fn reset(&mut self) {
        self.next = 0;
    }

    /// Whether a poll is due at `cycle`; taking a due poll schedules the next one.
    pub(crate) fn take_due(&mut self, cycle: u64) -> bool {
        if cycle < self.next {
            return false;
        }
        self.next = cycle + self.interval;
        true
    }

    /// How many cycles a batch may run before the next poll is due; never zero.
    pub(crate) fn delay(&self, cycle: u64) -> u64 {
        self.next.saturating_sub(cycle).max(1)
    }
}

/// The host files an Ethernet backend opens: pcap input to preload, pcap output for the TX
/// capture, and the live socket with its optional fixed peer.
#[derive(Default)]
pub(crate) struct EthernetFiles<'a> {
    pub pcap_in: Option<&'a Path>,
    pub pcap_out: Option<&'a Path>,
    pub socket: Option<&'a Path>,
    pub peer: Option<&'a Path>,
}

struct EthernetPacket {
    sequence: u64,
    cycle: u64,
    incoming: bool,
    bytes: Vec<u8>,
}

/// Host-side frame lifetimes. The machine supplies its clock; the fabric owns the MAC.
pub(crate) struct Ethernet {
    bridge: Option<Bridge>,
    poll: PollSchedule,
    port: usize,
    pcap_in: Option<Vec<Vec<u8>>>,
    pcap_out: Option<PathBuf>,
    transmitted: Vec<Vec<u8>>,
    sent_to_bridge: usize,
    pending_receive: VecDeque<Vec<u8>>,
    history: VecDeque<EthernetPacket>,
    next_sequence: u64,
    history_enabled: bool,
}

impl Ethernet {
    /// A backend with no host files: nothing is preloaded, polled or written out.
    pub(crate) fn offline() -> Self {
        Self {
            bridge: None,
            poll: PollSchedule::new(1),
            port: 0,
            pcap_in: None,
            pcap_out: None,
            transmitted: Vec::new(),
            sent_to_bridge: 0,
            pending_receive: VecDeque::new(),
            history: VecDeque::new(),
            next_sequence: 0,
            history_enabled: false,
        }
    }

    /// Reads the pcap input and binds the live socket; frames go to and from MAC `port`.
    pub(crate) fn open(poll: u64, port: usize, files: EthernetFiles) -> Result<Self, String> {
        Ok(Self {
            pcap_in: files.pcap_in.map(pcap::read).transpose()?,
            bridge: files
                .socket
                .map(|path| Bridge::bind(path, files.peer))
                .transpose()?,
            poll: PollSchedule::new(poll),
            port,
            pcap_out: files.pcap_out.map(Path::to_path_buf),
            ..Self::offline()
        })
    }

    /// Reinstalling the device resets polling, but retains host backlog and the full TX archive.
    pub(crate) fn reset_poll(&mut self) {
        self.poll.reset();
    }

    pub(crate) fn reset_history(&mut self) {
        self.history.clear();
        self.next_sequence = 0;
    }

    /// Pcap input is preloaded directly, not paced or backpressured like the live socket.
    pub(crate) fn preload(&mut self, fabric: &mut Fabric, cycle: u64) {
        let frames = self.pcap_in.take();
        for frame in frames.iter().flatten() {
            self.inject(fabric, cycle, frame.clone());
        }
        self.pcap_in = frames;
    }

    pub(crate) fn poll_delay(&self, cycle: u64) -> Option<u64> {
        self.bridge.as_ref().map(|_| self.poll.delay(cycle))
    }

    pub(crate) fn service(&mut self, fabric: &mut Fabric, cycle: u64) {
        if self.bridge.is_none() || !self.poll.take_due(cycle) {
            return;
        }
        self.pending_receive
            .extend(self.bridge.as_mut().unwrap().receive());
        // A host socket can outrun the modelled wire. Wait for FIFO room rather than
        // manufacturing overruns from host scheduling.
        while let Some(length) = self.pending_receive.front().map(Vec::len) {
            if !fabric.emac_rx_has_room(self.port, length) {
                break;
            }
            let frame = self.pending_receive.pop_front().expect("frame");
            self.inject(fabric, cycle, frame);
        }
        self.collect(fabric, cycle);
        self.send_pending();
        // Retry a full peer even if firmware transmitted nothing this poll.
        self.bridge.as_mut().unwrap().flush();
    }

    /// Drains the MAC's transmit queue into the archive. Draining is destructive, so a state
    /// dump, the history and the pcap written at exit all read the archive, never the MAC.
    pub(crate) fn collect(&mut self, fabric: &mut Fabric, cycle: u64) {
        for frame in fabric.take_tx_frames() {
            self.record(cycle, false, &frame);
            self.transmitted.push(frame);
        }
    }

    fn inject(&mut self, fabric: &mut Fabric, cycle: u64, frame: Vec<u8>) {
        self.record(cycle, true, &frame);
        fabric.inject_frame_on(self.port, frame);
    }

    pub(crate) fn history_enabled(&self) -> bool {
        self.history_enabled
    }

    pub(crate) fn enable_history(&mut self, cycle: u64) {
        self.history_enabled = true;
        self.reset_history();
        // These frames were admitted before inspection was enabled; do not inject twice.
        let frames = self.pcap_in.take();
        for frame in frames.iter().flatten() {
            self.record(cycle, true, frame);
        }
        self.pcap_in = frames;
    }

    fn record(&mut self, cycle: u64, incoming: bool, bytes: &[u8]) {
        if !self.history_enabled {
            return;
        }
        if self.history.len() == ETHERNET_HISTORY_LIMIT {
            self.history.pop_front();
        }
        self.history.push_back(EthernetPacket {
            sequence: self.next_sequence,
            cycle,
            incoming,
            bytes: bytes.to_vec(),
        });
        self.next_sequence = self.next_sequence.wrapping_add(1);
    }

    pub(crate) fn history_json(&mut self, fabric: &mut Fabric, cycle: u64) -> Value {
        self.collect(fabric, cycle);
        json!(self
            .history
            .iter()
            .map(|packet| json!({
                "sequence": packet.sequence,
                "cycle": packet.cycle,
                "direction": if packet.incoming { "rx" } else { "tx" },
                "length": packet.bytes.len(),
                "hex": hex(&packet.bytes),
            }))
            .collect::<Vec<_>>())
    }

    /// Every frame the program has transmitted this session, in order.
    pub(crate) fn transmitted(&mut self, fabric: &mut Fabric, cycle: u64) -> &[Vec<u8>] {
        self.collect(fabric, cycle);
        &self.transmitted
    }

    /// Frames the live peer never took, because it fell too far behind.
    pub(crate) fn dropped(&self) -> u64 {
        self.bridge.as_ref().map_or(0, Bridge::dropped)
    }

    fn send_pending(&mut self) {
        if let Some(bridge) = self.bridge.as_mut() {
            for frame in &self.transmitted[self.sent_to_bridge..] {
                bridge.send(frame);
            }
            self.sent_to_bridge = self.transmitted.len();
        }
    }

    pub(crate) fn finish(&mut self, fabric: &mut Fabric, cycle: u64) -> Result<(), String> {
        self.collect(fabric, cycle);
        // Anything still queued when the program exits still belongs on the wire.
        self.send_pending();
        if let Some(path) = &self.pcap_out {
            pcap::write(path, &self.transmitted)?;
        }
        Ok(())
    }
}

pub(crate) fn hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

/// Jumbo-frame headroom: the EMAC accepts up to RXMAXLEN, which firmware may raise to 9018.
const MAX_FRAME: usize = 9_216;
/// How many transmitted frames may wait for a peer that is behind before the backend starts
/// reporting them dropped. A reader that is this far behind is not coming back.
const MAX_OUTBOUND: usize = 4_096;

struct Bridge {
    socket: UnixDatagram,
    bound: PathBuf,
    /// Where to send: a peer named on the command line, or whoever last sent us a frame.
    peer: Option<PathBuf>,
    learned: Option<SocketAddr>,
    scratch: Vec<u8>,
    /// Frames the peer's socket buffer had no room for, oldest first. A host reader that is
    /// momentarily behind is not a wire fault, and a frame lost here costs the firmware its own
    /// retransmission timeout - tens of emulated milliseconds, which is seconds of wall time.
    outbound: std::collections::VecDeque<Vec<u8>>,
    dropped: u64,
}

impl Bridge {
    /// Bind `path` and, if given, aim at `peer`. A stale socket file from an earlier run is
    /// removed first, which is what makes a crashed emulator recoverable without cleanup.
    fn bind(path: &Path, peer: Option<&Path>) -> Result<Self, String> {
        if path.exists() {
            std::fs::remove_file(path).map_err(|error| format!("{}: {error}", path.display()))?;
        }
        let socket =
            UnixDatagram::bind(path).map_err(|error| format!("{}: {error}", path.display()))?;
        socket
            .set_nonblocking(true)
            .map_err(|error| error.to_string())?;
        Ok(Self {
            socket,
            bound: path.to_path_buf(),
            peer: peer.map(Path::to_path_buf),
            learned: None,
            scratch: vec![0; MAX_FRAME],
            outbound: std::collections::VecDeque::new(),
            dropped: 0,
        })
    }

    /// Every frame waiting on the socket, oldest first. Never blocks.
    fn receive(&mut self) -> Vec<Vec<u8>> {
        let mut frames = Vec::new();
        loop {
            match self.socket.recv_from(&mut self.scratch) {
                Ok((length, from)) => {
                    if self.peer.is_none() && from.as_pathname().is_some() {
                        self.learned = Some(from);
                    }
                    // A runt shorter than an Ethernet header is not a frame; drop it rather than
                    // feed the MAC something it could never have seen on a wire.
                    if length >= 14 {
                        frames.push(self.scratch[..length].to_vec());
                    } else {
                        self.dropped += 1;
                    }
                }
                Err(error) if error.kind() == ErrorKind::WouldBlock => return frames,
                Err(error) if error.kind() == ErrorKind::Interrupted => continue,
                Err(_) => {
                    self.dropped += 1;
                    return frames;
                }
            }
        }
    }

    /// Hand a transmitted frame to the peer. A peer that is not listening yet is not an error:
    /// the firmware's first frames are usually gratuitous ARP that nobody is waiting for. A peer
    /// whose socket buffer is momentarily full is not an error either - the frame waits and goes
    /// out in order on a later poll, because dropping it would cost the firmware a
    /// retransmission timeout for a host scheduling hiccup.
    fn send(&mut self, frame: &[u8]) {
        self.flush();
        if !self.outbound.is_empty() {
            self.enqueue(frame);
            return;
        }
        if !self.try_send(frame) {
            self.enqueue(frame);
        }
    }

    /// Retry the frames a full peer socket refused, oldest first. The run loop calls this every
    /// poll, so a queue drains without the firmware having to transmit again.
    fn flush(&mut self) {
        while let Some(frame) = self.outbound.pop_front() {
            if !self.try_send(&frame) {
                self.outbound.push_front(frame);
                return;
            }
        }
    }

    fn enqueue(&mut self, frame: &[u8]) {
        if self.outbound.len() >= MAX_OUTBOUND {
            self.dropped += 1;
            return;
        }
        self.outbound.push_back(frame.to_vec());
    }

    /// Whether the frame left. `false` means the peer's buffer was full, which is the one
    /// failure worth retrying; anything else is reported as a drop, as before.
    fn try_send(&mut self, frame: &[u8]) -> bool {
        let result = match (&self.peer, &self.learned) {
            (Some(peer), _) => self.socket.send_to(frame, peer),
            (None, Some(address)) => match address.as_pathname() {
                Some(path) => self.socket.send_to(frame, path),
                None => return true,
            },
            (None, None) => return true,
        };
        match result {
            Ok(_) => true,
            Err(error) if error.kind() == ErrorKind::WouldBlock => false,
            Err(_) => {
                self.dropped += 1;
                true
            }
        }
    }

    fn dropped(&self) -> u64 {
        self.dropped
    }
}

impl Drop for Bridge {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.bound);
    }
}

/// A datagram channel for an external peripheral master - the HPI host, the UTOPIA's ATM
/// master - polled on its own schedule. The caller interprets requests; replies go to the
/// configured peer or the sender of the most recently received datagram.
pub struct HostLink {
    socket: UnixDatagram,
    bound: PathBuf,
    peer: Option<PathBuf>,
    learned: Option<SocketAddr>,
    poll: PollSchedule,
}

impl HostLink {
    pub fn bind(path: &Path, peer: Option<&Path>, poll_interval: u64) -> Result<Self, String> {
        if path.exists() {
            std::fs::remove_file(path).map_err(|error| format!("{}: {error}", path.display()))?;
        }
        let socket =
            UnixDatagram::bind(path).map_err(|error| format!("{}: {error}", path.display()))?;
        socket
            .set_nonblocking(true)
            .map_err(|error| error.to_string())?;
        Ok(Self {
            socket,
            bound: path.to_path_buf(),
            peer: peer.map(Path::to_path_buf),
            learned: None,
            poll: PollSchedule::new(poll_interval),
        })
    }

    /// A reinstalled device is polled on its first cycle.
    pub(crate) fn reset_poll(&mut self) {
        self.poll.reset();
    }

    /// Whether the link is due a look at `cycle`; taking a due poll schedules the next one.
    pub(crate) fn take_due(&mut self, cycle: u64) -> bool {
        self.poll.take_due(cycle)
    }

    /// How many cycles a batch may run before the next poll is due; never zero.
    pub(crate) fn poll_delay(&self, cycle: u64) -> u64 {
        self.poll.delay(cycle)
    }

    /// The next HPI access: direction, port and little-endian value.
    pub fn receive(&mut self) -> Option<(u8, u8, u32)> {
        let mut bytes = [0; 16];
        while let Some(length) = self.receive_datagram(&mut bytes) {
            if length >= 8 {
                return Some((
                    bytes[0],
                    bytes[1],
                    u32::from_le_bytes(bytes[4..8].try_into().unwrap()),
                ));
            }
        }
        None
    }

    /// Receive into the caller's bounded buffer without allocating.
    pub fn receive_datagram(&mut self, bytes: &mut [u8]) -> Option<usize> {
        loop {
            match self.socket.recv_from(bytes) {
                Ok((length, from)) => {
                    if self.peer.is_none() && from.as_pathname().is_some() {
                        self.learned = Some(from);
                    }
                    return Some(length);
                }
                Err(error) if error.kind() == ErrorKind::Interrupted => continue,
                Err(_) => return None,
            }
        }
    }

    pub fn reply(&mut self, bytes: &[u8]) {
        let _ = match (&self.peer, &self.learned) {
            (Some(peer), _) => self.socket.send_to(bytes, peer),
            (None, Some(address)) => match address.as_pathname() {
                Some(path) => self.socket.send_to(bytes, path),
                None => return,
            },
            (None, None) => return,
        };
    }
}

impl Drop for HostLink {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.bound);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    struct Files(PathBuf);

    impl Files {
        fn new() -> Self {
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let path = std::env::temp_dir().join(format!(
                "c64xemu-net-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir(&path).unwrap();
            Self(path)
        }

        fn path(&self, name: &str) -> PathBuf {
            self.0.join(name)
        }
    }

    impl Drop for Files {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// EMAC port 0 registers (SPRU975E Table 29).
    const MAC: u32 = 0x02c8_0000;
    const TXCONTROL: u32 = MAC + 0x004;
    const RXCONTROL: u32 = MAC + 0x014;
    const RXMBPENABLE: u32 = MAC + 0x100;
    const MACCONTROL: u32 = MAC + 0x160;
    const RXSOFOVERRUNS: u32 = MAC + 0x284;
    const RXMOFOVERRUNS: u32 = MAC + 0x288;
    const TXHDP0: u32 = MAC + 0x600;
    const RXHDP0: u32 = MAC + 0x620;
    /// RXMBPENABLE.RXCAFEN: copy every frame to the promiscuous channel, RXPROMCH = 0
    /// (SPRU975E Table 49).
    const RXCAFEN: u32 = 1 << 21;
    /// MACCONTROL.GMIIEN takes the interface out of reset, written after the DMA enables
    /// (SPRU975E Table 57 and section 2.15.4 step 17).
    const GMIIEN: u32 = 1 << 5;
    /// One four-word descriptor and its buffer in L2 (SPRU975E section 2.5).
    const DESCRIPTOR: u32 = 0x0080_1000;
    const BUFFER: u32 = 0x0080_2000;
    const SOP: u32 = 1 << 31;
    const EOP: u32 = 1 << 30;
    const OWNER: u32 = 1 << 29;

    fn transmit(fabric: &mut Fabric, frame: &[u8]) {
        for (index, bytes) in frame.chunks_exact(4).enumerate() {
            fabric.write32(
                0,
                BUFFER + index as u32 * 4,
                u32::from_le_bytes(bytes.try_into().unwrap()),
            );
        }
        fabric.write32(0, DESCRIPTOR, 0);
        fabric.write32(0, DESCRIPTOR + 4, BUFFER);
        fabric.write32(0, DESCRIPTOR + 8, frame.len() as u32);
        fabric.write32(0, DESCRIPTOR + 12, OWNER | SOP | EOP | frame.len() as u32);
        fabric.write32(0, TXCONTROL, 1);
        fabric.write32(0, MACCONTROL, GMIIEN);
        fabric.write32(0, TXHDP0, DESCRIPTOR);
        for _ in 0..256 {
            fabric.tick_once();
        }
    }

    #[test]
    fn inspection_and_repeated_finish_preserve_capture_without_duplicate_socket_tx() {
        let files = Files::new();
        let peer = UnixDatagram::bind(files.path("peer")).unwrap();
        peer.set_nonblocking(true).unwrap();
        let mut ethernet = Ethernet::open(
            16,
            0,
            EthernetFiles {
                pcap_out: Some(&files.path("capture")),
                socket: Some(&files.path("emulator")),
                peer: Some(&files.path("peer")),
                ..EthernetFiles::default()
            },
        )
        .unwrap();
        let mut fabric = Fabric::default();
        ethernet.enable_history(0);
        let frame = vec![0x42; 64];
        transmit(&mut fabric, &frame);
        assert_eq!(
            ethernet.history_json(&mut fabric, 256)[0]["hex"],
            hex(&frame)
        );
        assert_eq!(
            ethernet.transmitted(&mut fabric, 256),
            std::slice::from_ref(&frame)
        );
        ethernet.service(&mut fabric, 256);
        ethernet.finish(&mut fabric, 256).unwrap();
        ethernet.finish(&mut fabric, 256).unwrap();
        let mut received = [0; 128];
        let length = peer.recv(&mut received).unwrap();
        assert_eq!(&received[..length], frame);
        assert_eq!(
            peer.recv(&mut received).unwrap_err().kind(),
            ErrorKind::WouldBlock
        );
        assert_eq!(pcap::read(&files.path("capture")).unwrap(), vec![frame]);
    }

    #[test]
    fn bounded_history_does_not_truncate_capture_or_reload_archive() {
        let files = Files::new();
        let capture = files.path("capture");
        let mut ethernet = Ethernet::open(
            1,
            0,
            EthernetFiles {
                pcap_out: Some(&capture),
                ..EthernetFiles::default()
            },
        )
        .unwrap();
        let mut fabric = Fabric::default();
        ethernet.enable_history(0);
        let mut expected = Vec::new();
        for index in 0..260u32 {
            let mut frame = vec![0x42; 64];
            frame[..4].copy_from_slice(&index.to_le_bytes());
            transmit(&mut fabric, &frame);
            ethernet.collect(&mut fabric, u64::from(index));
            expected.push(frame);
        }
        let history = ethernet.history_json(&mut fabric, 260);
        assert_eq!(history.as_array().unwrap().len(), 256);
        assert_eq!(history[0]["sequence"], 4);
        assert_eq!(history[255]["hex"], hex(&expected[259]));
        ethernet.reset_history();
        ethernet.reset_poll();
        fabric = Fabric::default();
        assert_eq!(ethernet.history_json(&mut fabric, 0), json!([]));
        let frame = vec![0x24; 64];
        transmit(&mut fabric, &frame);
        expected.push(frame);
        assert_eq!(ethernet.history_json(&mut fabric, 1)[0]["sequence"], 0);
        ethernet.finish(&mut fabric, 1).unwrap();
        assert_eq!(pcap::read(&capture).unwrap(), expected);
    }

    #[test]
    fn live_receive_waits_for_fifo_room_and_admits_backlog_only_when_due() {
        let files = Files::new();
        let peer = UnixDatagram::bind(files.path("peer")).unwrap();
        let mut ethernet = Ethernet::open(
            10,
            0,
            EthernetFiles {
                socket: Some(&files.path("emulator")),
                ..EthernetFiles::default()
            },
        )
        .unwrap();
        let mut fabric = Fabric::default();
        ethernet.enable_history(0);
        // Occupy all 68 FIFO cells without a running receive DMA.
        let filler = vec![0x44; 64];
        while fabric.emac_rx_has_room(0, filler.len()) {
            fabric.inject_frame_on(0, filler.clone());
        }
        let frame = vec![0x42; 64];
        peer.send_to(&frame, files.path("emulator")).unwrap();
        ethernet.service(&mut fabric, 0);
        assert_eq!(ethernet.history_json(&mut fabric, 0), json!([]));
        assert_eq!(ethernet.poll_delay(0), Some(10));
        // Rebuilding the device clears its FIFO, not the backend's pending host frames.
        fabric = Fabric::default();
        fabric.write32(0, DESCRIPTOR, 0);
        fabric.write32(0, DESCRIPTOR + 4, BUFFER);
        fabric.write32(0, DESCRIPTOR + 8, 256);
        fabric.write32(0, DESCRIPTOR + 12, OWNER);
        fabric.write32(0, RXMBPENABLE, RXCAFEN);
        fabric.write32(0, RXCONTROL, 1);
        fabric.write32(0, MACCONTROL, GMIIEN);
        fabric.write32(0, RXHDP0, DESCRIPTOR);
        ethernet.service(&mut fabric, 9);
        assert_eq!(ethernet.history_json(&mut fabric, 9), json!([]));
        ethernet.service(&mut fabric, 10);
        let history = ethernet.history_json(&mut fabric, 10);
        assert_eq!(history[0]["cycle"], 10);
        assert_eq!(history[0]["direction"], "rx");
        assert_eq!(history[0]["hex"], hex(&frame));
        for _ in 0..256 {
            fabric.tick_once();
        }
        assert_eq!(fabric.read_bytes(0, BUFFER, 64), frame);
        assert_eq!(fabric.read32(0, DESCRIPTOR + 12) & OWNER, 0);
        assert_eq!(fabric.read32(0, RXSOFOVERRUNS), 0, "no FIFO overrun");
        assert_eq!(
            fabric.read32(0, RXMOFOVERRUNS),
            0,
            "no partial FIFO overrun"
        );
    }
}
