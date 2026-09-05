//! A live Ethernet backend: raw frames over a Unix datagram socket.
//!
//! The emulator is single-threaded and cycle-stepped while a host network is asynchronous, so the
//! backend is polled from the run loop at a fixed cycle interval and never touches descriptor
//! memory itself - the frames it collects go through the same `inject_frame` path a pcap replay
//! uses. One datagram carries exactly one Ethernet frame, so no framing is needed on the wire.
//!
//! Keeping the socket here rather than a TAP device keeps the emulator free of ioctl bindings and
//! of any privilege requirement: `tools/tap_bridge.py` owns `/dev/net/tun` and relays frames to
//! this socket, and a test script can take the same place to drive a deterministic exchange.

use std::io::ErrorKind;
use std::os::unix::net::{SocketAddr, UnixDatagram};
use std::path::{Path, PathBuf};

/// Jumbo-frame headroom: the EMAC accepts up to RXMAXLEN, which firmware may raise to 9018.
const MAX_FRAME: usize = 9_216;
/// How many transmitted frames may wait for a peer that is behind before the backend starts
/// reporting them dropped. A reader that is this far behind is not coming back.
const MAX_OUTBOUND: usize = 4_096;

pub struct Bridge {
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
    pub fn bind(path: &Path, peer: Option<&Path>) -> Result<Self, String> {
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
    pub fn receive(&mut self) -> Vec<Vec<u8>> {
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
    pub fn send(&mut self, frame: &[u8]) {
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
    pub fn flush(&mut self) {
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

    pub fn dropped(&self) -> u64 {
        self.dropped
    }
}

impl Drop for Bridge {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.bound);
    }
}

/// A datagram channel for a host process that is not a network: the HPI's host side. One
/// datagram is one host access - `[direction, port, 0, 0, value]`, where direction is `r` or `w`
/// and port is the HCNTL encoding - and a read is answered with its four value bytes, or with an
/// empty datagram when the software reset is holding the host off (SPRU969B section 8.3).
pub struct HostLink {
    socket: UnixDatagram,
    bound: PathBuf,
    peer: Option<PathBuf>,
    learned: Option<SocketAddr>,
    scratch: [u8; 16],
}

impl HostLink {
    pub fn bind(path: &Path, peer: Option<&Path>) -> Result<Self, String> {
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
            scratch: [0; 16],
        })
    }

    /// The next host access waiting, if any: direction, port and value.
    pub fn receive(&mut self) -> Option<(u8, u8, u32)> {
        loop {
            match self.socket.recv_from(&mut self.scratch) {
                Ok((length, from)) => {
                    if self.peer.is_none() && from.as_pathname().is_some() {
                        self.learned = Some(from);
                    }
                    if length < 8 {
                        continue;
                    }
                    let value = u32::from_le_bytes(self.scratch[4..8].try_into().unwrap());
                    return Some((self.scratch[0], self.scratch[1], value));
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
