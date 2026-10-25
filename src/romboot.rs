//! The ROM bootloader's Ethernet boot mode: the boot the C6472 performs with BOOTMODE[3:0] set
//! to 9 or 10 (SPRUEC6G section 4.3.5, SPRS612G Table 2-4).
//!
//! Core 0 runs the on-chip ROM while cores 1-5 stay internally held in reset. The ROM brings up
//! one Ethernet MAC, sends a single BOOTP-shaped "Ethernet-ready" announcement so the host knows
//! which device is asking to be booted, then accepts UDP frames whose payloads are fragments of
//! an ordinary boot table (SPRUEC6G section 6.2.2). Each fragment is handed to boot-table
//! processing as it arrives, so the sections land wherever the table says - including another
//! core's memory through the global aliases, and the boot controller's own registers, which is
//! how the table both places the other cores' images and releases them. When the table's zero
//! count arrives the ROM clears the receive channel's head descriptor pointer and branches core 0
//! to the entry point the table named.
//!
//! The ROM's *instructions* are not modelled: TI ships them in on-chip ROM this project has no
//! copy of, and the manual explicitly warns that their addresses change between silicon
//! revisions. What is modelled is everything the ROM does that software or a host can observe -
//! the register programming, the frames on the wire, the memory writes, the release of the other
//! cores and the entry point - performed by this state machine on core 0's behalf while core 0
//! itself is held. So the device's own Ethernet MAC, its receive filter and its CPPI descriptors
//! are exercised exactly as they are by firmware: the frames arrive through whichever backend the
//! command line wired up, and the announcement leaves through the same path a transmitted frame
//! always takes.
//!
//! The boot code resets the MAC before it configures anything, so a frame that arrived before the
//! announcement went out is gone with the rest of the MAC's state. That is what the announcement
//! is for: a host waits for it and only then starts sending, and nothing in this protocol is
//! acknowledged or retransmitted.
//!
//! Two deliberate departures from the letter of SPRUEC6G section 4.3.5, both forced by the
//! device model rather than chosen:
//!
//! * TI's ROM keeps its ten 1600-byte packet buffers in L1D at 0x00F0 0000. No DMA master in
//!   this model can reach a megamodule's *local* L1D window - SPRS612G Table 2-2 marks it "DATA
//!   SPACE ON DMA" - so the descriptor ring and its buffers live in the top of core 0's L2
//!   instead, addressed through core 0's global alias so that the MAC and the boot code name the
//!   same bytes. That is inside the L2 region section 4.1 already reserves for the boot code.
//! * The MAC address comes from the fused MAC ID words at 0x02A8 0700, as section 4.3.5 says,
//!   but no public fuse value exists; the command line fuses one (`--mac-id`).

use crate::boot::BootStream;
use crate::system::{CoreBus, Endian, Fabric};

/// BOOTMODE[3:0] 9 and 10 are MAC port 0 and MAC port 1 (SPRS612G Table 2-4, SPRUEC6G Table 22).
pub(crate) const BOOTMODE_MAC_PORT0: u8 = 9;
pub(crate) const BOOTMODE_MAC_PORT1: u8 = 10;

/// Which MAC port a boot mode selects, for the boot modes that select one at all.
pub(crate) fn mac_boot_port(boot_mode: u8) -> Option<usize> {
    match boot_mode {
        BOOTMODE_MAC_PORT0 => Some(0),
        BOOTMODE_MAC_PORT1 => Some(1),
        _ => None,
    }
}

/// The fused MAC ID words (SPRS612G Table 3-2), which this mode reads its own address from.
const MACID0: u32 = 0x02a8_0700;
const MACID1: u32 = 0x02a8_0704;

/// EMAC register offsets from the port's base (SPRU975E Table 29, SPRAAU2A section 3).
const TXCONTROL: u32 = 0x004;
const RXCONTROL: u32 = 0x014;
const RXMBPENABLE: u32 = 0x100;
const RXUNICASTSET: u32 = 0x104;
const RXMAXLEN: u32 = 0x10c;
const RXBUFFEROFFSET: u32 = 0x110;
const RXFREEBUFFER0: u32 = 0x140;
const MACCONTROL: u32 = 0x160;
const SOFTRESET: u32 = 0x174;
const MACSRCADDRLO: u32 = 0x1d0;
const MACSRCADDRHI: u32 = 0x1d4;
const MACADDRLO: u32 = 0x500;
const MACADDRHI: u32 = 0x504;
const MACINDEX: u32 = 0x508;
const TXHDP0: u32 = 0x600;
const RXHDP0: u32 = 0x620;
const TXCP0: u32 = 0x640;
const RXCP0: u32 = 0x660;

/// MACCONTROL: GMII enabled, full duplex (SPRU975E Table 46). The boot parameter table's
/// duplex and interface bits (SPRUEC6G Table 27) have no meaning without a PHY model.
const MACCONTROL_FULLDUPLEX: u32 = 1 << 0;
const MACCONTROL_GMIIEN: u32 = 1 << 5;
/// RXMBPENABLE.RXBROADEN with RXBROADCH = 0: broadcast frames are accepted on channel 0
/// (SPRU975E Table 49). The unicast address match is enabled through RXUNICASTSET instead.
const RXMBP_BROADCAST_CHANNEL0: u32 = 1 << 13;
/// MACADDRLO: VALID and MATCHFILT with CHANNEL = 0, which makes the slot an address the
/// receiver matches rather than one it filters out (SPRU975E section 5.44).
const MACADDR_VALID_MATCH: u32 = (1 << 20) | (1 << 19);
/// The longest frame the receiver accepts. Anything longer is truncated and marked oversized.
const RX_MAX_LENGTH: u32 = 1518;

/// CPPI buffer descriptor flags (SPRU975E section 2.5.5).
const SOP: u32 = 1 << 31;
const EOP: u32 = 1 << 30;
const OWNER: u32 = 1 << 29;
/// The status bits that say the frame in a descriptor is not one to process.
const RX_BAD_STATUS: u32 = (1 << 25)   // JABBER
    | (1 << 24)                        // OVERSIZE
    | (1 << 23)                        // FRAGMENT
    | (1 << 22)                        // UNDERSIZED
    | (1 << 21)                        // CONTROL
    | (1 << 20)                        // OVERRUN
    | (1 << 19)                        // CODEERROR
    | (1 << 18)                        // ALIGNERROR
    | (1 << 17)                        // CRCERROR
    | (1 << 16); // NOMATCH

/// The receive ring: ten buffers of 1600 bytes, the geometry SPRUEC6G section 4.3.5 gives, in
/// the 20 KB below the L2 scratch the boot code reserves, through core 0's global alias.
const RING_BASE: u32 = 0x1089_1000;
const TX_DESCRIPTOR: u32 = RING_BASE;
const TX_BUFFER: u32 = RING_BASE + 0x010;
const RX_DESCRIPTORS: u32 = RING_BASE + 0x200;
const RX_BUFFERS_BASE: u32 = RING_BASE + 0x400;
const RX_BUFFERS: usize = 10;
const RX_BUFFER_SIZE: u32 = 1600;
const DESCRIPTOR_BYTES: u32 = 16;

/// The boot-table frame header (SPRUEC6G Table 28): a magic halfword, then an opcode byte and a
/// sequence byte, big-endian on the wire like every other header in the frame.
const FRAME_MAGIC: u16 = 0x544b;
const FRAME_OPCODE: u8 = 0x01;
const FRAME_HEADER_BYTES: usize = 4;

const ETHERTYPE_IPV4: u16 = 0x0800;
const ETHERTYPE_VLAN: u16 = 0x8100;
const IP_PROTOCOL_UDP: u8 = 17;
/// The announcement is a BOOTP request, client port to server port (SPRUEC6G section 4.3.5.2).
const BOOTP_CLIENT_PORT: u16 = 68;
const BOOTP_SERVER_PORT: u16 = 67;
const BOOTP_PAYLOAD_BYTES: usize = 300;
const BOOT_FILE_PREFIX: &[u8] = b"ti-boot-table-";
const BOOT_SERVER_NAME: &[u8] = b"ti-boot-table-svr";

/// What the ROM is doing.
enum Phase {
    /// Nothing has been programmed yet; the first cycle of the boot does all of it.
    Configure,
    /// The announcement is posted on the transmit channel and its descriptor is being watched.
    Announce,
    /// Frames are being taken off the receive ring and fed to boot-table processing.
    Receive,
    /// The table terminated and core 0 has been given its entry point.
    Done,
}

/// The boot parameters this mode runs with (SPRUEC6G Table 27), reduced to the ones that mean
/// something to a device model with no PHY and no I2C EEPROM behind it.
pub(crate) struct BootParameters {
    /// The UDP destination port boot-table frames must arrive on. TI's default is 9, the
    /// discard port.
    ///
    /// UNCERTAIN: SPRUEC6G Table 27 describes this field as "destination UDP port used for
    /// Ethernet-ready frame" while section 4.3.5.3 says the boot-table frames' ports are the
    /// ones "specified in the boot parameters" and section 4.3.5.2 fixes the announcement's
    /// ports at the BOOTP pair. Both statements can only hold if the field is the port the boot
    /// frames use, which is how it is modelled.
    pub(crate) udp_destination_port: u16,
    /// The UDP source port to accept; zero accepts any (Table 27).
    pub(crate) udp_source_port: u16,
    /// The second address the receiver accepts besides its own, TI's "M-MAC". The default is
    /// all ones, i.e. broadcast.
    pub(crate) group_mac: [u8; 6],
    /// Where the announcement is sent, TI's "H-MAC". The default is all ones.
    pub(crate) host_mac: [u8; 6],
    /// The four characters the announcement's file name ends with (Table 27's device id).
    pub(crate) device_id: [u8; 4],
}

impl Default for BootParameters {
    fn default() -> Self {
        Self {
            udp_destination_port: 9,
            udp_source_port: 0,
            group_mac: [0xff; 6],
            host_mac: [0xff; 6],
            device_id: *b"0000",
        }
    }
}

/// What the boot has done so far, for a front end to report.
#[derive(Clone, Copy, Default)]
pub(crate) struct Progress {
    pub(crate) accepted: u64,
    pub(crate) discarded: u64,
    pub(crate) table_bytes: u64,
    pub(crate) entry: Option<u32>,
}

pub(crate) struct EmacBoot {
    /// The register base of the MAC port the BOOTMODE pins chose.
    base: u32,
    parameters: BootParameters,
    /// The device's own address, read from the fused MAC ID words at configuration time.
    mac: [u8; 6],
    phase: Phase,
    stream: BootStream,
    /// The ring entry the boot code looks at next, and the frame sequence number it expects.
    next_descriptor: usize,
    sequence: u8,
    progress: Progress,
}

impl EmacBoot {
    pub(crate) fn new(base: u32, endian: Endian, parameters: BootParameters) -> Self {
        Self {
            base,
            parameters,
            mac: [0; 6],
            phase: Phase::Configure,
            stream: BootStream::new(endian),
            next_descriptor: 0,
            sequence: 0,
            progress: Progress::default(),
        }
    }

    pub(crate) fn progress(&self) -> Progress {
        self.progress
    }

    /// One cycle of the ROM's boot loop. Returns the entry point once the boot table has ended,
    /// which is the device's signal to start core 0 there.
    pub(crate) fn tick(&mut self, fabric: &mut Fabric) -> Option<u32> {
        // The boot code runs on core 0, so its accesses are core 0's accesses.
        let mut bus = CoreBus::new(0, fabric);
        match self.phase {
            Phase::Configure => {
                self.configure(&mut bus);
                self.phase = Phase::Announce;
                None
            }
            Phase::Announce => {
                // The announcement is sent once and nothing answers it, so the only thing to
                // wait for is the MAC handing the descriptor back.
                if bus.read32(TX_DESCRIPTOR + 12) & OWNER == 0 {
                    bus.write32(self.base + TXCP0, TX_DESCRIPTOR);
                    self.phase = Phase::Receive;
                }
                None
            }
            Phase::Receive => {
                if let Some(entry) = self.receive(&mut bus) {
                    // Clearing the head descriptor pointer stops reception, which is how the
                    // ROM leaves the MAC on the way out (SPRUEC6G section 4.3.5).
                    bus.write32(self.base + RXHDP0, 0);
                    self.progress.entry = Some(entry);
                    self.phase = Phase::Done;
                    return Some(entry);
                }
                None
            }
            Phase::Done => None,
        }
    }

    /// Bring up the MAC, post the receive ring and transmit the announcement.
    fn configure(&mut self, bus: &mut CoreBus<'_>) {
        let base = self.base;
        let fused = [bus.read32(MACID0), bus.read32(MACID1)];
        self.mac = [
            (fused[0] >> 24) as u8,
            (fused[0] >> 16) as u8,
            (fused[0] >> 8) as u8,
            fused[0] as u8,
            (fused[1] >> 24) as u8,
            (fused[1] >> 16) as u8,
        ];
        bus.write32(base + SOFTRESET, 1);
        let address_high = u32::from_le_bytes([self.mac[0], self.mac[1], self.mac[2], self.mac[3]]);
        let address_low = (u32::from(self.mac[5]) << 8) | u32::from(self.mac[4]);
        bus.write32(base + MACINDEX, 0);
        bus.write32(base + MACADDRHI, address_high);
        bus.write32(base + MACADDRLO, address_low | MACADDR_VALID_MATCH);
        bus.write32(base + MACSRCADDRHI, address_high);
        bus.write32(base + MACSRCADDRLO, address_low);
        // A single matched address plus broadcast, which is the filter SPRUEC6G section 4.3.5
        // describes; the group address is compared by the boot code itself.
        bus.write32(base + RXMBPENABLE, RXMBP_BROADCAST_CHANNEL0);
        bus.write32(base + RXUNICASTSET, 1);
        bus.write32(base + RXMAXLEN, RX_MAX_LENGTH);
        bus.write32(base + RXBUFFEROFFSET, 0);
        bus.write32(base + MACCONTROL, MACCONTROL_GMIIEN | MACCONTROL_FULLDUPLEX);
        // The ring is circular, so the MAC never runs off its end and never needs the head
        // pointer rewritten: it stalls on the first descriptor the boot code has not given back
        // and picks up again when it does (SPRU975E section 2.10.5).
        for index in 0..RX_BUFFERS {
            let descriptor = descriptor_address(index);
            let next = descriptor_address((index + 1) % RX_BUFFERS);
            bus.write32(descriptor, next);
            bus.write32(descriptor + 4, buffer_address(index));
            bus.write32(descriptor + 8, RX_BUFFER_SIZE);
            bus.write32(descriptor + 12, OWNER);
        }
        bus.write32(base + RXFREEBUFFER0, RX_BUFFERS as u32);
        bus.write32(base + RXCONTROL, 1);
        bus.write32(base + TXCONTROL, 1);
        bus.write32(base + RXHDP0, descriptor_address(0));

        let announcement = self.announcement();
        bus.host_write_bytes(TX_BUFFER, &announcement);
        let length = announcement.len() as u32;
        bus.write32(TX_DESCRIPTOR, 0);
        bus.write32(TX_DESCRIPTOR + 4, TX_BUFFER);
        bus.write32(TX_DESCRIPTOR + 8, length);
        bus.write32(TX_DESCRIPTOR + 12, SOP | EOP | OWNER | length);
        bus.write32(base + TXHDP0, TX_DESCRIPTOR);
    }

    /// Take at most one frame off the ring and process it, the way an interrupt-driven ROM
    /// handles one receive interrupt. Returns the entry point when the table has ended.
    fn receive(&mut self, bus: &mut CoreBus<'_>) -> Option<u32> {
        let descriptor = descriptor_address(self.next_descriptor);
        let flags = bus.read32(descriptor + 12);
        if flags & OWNER != 0 {
            return None;
        }
        let length = (flags & 0xffff) as usize;
        let frame = if flags & (SOP | EOP) == (SOP | EOP) && flags & RX_BAD_STATUS == 0 {
            bus.host_read_bytes(buffer_address(self.next_descriptor), length)
        } else {
            // A frame the MAC could only place in pieces, or one it marked, is not a frame this
            // protocol can use; the checks below then count it as discarded.
            Vec::new()
        };
        bus.write32(self.base + RXCP0, descriptor);
        bus.write32(descriptor + 8, RX_BUFFER_SIZE);
        bus.write32(descriptor + 12, OWNER);
        bus.write32(self.base + RXFREEBUFFER0, 1);
        self.next_descriptor = (self.next_descriptor + 1) % RX_BUFFERS;
        self.process(bus, &frame)
    }

    /// Validate one frame and hand its payload to boot-table processing.
    fn process(&mut self, bus: &mut CoreBus<'_>, frame: &[u8]) -> Option<u32> {
        let Some((sequence, payload)) = self.boot_frame(frame) else {
            self.progress.discarded += 1;
            return None;
        };
        if sequence != self.sequence {
            // There is no retransmission in this protocol, so a gap is fatal to the boot; it is
            // reported through the frame counters rather than guessed at.
            self.progress.discarded += 1;
            return None;
        }
        self.sequence = self.sequence.wrapping_add(1);
        self.progress.accepted += 1;
        self.progress.table_bytes += payload.len() as u64;
        match self.stream.feed(payload, bus) {
            Ok(true) => self.stream.entry(),
            Ok(false) => None,
            Err(_) => {
                // A table this malformed cannot name an entry point; the boot stalls, which is
                // what the ROM does when the frames stop making sense.
                self.progress.discarded += 1;
                None
            }
        }
    }

    /// The frame checks of SPRUEC6G section 4.3.5.3, in the order the manual lists them: MAC
    /// header (DIX or 802.3 with SNAP/LLC, with or without a VLAN tag), IPv4, UDP, then the
    /// boot-table frame header. Anything that fails is silently discarded.
    fn boot_frame<'frame>(&self, frame: &'frame [u8]) -> Option<(u8, &'frame [u8])> {
        let destination = frame.get(0..6)?;
        if destination != self.mac && destination != self.parameters.group_mac {
            return None;
        }
        let mut cursor = 12;
        let mut type_or_length = be16(frame, cursor)?;
        if type_or_length == ETHERTYPE_VLAN {
            // The VLAN fields other than the type that follows them are ignored.
            cursor += 4;
            type_or_length = be16(frame, cursor)?;
        }
        cursor += 2;
        if type_or_length < 0x0600 {
            // 802.3: the LLC header must be the SNAP one, and the type is in the SNAP header.
            if frame.get(cursor..cursor + 3)? != [0xaa, 0xaa, 0x03] {
                return None;
            }
            cursor += 6;
            if be16(frame, cursor)? != ETHERTYPE_IPV4 {
                return None;
            }
            cursor += 2;
        } else if type_or_length != ETHERTYPE_IPV4 {
            return None;
        }
        let ip = frame.get(cursor..)?;
        if ip.len() < 20 || ip[0] >> 4 != 4 {
            return None;
        }
        let header_length = usize::from(ip[0] & 0xf) * 4;
        if header_length < 20 || ip.len() < header_length {
            return None;
        }
        if ip[9] != IP_PROTOCOL_UDP {
            return None;
        }
        // Neither flag bit that matters nor a fragment offset: this mode reassembles nothing.
        if be16(ip, 6)? & 0x3fff != 0 {
            return None;
        }
        let total_length = usize::from(be16(ip, 2)?);
        if total_length < header_length + 8 || total_length > ip.len() {
            return None;
        }
        let udp = &ip[header_length..total_length];
        let source_port = be16(udp, 0)?;
        let destination_port = be16(udp, 2)?;
        let udp_length = usize::from(be16(udp, 4)?);
        if destination_port != self.parameters.udp_destination_port {
            return None;
        }
        if self.parameters.udp_source_port != 0 && source_port != self.parameters.udp_source_port {
            return None;
        }
        if udp_length < 8 + FRAME_HEADER_BYTES || udp_length > udp.len() || udp_length % 2 != 0 {
            return None;
        }
        let udp = &udp[..udp_length];
        if be16(udp, 6)? != 0 && udp_checksum(&ip[12..20], udp) != 0 {
            return None;
        }
        let body = &udp[8..];
        if be16(body, 0)? != FRAME_MAGIC {
            return None;
        }
        let opcode_sequence = be16(body, 2)?;
        if (opcode_sequence >> 8) as u8 != FRAME_OPCODE {
            return None;
        }
        let payload = &body[FRAME_HEADER_BYTES..];
        if payload.len() % 4 != 0 {
            return None;
        }
        Some((opcode_sequence as u8, payload))
    }

    /// The Ethernet-ready announcement of SPRUEC6G section 4.3.5.2: a BOOTP request built so
    /// that a real BOOTP or DHCP server discards it, sent once and never retransmitted.
    fn announcement(&self) -> Vec<u8> {
        let mut bootp = vec![0u8; BOOTP_PAYLOAD_BYTES];
        bootp[0] = 1; // request
        bootp[1] = 1; // Ethernet
        bootp[2] = 6; // hardware address length
        bootp[4..8].copy_from_slice(&0x1234_5678u32.to_be_bytes());
        bootp[8..10].copy_from_slice(&1u16.to_be_bytes()); // seconds
        bootp[28..34].copy_from_slice(&self.mac);
        bootp[44..44 + BOOT_SERVER_NAME.len()].copy_from_slice(BOOT_SERVER_NAME);
        let file = &mut bootp[108..108 + BOOT_FILE_PREFIX.len() + 4];
        file[..BOOT_FILE_PREFIX.len()].copy_from_slice(BOOT_FILE_PREFIX);
        file[BOOT_FILE_PREFIX.len()..].copy_from_slice(&self.parameters.device_id);

        let mut udp = Vec::with_capacity(8 + bootp.len());
        udp.extend_from_slice(&BOOTP_CLIENT_PORT.to_be_bytes());
        udp.extend_from_slice(&BOOTP_SERVER_PORT.to_be_bytes());
        udp.extend_from_slice(&((8 + bootp.len()) as u16).to_be_bytes());
        udp.extend_from_slice(&0u16.to_be_bytes()); // the manual does not calculate it either
        udp.extend_from_slice(&bootp);

        let mut ip = Vec::with_capacity(20 + udp.len());
        ip.push(0x45);
        ip.push(0);
        ip.extend_from_slice(&((20 + udp.len()) as u16).to_be_bytes());
        ip.extend_from_slice(&1u16.to_be_bytes()); // identification
        ip.extend_from_slice(&0u16.to_be_bytes()); // no flags, no fragment
        ip.push(0x10); // time to live
        ip.push(IP_PROTOCOL_UDP);
        ip.extend_from_slice(&0u16.to_be_bytes()); // checksum, filled in below
        ip.extend_from_slice(&[0; 4]); // source 0.0.0.0
        ip.extend_from_slice(&[0; 4]); // destination 0.0.0.0
        let checksum = internet_checksum(&ip[..20]);
        ip[10..12].copy_from_slice(&checksum.to_be_bytes());
        ip.extend_from_slice(&udp);

        let mut frame = Vec::with_capacity(14 + ip.len());
        frame.extend_from_slice(&self.parameters.host_mac);
        frame.extend_from_slice(&self.mac);
        frame.extend_from_slice(&ETHERTYPE_IPV4.to_be_bytes());
        frame.extend_from_slice(&ip);
        frame
    }
}

fn descriptor_address(index: usize) -> u32 {
    RX_DESCRIPTORS + index as u32 * DESCRIPTOR_BYTES
}

fn buffer_address(index: usize) -> u32 {
    RX_BUFFERS_BASE + index as u32 * RX_BUFFER_SIZE
}

fn be16(bytes: &[u8], offset: usize) -> Option<u16> {
    Some(u16::from_be_bytes(
        bytes.get(offset..offset + 2)?.try_into().ok()?,
    ))
}

/// The internet checksum of a byte range (RFC 1071), zero-padded to a whole halfword: the value
/// a header field carries, and zero when computed over a range that already holds a right one.
fn internet_checksum(bytes: &[u8]) -> u16 {
    let mut sum = 0u32;
    let mut chunks = bytes.chunks_exact(2);
    for chunk in &mut chunks {
        sum += u32::from(u16::from_be_bytes([chunk[0], chunk[1]]));
    }
    if let [last] = chunks.remainder() {
        sum += u32::from(*last) << 8;
    }
    while sum >> 16 != 0 {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    !(sum as u16)
}

/// The UDP checksum over its pseudo-header (RFC 768): the addresses, the protocol and the UDP
/// length, then the datagram itself. Computed over a datagram that carries its own checksum
/// this is zero exactly when that checksum is right.
fn udp_checksum(addresses: &[u8], udp: &[u8]) -> u16 {
    let mut block = Vec::with_capacity(addresses.len() + 4 + udp.len());
    block.extend_from_slice(addresses);
    block.push(0);
    block.push(IP_PROTOCOL_UDP);
    block.extend_from_slice(&(udp.len() as u16).to_be_bytes());
    block.extend_from_slice(udp);
    internet_checksum(&block)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The header fields a boot-table frame needs, with the parts a test varies as arguments.
    fn boot_frame(
        destination: [u8; 6],
        destination_port: u16,
        sequence: u8,
        payload: &[u8],
    ) -> Vec<u8> {
        let mut udp = Vec::new();
        udp.extend_from_slice(&4096u16.to_be_bytes());
        udp.extend_from_slice(&destination_port.to_be_bytes());
        udp.extend_from_slice(&((8 + FRAME_HEADER_BYTES + payload.len()) as u16).to_be_bytes());
        udp.extend_from_slice(&0u16.to_be_bytes());
        udp.extend_from_slice(&FRAME_MAGIC.to_be_bytes());
        udp.extend_from_slice(&[FRAME_OPCODE, sequence]);
        udp.extend_from_slice(payload);

        let mut ip = vec![0x45, 0];
        ip.extend_from_slice(&((20 + udp.len()) as u16).to_be_bytes());
        ip.extend_from_slice(&[0, 0, 0, 0, 64, IP_PROTOCOL_UDP, 0, 0]);
        ip.extend_from_slice(&[192, 168, 64, 1]);
        ip.extend_from_slice(&[192, 168, 64, 10]);
        let checksum = internet_checksum(&ip[..20]);
        ip[10..12].copy_from_slice(&checksum.to_be_bytes());
        ip.extend_from_slice(&udp);

        let mut frame = Vec::new();
        frame.extend_from_slice(&destination);
        frame.extend_from_slice(&[0x02, 0, 0, 0, 0, 1]);
        frame.extend_from_slice(&ETHERTYPE_IPV4.to_be_bytes());
        frame.extend_from_slice(&ip);
        // Every frame on the wire is padded to the 60-byte minimum by its sender.
        frame.resize(frame.len().max(60), 0);
        frame
    }

    fn boot() -> EmacBoot {
        let mut boot = EmacBoot::new(0x02c8_0000, Endian::Little, BootParameters::default());
        boot.mac = [0x02, 0x00, 0x00, 0x00, 0x64, 0x72];
        boot
    }

    #[test]
    fn a_boot_frame_for_this_device_is_accepted_with_its_payload() {
        let boot = boot();
        let payload = [1u8, 2, 3, 4, 5, 6, 7, 8];
        let frame = boot_frame(boot.mac, 9, 0, &payload);
        assert_eq!(boot.boot_frame(&frame), Some((0, &payload[..])));
    }

    #[test]
    fn a_broadcast_boot_frame_is_accepted_because_the_group_address_defaults_to_it() {
        let boot = boot();
        let payload = [0u8; 4];
        let frame = boot_frame([0xff; 6], 9, 7, &payload);
        assert_eq!(boot.boot_frame(&frame), Some((7, &payload[..])));
    }

    #[test]
    fn frames_the_checks_reject_are_discarded() {
        let boot = boot();
        let payload = [0u8; 4];
        // Another station's address.
        let other = boot_frame([0x02, 0, 0, 0, 0, 0xff], 9, 0, &payload);
        assert_eq!(boot.boot_frame(&other), None);
        // The wrong UDP destination port.
        let port = boot_frame(boot.mac, 10, 0, &payload);
        assert_eq!(boot.boot_frame(&port), None);
        // A payload that is not a whole number of words.
        let ragged = boot_frame(boot.mac, 9, 0, &[0u8; 5]);
        assert_eq!(boot.boot_frame(&ragged), None);
        // A corrupted magic number.
        let mut magic = boot_frame(boot.mac, 9, 0, &payload);
        magic[42] ^= 0xff;
        assert_eq!(boot.boot_frame(&magic), None);
        // The announcement itself, looped back: BOOTP ports are not the boot table's.
        let announcement = boot.announcement();
        assert_eq!(boot.boot_frame(&announcement), None);
    }

    #[test]
    fn a_frame_with_a_wrong_udp_checksum_is_discarded_but_zero_means_unchecked() {
        let boot = boot();
        let payload = [9u8, 8, 7, 6];
        let mut frame = boot_frame(boot.mac, 9, 0, &payload);
        // The UDP checksum sits 6 bytes into the UDP header, which starts at 14 + 20.
        frame[40..42].copy_from_slice(&0x1234u16.to_be_bytes());
        assert_eq!(boot.boot_frame(&frame), None, "a wrong checksum is checked");
        let datagram = 34..34 + 8 + FRAME_HEADER_BYTES + payload.len();
        frame[40..42].copy_from_slice(&0u16.to_be_bytes());
        let good = udp_checksum(&frame[26..34], &frame[datagram]);
        frame[40..42].copy_from_slice(&good.to_be_bytes());
        assert_eq!(boot.boot_frame(&frame), Some((0, &payload[..])));
    }

    #[test]
    fn the_announcement_is_a_bootp_request_naming_this_device() {
        let boot = boot();
        let frame = boot.announcement();
        assert_eq!(frame.len(), 14 + 20 + 8 + BOOTP_PAYLOAD_BYTES);
        assert_eq!(&frame[0..6], &[0xff; 6], "sent to the host address");
        assert_eq!(&frame[6..12], &boot.mac);
        assert_eq!(be16(&frame, 12), Some(ETHERTYPE_IPV4));
        // A correct IPv4 header sums to zero, including its own checksum.
        assert_eq!(internet_checksum(&frame[14..34]), 0);
        let udp = &frame[34..];
        assert_eq!(be16(udp, 0), Some(BOOTP_CLIENT_PORT));
        assert_eq!(be16(udp, 2), Some(BOOTP_SERVER_PORT));
        let bootp = &udp[8..];
        assert_eq!(bootp[0..3], [1, 1, 6]);
        assert_eq!(&bootp[28..34], &boot.mac);
        assert_eq!(&bootp[44..61], BOOT_SERVER_NAME);
        assert_eq!(&bootp[108..126], b"ti-boot-table-0000");
    }

    /// The whole boot, on the device: the announcement leaves the MAC, the frames arrive through
    /// it, the sections land where the table says - including in another core's memory and in the
    /// boot controller - and core 0 ends up at the entry point with the released core running.
    #[test]
    fn an_ethernet_boot_places_both_cores_images_and_starts_them() {
        use crate::device::{Device, C6472};

        const DSP_BOOT_ADDR1: u32 = 0x02ab_0224;
        const BOOT_COMPLETE_STAT: u32 = 0x02ab_0004;
        /// The signature word the table writes into core 1's memory, clear of the entry point so
        /// that the core starts on zeroed memory rather than trying to execute the signature.
        const CORE1_SIGNATURE: u32 = 0x0080_1000;
        const CORE1_SIGNATURE_GLOBAL: u32 = 0x1180_1000;
        const ENTRY: u32 = 0x0080_0000;
        let mac = [0x02, 0x00, 0x00, 0x00, 0x64, 0x72];

        let mut device = Device::with_boot_mode(&C6472, ENTRY, Endian::Little, 9);
        device.fabric().fuse_mac_id(mac);
        device.begin_ethernet_boot(0x02c8_0000, BootParameters::default());
        for core in 0..6 {
            assert!(
                device.fabric().held_in_reset(core),
                "core {core} runs already"
            );
        }

        // A table that writes core 1's image, points core 1 at it and releases just core 1.
        let words: [u32; 11] = [
            ENTRY,
            4,
            CORE1_SIGNATURE_GLOBAL,
            0xc0de_0001,
            4,
            DSP_BOOT_ADDR1,
            ENTRY >> 10,
            4,
            BOOT_COMPLETE_STAT,
            0b10,
            0,
        ];
        let table: Vec<u8> = words.iter().flat_map(|word| word.to_le_bytes()).collect();

        // The host waits for the announcement before it sends anything, which is the whole point
        // of the announcement: the boot code resets the MAC on its way up, so a frame that
        // arrived before that is gone with the rest of the MAC's state.
        let mut announcement = Vec::new();
        for _ in 0..16 {
            device.step_cycle();
            announcement = device.fabric().take_tx_frames();
            if !announcement.is_empty() {
                break;
            }
        }
        assert_eq!(announcement.len(), 1, "one announcement, and only one");
        assert_eq!(&announcement[0][6..12], &mac, "it names the fused address");

        // Two frames, split so that a section's header straddles them.
        for (sequence, fragment) in table.chunks(20).enumerate() {
            let frame = boot_frame(mac, 9, sequence as u8, fragment);
            device.fabric().inject_frame_on(0, frame);
        }
        for _ in 0..256 {
            device.step_cycle();
            if device.ethernet_boot().and_then(|boot| boot.entry).is_some() {
                break;
            }
        }

        let boot = device
            .ethernet_boot()
            .expect("the device is booting over Ethernet");
        assert_eq!(boot.entry, Some(ENTRY));
        assert_eq!(boot.accepted, table.len().div_ceil(20) as u64);
        assert_eq!(boot.discarded, 0);
        assert_eq!(boot.table_bytes, table.len() as u64);
        assert!(
            device.fabric().take_tx_frames().is_empty(),
            "nothing else goes out: the announcement is never retransmitted"
        );
        // Core 0 is where the boot table's entry point says, with nothing executed yet: the
        // cycle that finished the boot is the cycle before its first fetch.
        assert!(!device.fabric().held_in_reset(0));
        assert_eq!(device.core(0).pc(), ENTRY);
        assert_eq!(device.core(0).cycles(), 0);
        // Core 1 was released by the table's own BOOT_COMPLETE_STAT write, started at the
        // address the table put in DSP_BOOT_ADDR1, and has been running since.
        assert!(!device.fabric().held_in_reset(1));
        assert!(
            device.core(1).pc() >= ENTRY,
            "core 1 started at its boot address"
        );
        assert!(device.core(1).cycles() > 0, "core 1 has executed");
        assert_eq!(device.core(1).fault(), None);
        assert_eq!(device.fabric().read32(1, CORE1_SIGNATURE), 0xc0de_0001);
        for core in 2..6 {
            assert!(
                device.fabric().held_in_reset(core),
                "core {core} was released by a table that never named it"
            );
        }
    }
}
