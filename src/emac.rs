use crate::port::{PortContext, PortWrite, SlavePort};
use std::collections::{BTreeMap, VecDeque};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const TXIDVER: u32 = 0x02c8_0000;
const RXIDVER: u32 = 0x02c8_0010;
const TXTEARDOWN: u32 = 0x02c8_0008;
const RXTEARDOWN: u32 = 0x02c8_0018;
const TXCONTROL: u32 = 0x02c8_0004;
const RXCONTROL: u32 = 0x02c8_0014;
const TXINTSTATRAW: u32 = 0x02c8_0080;
const TXINTSTATMASKED: u32 = 0x02c8_0084;
const TXINTMASKSET: u32 = 0x02c8_0088;
const TXINTMASKCLEAR: u32 = 0x02c8_008c;
const RXINTSTATRAW: u32 = 0x02c8_00a0;
/// MACINVECTOR is composed one level up, in the control module that also sees the MDIO.
const MACINTSTATRAW: u32 = 0x02c8_00b0;
const MACINTSTATMASKED: u32 = 0x02c8_00b4;
const MACINTMASKSET: u32 = 0x02c8_00b8;
const MACINTMASKCLEAR: u32 = 0x02c8_00bc;
const RXINTSTATMASKED: u32 = 0x02c8_00a4;
const RXINTMASKSET: u32 = 0x02c8_00a8;
const RXINTMASKCLEAR: u32 = 0x02c8_00ac;
const RXMBPENABLE: u32 = 0x02c8_0100;
const RXUNICASTSET: u32 = 0x02c8_0104;
const RXUNICASTCLEAR: u32 = 0x02c8_0108;
const RXMAXLEN: u32 = 0x02c8_010c;
const MACCONTROL: u32 = 0x02c8_0160;
const MACSTATUS: u32 = 0x02c8_0164;
const SOFTRESET: u32 = 0x02c8_0174;
const FIFOCONTROL: u32 = 0x02c8_016c;
const MACSRCADDRLO: u32 = 0x02c8_01d0;
const MACSRCADDRHI: u32 = 0x02c8_01d4;
const RXPAUSE: u32 = 0x02c8_01e8;
const TXPAUSE: u32 = 0x02c8_01ec;
const MACHASH1: u32 = 0x02c8_01d8;
const MACHASH2: u32 = 0x02c8_01dc;
const RXGOODFRAMES: u32 = 0x02c8_0200;
const RXBCASTFRAMES: u32 = 0x02c8_0204;
const RXMCASTFRAMES: u32 = 0x02c8_0208;
const RXPAUSEFRAMES: u32 = 0x02c8_020c;
const RXCRCERRORS: u32 = 0x02c8_0210;
const RXALIGNCODEERRORS: u32 = 0x02c8_0214;
const RXOVERSIZED: u32 = 0x02c8_0218;
const RXJABBER: u32 = 0x02c8_021c;
const RXUNDERSIZED: u32 = 0x02c8_0220;
const RXFRAGMENTS: u32 = 0x02c8_0224;
const RXFILTERED: u32 = 0x02c8_0228;
const RXQOSFILTERED: u32 = 0x02c8_022c;
const RXOCTETS: u32 = 0x02c8_0230;
const TXGOODFRAMES: u32 = 0x02c8_0234;
const TXBCASTFRAMES: u32 = 0x02c8_0238;
const TXMCASTFRAMES: u32 = 0x02c8_023c;
const TXPAUSEFRAMES: u32 = 0x02c8_0240;
const TXOCTETS: u32 = 0x02c8_0264;
const FRAME64: u32 = 0x02c8_0268;
const FRAME65T127: u32 = 0x02c8_026c;
const FRAME128T255: u32 = 0x02c8_0270;
const FRAME256T511: u32 = 0x02c8_0274;
const FRAME512T1023: u32 = 0x02c8_0278;
const FRAME1024TUP: u32 = 0x02c8_027c;
const NETOCTETS: u32 = 0x02c8_0280;
const RXSOFOVERRUNS: u32 = 0x02c8_0284;
const RXMOFOVERRUNS: u32 = 0x02c8_0288;
const RXDMAOVERRUNS: u32 = 0x02c8_028c;
const MACADDRLO: u32 = 0x02c8_0500;
const MACADDRHI: u32 = 0x02c8_0504;
const MACINDEX: u32 = 0x02c8_0508;
const RXBUFFEROFFSET: u32 = 0x02c8_0110;
const RXFILTERLOWTHRESH: u32 = 0x02c8_0114;
const RXFLOWTHRESH_BASE: u32 = 0x02c8_0120;
/// The per-channel register blocks: eight words each, one per channel (SPRU975E Table 29).
const RXFREEBUFFER_BASE: u32 = 0x02c8_0140;
const TXHDP_BASE: u32 = 0x02c8_0600;
const RXHDP_BASE: u32 = 0x02c8_0620;
const TXCP_BASE: u32 = 0x02c8_0640;
const RXCP_BASE: u32 = 0x02c8_0660;
const CHANNELS: usize = 8;
const EWCTL: u32 = 0x02c8_1004;

const SOP: u32 = 1 << 31;
const EOP: u32 = 1 << 30;
const OWNER: u32 = 1 << 29;
const EOQ: u32 = 1 << 28;
const TDOWNCMPLT: u32 = 1 << 27;
/// The SOP descriptor's overrun flag, set when the chain ran out of buffer space.
const OVERRUN: u32 = 1 << 20;
/// The SOP descriptor's frame-class and match flags (SPRU975E §2.5.5). A transferred long
/// frame is marked OVERSIZE, a transferred short frame UNDERSIZED, a transferred MAC control
/// frame CONTROL, and a frame that only arrived because RXCAFEN was set NOMATCH.
const JABBER: u32 = 1 << 25;
const OVERSIZE: u32 = 1 << 24;
const FRAGMENT: u32 = 1 << 23;
const UNDERSIZED: u32 = 1 << 22;
const CONTROL: u32 = 1 << 21;
const CODEERROR: u32 = 1 << 19;
const ALIGNERROR: u32 = 1 << 18;
const CRCERROR: u32 = 1 << 17;
const NOMATCH: u32 = 1 << 16;

/// The wire errors a frame can arrive with (SPRU975E sections 2.5.5.18 to 2.5.5.20). A CRC
/// error is a frame of whole nibbles that fails the frame check sequence, an alignment error
/// one with an odd nibble that fails it, and a code error a frame the MRXER pin condemned.
pub const RX_ERROR_CRC: u32 = 1;
pub const RX_ERROR_ALIGN: u32 = 2;
pub const RX_ERROR_CODE: u32 = 4;

/// The completion pointer a torn-down channel reports, and the value software acknowledges
/// it with (SPRU975E §2.11).
const TEARDOWN_MARKER: u32 = 0xffff_fffc;
/// RXMBPENABLE bits this model honours.
const RXMBP_MULTICAST: u32 = 1 << 5;
const RXMBP_BROADCAST: u32 = 1 << 13;
const RXMBP_PROMISCUOUS: u32 = 1 << 21;
const RXMBP_QOS: u32 = 1 << 29;
/// RXCMFEN, RXCSFEN and RXCEFEN decide whether a control, short or error frame the address
/// logic accepted is copied to memory at all (SPRU975E Table 8).
const RXMBP_CMFEN: u32 = 1 << 24;
const RXMBP_CSFEN: u32 = 1 << 23;
const RXMBP_CEFEN: u32 = 1 << 22;
const RXMBP_NOCHAIN: u32 = 1 << 28;
/// The two MAC interrupt sources, and their positions in the control module's input vector
/// (SPRU975E Tables 45 and 40).
const MACINT_STATPEND: u32 = 1 << 0;
const MACINT_HOSTPEND: u32 = 1 << 1;
const VECTOR_STATPEND: u32 = 1 << 16;
const VECTOR_HOSTPEND: u32 = 1 << 17;
/// The statistics level a set most significant bit reports through STATPEND (SPRU975E
/// §2.16.1.3).
const STATISTIC_LEVEL: u32 = 0x8000_0000;
/// Transmit and receive host error codes (SPRU975E Table 58). A zero packet length is an
/// error the hardware itself does not detect, so this model does not report it either.
const TXERR_SOP: u32 = 1;
const TXERR_OWNER: u32 = 2;
const TXERR_NO_EOP: u32 = 3;
const TXERR_BUFFER: u32 = 4;
const TXERR_LENGTH: u32 = 5;
const RXERR_BUFFER: u32 = 4;
/// A proper frame is 64 to RXMAXLEN bytes on the wire, which is four CRC bytes more than the
/// data this model carries (SPRU975E §2.10.7).
const MIN_WIRE_LENGTH: usize = 64;
const CRC_LENGTH: usize = 4;

/// SPRU975E §2.10.7: frames shorter than 64 wire bytes are short and frames longer than
/// RXMAXLEN are long. Short frames without errors are undersized and with errors fragments;
/// long frames without errors are oversized and with errors jabber. The errors themselves come
/// in with the frame, because nothing inside the module can produce them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FrameClass {
    Proper,
    Undersized,
    Oversized,
}
/// RXMBPENABLE also names the channel each class of frame is copied to: RXMULTCH in bits 2-0,
/// RXBROADCH in bits 10-8 and RXPROMCH in bits 18-16 (SPRU975E Table 49).
fn mbp_channel(rx_mbp_enable: u32, shift: u32) -> usize {
    ((rx_mbp_enable >> shift) & 7) as usize
}
/// MACADDRLO fields: the address is only used when VALID is set, MATCHFILT chooses between
/// matching and filtering, and CHANNEL names the receive channel it belongs to.
const MACADDR_VALID: u32 = 1 << 20;
const MACADDR_MATCH: u32 = 1 << 19;
/// MACCONTROL.GMIIEN, which holds the wire in reset until software sets it: SPRU975E Table 57
/// reads "GMII RX and TX are held in reset" for 0 and "GMII RX and TX are enabled for receive
/// and transmit" for 1, and its reset value is 0. It is the last thing TI's own initialization
/// sequence writes - section 2.15.4 step 12 says "do not set the GMIIEN bit yet" and step 17
/// "Enable the receive and transmit DMA controllers ... Then set the GMIIEN bit in MACCONTROL" -
/// so the DMA enables and the wire enable are two separate gates and this is the outer one.
const MACCONTROL_GMIIEN: u32 = 1 << 5;
const MACCONTROL_TXPTYPE: u32 = 1 << 9;
const MACCONTROL_FULLDUPLEX: u32 = 1 << 0;
const MACCONTROL_RXBUFFERFLOWEN: u32 = 1 << 3;
const MACCONTROL_TXFLOWEN: u32 = 1 << 4;
const MACCONTROL_RXFIFOFLOWEN: u32 = 1 << 12;
const RX_FIFO_CELLS: u32 = 68;

/// One entry of the address RAM: six address bytes plus the MACADDRLO control bits.
#[derive(Clone, Copy, Default)]
struct AddressSlot {
    bytes: [u8; 6],
    control: u32,
}

impl AddressSlot {
    fn matches(&self, destination: &[u8]) -> bool {
        self.control & MACADDR_VALID != 0
            && self.control & MACADDR_MATCH != 0
            && destination == self.bytes
    }

    fn filters(&self, destination: &[u8]) -> bool {
        self.control & MACADDR_VALID != 0
            && self.control & MACADDR_MATCH == 0
            && destination == self.bytes
    }

    fn channel(&self) -> u32 {
        (self.control >> 16) & 7
    }
}

/// The six-bit hash of a destination address: bit i of the result exclusive-ors every eighth
/// bit of the address starting at bit i (SPRU975E §5.36).
fn address_hash(destination: &[u8]) -> u32 {
    let mut bits = 0u64;
    for (index, byte) in destination.iter().take(6).enumerate() {
        bits |= u64::from(*byte) << (index * 8);
    }
    let mut hash = 0u32;
    for slot in 0..6 {
        let mut value = 0u64;
        for step in 0..8 {
            value ^= (bits >> (slot + step * 6)) & 1;
        }
        hash |= (value as u32) << slot;
    }
    hash
}

/// A frame waiting in the receive FIFO, with the wire errors it arrived with.
struct RxFrame {
    data: Vec<u8>,
    errors: u32,
}

pub struct Emac {
    /// How far this port's register block sits above the C6455's, so one model can serve both
    /// of a C6472's MACs: EMAC0 at 0x02C8 0000 and EMAC1 at 0x02CC 0000 (SPRS612G Tables 7-101
    /// and 7-103 give the second block the same registers 0x0004 0000 higher). Every constant
    /// in this file stays written as a C6455 address and an access is translated once, on the
    /// way in.
    offset: u32,
    /// The C6455 gates its single MAC's interrupts through EWCTL in the EMAC control module
    /// (SPRS276M section 7.15), while each C6472 MAC has an EMIC at that address instead
    /// (SPRS612G Tables 7-105 and 7-106).
    wrapper_control: bool,
    tx_control: u32,
    rx_control: u32,
    tx_mask: u32,
    rx_mask: u32,
    rx_mbp_enable: u32,
    rx_buffer_offset: u32,
    rx_free_buffers: [u32; CHANNELS],
    rx_flow_thresholds: [u32; CHANNELS],
    rx_filter_low_threshold: u32,
    tx_hdp: [u32; CHANNELS],
    tx_active: u8,
    tx_cursor: usize,
    rx_hdp: [u32; CHANNELS],
    tx_completion: [u32; CHANNELS],
    rx_completion: [u32; CHANNELS],
    tx_acknowledged: [u32; CHANNELS],
    rx_acknowledged: [u32; CHANNELS],
    ewctl: u32,
    unicast_enable: u32,
    rx_max_length: u32,
    mac_int_mask: u32,
    /// The latched host error code and channel for each direction. A host error stops the DMA
    /// until the module is reset, and reading MACSTATUS clears the channel field.
    tx_error_code: u32,
    tx_error_channel: u32,
    rx_error_code: u32,
    rx_error_channel: u32,
    mac_control: u32,
    fifo_control: u32,
    fifo_control_written: bool,
    rx_fifo_cells: u32,
    mac_source: [u8; 6],
    rx_pause_timer: u16,
    tx_pause_timer: u16,
    rx_flow_active: bool,
    hash: [u32; 2],
    addresses: [AddressSlot; 32],
    mac_index: u32,
    statistics: BTreeMap<u32, u32>,
    rx_teardown: u32,
    tx_teardown: u32,
    interrupt_line: bool,
    transmitted: Vec<Vec<u8>>,
    received: VecDeque<RxFrame>,
    /// Set while the frame at the head of the receive queue is a MAC control frame the MAC
    /// has already acted on, so RXCMFEN can still copy it without a second pause action.
    control_acted: bool,
}

impl Default for Emac {
    fn default() -> Self {
        Self {
            offset: 0,
            wrapper_control: true,
            tx_control: 0,
            rx_control: 0,
            tx_mask: 0,
            rx_mask: 0,
            rx_mbp_enable: 0,
            rx_buffer_offset: 0,
            rx_free_buffers: [0; CHANNELS],
            rx_flow_thresholds: [0; CHANNELS],
            rx_filter_low_threshold: 0,
            tx_hdp: [0; CHANNELS],
            tx_active: 0,
            tx_cursor: 0,
            rx_hdp: [0; CHANNELS],
            tx_completion: [0; CHANNELS],
            rx_completion: [0; CHANNELS],
            tx_acknowledged: [0; CHANNELS],
            rx_acknowledged: [0; CHANNELS],
            ewctl: 0,
            unicast_enable: 0,
            rx_max_length: 1518,
            mac_int_mask: 0,
            tx_error_code: 0,
            tx_error_channel: 0,
            rx_error_code: 0,
            rx_error_channel: 0,
            mac_control: 0,
            fifo_control: (2 << 16) | 24,
            fifo_control_written: true,
            rx_fifo_cells: 0,
            mac_source: [0; 6],
            rx_pause_timer: 0,
            tx_pause_timer: 0,
            rx_flow_active: false,
            hash: [0; 2],
            addresses: [AddressSlot::default(); 32],
            mac_index: 0,
            statistics: BTreeMap::new(),
            rx_teardown: 0,
            tx_teardown: 0,
            interrupt_line: false,
            transmitted: Vec::new(),
            received: VecDeque::new(),
            control_acted: false,
        }
    }
}

impl Emac {
    /// A port whose register block starts at `base`: 0x02C8 0000 for a C6455's only MAC or a
    /// C6472's EMAC0, 0x02CC 0000 for a C6472's EMAC1. `wrapper_control` says whether the device
    /// has the C6455's control-module gate rather than the C6472's per-MAC EMIC.
    pub fn new(base: u32, wrapper_control: bool) -> Self {
        Self {
            offset: base.wrapping_sub(0x02c8_0000),
            wrapper_control,
            ..Self::default()
        }
    }

    /// The interrupt sources an interrupt-control module gates, after this port's own masks:
    /// the eight receive and eight transmit channel interrupts, the statistics interrupt and
    /// the host error (SPRUEF8F section 3.1 lists them as the EMIC's inputs).
    pub fn masked_sources(&self) -> (u8, u8, bool, bool) {
        let mac = self.mac_int_raw() & self.mac_int_mask;
        (
            (self.rx_raw() & self.rx_mask) as u8,
            (self.tx_raw() & self.tx_mask) as u8,
            mac & MACINT_STATPEND != 0,
            mac & MACINT_HOSTPEND != 0,
        )
    }
}

impl SlavePort for Emac {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let address = address.wrapping_sub(self.offset);
        let value = match address {
            TXIDVER | RXIDVER => 0x000c_0a08,
            SOFTRESET => 0,
            TXCONTROL => self.tx_control,
            RXCONTROL => self.rx_control,
            TXINTSTATRAW => self.tx_raw(),
            TXINTSTATMASKED => self.tx_raw() & self.tx_mask,
            TXINTMASKSET | TXINTMASKCLEAR => self.tx_mask,
            RXINTSTATRAW => self.rx_raw(),
            RXINTSTATMASKED => self.rx_raw() & self.rx_mask,
            RXINTMASKSET | RXINTMASKCLEAR => self.rx_mask,
            MACINTSTATRAW => self.mac_int_raw(),
            MACINTSTATMASKED => self.mac_int_raw() & self.mac_int_mask,
            MACINTMASKSET | MACINTMASKCLEAR => self.mac_int_mask,
            RXMBPENABLE => self.rx_mbp_enable,
            RXBUFFEROFFSET => self.rx_buffer_offset,
            RXFILTERLOWTHRESH => self.rx_filter_low_threshold,
            EWCTL if self.wrapper_control => self.ewctl,
            RXUNICASTSET | RXUNICASTCLEAR => self.unicast_enable,
            RXMAXLEN => self.rx_max_length,
            MACCONTROL => self.mac_control,
            MACSTATUS => {
                let status = (self.tx_error_code << 20)
                    | (self.tx_error_channel << 16)
                    | (self.rx_error_code << 12)
                    | (self.rx_error_channel << 8)
                    | (u32::from(self.qos_active()) << 2)
                    | (u32::from(self.rx_flow_active) << 1)
                    | u32::from(self.tx_pause_timer != 0);
                // The host error channel fields read as zero once the host has seen them
                // (SPRU975E Table 58); the error codes stay until the module is reset.
                self.tx_error_channel = 0;
                self.rx_error_channel = 0;
                status
            }
            FIFOCONTROL => self.effective_fifo_control(),
            MACSRCADDRLO => (u32::from(self.mac_source[5]) << 8) | u32::from(self.mac_source[4]),
            MACSRCADDRHI => {
                (u32::from(self.mac_source[3]) << 24)
                    | (u32::from(self.mac_source[2]) << 16)
                    | (u32::from(self.mac_source[1]) << 8)
                    | u32::from(self.mac_source[0])
            }
            RXPAUSE => u32::from(self.rx_pause_timer),
            TXPAUSE => u32::from(self.tx_pause_timer),
            MACHASH1 => self.hash[0],
            MACHASH2 => self.hash[1],
            // In wire order, LO holds bytes 5 and 4 in bits 15:8 and 7:0; HI holds bytes
            // 3, 2, 1, 0 from most to least significant (SPRU975E Tables 71 and 72).
            MACADDRLO => {
                let slot = self.addresses[self.slot_index()];
                (u32::from(slot.bytes[5]) << 8) | u32::from(slot.bytes[4]) | slot.control
            }
            MACADDRHI => {
                let slot = self.addresses[self.slot_index()];
                (u32::from(slot.bytes[3]) << 24)
                    | (u32::from(slot.bytes[2]) << 16)
                    | (u32::from(slot.bytes[1]) << 8)
                    | u32::from(slot.bytes[0])
            }
            TXTEARDOWN | RXTEARDOWN => 0,
            _ if statistics_register(address) => {
                self.statistics.get(&address).copied().unwrap_or(0)
            }
            _ => match channel_register(address) {
                Some((RXFLOWTHRESH_BASE, channel)) => self.rx_flow_thresholds[channel],
                Some((RXFREEBUFFER_BASE, channel)) => self.rx_free_buffers[channel],
                Some((TXHDP_BASE, channel)) => self.tx_hdp[channel],
                Some((RXHDP_BASE, channel)) => self.rx_hdp[channel],
                Some((TXCP_BASE, channel)) => self.tx_completion[channel],
                Some((RXCP_BASE, channel)) => self.rx_completion[channel],
                _ => return None,
            },
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        let address = address.wrapping_sub(self.offset);
        match address {
            TXCONTROL => self.tx_control = value & 1,
            RXCONTROL => self.rx_control = value & 1,
            TXINTMASKSET => self.tx_mask |= value & 0xff,
            TXINTMASKCLEAR => self.tx_mask &= !(value & 0xff),
            RXINTMASKSET => self.rx_mask |= value & 0xff,
            RXINTMASKCLEAR => self.rx_mask &= !(value & 0xff),
            RXMBPENABLE => self.rx_mbp_enable = value,
            TXIDVER | RXIDVER => {}
            // SPRU975E §2.14.1: only the EMAC resets, not its separate control
            // module. The address and wrapper selection are wiring, not reset state.
            SOFTRESET if value & 1 != 0 => {
                let offset = self.offset;
                let wrapper_control = self.wrapper_control;
                let ewctl = self.ewctl;
                *self = Self::default();
                self.offset = offset;
                self.wrapper_control = wrapper_control;
                self.ewctl = ewctl;
            }
            SOFTRESET => {}
            RXBUFFEROFFSET => self.rx_buffer_offset = value & 0xffff,
            RXFILTERLOWTHRESH => self.rx_filter_low_threshold = value & 0xff,
            EWCTL if self.wrapper_control => self.ewctl = value & 1,
            RXUNICASTSET => self.unicast_enable |= value & 0xff,
            RXUNICASTCLEAR => self.unicast_enable &= !(value & 0xff),
            RXMAXLEN => self.rx_max_length = value & 0xffff,
            MACCONTROL => self.mac_control = value,
            FIFOCONTROL => {
                self.fifo_control = value & 0x007f_001f;
                self.fifo_control_written = true;
            }
            MACSRCADDRLO => {
                self.mac_source[5] = (value >> 8) as u8;
                self.mac_source[4] = value as u8;
            }
            MACSRCADDRHI => {
                self.mac_source[3] = (value >> 24) as u8;
                self.mac_source[2] = (value >> 16) as u8;
                self.mac_source[1] = (value >> 8) as u8;
                self.mac_source[0] = value as u8;
            }
            RXPAUSE | TXPAUSE => {}
            MACHASH1 => self.hash[0] = value,
            MACHASH2 => self.hash[1] = value,
            MACINDEX => self.mac_index = value & 0x1f,
            MACADDRLO => {
                let index = self.slot_index();
                let slot = &mut self.addresses[index];
                slot.bytes[5] = (value >> 8) as u8;
                slot.bytes[4] = value as u8;
                slot.control = value & 0x001f_0000;
            }
            MACADDRHI => {
                let index = self.slot_index();
                let slot = &mut self.addresses[index];
                slot.bytes[3] = (value >> 24) as u8;
                slot.bytes[2] = (value >> 16) as u8;
                slot.bytes[1] = (value >> 8) as u8;
                slot.bytes[0] = value as u8;
            }
            // The teardown registers carry the channel to tear down in their low three bits.
            RXTEARDOWN => self.rx_teardown |= 1 << (value & 7),
            TXTEARDOWN => self.tx_teardown |= 1 << (value & 7),
            MACINTMASKSET => self.mac_int_mask |= value & 3,
            MACINTMASKCLEAR => self.mac_int_mask &= !(value & 3),
            MACINTSTATRAW | MACINTSTATMASKED => {}
            _ if statistics_register(address) => {
                // With GMIIEN set the statistics are write-to-decrement, saturating at zero;
                // with it clear they are ordinary read/write registers (SPRU975E §5.49).
                let counter = self.statistics.entry(address).or_insert(0);
                *counter = if self.mac_control & MACCONTROL_GMIIEN != 0 {
                    counter.saturating_sub(value)
                } else {
                    value
                };
            }
            _ => match channel_register(address) {
                Some((RXFLOWTHRESH_BASE, channel)) => {
                    self.rx_flow_thresholds[channel] = value & 0xff;
                }
                Some((RXFREEBUFFER_BASE, channel)) => {
                    self.rx_free_buffers[channel] =
                        self.rx_free_buffers[channel].wrapping_add(value) & 0xffff;
                }
                Some((TXHDP_BASE, channel)) => {
                    self.tx_hdp[channel] = value;
                    if value == 0 {
                        self.tx_active &= !(1 << channel);
                    } else {
                        self.tx_active |= 1 << channel;
                    }
                }
                Some((RXHDP_BASE, channel)) => self.rx_hdp[channel] = value,
                Some((TXCP_BASE, channel)) => self.tx_acknowledged[channel] = value,
                Some((RXCP_BASE, channel)) => self.rx_acknowledged[channel] = value,
                _ => return PortWrite::Unhandled,
            },
        }
        self.update_flow_control();
        self.update_interrupt();
        PortWrite::Done
    }
}

impl Emac {
    fn slot_index(&self) -> usize {
        (self.mac_index & 0x1f) as usize
    }

    fn count(&mut self, register: u32) {
        self.add_count(register, 1);
    }

    fn add_count(&mut self, register: u32, value: u32) {
        let counter = self.statistics.entry(register).or_insert(0);
        *counter = counter.wrapping_add(value);
    }

    fn count_size(&mut self, length: usize) {
        let wire_length = length.saturating_add(4);
        let register = match wire_length {
            0..=64 => FRAME64,
            65..=127 => FRAME65T127,
            128..=255 => FRAME128T255,
            256..=511 => FRAME256T511,
            512..=1023 => FRAME512T1023,
            _ => FRAME1024TUP,
        };
        self.count(register);
    }

    fn record_transmit(&mut self, frame: &[u8]) {
        self.count(TXGOODFRAMES);
        if frame.get(..6) == Some(&[0xff; 6]) {
            self.count(TXBCASTFRAMES);
        } else if frame.first().is_some_and(|byte| byte & 1 != 0) {
            self.count(TXMCASTFRAMES);
        }
        let octets = frame.len().saturating_add(4).min(u32::MAX as usize) as u32;
        self.add_count(TXOCTETS, octets);
        self.add_count(NETOCTETS, octets);
        self.count_size(frame.len());
    }

    fn record_receive(&mut self, destination: [u8; 6], length: usize) {
        self.count(RXGOODFRAMES);
        if destination == [0xff; 6] {
            self.count(RXBCASTFRAMES);
        } else if destination[0] & 1 != 0 {
            self.count(RXMCASTFRAMES);
        }
        let octets = length.saturating_add(CRC_LENGTH).min(u32::MAX as usize) as u32;
        self.add_count(RXOCTETS, octets);
        self.add_count(NETOCTETS, octets);
        self.count_size(length);
    }
    fn effective_fifo_control(&self) -> u32 {
        if self.fifo_control_written {
            self.fifo_control
        } else {
            (2 << 16) | 24
        }
    }

    fn frame_cells(frame: &[u8]) -> u32 {
        Self::frame_cells_for(frame.len())
    }

    /// The 64-byte cells a frame of this length occupies in either FIFO.
    fn frame_cells_for(length: usize) -> u32 {
        (length as u32).div_ceil(64).max(1)
    }

    fn pop_received(&mut self) -> Option<RxFrame> {
        let frame = self.received.pop_front()?;
        self.rx_fifo_cells = self
            .rx_fifo_cells
            .saturating_sub(Self::frame_cells(&frame.data));
        self.control_acted = false;
        Some(frame)
    }

    fn channel_enabled(&self, channel: usize) -> bool {
        self.unicast_enable & (1 << channel) != 0
            || self.rx_mbp_enable & RXMBP_BROADCAST != 0
                && mbp_channel(self.rx_mbp_enable, 8) == channel
            || self.rx_mbp_enable & RXMBP_MULTICAST != 0
                && mbp_channel(self.rx_mbp_enable, 0) == channel
            || self.rx_mbp_enable & RXMBP_PROMISCUOUS != 0
                && mbp_channel(self.rx_mbp_enable, 16) == channel
    }
    fn qos_active(&self) -> bool {
        self.rx_mbp_enable & RXMBP_QOS != 0
            && (0..CHANNELS).any(|channel| {
                self.channel_enabled(channel)
                    && self.rx_free_buffers[channel] <= self.rx_filter_low_threshold
            })
    }

    fn low_priority(frame: &[u8]) -> bool {
        frame.len() < 16
            || frame[12..14] != [0x81, 0x00]
            || u16::from_be_bytes([frame[14], frame[15]]) >> 13 < 4
    }

    fn flow_triggered(&self) -> bool {
        let buffer_flow = self.mac_control & MACCONTROL_RXBUFFERFLOWEN != 0
            && (0..CHANNELS).any(|channel| {
                self.channel_enabled(channel)
                    && self.rx_free_buffers[channel] <= self.rx_flow_thresholds[channel]
            });
        let fifo_threshold = (self.effective_fifo_control() >> 16) & 0x7f;
        let fifo_flow = self.mac_control & (MACCONTROL_FULLDUPLEX | MACCONTROL_RXFIFOFLOWEN)
            == (MACCONTROL_FULLDUPLEX | MACCONTROL_RXFIFOFLOWEN)
            && (1..=66).contains(&fifo_threshold)
            && self.rx_fifo_cells >= fifo_threshold;
        buffer_flow || fifo_flow
    }

    fn update_flow_control(&mut self) {
        let active = self.flow_triggered();
        if active == self.rx_flow_active {
            return;
        }
        self.rx_flow_active = active;
        if self.mac_control & MACCONTROL_FULLDUPLEX == 0 {
            return;
        }
        self.send_pause(if active { 0xffff } else { 0 });
    }

    /// Whether the media-independent interface is out of reset, which is the outer gate on both
    /// directions: SPRU975E Table 57 reads GMIIEN = 0 as "GMII RX and TX are held in reset".
    /// The DMA enables RXEN and TXEN are the inner gate, and TI's initialization sequence sets
    /// them first (section 2.15.4 step 17), so a model that only had them would move frames in
    /// a configuration the interface is not running in.
    fn wire_enabled(&self) -> bool {
        self.mac_control & MACCONTROL_GMIIEN != 0
    }

    fn send_pause(&mut self, time: u16) {
        // A pause frame is a frame: it goes out over the same interface, so it does not go out
        // while that interface is held in reset.
        if !self.wire_enabled() {
            return;
        }
        let mut frame = Vec::with_capacity(60);
        frame.extend_from_slice(&[0x01, 0x80, 0xc2, 0x00, 0x00, 0x01]);
        frame.extend_from_slice(&self.mac_source);
        frame.extend_from_slice(&[0x88, 0x08, 0x00, 0x01]);
        frame.extend_from_slice(&time.to_be_bytes());
        frame.resize(60, 0);
        self.record_transmit(&frame);
        self.transmitted.push(frame);
        self.count(TXPAUSEFRAMES);
        self.rx_pause_timer = if time == 0 { 0 } else { 0xff00 };
    }

    /// SPRU975E §2.10.7: classification uses the wire length, which is four CRC bytes more
    /// than the frame data this model carries.
    fn classify(&self, frame: &[u8]) -> FrameClass {
        let wire = frame.len().saturating_add(CRC_LENGTH);
        if wire < MIN_WIRE_LENGTH {
            FrameClass::Undersized
        } else if self.rx_max_length != 0 && wire > self.rx_max_length as usize {
            FrameClass::Oversized
        } else {
            FrameClass::Proper
        }
    }

    /// The MAC control frames this device sees are pause frames: the reserved or a matching
    /// destination, the 8808 length/type with opcode 1, and a proper length (SPRU975E §5.49.4).
    fn is_control_frame(&self, frame: &[u8]) -> bool {
        frame.len() >= 18
            && self.classify(frame) == FrameClass::Proper
            && (frame[..6] == [0x01, 0x80, 0xc2, 0x00, 0x00, 0x01]
                || self.addresses.iter().any(|slot| slot.matches(&frame[..6])))
            && frame[12..16] == [0x88, 0x08, 0x00, 0x01]
    }

    /// A control frame is acted upon whether or not it is copied to memory, so the pause
    /// action and its statistics happen here and RXCMFEN alone decides whether the receive
    /// DMA still sees the frame (SPRU975E Table 49).
    fn process_pause_frame(&mut self) {
        if self.control_acted {
            return;
        }
        let Some(frame) = self.received.front() else {
            return;
        };
        // A pause frame with a CRC, alignment or code error is not one: the statistic and the
        // action both require an error-free frame (SPRU975E §5.49.4).
        if frame.errors != 0 || !self.is_control_frame(&frame.data) {
            return;
        }
        let mut destination = [0u8; 6];
        destination.copy_from_slice(&frame.data[..6]);
        let length = frame.data.len();
        let pause_time = u16::from_be_bytes([frame.data[16], frame.data[17]]);
        self.record_receive(destination, length);
        self.count(RXPAUSEFRAMES);
        if self.mac_control & (MACCONTROL_FULLDUPLEX | MACCONTROL_TXFLOWEN)
            == MACCONTROL_FULLDUPLEX | MACCONTROL_TXFLOWEN
        {
            self.tx_pause_timer = pause_time;
        }
        if self.rx_mbp_enable & RXMBP_CMFEN == 0 {
            self.pop_received();
        } else {
            self.control_acted = true;
        }
    }

    /// Receive filtering: a frame reaches a channel when an address slot matches it, when it
    /// is broadcast or hashed multicast and the matching RXMBPENABLE bit is set, or - marked
    /// NOMATCH - when RXCAFEN routes non-matching frames to the promiscuous channel. A slot
    /// marked for filtering rejects the address outright (SPRU975E §2.10).
    fn receive_channel(&self, frame: &[u8]) -> Option<(usize, u32)> {
        let destination = &frame[..6.min(frame.len())];
        if destination.len() < 6 {
            return None;
        }
        if self.addresses.iter().any(|slot| slot.filters(destination)) {
            return None;
        }
        // An address slot that matches names the channel the frame goes to, provided that
        // channel is enabled in RXUNICASTSET.
        if let Some(slot) = self.addresses.iter().find(|slot| {
            slot.matches(destination) && self.unicast_enable & (1 << slot.channel()) != 0
        }) {
            return Some((slot.channel() as usize, 0));
        }
        if destination == [0xff; 6] {
            if self.rx_mbp_enable & RXMBP_BROADCAST != 0 {
                return Some((mbp_channel(self.rx_mbp_enable, 8), 0));
            }
        } else if destination[0] & 1 != 0 && self.rx_mbp_enable & RXMBP_MULTICAST != 0 {
            let hash = address_hash(destination);
            if self.hash[(hash / 32) as usize] & (1 << (hash % 32)) != 0 {
                return Some((mbp_channel(self.rx_mbp_enable, 0), 0));
            }
        }
        // RXCAFEN takes whatever is left, on the channel RXPROMCH names.
        if self.rx_mbp_enable & RXMBP_PROMISCUOUS != 0 {
            return Some((mbp_channel(self.rx_mbp_enable, 16), NOMATCH));
        }
        None
    }

    pub fn inject_frame(&mut self, frame: Vec<u8>) {
        self.inject_frame_with_errors(frame, 0);
    }

    /// A frame arriving with wire errors the receiver cannot produce by itself: a failed frame
    /// check sequence, an odd nibble count, or the MRXER pin driven high during reception.
    pub fn inject_frame_with_errors(&mut self, frame: Vec<u8>, errors: u32) {
        let cells = Self::frame_cells(&frame);
        if self.rx_fifo_cells + cells > RX_FIFO_CELLS {
            // Reception never starts when no cell is free at all, which is a start-of-frame
            // overrun; with room for part of the frame it starts and then runs out of cells,
            // which is a middle-of-frame overrun (SPRU975E §2.10.9).
            if self.rx_fifo_cells >= RX_FIFO_CELLS {
                self.count(RXSOFOVERRUNS);
            } else {
                self.count(RXMOFOVERRUNS);
            }
            return;
        }
        self.rx_fifo_cells += cells;
        self.received.push_back(RxFrame {
            data: frame,
            errors: errors & (RX_ERROR_CRC | RX_ERROR_ALIGN | RX_ERROR_CODE),
        });
        self.update_flow_control();
    }

    /// Whether the receive FIFO has room for a frame of this length. A host backend asks
    /// before delivering, because a Unix socket has no rate of its own: a frame the emulator
    /// was too slow to read in time would otherwise overrun a FIFO that a 100 Mbit/s line
    /// could never have filled that fast.
    pub fn rx_fifo_has_room(&self, length: usize) -> bool {
        self.rx_fifo_cells + Self::frame_cells_for(length) <= RX_FIFO_CELLS
    }

    pub fn take_transmitted(&mut self) -> Vec<Vec<u8>> {
        std::mem::take(&mut self.transmitted)
    }

    /// Whether a cycle can change MAC state. Programmed RX descriptors alone are dormant until
    /// a frame arrives; completion interrupts are levels and are latched separately by System.
    pub fn active(&self) -> bool {
        !self.received.is_empty()
            || self.tx_active != 0
            || self.tx_teardown != 0
            || self.rx_teardown != 0
            || self.tx_pause_timer != 0
            || self.rx_pause_timer != 0
    }

    pub fn tick(&mut self, memory: &mut impl DmaMemory) {
        self.process_pause_frame();
        self.update_flow_control();
        // TXPTYPE selects fixed priority (7 highest) or round robin. One complete packet
        // leaves the 24-cell FIFO per arbitration cycle.
        let selected = if self.mac_control & MACCONTROL_TXPTYPE != 0 {
            (0..CHANNELS).rev().find(|channel| {
                self.tx_hdp[*channel] != 0 || self.tx_teardown & (1 << channel) != 0
            })
        } else {
            (0..CHANNELS)
                .map(|offset| (self.tx_cursor + offset) % CHANNELS)
                .find(|channel| {
                    self.tx_hdp[*channel] != 0 || self.tx_teardown & (1 << channel) != 0
                })
        };
        if let Some(channel) = selected {
            self.process_tx(memory, channel);
            if self.mac_control & MACCONTROL_TXPTYPE == 0 {
                self.tx_cursor = (channel + 1) % CHANNELS;
            }
        }
        self.process_rx(memory);
        self.update_flow_control();
        if self.tx_pause_timer != 0 {
            self.tx_pause_timer -= 1;
        }
        if self.rx_pause_timer != 0 {
            self.rx_pause_timer -= 1;
            if self.rx_pause_timer == 0 && self.rx_flow_active {
                self.send_pause(0xffff);
            }
        }
        self.update_interrupt();
    }

    /// MACINT is a level: it stays asserted until the driver writes the completion pointer of
    /// the descriptor the hardware finished, so the megamodule keeps latching the event.
    pub fn interrupt_pending(&self) -> bool {
        self.interrupt_line
    }

    /// SPRU975E §3.2 Table 12: EWCTL gates the MDIO sources as well as the MAC's.
    pub fn combined_interrupt_pending(&self, mdio_pending: bool) -> bool {
        self.interrupt_pending() || ((!self.wrapper_control || self.ewctl & 1 != 0) && mdio_pending)
    }

    fn process_tx(&mut self, memory: &mut impl DmaMemory, channel: usize) {
        if self.tx_control == 0 || self.host_error() {
            return;
        }
        if self.tx_teardown & (1 << channel) != 0 {
            self.tx_teardown &= !(1 << channel);
            if self.tx_hdp[channel] != 0 {
                let flags = memory.dma_read_u32(self.tx_hdp[channel] + 12);
                memory.dma_write_u32(self.tx_hdp[channel] + 12, (flags & !OWNER) | TDOWNCMPLT);
            }
            self.tx_hdp[channel] = 0;
            self.tx_active &= !(1 << channel);
            self.tx_completion[channel] = TEARDOWN_MARKER;
            self.update_interrupt();
            return;
        }
        // The wire itself. Teardown above is a DMA command and completes whatever the interface
        // is doing, but a frame only leaves once GMII TX is out of reset.
        if !self.wire_enabled() {
            return;
        }
        if self.tx_pause_timer != 0 {
            return;
        }
        if self.tx_hdp[channel] == 0 {
            return;
        }
        let first = self.tx_hdp[channel];
        let first_flags = memory.dma_read_u32(first + 12);
        // A queued descriptor the host did not hand over, or a first buffer without SOP, is a
        // host error rather than an empty queue (SPRU975E §2.16.1.4 and Table 58).
        if first_flags & OWNER == 0 {
            self.raise_tx_error(TXERR_OWNER, channel);
            return;
        }
        if first_flags & SOP == 0 {
            self.raise_tx_error(TXERR_SOP, channel);
            return;
        }

        let mut descriptor = first;
        let mut packet = Vec::with_capacity((first_flags & 0xffff) as usize);
        let last;
        loop {
            let next = memory.dma_read_u32(descriptor);
            let buffer = memory.dma_read_u32(descriptor + 4);
            let offset_length = memory.dma_read_u32(descriptor + 8);
            let flags_length = memory.dma_read_u32(descriptor + 12);
            let offset = if descriptor == first {
                offset_length >> 16
            } else {
                0
            };
            let length = (offset_length & 0xffff) as usize;
            if buffer == 0 {
                self.raise_tx_error(TXERR_BUFFER, channel);
                return;
            }
            if length == 0 {
                self.raise_tx_error(TXERR_LENGTH, channel);
                return;
            }
            packet.extend(memory.dma_read_bytes(buffer.wrapping_add(offset), length));
            if flags_length & EOP != 0 {
                last = descriptor;
                if next == 0 {
                    memory.dma_write_u32(descriptor + 12, (flags_length & !OWNER) | EOQ);
                    self.tx_hdp[channel] = 0;
                    self.tx_active &= !(1 << channel);
                } else {
                    memory.dma_write_u32(descriptor + 12, flags_length & !OWNER);
                    self.tx_hdp[channel] = next;
                    self.tx_active |= 1 << channel;
                }
                break;
            }
            if next == 0 {
                self.raise_tx_error(TXERR_NO_EOP, channel);
                return;
            }
            descriptor = next;
        }

        if last != first {
            memory.dma_write_u32(first + 12, first_flags & !OWNER);
        }
        packet.truncate((first_flags & 0xffff) as usize);
        self.record_transmit(&packet);
        self.transmitted.push(packet);
        self.tx_completion[channel] = last;
    }

    fn process_rx(&mut self, memory: &mut impl DmaMemory) {
        if self.rx_control == 0 || self.host_error() {
            return;
        }
        for channel in 0..CHANNELS {
            if self.rx_teardown & (1 << channel) == 0 {
                continue;
            }
            // Teardown clears the head pointer, marks the next descriptor and reports the
            // teardown marker in the completion pointer (SPRU975E §2.11).
            self.rx_teardown &= !(1 << channel);
            if self.rx_hdp[channel] != 0 {
                let flags = memory.dma_read_u32(self.rx_hdp[channel] + 12);
                memory.dma_write_u32(self.rx_hdp[channel] + 12, (flags & !OWNER) | TDOWNCMPLT);
            }
            self.rx_hdp[channel] = 0;
            self.rx_completion[channel] = TEARDOWN_MARKER;
            self.update_interrupt();
        }
        // As on the transmit side, teardown above is a DMA command; taking a frame off the wire
        // needs GMII RX out of reset.
        if self.received.is_empty() || !self.wire_enabled() {
            return;
        }
        let class = self.classify(&self.received[0].data);
        let errors = self.received[0].errors;
        let control = self.control_acted;
        let Some((channel, mut status)) = self.receive_channel(&self.received[0].data) else {
            // Only a data frame the address logic rejected counts as filtered; the class
            // counters cover the frames it accepted (SPRU975E §5.49.11). A fragment is the one
            // exception: address matching does not matter to it (SPRU975E §5.49.10).
            self.pop_received();
            if class == FrameClass::Undersized && errors != 0 {
                self.count(RXFRAGMENTS);
            } else if !control {
                self.count(RXFILTERED);
            }
            return;
        };
        // The copy enables decide whether an accepted short, long, errored or control frame
        // reaches memory at all, and what it arrived as is recorded in its SOP descriptor
        // (SPRU975E Table 49 and §2.5.5). A short frame needs RXCSFEN, a long one RXCEFEN, and
        // an errored one RXCEFEN whatever its length - so a fragment, being both short and
        // errored, needs both.
        match class {
            FrameClass::Undersized => {
                if self.rx_mbp_enable & RXMBP_CSFEN == 0 {
                    self.pop_received();
                    self.record_class(class, errors, control, &[]);
                    return;
                }
                status |= if errors != 0 { FRAGMENT } else { UNDERSIZED };
            }
            FrameClass::Oversized => {
                if self.rx_mbp_enable & RXMBP_CEFEN == 0 {
                    self.pop_received();
                    self.record_class(class, errors, control, &[]);
                    return;
                }
                status |= if errors != 0 { JABBER } else { OVERSIZE };
            }
            FrameClass::Proper => {}
        }
        if errors != 0 {
            if self.rx_mbp_enable & RXMBP_CEFEN == 0 {
                self.pop_received();
                self.record_class(class, errors, control, &[]);
                return;
            }
            if errors & RX_ERROR_CRC != 0 {
                status |= CRCERROR;
            }
            if errors & RX_ERROR_ALIGN != 0 {
                status |= ALIGNERROR;
            }
            if errors & RX_ERROR_CODE != 0 {
                status |= CODEERROR;
            }
        }
        if control {
            if self.rx_mbp_enable & RXMBP_CMFEN == 0 {
                self.pop_received();
                return;
            }
            status |= CONTROL;
        }
        // QOS filtering only ever looks at proper frames (SPRU975E §5.49.12).
        if class == FrameClass::Proper
            && self.rx_mbp_enable & RXMBP_QOS != 0
            && Self::low_priority(&self.received[0].data)
            && self.rx_free_buffers[channel] <= self.rx_filter_low_threshold
        {
            self.pop_received();
            self.count(RXQOSFILTERED);
            return;
        }
        if self.rx_hdp[channel] == 0 {
            return;
        }

        let first = self.rx_hdp[channel];
        if memory.dma_read_u32(first + 12) & OWNER == 0 {
            return;
        }
        let mut frame = self.pop_received().unwrap().data;
        if class == FrameClass::Oversized {
            // A long frame is truncated to RXMAXLEN wire bytes (SPRU975E §2.10.7).
            frame.truncate((self.rx_max_length as usize).saturating_sub(CRC_LENGTH));
        }
        // A packet may span one descriptor per fragment: each one takes as much as its buffer
        // holds, the first carries SOP and the packet length, the last carries EOP, and the
        // buffer offset applies to the start of the packet (SPRU975E §2.5.5). RXNOCHAIN
        // instead uses one buffer per frame and discards whatever does not fit, reporting the
        // whole frame byte count in the buffer length field (SPRU975E Table 49).
        let no_chain = self.rx_mbp_enable & RXMBP_NOCHAIN != 0;
        let mut descriptor = first;
        let mut stored = 0usize;
        let mut last = first;
        let mut next_after = 0u32;
        let mut overrun = false;
        loop {
            let flags = memory.dma_read_u32(descriptor + 12);
            if flags & OWNER == 0 {
                overrun = true;
                break;
            }
            let next = memory.dma_read_u32(descriptor);
            let buffer = memory.dma_read_u32(descriptor + 4);
            if buffer == 0 {
                // A posted descriptor with no buffer is a receive host error (SPRU975E
                // Table 58 code 4h).
                self.raise_rx_error(RXERR_BUFFER, channel);
                return;
            }
            let capacity = (memory.dma_read_u32(descriptor + 8) & 0xffff) as usize;
            let offset = if descriptor == first {
                self.rx_buffer_offset
            } else {
                0
            };
            let fragment = (frame.len() - stored).min(capacity);
            memory.dma_write_bytes(
                buffer.wrapping_add(offset),
                &frame[stored..stored + fragment],
            );
            let reported = if no_chain { frame.len() } else { fragment };
            memory.dma_write_u32(descriptor + 8, (offset << 16) | (reported & 0xffff) as u32);
            stored += fragment;
            last = descriptor;
            next_after = next;
            if no_chain || stored == frame.len() {
                break;
            }
            if next == 0 {
                overrun = true;
                break;
            }
            descriptor = next;
        }
        if overrun {
            // Running out of descriptors mid-frame is a DMA overrun. Only RXCEFEN hands the
            // partial frame over; otherwise it is filtered and the descriptors it started to
            // fill stay with the EMAC (SPRU975E §2.10.9 and Table 9).
            self.count(RXDMAOVERRUNS);
            if self.rx_mbp_enable & RXMBP_CEFEN == 0 {
                self.record_class(class, errors, control, &frame);
                return;
            }
            status |= OVERRUN;
        }
        let reported_length = if no_chain { frame.len() } else { stored };

        let mut walk = first;
        loop {
            let flags = memory.dma_read_u32(walk + 12);
            let mut updated = flags & !OWNER;
            if walk == first {
                updated |= SOP | status | (reported_length & 0xffff) as u32;
            } else {
                updated &= !0xffff;
            }
            if walk == last {
                updated |= EOP;
                if next_after == 0 {
                    updated |= EOQ;
                }
                memory.dma_write_u32(walk + 12, updated);
                break;
            }
            memory.dma_write_u32(walk + 12, updated);
            walk = memory.dma_read_u32(walk);
        }
        self.rx_completion[channel] = last;
        self.rx_hdp[channel] = next_after;
        self.rx_free_buffers[channel] = self.rx_free_buffers[channel].saturating_sub(1);
        self.record_class(class, errors, control, &frame);
    }

    /// Overruns have no effect on the frame-class statistics, so a frame the DMA could not
    /// place is still counted for the class it arrived as (SPRU975E §5.49). Each length class
    /// splits on whether the frame carried a wire error: a short one is a fragment rather than
    /// undersized, a long one jabber rather than oversized, and a proper one counts as an
    /// alignment/code error or a CRC error instead of a good frame. Alignment and code errors
    /// outrank a CRC error, which the manual defines as excluding them (SPRU975E §5.49.5).
    fn record_class(&mut self, class: FrameClass, errors: u32, control: bool, frame: &[u8]) {
        match class {
            FrameClass::Proper if errors & (RX_ERROR_ALIGN | RX_ERROR_CODE) != 0 => {
                self.count(RXALIGNCODEERRORS)
            }
            FrameClass::Proper if errors & RX_ERROR_CRC != 0 => self.count(RXCRCERRORS),
            // A good frame is 64 to RXMAXLEN wire bytes, and an acted-upon control frame has
            // already been recorded by the pause path (SPRU975E §5.49.1).
            FrameClass::Proper if !control => {
                let mut destination = [0u8; 6];
                destination.copy_from_slice(&frame[..6]);
                self.record_receive(destination, frame.len());
            }
            FrameClass::Proper => {}
            FrameClass::Undersized if errors != 0 => self.count(RXFRAGMENTS),
            FrameClass::Undersized => self.count(RXUNDERSIZED),
            FrameClass::Oversized if errors != 0 => self.count(RXJABBER),
            FrameClass::Oversized => self.count(RXOVERSIZED),
        }
    }

    /// Bit n of the raw status is set while channel n has a completion the driver has not
    /// acknowledged by writing its completion pointer (SPRU975E sections 5.7 and 5.13).
    fn tx_raw(&self) -> u32 {
        (0..CHANNELS)
            .filter(|channel| self.tx_completion[*channel] != self.tx_acknowledged[*channel])
            .fold(0, |mask, channel| mask | 1 << channel)
    }

    fn rx_raw(&self) -> u32 {
        (0..CHANNELS)
            .filter(|channel| self.rx_completion[*channel] != self.rx_acknowledged[*channel])
            .fold(0, |mask, channel| mask | 1 << channel)
    }

    /// STATPEND stands while any statistic has its most significant bit set; HOSTPEND latches
    /// a descriptor error until the module is reset (SPRU975E sections 2.16.1.3 and 2.16.1.4).
    fn mac_int_raw(&self) -> u32 {
        let level = self
            .statistics
            .values()
            .any(|value| value & STATISTIC_LEVEL != 0);
        let host = self.tx_error_code != 0 || self.rx_error_code != 0;
        (u32::from(level) * MACINT_STATPEND) | (u32::from(host) * MACINT_HOSTPEND)
    }

    /// The control module's view of every EMAC interrupt source (SPRU975E Table 40). The MDIO
    /// link and user bits are added by the caller, which is the only place that sees both.
    pub fn input_vector(&self) -> u32 {
        let mac = self.mac_int_raw();
        (self.rx_raw() << 8)
            | self.tx_raw()
            | (u32::from(mac & MACINT_STATPEND != 0) * VECTOR_STATPEND)
            | (u32::from(mac & MACINT_HOSTPEND != 0) * VECTOR_HOSTPEND)
    }

    /// A host error stops both DMA engines until the module is reset (SPRU975E §2.16.1.4).
    fn host_error(&self) -> bool {
        self.tx_error_code != 0 || self.rx_error_code != 0
    }

    fn raise_tx_error(&mut self, code: u32, channel: usize) {
        if self.tx_error_code == 0 {
            self.tx_error_code = code;
            self.tx_error_channel = channel as u32;
        }
        self.update_interrupt();
    }

    fn raise_rx_error(&mut self, code: u32, channel: usize) {
        if self.rx_error_code == 0 {
            self.rx_error_code = code;
            self.rx_error_channel = channel as u32;
        }
        self.update_interrupt();
    }

    fn update_interrupt(&mut self) {
        let pending = self.tx_raw() & self.tx_mask != 0
            || self.rx_raw() & self.rx_mask != 0
            || self.mac_int_raw() & self.mac_int_mask != 0;
        self.interrupt_line = self.ewctl & 1 != 0 && pending;
    }
}
fn statistics_register(address: u32) -> bool {
    (RXGOODFRAMES..=RXDMAOVERRUNS).contains(&address) && address % 4 == 0
}

/// Decode one of the per-channel register blocks: eight consecutive words starting at the
/// block's base address.
fn channel_register(address: u32) -> Option<(u32, usize)> {
    for base in [
        RXFREEBUFFER_BASE,
        RXFLOWTHRESH_BASE,
        TXHDP_BASE,
        RXHDP_BASE,
        TXCP_BASE,
        RXCP_BASE,
    ] {
        if address >= base && address < base + 4 * CHANNELS as u32 && address % 4 == 0 {
            return Some((base, ((address - base) / 4) as usize));
        }
    }
    None
}

pub trait DmaMemory {
    fn dma_read_bytes(&mut self, address: u32, length: usize) -> Vec<u8>;
    fn dma_write_bytes(&mut self, address: u32, bytes: &[u8]);

    fn dma_read_u32(&mut self, address: u32) -> u32 {
        let bytes = self.dma_read_bytes(address, 4);
        u32::from_le_bytes(bytes.try_into().unwrap())
    }

    fn dma_write_u32(&mut self, address: u32, value: u32) {
        self.dma_write_bytes(address, &value.to_le_bytes());
    }
}

impl DmaMemory for BTreeMap<u32, u8> {
    fn dma_read_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
        (0..length)
            .map(|offset| {
                self.get(&address.wrapping_add(offset as u32))
                    .copied()
                    .unwrap_or(0)
            })
            .collect()
    }

    fn dma_write_bytes(&mut self, address: u32, bytes: &[u8]) {
        for (offset, byte) in bytes.iter().copied().enumerate() {
            self.insert(address.wrapping_add(offset as u32), byte);
        }
    }
}

#[cfg(test)]
fn read_u32(memory: &BTreeMap<u32, u8>, address: u32) -> u32 {
    u32::from_le_bytes([
        memory.get(&address).copied().unwrap_or(0),
        memory.get(&address.wrapping_add(1)).copied().unwrap_or(0),
        memory.get(&address.wrapping_add(2)).copied().unwrap_or(0),
        memory.get(&address.wrapping_add(3)).copied().unwrap_or(0),
    ])
}

#[cfg(test)]
fn read_bytes(memory: &BTreeMap<u32, u8>, address: u32, length: usize) -> Vec<u8> {
    (0..length)
        .map(|offset| {
            memory
                .get(&address.wrapping_add(offset as u32))
                .copied()
                .unwrap_or(0)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    #[test]
    fn address_registers_encode_wire_order_bytes() {
        let mut emac = Emac::default();
        let wire = [0x02, 0x13, 0x24, 0x35, 0x46, 0x57];
        let low = 0x5746 | MACADDR_VALID | MACADDR_MATCH | (3 << 16);
        emac.write(MACADDRHI, 0x3524_1302, TEST_CONTEXT);
        emac.write(MACADDRLO, low, TEST_CONTEXT);
        emac.write(MACSRCADDRHI, 0x3524_1302, TEST_CONTEXT);
        emac.write(MACSRCADDRLO, 0x5746, TEST_CONTEXT);
        assert_eq!(emac.addresses[0].bytes, wire);
        assert_eq!(emac.mac_source, wire);
        assert_eq!(emac.read(MACADDRHI, TEST_CONTEXT), Some(0x3524_1302));
        assert_eq!(emac.read(MACADDRLO, TEST_CONTEXT), Some(low));
        assert_eq!(emac.read(MACSRCADDRHI, TEST_CONTEXT), Some(0x3524_1302));
        assert_eq!(emac.read(MACSRCADDRLO, TEST_CONTEXT), Some(0x5746));
    }

    #[test]
    fn each_class_of_frame_goes_to_the_channel_its_rules_name() {
        // SPRU975E section 2.10 and Table 49: an address RAM match routes to the slot's CHANNEL
        // when that channel is enabled in RXUNICASTSET, broadcast goes to RXBROADCH, a
        // hash-matching multicast to RXMULTCH, and anything left over only to RXPROMCH when
        // promiscuous reception is on.
        let mut emac = Emac::default();
        emac.write(MACINDEX, 0, TEST_CONTEXT);
        emac.write(MACADDRHI, 0x1100_0002, TEST_CONTEXT);
        emac.write(
            MACADDRLO,
            0x3322 | MACADDR_VALID | MACADDR_MATCH,
            TEST_CONTEXT,
        );
        emac.write(MACINDEX, 1, TEST_CONTEXT);
        emac.write(MACADDRHI, 0x4400_0002, TEST_CONTEXT);
        emac.write(
            MACADDRLO,
            0x6655 | MACADDR_VALID | MACADDR_MATCH | (2 << 16),
            TEST_CONTEXT,
        );
        emac.write(RXUNICASTSET, 0b101, TEST_CONTEXT);
        // Broadcast to channel 3, hashed multicast to channel 1.
        emac.write(
            RXMBPENABLE,
            RXMBP_BROADCAST | (3 << 8) | RXMBP_MULTICAST | 1,
            TEST_CONTEXT,
        );
        let multicast = [0x01, 0x00, 0x5e, 0x01, 0x02, 0x03];
        let hash = address_hash(&multicast);
        emac.write(MACHASH1, 1 << hash, TEST_CONTEXT);

        let frame = |destination: [u8; 6]| {
            let mut bytes = destination.to_vec();
            bytes.extend_from_slice(&[0x02, 0, 0, 0xaa, 0xbb, 0xcc, 0x08, 0x00]);
            bytes.resize(60, 0);
            bytes
        };
        assert_eq!(
            emac.receive_channel(&frame([0x02, 0, 0, 0x11, 0x22, 0x33])),
            Some((0, 0))
        );
        assert_eq!(
            emac.receive_channel(&frame([0x02, 0, 0, 0x44, 0x55, 0x66])),
            Some((2, 0))
        );
        assert_eq!(emac.receive_channel(&frame([0xff; 6])), Some((3, 0)));
        assert_eq!(emac.receive_channel(&frame(multicast)), Some((1, 0)));
        // An address the RAM does not hold is filtered until RXCAFEN is enabled, and then it
        // lands on the promiscuous channel carrying NOMATCH.
        let stranger = frame([0x02, 0, 0, 0x99, 0x88, 0x77]);
        assert_eq!(emac.receive_channel(&stranger), None);
        emac.write(RXMBPENABLE, RXMBP_PROMISCUOUS | (6 << 16), TEST_CONTEXT);
        assert_eq!(emac.receive_channel(&stranger), Some((6, NOMATCH)));
        // A slot whose match bit is clear filters its address even in promiscuous mode.
        emac.write(MACINDEX, 2, TEST_CONTEXT);
        emac.write(MACADDRHI, 0x9900_0002, TEST_CONTEXT);
        emac.write(MACADDRLO, 0x7788 | MACADDR_VALID, TEST_CONTEXT);
        assert_eq!(emac.receive_channel(&stranger), None);
    }

    #[test]
    fn the_raw_interrupt_status_names_the_channel() {
        // Bit n stays set while channel n holds a completion the driver has not acknowledged
        // (SPRU975E sections 5.7 and 5.13).
        let mut emac = Emac::default();
        emac.tx_completion[5] = 0x1000;
        emac.rx_completion[2] = 0x2000;
        assert_eq!(emac.read(TXINTSTATRAW, TEST_CONTEXT), Some(1 << 5));
        assert_eq!(emac.read(RXINTSTATRAW, TEST_CONTEXT), Some(1 << 2));
        emac.write(TXCP_BASE + 4 * 5, 0x1000, TEST_CONTEXT);
        assert_eq!(emac.read(TXINTSTATRAW, TEST_CONTEXT), Some(0));
        emac.write(RXCP_BASE + 4 * 2, 0x2000, TEST_CONTEXT);
        assert_eq!(emac.read(RXINTSTATRAW, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn the_address_hash_folds_the_destination_into_six_bits() {
        // SPRU975E §5.36: bit i of the hash exclusive-ors every sixth bit of the 48-bit
        // destination address starting at bit i.
        assert_eq!(address_hash(&[0x01, 0x00, 0x5e, 0x11, 0x22, 0x33]), 17);
        assert_eq!(address_hash(&[0x01, 0x00, 0x5e, 0x44, 0x55, 0x66]), 27);
        assert_eq!(address_hash(&[0xff; 6]), 0);
    }

    #[test]
    fn receive_filtering_follows_the_enable_bits_and_the_address_ram() {
        let mut emac = Emac::default();
        emac.write(MACINDEX, 0, TEST_CONTEXT);
        emac.write(MACADDRHI, 0x1100_0002, TEST_CONTEXT);
        emac.write(
            MACADDRLO,
            0x3322 | MACADDR_VALID | MACADDR_MATCH,
            TEST_CONTEXT,
        );
        emac.write(RXUNICASTSET, 1, TEST_CONTEXT);
        emac.write(RXMAXLEN, 100, TEST_CONTEXT);
        emac.write(MACHASH1, 1 << 17, TEST_CONTEXT);

        let frame = |destination: [u8; 6], length: usize| {
            let mut bytes = destination.to_vec();
            bytes.resize(length, 0x5a);
            bytes
        };
        let station = [0x02, 0x00, 0x00, 0x11, 0x22, 0x33];
        let stranger = [0x02, 0x00, 0x00, 0x99, 0x88, 0x77];
        let hashed = [0x01, 0x00, 0x5e, 0x11, 0x22, 0x33];
        let unhashed = [0x01, 0x00, 0x5e, 0x44, 0x55, 0x66];

        // Unicast needs both an address match and the channel's unicast enable.
        assert!(emac.receive_channel(&frame(station, 60)).is_some());
        assert!(emac.receive_channel(&frame(stranger, 60)).is_none());
        emac.write(RXUNICASTCLEAR, 1, TEST_CONTEXT);
        assert!(emac.receive_channel(&frame(station, 60)).is_none());
        emac.write(RXUNICASTSET, 1, TEST_CONTEXT);

        // Broadcast and multicast each need their enable bit, and multicast also needs its
        // hash bucket.
        assert!(emac.receive_channel(&frame([0xff; 6], 60)).is_none());
        assert!(emac.receive_channel(&frame(hashed, 60)).is_none());
        emac.write(RXMBPENABLE, RXMBP_BROADCAST | RXMBP_MULTICAST, TEST_CONTEXT);
        assert!(emac.receive_channel(&frame([0xff; 6], 60)).is_some());
        assert!(emac.receive_channel(&frame(hashed, 60)).is_some());
        assert!(emac.receive_channel(&frame(unhashed, 60)).is_none());

        // Length no longer decides routing: the class decides which statistic the frame
        // reaches and which copy enable has to be set for it to be transferred at all.
        assert_eq!(emac.classify(&frame(station, 60)), FrameClass::Proper);
        assert_eq!(emac.classify(&frame(station, 200)), FrameClass::Oversized);
        assert_eq!(emac.classify(&frame(station, 40)), FrameClass::Undersized);
        emac.write(RXMBPENABLE, RXMBP_PROMISCUOUS, TEST_CONTEXT);
        assert!(emac.receive_channel(&frame(stranger, 60)).is_some());
    }

    #[test]
    fn a_packet_spans_one_descriptor_per_fragment() {
        // SPRU975E §2.5.5: a receive packet fills as many descriptors as it needs; the first
        // carries SOP and the packet length, the last carries EOP and, when the chain ends
        // there, EOQ. The buffer offset applies to the start of the packet only.
        let mut memory = BTreeMap::new();
        let mut emac = Emac::default();
        // A twenty-byte frame is short, so RXCSFEN has to be set for it to reach memory.
        emac.write(RXMBPENABLE, RXMBP_PROMISCUOUS | RXMBP_CSFEN, TEST_CONTEXT);
        emac.write(RXBUFFEROFFSET, 2, TEST_CONTEXT);
        emac.write(RXCONTROL, 1, TEST_CONTEXT);
        // The DMA enable is the inner gate; GMIIEN takes the interface out of reset, and TI's
        // own sequence writes it last (SPRU975E section 2.15.4 step 17).
        emac.write(MACCONTROL, MACCONTROL_GMIIEN, TEST_CONTEXT);

        // Three descriptors of eight bytes each at 0x1000, buffers at 0x2000.
        for slot in 0..3u32 {
            let descriptor = 0x1000 + slot * 16;
            let next = if slot == 2 { 0 } else { descriptor + 16 };
            memory.dma_write_u32(descriptor, next);
            memory.dma_write_u32(descriptor + 4, 0x2000 + slot * 0x100);
            memory.dma_write_u32(descriptor + 8, 8);
            memory.dma_write_u32(descriptor + 12, OWNER);
        }
        emac.write(RXHDP_BASE, 0x1000, TEST_CONTEXT);
        let frame: Vec<u8> = (0..20u8).collect();
        emac.inject_frame(frame.clone());
        emac.tick(&mut memory);

        // Eight, eight and four bytes, the first fragment two bytes into its buffer.
        assert_eq!(read_u32(&memory, 0x1008), (2 << 16) | 8);
        assert_eq!(read_u32(&memory, 0x1018), 8);
        assert_eq!(read_u32(&memory, 0x1028), 4);
        assert_eq!(read_bytes(&memory, 0x2002, 8), frame[..8]);
        assert_eq!(read_bytes(&memory, 0x2100, 8), frame[8..16]);
        assert_eq!(read_bytes(&memory, 0x2200, 4), frame[16..]);

        let first = memory.dma_read_u32(0x100c);
        let middle = memory.dma_read_u32(0x101c);
        let last = memory.dma_read_u32(0x102c);
        assert_eq!(first & (SOP | EOP | OWNER), SOP);
        // The class and match flags the frame arrived with land in the SOP descriptor.
        assert_eq!(first & (UNDERSIZED | NOMATCH), UNDERSIZED | NOMATCH);
        assert_eq!(first & 0xffff, 20);
        assert_eq!(middle & (SOP | EOP | OWNER), 0);
        assert_eq!(last & (SOP | EOP | OWNER | EOQ), EOP | EOQ);
        assert_eq!(emac.read(RXCP_BASE, TEST_CONTEXT), Some(0x1020));
        assert_eq!(emac.read(RXHDP_BASE, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn statistics_are_write_to_decrement_once_gmiien_is_set() {
        let mut emac = Emac::default();
        emac.count(RXGOODFRAMES);
        emac.count(RXGOODFRAMES);
        emac.count(RXGOODFRAMES);
        assert_eq!(emac.read(RXGOODFRAMES, TEST_CONTEXT), Some(3));
        // Without GMIIEN a statistics register is an ordinary read/write location; with it the
        // written value is subtracted and the result saturates at zero (SPRU975E §5.49).
        emac.write(RXGOODFRAMES, 2, TEST_CONTEXT);
        assert_eq!(emac.read(RXGOODFRAMES, TEST_CONTEXT), Some(2));
        emac.write(RXGOODFRAMES, 3, TEST_CONTEXT);
        emac.write(MACCONTROL, MACCONTROL_GMIIEN, TEST_CONTEXT);
        emac.write(RXGOODFRAMES, 2, TEST_CONTEXT);
        assert_eq!(emac.read(RXGOODFRAMES, TEST_CONTEXT), Some(1));
        emac.write(RXGOODFRAMES, 5, TEST_CONTEXT);
        assert_eq!(emac.read(RXGOODFRAMES, TEST_CONTEXT), Some(0));
    }
}
