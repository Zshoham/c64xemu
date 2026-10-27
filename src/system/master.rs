//! Master access: how a system master reaches the device.
//!
//! The HPI's DMA logic, each megamodule's IDMA, the EDMA3's transfer controllers, the EMACs'
//! buffer and descriptor DMA and the UTOPIA's PDMA all move data the same way: a [`Privilege`],
//! an address and bytes or a word, on a device whose memories are cached and protected and
//! whose peripherals answer through slave ports. Each of those masters used to pick its own
//! privilege, width and route; this module is the one place those are decided, so a master
//! says what it wants moved and under whose identity ([`MasterAccess`]) and the fabric
//! decides everything else:
//!
//! - **Route.** An address that is memory (`is_memory`) goes to the backing store; an address
//!   inside a peripheral aperture reaches that peripheral's register through its slave port,
//!   with every side effect a CPU access would have, and the identity is presented to the
//!   EDMA3 first because its registers are judged by it (SPRU966C section 2.10.2) and the
//!   device is marked dirty (`Shared::touched`).
//! - **Megamodule registers.** The megamodules' own registers - interrupt controller, cache
//!   and protection configuration - are "only accessible by the CPU" (SPRU871K section
//!   5.3.1): a master reads zero there and its writes go nowhere.
//! - **Protection.** A master's access to memory is judged by the memory protection unit of the
//!   megamodule that owns the page, under the PrivID and mode it carries. The first protected
//!   byte latches the fault and refuses the whole access: a refused read gives nothing, a
//!   refused write stores nothing and does not reach the L1D snoop (SPRU871K section 4.6.1).
//! - **Coherence.** Only L2 SRAM is snooped against L1D, under the local and the global tag;
//!   external memory is not snooped at all (SPRU862B section 2.4).
//! - **Byte order.** A word is its four bytes in the device's byte order; an element narrower
//!   than a word is its low bytes, right-justified (SPRUEG2E Table 32). Every assembly of a word
//!   from bytes, and every split of one into bytes, is in the two functions below.
//!
//! A master that is lent out of the fabric for its own transfer (`Fabric::lend_emac`,
//! `lend_edma`) has an empty slot, so its own aperture is claimed by no port and a master's
//! access to it falls through to the memory behind: the EMAC reads its CPPI descriptor RAM
//! this way, outside memory protection, as it does any register access.
//!
//! Every access passes three stages in `Fabric::read`/`write`: the route, `admit` - the one
//! place a judge of master accesses sits - and the transfer. The shared-memory controller is not
//! a judge: it has no protection of its own (SPRUEG5D section 1.2), so it sits in the transfer
//! stage, where a transfer to the shared L2 becomes requests to it (`note_sl2_transfer`).
//! CPU accesses cross the core bus instead, and the debug port crosses `host_read_bytes`.

use super::{
    mpu_page, permission_bit, Endian, Fabric, Permission, Placement, Requester,
    MEGAMODULE_REGISTERS,
};
use crate::memory::is_memory;

/// The width of an SMC bank, 256 bits (SPRUEG5D Figure 2): one master request.
const BANK_WORD_BYTES: u32 = 32;

/// Whom a master's access is made for: the PrivID and privilege mode memory protection judges
/// it by. Not to be confused with `Requester`, which only picks the address view an access
/// resolves through. A core's PrivID is its index - AID0 to AID5 on the C6472 (SPRS612G Table
/// 5-3), 0 on the one-core C6455 (SPRS276M section 5.2) - and an EDMA3 transfer carries the
/// one stamped on its PaRAM set (SPRU966C section 2.10.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Privilege {
    pub id: u8,
    pub user: bool,
}

impl Privilege {
    /// The HPI's DMA and the IDMA: PrivID 0, supervisor (SPRS276M section 5.2, SPRS612G
    /// Table 5-3).
    pub const ID0_SUPERVISOR: Self = Self { id: 0, user: false };
    /// The EMACs and the UTOPIA PDMA: PrivID 0, always user (SPRS612G section 5.2: "For most
    /// peripheral masters (EMAC0, EMAC1, UTOPIA, TSIP0, TSIP1, and TSIP2), the privilege mode
    /// is always user mode").
    pub const ID0_USER: Self = Self { id: 0, user: true };

    /// The bit of a page attribute register that admits this PrivID: AID0 to AID5 at bits 10
    /// to 15, and every id above them with AIDX at bit 9 (SPRU966C Figure 4-28; the
    /// megamodule's MPPA registers share the layout).
    pub fn allowed_id_bit(self) -> u32 {
        if self.id < 6 {
            1 << (10 + u32::from(self.id))
        } else {
            1 << 9
        }
    }
}

/// What a system master moves data through. Each access names the privilege it is made with,
/// so the device can apply that identity's protection; the fabric is the only production
/// implementation, and the test adapters (`BTreeMap<u32, u8>` here, the witness in the EDMA3's
/// tests) are what make the seam real.
pub trait MasterAccess {
    /// The device's byte order, which a word's bytes are assembled in.
    fn byte_order(&self) -> Endian;

    /// `length` bytes at `address`, or `None` when protection refuses the access.
    fn read(&mut self, privilege: Privilege, address: u32, length: usize) -> Option<Vec<u8>>;

    /// Store `bytes` at `address`; a refused access stores nothing.
    fn write(&mut self, privilege: Privilege, address: u32, bytes: &[u8]);

    /// A refused read as a master that cannot tell reads it: zeros.
    fn read_or_zero(&mut self, privilege: Privilege, address: u32, length: usize) -> Vec<u8> {
        self.read(privilege, address, length)
            .unwrap_or_else(|| vec![0; length])
    }

    /// The word at `address` in the device's byte order, or `None` when protection refuses it.
    fn read_word(&mut self, privilege: Privilege, address: u32) -> Option<u32> {
        read_element_bytes(self, privilege, address, 4)
    }

    fn write_word(&mut self, privilege: Privilege, address: u32, value: u32) {
        write_element_bytes(self, privilege, address, value, 4);
    }

    /// An element of `width` bytes (1 to 4), zero-extended into a word: the low bytes of the
    /// word, right-justified, in the device's byte order (SPRUEG2E Table 32). A word is the
    /// element of width 4.
    fn read_element(&mut self, privilege: Privilege, address: u32, width: usize) -> Option<u32> {
        if width == 4 {
            self.read_word(privilege, address)
        } else {
            read_element_bytes(self, privilege, address, width)
        }
    }

    fn write_element(&mut self, privilege: Privilege, address: u32, value: u32, width: usize) {
        if width == 4 {
            self.write_word(privilege, address, value);
        } else {
            write_element_bytes(self, privilege, address, value, width);
        }
    }
}

/// An element assembled from its bytes: right-justified in the device's byte order.
fn element_from_bytes(order: Endian, bytes: &[u8]) -> u32 {
    let mut word = [0; 4];
    match order {
        Endian::Little => {
            word[..bytes.len()].copy_from_slice(bytes);
            u32::from_le_bytes(word)
        }
        Endian::Big => {
            word[4 - bytes.len()..].copy_from_slice(bytes);
            u32::from_be_bytes(word)
        }
    }
}

/// The bytes an element of `width` bytes is stored as: the first `width` of the returned array
/// from the offset it also returns.
fn element_to_bytes(order: Endian, value: u32, width: usize) -> ([u8; 4], usize) {
    match order {
        Endian::Little => (value.to_le_bytes(), 0),
        Endian::Big => (value.to_be_bytes(), 4 - width),
    }
}

fn read_element_bytes<M: MasterAccess + ?Sized>(
    master: &mut M,
    privilege: Privilege,
    address: u32,
    width: usize,
) -> Option<u32> {
    let bytes = master.read(privilege, address, width)?;
    Some(element_from_bytes(master.byte_order(), &bytes))
}

fn write_element_bytes<M: MasterAccess + ?Sized>(
    master: &mut M,
    privilege: Privilege,
    address: u32,
    value: u32,
    width: usize,
) {
    let (bytes, start) = element_to_bytes(master.byte_order(), value, width);
    master.write(privilege, address, &bytes[start..start + width]);
}

/// The fabric is the one production adapter.
///
/// Its word accesses differ from its byte accesses in one respect, deliberately: a word access
/// to a register is one access to that register, while a byte access assembles whole words
/// around the bytes it is given and so reads the register before writing it back. That read
/// has the side effects a read has, which a word master (the HPI, the IDMA, a PDMA word, an
/// EMAC descriptor) never asked for.
impl MasterAccess for Fabric {
    fn byte_order(&self) -> Endian {
        self.endian()
    }

    fn read(&mut self, privilege: Privilege, address: u32, length: usize) -> Option<Vec<u8>> {
        if !is_memory(address) {
            return Some(self.register_bytes(address, length, privilege));
        }
        if !self.admit(address, length, Permission::Read, privilege) {
            return None;
        }
        Some(self.memory_bytes(address, length))
    }

    fn write(&mut self, privilege: Privilege, address: u32, bytes: &[u8]) {
        if !is_memory(address) {
            self.store_register_bytes(address, bytes, privilege);
            return;
        }
        if self.admit(address, bytes.len(), Permission::Write, privilege) {
            self.store_memory_bytes(address, bytes);
        }
    }

    fn read_word(&mut self, privilege: Privilege, address: u32) -> Option<u32> {
        if !is_memory(address) {
            return Some(self.register_word(address, privilege));
        }
        read_element_bytes(self, privilege, address, 4)
    }

    fn write_word(&mut self, privilege: Privilege, address: u32, value: u32) {
        if !is_memory(address) {
            self.store_register_word(address, value, privilege);
            return;
        }
        write_element_bytes(self, privilege, address, value, 4);
    }
}

impl Fabric {
    /// A master's word in a peripheral aperture. A chip-level peripheral answers through its
    /// slave port, with every effect a CPU access would have. The megamodules' own registers
    /// are "only accessible by the CPU" (SPRU871K section 5.3.1): a master reads zero there and
    /// its writes go nowhere. Whatever else the aperture holds that no port claims is device
    /// memory, reached as any other memory is - which is also how a master that is lent out
    /// reaches its own aperture, since nothing answers for it while it is out.
    fn register_word(&mut self, address: u32, privilege: Privilege) -> u32 {
        self.present_to_edma(privilege);
        self.shared.touched();
        let context = self.port_context();
        if let Some(value) = self.peripheral_read(address, context) {
            return value;
        }
        if MEGAMODULE_REGISTERS.contains(&address) {
            return 0;
        }
        let bytes = self.memory_bytes(address, 4);
        element_from_bytes(self.endian(), &bytes)
    }

    fn store_register_word(&mut self, address: u32, value: u32, privilege: Privilege) {
        self.present_to_edma(privilege);
        self.shared.touched();
        let context = self.port_context();
        if self.peripheral_write(address, value, context) || MEGAMODULE_REGISTERS.contains(&address)
        {
            return;
        }
        let (bytes, _) = element_to_bytes(self.endian(), value, 4);
        self.store_memory_bytes(address, &bytes);
    }

    /// `length` bytes of a peripheral aperture, a register word at a time.
    fn register_bytes(&mut self, address: u32, length: usize, privilege: Privilege) -> Vec<u8> {
        let order = self.endian();
        let mut bytes = Vec::with_capacity(length);
        while bytes.len() < length {
            let byte_address = address.wrapping_add(bytes.len() as u32);
            let word = self.register_word(byte_address & !3, privilege);
            let (word, _) = element_to_bytes(order, word, 4);
            for byte in word.into_iter().skip((byte_address & 3) as usize) {
                if bytes.len() == length {
                    break;
                }
                bytes.push(byte);
            }
        }
        bytes
    }

    /// Write a peripheral aperture, assembling whole words around the bytes the transfer
    /// supplies.
    fn store_register_bytes(&mut self, address: u32, bytes: &[u8], privilege: Privilege) {
        let order = self.endian();
        let mut written = 0usize;
        while written < bytes.len() {
            let byte_address = address.wrapping_add(written as u32);
            let word_address = byte_address & !3;
            let current = self.register_word(word_address, privilege);
            let (mut word, _) = element_to_bytes(order, current, 4);
            for slot in word.iter_mut().skip((byte_address & 3) as usize) {
                if written == bytes.len() {
                    break;
                }
                *slot = bytes[written];
                written += 1;
            }
            let value = element_from_bytes(order, &word);
            self.store_register_word(word_address, value, privilege);
        }
    }

    /// The one judge of a master's memory access: whether protection lets a master with
    /// `privilege` make `permission` accesses to all of `length` bytes at `address`. The first
    /// protected byte latches the fault and refuses the whole access.
    fn admit(
        &mut self,
        address: u32,
        length: usize,
        permission: Permission,
        privilege: Privilege,
    ) -> bool {
        if !self.dma_protection_active() {
            return true;
        }
        (0..length).all(|offset| {
            let byte_address = address.wrapping_add(offset as u32);
            self.check_dma_access(byte_address, permission, privilege)
        })
    }

    /// Whether any megamodule restricts DMA masters at all; until one does, no DMA access is
    /// checked.
    fn dma_protection_active(&self) -> bool {
        (0..self.cores).any(|core| self.mm[core].mpu_dma_restrictive)
    }

    /// DMA accesses carry a requestor PrivID rather than LOCAL, and the privilege mode the
    /// master transfers in. IDMA, the HPI's DMA, the UTOPIA PDMA and the EMACs use ID 0; an
    /// EDMA3 transfer carries the identity stamped on its PaRAM set. The unit consulted belongs
    /// to whichever megamodule owns the memory, which on a multicore device is decided by the
    /// global address the master used.
    fn check_dma_access(
        &mut self,
        address: u32,
        permission: Permission,
        privilege: Privilege,
    ) -> bool {
        let Some((core, unit, page)) = self.dma_mpu_page(address) else {
            return true;
        };
        let attributes = self.mm[core].mpu_pages[unit][page];
        let permission_bit = permission_bit(permission, privilege.user);
        if attributes & privilege.allowed_id_bit() != 0 && attributes & permission_bit != 0 {
            return true;
        }
        let status = (u32::from(privilege.id) << 9) | permission_bit;
        self.mm[core].record_mpu_fault(unit, address, status, self.profile.events);
        false
    }

    /// The megamodule, unit and page a DMA address protects, resolving a global alias back to
    /// the local address the protection registers are written in terms of.
    fn dma_mpu_page(&self, address: u32) -> Option<(usize, usize, usize)> {
        let map = self.profile.map;
        let (core, local) = match map.local_of_global(address) {
            Some(pair) => pair,
            // Without global aliases a DMA master addresses the only megamodule's local
            // windows directly.
            None if !map.has_global_aliases() => (0, address),
            None => return None,
        };
        let (unit, page) = mpu_page(self.profile.mpu_regions, local)?;
        (core < self.mm.len()).then_some((core, unit, page))
    }

    /// A DMA read of memory: the stored bytes with the L2 SRAM snoop applied.
    ///
    /// The L2 controller serves a DMA read of L2 SRAM from L1D when the CPU holds that line
    /// dirty, so a buffer the CPU has only written into its cache still goes out correctly
    /// (SPRU862B section 2.4.2). External memory is not snooped at all. On a multicore device
    /// the line may be tagged with either the local or the global address, depending on which
    /// the core used, so both are looked for.
    fn memory_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
        self.note_sl2_transfer(address, length, true);
        self.note_coherence(address, length, false);
        let mut data = self
            .shared
            .backing
            .read_bytes(Requester::Dma, address, length);
        if let Some((core, local)) = self.snooped_l2(address) {
            self.mm[core].memory.snoop_read(local, &mut data);
            if local != address {
                self.mm[core].memory.snoop_read(address, &mut data);
            }
        }
        data
    }

    /// A DMA write to L2 SRAM is snoop-written into L1D, so the CPU sees it without any cache
    /// operation. Outside L2 SRAM the CPU keeps its stale line, and maintaining that is the
    /// firmware's job (SPRU862B sections 2.4 and 2.6).
    fn store_memory_bytes(&mut self, address: u32, bytes: &[u8]) {
        self.note_sl2_transfer(address, bytes.len(), false);
        self.shared.backing.note_memory_write(address, bytes.len());
        self.shared
            .backing
            .write_bytes(Requester::Dma, address, bytes);
        if let Some((core, local)) = self.snooped_l2(address) {
            self.mm[core].memory.snoop_write(local, bytes);
            if local != address {
                self.mm[core].memory.snoop_write(address, bytes);
            }
        }
        self.note_coherence(address, bytes.len(), true);
    }

    /// Hand a master's transfer to every core's coherence check, under each address that core
    /// could have cached the bytes by: the one the master used, the core's local name for its
    /// own memory behind a global alias, and - for the shared L2 - its local window and its alias
    /// in every core's global block, since a core may cache it through any of them. The owner of
    /// snooped L2 SRAM is the one core the hardware keeps coherent; for every other core the
    /// same bytes are someone else's memory.
    fn note_coherence(&mut self, address: u32, length: usize, write: bool) {
        if !self.mm[0].memory.checks_coherence() {
            return;
        }
        let map = self.profile.map;
        let owner = self.snooped_l2(address).map(|(core, _)| core);
        let local = map.local_of_global(address);
        let mut common = vec![address];
        if let Some((_, offset)) = map.shared_ram_alias(address) {
            let shared = map.shared_ram_base() + offset;
            common.push(shared);
            common.extend(
                (0..self.cores)
                    .filter_map(|core| map.global_alias_base(core).map(|base| base + shared)),
            );
        }
        let length = length as u32;
        for core in 0..self.cores {
            let snooped = owner == Some(core);
            let mut tags = common.clone();
            tags.extend(
                local
                    .filter(|(owner, _)| *owner == core)
                    .map(|(_, offset)| offset),
            );
            tags.sort_unstable();
            tags.dedup();
            let memory = &mut self.mm[core].memory;
            for tag in tags {
                if write {
                    memory.master_wrote(tag, length, snooped);
                } else {
                    memory.master_read(tag, length, snooped);
                }
            }
        }
    }

    /// A master's transfer to the shared L2 is a series of requests to the shared-memory
    /// controller, one per 256-bit bank word it touches (SPRUEG5D Figure 2), and there is no
    /// cache in front of it to answer any: L1D is not snooped against SL2 (the snoop is of L2
    /// SRAM only), so every word reaches the SMC.
    ///
    /// UNCERTAIN: the SMC has six ports, one per megamodule, and its guide says "all the accesses
    /// to the SMC go through the L2 controller" (section 1.3) without saying which megamodule a
    /// master's access enters through. The only address a master can use is a megamodule's global
    /// alias of SL2 (`AddressMap::shared_ram_alias`), which names one, so the request is that
    /// megamodule's: its profiler counts it. The first byte decides whether a transfer is SL2's;
    /// the SMC never judges it, because "no memory protection support on SMC" (section 1.2) -
    /// the L2 controller's `admit` has already.
    fn note_sl2_transfer(&mut self, address: u32, length: usize, read: bool) {
        let Some((core, offset)) = self.profile.map.shared_ram_alias(address) else {
            return;
        };
        let end =
            (u64::from(offset) + length as u64).min(u64::from(self.profile.map.shared_ram_bytes()));
        let mut word = u64::from(offset & !(BANK_WORD_BYTES - 1));
        while word < end {
            self.shared.sl2_request(core, word as u32, read);
            word += u64::from(BANK_WORD_BYTES);
        }
    }

    /// The megamodule whose L2 SRAM a DMA address lands in, and the local address that core
    /// would have used - the only memory the coherence protocol snoops. Memory the core has
    /// converted into cache is not L2 SRAM and is not snooped.
    fn snooped_l2(&self, address: u32) -> Option<(usize, u32)> {
        let map = self.profile.map;
        match self.shared.backing.placement(Requester::Dma, address) {
            Placement::CoreL2 { core, offset } => Some((core, map.local_l2().0 + offset)),
            _ => None,
        }
    }
}

/// The test adapter that backs every master's tests without a device: a sparse little-endian
/// memory that reads zero where nothing was written and judges nothing.
#[cfg(test)]
impl MasterAccess for std::collections::BTreeMap<u32, u8> {
    fn byte_order(&self) -> Endian {
        Endian::Little
    }

    fn read(&mut self, _privilege: Privilege, address: u32, length: usize) -> Option<Vec<u8>> {
        Some(
            (0..length)
                .map(|offset| {
                    self.get(&address.wrapping_add(offset as u32))
                        .copied()
                        .unwrap_or(0)
                })
                .collect(),
        )
    }

    fn write(&mut self, _privilege: Privilege, address: u32, bytes: &[u8]) {
        for (offset, byte) in bytes.iter().copied().enumerate() {
            self.insert(address.wrapping_add(offset as u32), byte);
        }
    }
}

/// The seam every master crosses, exercised for each master's privilege on both devices: what
/// one master gets, every master gets, and the identity it carries is what the device judges.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::{DeviceProfile, C6455, C6472};
    use crate::memory::{CoherenceHazard, DataCache, L1DCFG, L2CFG, MAR_BASE};
    use std::collections::BTreeMap;

    const L2MPFAR: u32 = 0x0184_a000;
    const L2MPFSR: u32 = 0x0184_a004;
    /// L2MPPA0 covers the first page of L2 SRAM and L2MPPA1 the one after it.
    const L2MPPA1: u32 = 0x0184_a204;
    const L2_SRAM: u32 = 0x0080_0000;
    const LOCAL: u32 = 1 << 8;
    const PERMISSIONS: u32 = 0x3f;
    const PARAM: u32 = 0x02a0_4000;
    const ESR: u32 = 0x02a0_1010;

    /// Every kind of master the device has, by the privilege its accesses carry: the HPI and
    /// the IDMA (PrivID 0, supervisor), the EMACs and the UTOPIA PDMA (PrivID 0, user), and an
    /// EDMA3 transfer made for the core that programmed it - which on the six-core device is
    /// not PrivID 0 (SPRS612G Table 5-3).
    fn masters(profile: &DeviceProfile) -> Vec<(&'static str, Privilege)> {
        let mut masters = vec![
            ("HPI or IDMA", Privilege::ID0_SUPERVISOR),
            ("EMAC or PDMA", Privilege::ID0_USER),
        ];
        if profile.map.has_global_aliases() {
            masters.push((
                "EDMA3 for core 3 supervisor",
                Privilege { id: 3, user: false },
            ));
            masters.push(("EDMA3 for core 3 user", Privilege { id: 3, user: true }));
        }
        masters
    }

    /// The core whose memory the tests use: the only one on the C6455, a peer on the C6472.
    fn owner(profile: &DeviceProfile) -> usize {
        if profile.map.has_global_aliases() {
            3
        } else {
            0
        }
    }

    /// The address a DMA master uses for `local` in `core`'s memory: the C6472 gives each
    /// core's memories a global alias, the C6455 has none.
    fn global(profile: &DeviceProfile, core: usize, local: u32) -> u32 {
        profile
            .map
            .global_alias_base(core)
            .map_or(local, |base| base + local)
    }

    fn l2_page_bytes(profile: &DeviceProfile) -> u32 {
        profile
            .mpu_regions
            .iter()
            .find(|region| region.unit == 0 && region.base == L2_SRAM)
            .unwrap()
            .page_bytes
    }

    #[test]
    fn a_protected_byte_refuses_the_whole_access_and_latches_one_fault() {
        for profile in [&C6455, &C6472] {
            let core = owner(profile);
            for (name, privilege) in masters(profile) {
                for permission in [Permission::Read, Permission::Write] {
                    let context = format!("{} {name} {}", profile.name, permission as u8);
                    let edge = L2_SRAM + l2_page_bytes(profile);
                    let start = global(profile, core, edge - 2);
                    let bit = permission_bit(permission, privilege.user);

                    let mut fabric = Fabric::new(profile);
                    fabric.load_bytes(core, edge - 2, &[1, 2, 3, 4]);
                    // The second page grants no requestor anything; the first is open, so the
                    // access starts where it would be let through and crosses into the page
                    // that is not.
                    fabric.write32(core, L2MPPA1, LOCAL | PERMISSIONS);
                    let intact = |fabric: &Fabric| fabric.host_read_bytes(core, edge - 2, 4);
                    match permission {
                        Permission::Read => {
                            assert_eq!(fabric.read(privilege, start, 4), None, "{context}");
                            assert_eq!(fabric.read_word(privilege, start), None, "{context}");
                            assert_eq!(fabric.read_or_zero(privilege, start, 4), [0; 4]);
                        }
                        _ => {
                            fabric.write(privilege, start, &[9; 4]);
                            assert_eq!(intact(&fabric), [1, 2, 3, 4], "{context}: nothing stored");
                            fabric.write_word(privilege, start, 0x0909_0909);
                        }
                    }
                    assert_eq!(intact(&fabric), [1, 2, 3, 4], "{context}: nothing stored");
                    // The first protected byte is the one named, with who asked and how.
                    assert_eq!(
                        fabric.read32(core, L2MPFAR),
                        global(profile, core, edge),
                        "{context}"
                    );
                    assert_eq!(
                        fabric.read32(core, L2MPFSR),
                        (u32::from(privilege.id) << 9) | bit,
                        "{context}"
                    );

                    // The same page granting this identity this permission lets it through.
                    let mut fabric = Fabric::new(profile);
                    fabric.load_bytes(core, edge - 2, &[1, 2, 3, 4]);
                    fabric.write32(core, L2MPPA1, privilege.allowed_id_bit() | LOCAL | bit);
                    match permission {
                        Permission::Read => {
                            assert_eq!(fabric.read(privilege, start, 4), Some(vec![1, 2, 3, 4]));
                        }
                        _ => {
                            fabric.write(privilege, start, &[9; 4]);
                            assert_eq!(intact(&fabric), [9; 4], "{context}");
                        }
                    }
                    assert_eq!(fabric.read32(core, L2MPFSR), 0, "{context}: no fault");
                }
            }
        }
    }

    #[test]
    fn a_refused_write_does_not_snoop_a_dirty_l1d_line() {
        // SPRU871K section 4.6.1: protection is checked before snoop-write commands are issued
        // to L1D, so a refused write leaves the CPU's line alone. The granted control proves
        // the same write does reach the snoop.
        for profile in [&C6455, &C6472] {
            let core = owner(profile);
            for (name, privilege) in masters(profile) {
                for granted in [false, true] {
                    let mut fabric = Fabric::new(profile);
                    let address = global(profile, core, L2_SRAM + 0x100);
                    fabric.load_bytes(core, L2_SRAM + 0x100, &[0; 4]);
                    fabric.write32(core, L1DCFG, 4);
                    assert_eq!(fabric.read32(core, L2_SRAM + 0x100), 0);
                    fabric.write32(core, L2_SRAM + 0x100, 0x1234_5678);
                    let bit = permission_bit(Permission::Write, privilege.user);
                    let page = if granted {
                        privilege.allowed_id_bit() | bit
                    } else {
                        0
                    };
                    fabric.write32(core, 0x0184_a200, LOCAL | PERMISSIONS & !bit | page);
                    fabric.write_word(privilege, address, 0x0bad_c0de);
                    assert_eq!(
                        fabric.read32(core, L2_SRAM + 0x100),
                        if granted { 0x0bad_c0de } else { 0x1234_5678 },
                        "{} {name} granted {granted}",
                        profile.name
                    );
                }
            }
        }
    }

    #[test]
    fn l2_sram_is_coherent_with_l1d_under_either_tag() {
        // SPRU862B section 2.4.2 and SPRS612G Table 2-2: a DMA read of L2 SRAM is served from
        // L1D when the CPU holds the line dirty, and a DMA write is snoop-written into it,
        // whichever of the local and the global address the CPU tagged the line with.
        for profile in [&C6455, &C6472] {
            let core = owner(profile);
            let tags = if profile.map.has_global_aliases() {
                vec![L2_SRAM + 0x2000, global(profile, core, L2_SRAM + 0x2000)]
            } else {
                vec![L2_SRAM + 0x2000]
            };
            for (name, privilege) in masters(profile) {
                for cpu_address in &tags {
                    let context = format!("{} {name} cpu {cpu_address:#x}", profile.name);
                    let dma = global(profile, core, L2_SRAM + 0x2000);
                    let mut fabric = Fabric::new(profile);
                    fabric.write32(core, L1DCFG, 4);
                    // A line is only tagged with the global address when its MAR bit lets L1D
                    // cache that region.
                    fabric.write32(core, MAR_BASE + (cpu_address >> 24) * 4, 1);
                    fabric.load_bytes(core, L2_SRAM + 0x2000, &[0; 64]);
                    // L1D allocates on a read miss, so the line is read before the store leaves
                    // it dirty in the cache (SPRU862B section 3.4).
                    assert_eq!(fabric.read32(core, *cpu_address), 0);
                    fabric.write32(core, *cpu_address, 0xdead_beef);
                    assert_eq!(
                        fabric.shared.backing.read_bytes(Requester::Dma, dma, 4),
                        [0; 4],
                        "{context}: the line has not reached the SRAM"
                    );
                    assert_eq!(
                        fabric.read_word(privilege, dma),
                        Some(0xdead_beef),
                        "{context}"
                    );
                    fabric.write_word(privilege, dma, 0x0bad_c0de);
                    assert_eq!(fabric.read32(core, *cpu_address), 0x0bad_c0de, "{context}");
                    if profile.map.has_global_aliases() {
                        assert_eq!(fabric.read32(core - 1, L2_SRAM + 0x2000), 0, "{context}");
                    }
                }
            }
        }
    }

    #[test]
    fn external_memory_is_not_snooped() {
        // SPRU862B section 2.4: only L2 SRAM is. A line the CPU holds dirty in cache is
        // invisible to a DMA read, and a DMA write leaves the CPU its stale line.
        const DDR: u32 = 0xe000_1000;
        for profile in [&C6455, &C6472] {
            for (name, privilege) in masters(profile) {
                let mut fabric = Fabric::new(profile);
                fabric.load_bytes(0, DDR, &0x1111_1111u32.to_le_bytes());
                fabric.write32(0, MAR_BASE + 224 * 4, 1);
                fabric.write32(0, L2CFG, 1);
                fabric.write32(0, L1DCFG, 1);
                assert_eq!(fabric.read32(0, DDR), 0x1111_1111);
                fabric.write32(0, DDR, 0xdead_beef);
                let context = format!("{} {name}", profile.name);
                assert_eq!(
                    fabric.read_word(privilege, DDR),
                    Some(0x1111_1111),
                    "{context}"
                );
                fabric.write_word(privilege, DDR, 0x0bad_c0de);
                assert_eq!(
                    fabric.read32(0, DDR),
                    0xdead_beef,
                    "{context}: CPU keeps its line"
                );
                assert_eq!(
                    fabric.shared.backing.read_bytes(Requester::Dma, DDR, 4),
                    0x0bad_c0deu32.to_le_bytes(),
                    "{context}"
                );
            }
        }
    }

    /// The coherence check names each of the three ways a program lets a master and the CPU
    /// disagree about memory nothing snoops, at the access that goes wrong, and stays silent for
    /// the cache operations SPRU862B Table 2-2 asks for.
    #[test]
    fn the_coherence_check_names_what_firmware_left_incoherent() {
        const DDR: u32 = 0xe000_1000;
        const L1D_LINE: u32 = 0x0184_4040; // L1DWBAR; L1DWWC, L1DIBAR, L1DIWC follow
        let privilege = Privilege::ID0_SUPERVISOR;
        let cached = || {
            let mut fabric = Fabric::new(&C6455);
            fabric.set_coherence_check(true);
            fabric.write32(0, MAR_BASE + 224 * 4, 1);
            fabric.write32(0, L1DCFG, 1);
            assert_eq!(fabric.read32(0, DDR), 0);
            fabric
        };

        // A buffer written into the cache and handed to a master unflushed.
        let mut fabric = cached();
        fabric.write32(0, DDR, 1);
        fabric.read_word(privilege, DDR);
        assert_eq!(
            fabric.take_coherence_hazards(),
            [(
                0,
                CoherenceHazard::UnflushedRead {
                    cache: DataCache::L1d,
                    line: DDR
                }
            )]
        );
        // Written back first, the same transfer is fine.
        fabric.write32(0, L1D_LINE, DDR);
        fabric.write32(0, L1D_LINE + 4, 1);
        fabric.read_word(privilege, DDR);
        assert_eq!(fabric.take_coherence_hazards(), []);

        // A master's write read back through the line cached before it.
        let mut fabric = cached();
        fabric.write_word(privilege, DDR + 8, 7);
        assert_eq!(fabric.take_coherence_hazards(), []);
        assert_eq!(
            fabric.read32(0, DDR + 4),
            0,
            "a byte the master did not write"
        );
        assert_eq!(fabric.take_coherence_hazards(), []);
        assert_eq!(fabric.read32(0, DDR + 8), 0);
        assert_eq!(
            fabric.take_coherence_hazards(),
            [(
                0,
                CoherenceHazard::StaleRead {
                    cache: DataCache::L1d,
                    address: DDR + 8
                }
            )]
        );
        // Invalidated after the transfer, the read is fresh and silent.
        fabric.write32(0, L1D_LINE + 8, DDR);
        fabric.write32(0, L1D_LINE + 12, 1);
        assert_eq!(fabric.read32(0, DDR + 8), 7);
        assert_eq!(fabric.take_coherence_hazards(), []);

        // A dirty line written back over what a master wrote after it.
        let mut fabric = cached();
        fabric.write32(0, DDR, 1);
        fabric.write_word(privilege, DDR + 8, 7);
        fabric.write32(0, L1D_LINE, DDR);
        fabric.write32(0, L1D_LINE + 4, 1);
        assert_eq!(
            fabric.take_coherence_hazards(),
            [(
                0,
                CoherenceHazard::Clobber {
                    cache: DataCache::L1d,
                    line: DDR
                }
            )]
        );
        assert_eq!(
            fabric.shared.backing.read_bytes(Requester::Dma, DDR + 8, 4),
            [0; 4],
            "the master's word was lost, which is what the report is about"
        );
    }

    /// L2 SRAM is snooped against L1D and nothing else is: a master writing the owner's L2 SRAM
    /// leaves its L1D coherent and its L1P stale, and another core caching the same SRAM through
    /// the global alias, or the shared L2 through its local window, is not kept coherent at all.
    #[test]
    fn the_coherence_check_follows_what_the_snoop_covers() {
        const L1PCFG: u32 = 0x0184_0020;
        const SRAM: u32 = 0x0080_2000;
        const SL2: u32 = 0x0020_1000;
        let privilege = Privilege::ID0_SUPERVISOR;
        let mut fabric = Fabric::new(&C6472);
        fabric.set_coherence_check(true);
        for core in 0..2 {
            fabric.write32(core, L1PCFG, 1);
            fabric.write32(core, L1DCFG, 1);
        }
        fabric.write32(1, MAR_BASE + 16 * 4, 1);
        let global = 0x1000_0000 + SRAM;
        assert_eq!(fabric.fetch32(0, SRAM), 0);
        assert_eq!(fabric.read32(0, SRAM), 0);
        assert_eq!(fabric.read32(1, global), 0);
        assert_eq!(fabric.read32(1, SL2), 0);

        fabric.write_word(privilege, global, 5);
        fabric.write_word(privilege, 0x1100_0000 + SL2, 6);
        assert_eq!(fabric.read32(0, SRAM), 5, "snooped into core 0's L1D");
        assert_eq!(fabric.fetch32(0, SRAM), 0);
        assert_eq!(fabric.read32(1, global), 0);
        assert_eq!(fabric.read32(1, SL2), 0);
        assert_eq!(
            fabric.take_coherence_hazards(),
            [
                (0, CoherenceHazard::StaleFetch { address: SRAM }),
                (
                    1,
                    CoherenceHazard::StaleRead {
                        cache: DataCache::L1d,
                        address: global
                    }
                ),
                (
                    1,
                    CoherenceHazard::StaleRead {
                        cache: DataCache::L1d,
                        address: SL2
                    }
                ),
            ]
        );
    }

    /// The shared L2 has a global alias in every core's block, and a core may cache it through
    /// any of them: a master writing through one alias leaves stale a line another alias tags.
    #[test]
    fn the_coherence_check_follows_every_alias_of_the_shared_l2() {
        const OWN_ALIAS: u32 = 0x1020_1000;
        const OTHER_ALIAS: u32 = 0x1120_1000;
        let mut fabric = Fabric::new(&C6472);
        fabric.set_coherence_check(true);
        fabric.write32(0, L1DCFG, 1);
        fabric.write32(0, MAR_BASE + 16 * 4, 1);
        assert_eq!(fabric.read32(0, OWN_ALIAS), 0);

        fabric.write_word(Privilege::ID0_SUPERVISOR, OTHER_ALIAS, 7);
        assert_eq!(fabric.read32(0, OWN_ALIAS), 0);
        assert_eq!(
            fabric.take_coherence_hazards(),
            [(
                0,
                CoherenceHazard::StaleRead {
                    cache: DataCache::L1d,
                    address: OWN_ALIAS
                }
            )]
        );
    }

    /// A line filled from a stale L2 line inherits which of its bytes are stale: the fill is not
    /// a CPU read and reports nothing, the CPU's read of an untouched byte reports nothing, and
    /// its read of the byte the master wrote does.
    #[test]
    fn an_l1d_fill_carries_the_l2_lines_stale_bytes_without_reporting_them() {
        const DDR: u32 = 0xe000_1000;
        let mut fabric = Fabric::new(&C6455);
        fabric.set_coherence_check(true);
        fabric.write32(0, MAR_BASE + 224 * 4, 1);
        fabric.write32(0, L2CFG, 1);
        assert_eq!(fabric.read32(0, DDR), 0, "the line is now in L2 alone");
        fabric.write32(0, L1DCFG, 1);

        fabric.write_word(Privilege::ID0_SUPERVISOR, DDR + 8, 7);
        assert_eq!(fabric.read32(0, DDR), 0);
        assert_eq!(fabric.take_coherence_hazards(), [], "the fill reported");
        assert_eq!(fabric.read32(0, DDR + 8), 0);
        assert_eq!(
            fabric.take_coherence_hazards(),
            [(
                0,
                CoherenceHazard::StaleRead {
                    cache: DataCache::L1d,
                    address: DDR + 8
                }
            )]
        );
    }

    /// Every stale line a cycle reads is reported, not only the first: an execute packet with
    /// two loads from two buffers a master has overwritten names both.
    #[test]
    fn two_stale_reads_between_collections_are_both_reported() {
        const FIRST: u32 = 0xe000_1000;
        const SECOND: u32 = 0xe000_2000;
        let mut fabric = Fabric::new(&C6455);
        fabric.set_coherence_check(true);
        fabric.write32(0, MAR_BASE + 224 * 4, 1);
        fabric.write32(0, L1DCFG, 1);
        assert_eq!(fabric.read32(0, FIRST), 0);
        assert_eq!(fabric.read32(0, SECOND), 0);
        fabric.write_word(Privilege::ID0_SUPERVISOR, FIRST, 1);
        fabric.write_word(Privilege::ID0_SUPERVISOR, SECOND, 2);

        assert_eq!(fabric.read32(0, FIRST), 0);
        assert_eq!(fabric.read32(0, FIRST + 4), 0, "the same line again");
        assert_eq!(fabric.read32(0, SECOND), 0);
        let stale = |address| {
            (
                0,
                CoherenceHazard::StaleRead {
                    cache: DataCache::L1d,
                    address,
                },
            )
        };
        assert_eq!(
            fabric.take_coherence_hazards(),
            [stale(FIRST), stale(SECOND)]
        );
    }

    /// Cache statistics count byte lookups. A line fill a frozen L2 sends to memory looks up
    /// every byte of the line and finds none of them, as the per-byte path it replaced counted.
    #[test]
    fn an_l1d_fill_past_a_frozen_l2_misses_on_every_byte() {
        const DDR: u32 = 0xe000_1000;
        let mut fabric = Fabric::new(&C6455);
        fabric.write32(0, MAR_BASE + 224 * 4, 1);
        fabric.write32(0, L2CFG, 1 | 1 << 3);
        fabric.write32(0, L1DCFG, 1);
        assert_eq!(fabric.read32(0, DDR), 0);
        let [_, l1d, l2] = fabric.megamodule(0).cache_views();
        assert_eq!(l1d.stats.line_fills, 1);
        assert_eq!(l2.stats.read_misses, 64);
        assert_eq!(l2.stats.read_hits, 0);
        assert_eq!(l2.stats.line_fills, 0);
    }

    #[test]
    fn megamodule_registers_read_zero_for_a_master_and_ignore_its_writes() {
        // SPRU871K section 5.3.1: they are only accessible by the CPU. Beside them, the
        // configuration space belongs to whatever device memory the address map puts there.
        const MAR224: u32 = MAR_BASE + 224 * 4;
        for profile in [&C6455, &C6472] {
            for (name, privilege) in masters(profile) {
                let context = format!("{} {name}", profile.name);
                let mut fabric = Fabric::new(profile);
                fabric.write32(0, MAR224, 1);
                assert_eq!(fabric.read32(0, MAR224), 1);
                assert_eq!(fabric.read_word(privilege, MAR224), Some(0), "{context}");
                assert_eq!(
                    fabric.read(privilege, MAR224, 4),
                    Some(vec![0; 4]),
                    "{context}"
                );
                fabric.write_word(privilege, MAR224, 0);
                fabric.write(privilege, MAR224, &[0; 4]);
                assert_eq!(
                    fabric.read32(0, MAR224),
                    1,
                    "{context}: the write went nowhere"
                );
                // The last word of the megamodule's window is still its own.
                fabric.write_word(privilege, 0x01bf_fffc, 0xffff_ffff);
                assert_eq!(
                    fabric.read_word(privilege, 0x01bf_fffc),
                    Some(0),
                    "{context}"
                );
            }
        }
    }

    #[test]
    fn a_peripheral_register_answers_through_its_slave_port_for_the_identity_asking() {
        for profile in [&C6455, &C6472] {
            let cpu = owner(profile);
            for (name, privilege) in masters(profile) {
                let context = format!("{} {name}", profile.name);
                let mut fabric = Fabric::new(profile);
                // Whoever last touched the bus - a user-mode core - is not the master.
                assert!(fabric.check_access(cpu, ESR, Permission::Write, true));
                fabric.write_word(privilege, PARAM + 4, 0x1234_5678);
                // The port took the word, and the set it wrote carries the master's own
                // PRIV and PRIVID (SPRU966C section 2.10.2), not the CPU's.
                assert_eq!(fabric.read32(cpu, PARAM + 4), 0x1234_5678, "{context}");
                assert_eq!(
                    fabric.read32(cpu, PARAM) >> 24,
                    (u32::from(!privilege.user) << 7) | u32::from(privilege.id),
                    "{context}"
                );
                assert_eq!(fabric.read_word(privilege, PARAM + 4), Some(0x1234_5678));
                // A byte of it is a whole register word read, patched and written back.
                fabric.write(privilege, PARAM + 5, &[0xab]);
                assert_eq!(fabric.read32(cpu, PARAM + 4), 0x1234_ab78, "{context}");
                assert_eq!(
                    fabric.read(privilege, PARAM + 4, 4),
                    Some(vec![0x78, 0xab, 0x34, 0x12]),
                    "{context}"
                );
            }
        }
    }

    #[test]
    fn a_word_is_its_four_bytes_in_the_device_byte_order() {
        for profile in [&C6455, &C6472] {
            let core = owner(profile);
            let at = global(profile, core, L2_SRAM + 0x4000);
            for big in [false, true] {
                let order = if big { Endian::Big } else { Endian::Little };
                let ordered = |bytes: [u8; 4]| {
                    if big {
                        bytes.into_iter().rev().collect()
                    } else {
                        bytes.to_vec()
                    }
                };
                let master = Privilege::ID0_SUPERVISOR;
                let context = format!("{} big {big}", profile.name);
                let mut fabric = Fabric::new(profile);
                fabric.set_endian(order);
                assert!(fabric.byte_order() == order);

                fabric.write_word(master, at, 0x1122_3344);
                assert_eq!(
                    fabric.shared.backing.read_bytes(Requester::Dma, at, 4),
                    ordered([0x44, 0x33, 0x22, 0x11]),
                    "{context}"
                );
                fabric.write(master, at + 8, &[1, 2, 3, 4]);
                assert_eq!(
                    fabric.read_word(master, at + 8),
                    Some(if big { 0x0102_0304 } else { 0x0403_0201 }),
                    "{context}"
                );

                // An element narrower than a word is its low bytes, right-justified, and reads
                // back zero-extended (SPRUEG2E Table 32).
                for (width, stored) in [(1, 0x78u32), (2, 0x5678), (3, 0x34_5678)] {
                    let address = at + 0x20 + 0x10 * width as u32;
                    fabric.write_element(master, address, 0x1234_5678, width);
                    let bytes = fabric.shared.backing.read_bytes(Requester::Dma, address, 4);
                    let expected: Vec<u8> = if big {
                        stored.to_be_bytes()[4 - width..].to_vec()
                    } else {
                        stored.to_le_bytes()[..width].to_vec()
                    };
                    assert_eq!(bytes[..width], expected, "{context} width {width}");
                    assert_eq!(
                        bytes[width..],
                        [0; 4][width..],
                        "{context}: nothing past it"
                    );
                    assert_eq!(fabric.read_element(master, address, width), Some(stored));
                }

                // A register is assembled in the same order: byte 1 is bits 15:8 on a
                // little-endian device and 23:16 on a big-endian one.
                fabric.write_word(master, PARAM + 4, 0);
                fabric.write(master, PARAM + 5, &[0xab]);
                assert_eq!(
                    fabric.read_word(master, PARAM + 4),
                    Some(if big { 0x00ab_0000 } else { 0x0000_ab00 }),
                    "{context}"
                );
            }
        }
    }

    #[test]
    fn the_sparse_test_memory_is_a_little_endian_master_target() {
        let mut memory = BTreeMap::new();
        let master = Privilege::ID0_USER;
        memory.write_word(master, 0x100, 0x0403_0201);
        assert_eq!(memory.read(master, 0x100, 4), Some(vec![1, 2, 3, 4]));
        assert_eq!(memory.read_element(master, 0x101, 2), Some(0x0302));
        assert_eq!(memory.read_or_zero(master, 0x200, 2), [0; 2]);
    }

    #[test]
    fn a_lent_out_master_reaches_its_own_aperture_as_memory() {
        // An EMAC's descriptor RAM sits in its own aperture. While the EMAC is lent out for
        // its tick nothing answers for the aperture, so the access finds the memory behind it:
        // outside memory protection, as every register access is, and the same bytes the CPU
        // sees once the EMAC is back.
        for profile in [&C6455, &C6472] {
            let descriptor = profile.emac_bases[0] + 0x2000;
            let mut fabric = Fabric::new(profile);
            fabric.write32(0, descriptor, 0x1111_1111);
            fabric.lend_emac(0, |_emac, fabric| {
                let emac = Privilege::ID0_USER;
                assert_eq!(fabric.read_word(emac, descriptor), Some(0x1111_1111));
                fabric.write_word(emac, descriptor, 0xcafe_f00d);
                assert_eq!(fabric.read_word(emac, descriptor), Some(0xcafe_f00d));
            });
            assert_eq!(
                fabric.read32(0, descriptor),
                0xcafe_f00d,
                "{}",
                profile.name
            );
        }
    }
}
