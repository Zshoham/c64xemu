//! The boot tables the on-chip ROM bootloader processes.
//!
//! SPRUEC6G sections 6.2.2 and 6.2.3. A boot table is a 32-bit entry point followed by sections,
//! each a 32-bit byte count and a 32-bit destination address with the bytes to copy, terminated by
//! a zero count; the bootloader copies every section and branches to the entry point. A boot
//! configuration table is a list of three-word entries - address, set mask, clear mask - and the
//! bootloader reads the address, sets the bits in the set mask, clears the bits in the clear mask
//! and toggles the bits that appear in both. An entry whose two masks are zero names a function
//! the bootloader calls with the return address in B3, so this layer reports those addresses to
//! its caller rather than branching itself. The table ends when all three words are zero.
//!
//! The same table also arrives over a link rather than sitting in memory - that is what the EMAC,
//! SRIO and UTOPIA boot modes receive - so [`BootStream`] processes the identical records as the
//! bytes turn up, in fragments no record is guaranteed to fit inside.
//!
//! Words are read in the device's byte order, which is what the hex conversion utility's `-order`
//! option matches to the build.

use crate::system::Endian;

/// A section count large enough for any real image; a malformed table hits it instead of looping.
const MAX_SECTIONS: usize = 4096;
const MAX_ENTRIES: usize = 4096;

/// What the bootloader needs from the machine it is loading.
pub trait BootMemory {
    fn boot_read_word(&mut self, address: u32) -> u32;
    fn boot_read_bytes(&mut self, address: u32, length: usize) -> Vec<u8>;
    fn boot_write_word(&mut self, address: u32, value: u32);
    fn boot_write_bytes(&mut self, address: u32, bytes: &[u8]);
}

/// Copy every section of the boot table at `address` and return the entry point it names.
pub fn load_boot_table(memory: &mut impl BootMemory, address: u32) -> Result<u32, String> {
    let entry = memory.boot_read_word(address);
    let mut cursor = address.wrapping_add(4);
    for _ in 0..MAX_SECTIONS {
        let count = memory.boot_read_word(cursor);
        if count == 0 {
            return Ok(entry);
        }
        let destination = memory.boot_read_word(cursor.wrapping_add(4));
        let data = memory.boot_read_bytes(cursor.wrapping_add(8), count as usize);
        memory.boot_write_bytes(destination, &data);
        cursor = cursor.wrapping_add(8).wrapping_add(count);
    }
    Err("boot table has no termination record".into())
}

/// Apply every read/modify/write entry of the boot configuration table at `address` and return
/// the function addresses it asks the bootloader to call, in order.
pub fn apply_boot_configuration(
    memory: &mut impl BootMemory,
    address: u32,
) -> Result<Vec<u32>, String> {
    let mut calls = Vec::new();
    let mut cursor = address;
    for _ in 0..MAX_ENTRIES {
        let target = memory.boot_read_word(cursor);
        let set = memory.boot_read_word(cursor.wrapping_add(4));
        let clear = memory.boot_read_word(cursor.wrapping_add(8));
        if target == 0 && set == 0 && clear == 0 {
            return Ok(calls);
        }
        if set == 0 && clear == 0 {
            calls.push(target);
        } else {
            let both = set & clear;
            let value = memory.boot_read_word(target);
            let value = (value | (set & !both)) & !(clear & !both);
            memory.boot_write_word(target, value ^ both);
        }
        cursor = cursor.wrapping_add(12);
    }
    Err("boot configuration table has no termination record".into())
}

/// The boot table processed as it arrives, a fragment at a time (SPRUEC6G section 4.3.5.3: the
/// EMAC boot's frame payloads are handed to "the boot-table processing function" one frame at a
/// time). The records are the memory-resident table's - entry point, then count, destination and
/// data per section, terminated by a zero count - but a fragment boundary can fall anywhere,
/// including inside a header word, so where the stream stands inside a record is state rather
/// than a cursor over memory. Section data is written through as it arrives and never buffered.
pub struct BootStream {
    endian: Endian,
    state: State,
    /// A header word being assembled across a fragment boundary, and how much of it has arrived.
    partial: [u8; 4],
    held: usize,
    entry: u32,
    sections: usize,
}

enum State {
    Entry,
    Count,
    Destination { count: u32 },
    Data { address: u32, remaining: u32 },
    Done,
}

impl BootStream {
    pub fn new(endian: Endian) -> Self {
        Self {
            endian,
            state: State::Entry,
            partial: [0; 4],
            held: 0,
            entry: 0,
            sections: 0,
        }
    }

    /// The entry point the table named, once its terminating zero count has arrived.
    pub fn entry(&self) -> Option<u32> {
        matches!(self.state, State::Done).then_some(self.entry)
    }

    /// Process `bytes` and report whether the table has ended. Bytes that follow the terminator
    /// are ignored, which is what lets a sender pad its last frame to a length the link needs.
    pub fn feed(&mut self, bytes: &[u8], memory: &mut impl BootMemory) -> Result<bool, String> {
        let mut rest = bytes;
        while !rest.is_empty() && !matches!(self.state, State::Done) {
            if let State::Data { address, remaining } = self.state {
                let take = (remaining as usize).min(rest.len());
                memory.boot_write_bytes(address, &rest[..take]);
                rest = &rest[take..];
                let left = remaining - take as u32;
                self.state = if left == 0 {
                    State::Count
                } else {
                    State::Data {
                        address: address.wrapping_add(take as u32),
                        remaining: left,
                    }
                };
                continue;
            }
            let Some(word) = self.take_word(&mut rest) else {
                break;
            };
            self.state = match self.state {
                State::Entry => {
                    self.entry = word;
                    State::Count
                }
                State::Count => {
                    if word == 0 {
                        State::Done
                    } else {
                        self.sections += 1;
                        if self.sections > MAX_SECTIONS {
                            return Err(format!(
                                "boot table has more than {MAX_SECTIONS} sections and no \
                                 termination record"
                            ));
                        }
                        State::Destination { count: word }
                    }
                }
                State::Destination { count } => State::Data {
                    address: word,
                    remaining: count,
                },
                State::Data { .. } | State::Done => unreachable!("handled above"),
            };
            if matches!(self.state, State::Done) {
                return Ok(true);
            }
        }
        Ok(matches!(self.state, State::Done))
    }

    /// The next header word, or nothing when the fragment ended part-way through one.
    fn take_word(&mut self, rest: &mut &[u8]) -> Option<u32> {
        let take = (4 - self.held).min(rest.len());
        self.partial[self.held..self.held + take].copy_from_slice(&rest[..take]);
        self.held += take;
        *rest = &rest[take..];
        if self.held < 4 {
            return None;
        }
        self.held = 0;
        Some(match self.endian {
            Endian::Little => u32::from_le_bytes(self.partial),
            Endian::Big => u32::from_be_bytes(self.partial),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    #[derive(Default)]
    struct Memory {
        bytes: BTreeMap<u32, u8>,
    }

    impl Memory {
        fn word(&mut self, address: u32, value: u32) {
            self.boot_write_word(address, value);
        }

        fn bytes_at(&self, address: u32, length: usize) -> Vec<u8> {
            (0..length)
                .map(|offset| {
                    *self
                        .bytes
                        .get(&address.wrapping_add(offset as u32))
                        .unwrap_or(&0)
                })
                .collect()
        }
    }

    impl BootMemory for Memory {
        fn boot_read_word(&mut self, address: u32) -> u32 {
            u32::from_le_bytes(self.bytes_at(address, 4).try_into().unwrap())
        }

        fn boot_read_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
            self.bytes_at(address, length)
        }

        fn boot_write_word(&mut self, address: u32, value: u32) {
            self.boot_write_bytes(address, &value.to_le_bytes());
        }

        fn boot_write_bytes(&mut self, address: u32, bytes: &[u8]) {
            for (offset, byte) in bytes.iter().enumerate() {
                self.bytes
                    .insert(address.wrapping_add(offset as u32), *byte);
            }
        }
    }

    #[test]
    fn a_boot_table_copies_every_section_and_names_the_entry_point() {
        // SPRUEC6G section 6.2.2.1: entry point, then count/address/data per section, then a
        // zero count.
        let mut memory = Memory::default();
        memory.word(0x1000, 0x0080_0100);
        memory.word(0x1004, 8);
        memory.word(0x1008, 0x0080_0000);
        memory.word(0x100c, 0x1122_3344);
        memory.word(0x1010, 0x5566_7788);
        memory.word(0x1014, 4);
        memory.word(0x1018, 0x0090_0000);
        memory.word(0x101c, 0xdead_beef);
        memory.word(0x1020, 0);
        assert_eq!(load_boot_table(&mut memory, 0x1000), Ok(0x0080_0100));
        assert_eq!(memory.boot_read_word(0x0080_0000), 0x1122_3344);
        assert_eq!(memory.boot_read_word(0x0080_0004), 0x5566_7788);
        assert_eq!(memory.boot_read_word(0x0090_0000), 0xdead_beef);
        // The section data itself is untouched at its source.
        assert_eq!(memory.boot_read_word(0x100c), 0x1122_3344);
    }

    #[test]
    fn a_section_count_that_is_not_a_whole_word_still_advances_exactly() {
        let mut memory = Memory::default();
        memory.word(0x2000, 0x0080_0200);
        memory.word(0x2004, 2);
        memory.word(0x2008, 0x0080_0000);
        memory.word(0x200c, 0x0000_a55a);
        memory.word(0x200e, 0);
        assert_eq!(load_boot_table(&mut memory, 0x2000), Ok(0x0080_0200));
        assert_eq!(memory.bytes_at(0x0080_0000, 2), vec![0x5a, 0xa5]);
    }

    #[test]
    fn a_configuration_table_sets_clears_toggles_and_reports_calls() {
        // SPRUEC6G Table 44: the three documented operations plus the termination record.
        let mut memory = Memory::default();
        memory.word(0x0093_001c, 0x0000_ffff);
        memory.word(0x0093_0010, 0x0000_0000);
        memory.word(0x3000, 0x0093_001c);
        memory.word(0x3004, 0xffff_0000);
        memory.word(0x3008, 0x0000_ffff);
        memory.word(0x300c, 0x0093_0010);
        memory.word(0x3010, 0x0101_0101);
        memory.word(0x3014, 0x0101_0101);
        memory.word(0x3018, 0x0093_0018);
        memory.word(0x301c, 0);
        memory.word(0x3020, 0);
        memory.word(0x3024, 0);
        memory.word(0x3028, 0);
        memory.word(0x302c, 0);
        assert_eq!(
            apply_boot_configuration(&mut memory, 0x3000),
            Ok(vec![0x0093_0018])
        );
        assert_eq!(memory.boot_read_word(0x0093_001c), 0xffff_0000);
        assert_eq!(memory.boot_read_word(0x0093_0010), 0x0101_0101);
    }

    #[test]
    fn an_unterminated_table_is_an_error_rather_than_a_loop() {
        let mut memory = Memory::default();
        for index in 0..(MAX_SECTIONS as u32 + 2) {
            memory.word(0x4004 + index * 12, 4);
            memory.word(0x4008 + index * 12, 0x0080_0000);
            memory.word(0x400c + index * 12, 1);
        }
        assert!(load_boot_table(&mut memory, 0x4000).is_err());
    }

    /// The words of a table, in the byte order a device would read them.
    fn table_bytes(words: &[u32], endian: Endian) -> Vec<u8> {
        words
            .iter()
            .flat_map(|word| match endian {
                Endian::Little => word.to_le_bytes(),
                Endian::Big => word.to_be_bytes(),
            })
            .collect()
    }

    #[test]
    fn a_streamed_table_lands_the_same_bytes_as_one_in_memory() {
        // The table of the first test, delivered in six-byte fragments so that every header
        // word is split across two of them.
        let words = [
            0x0080_0100,
            8,
            0x0080_0000,
            0x1122_3344,
            0x5566_7788,
            4,
            0x0090_0000,
            0xdead_beef,
            0,
        ];
        let bytes = table_bytes(&words, Endian::Little);
        let mut memory = Memory::default();
        let mut stream = BootStream::new(Endian::Little);
        let mut finished = false;
        for fragment in bytes.chunks(6) {
            finished = stream.feed(fragment, &mut memory).unwrap();
        }
        assert!(finished);
        assert_eq!(stream.entry(), Some(0x0080_0100));
        assert_eq!(memory.boot_read_word(0x0080_0000), 0x1122_3344);
        assert_eq!(memory.boot_read_word(0x0080_0004), 0x5566_7788);
        assert_eq!(memory.boot_read_word(0x0090_0000), 0xdead_beef);
    }

    #[test]
    fn a_stream_ends_at_the_terminator_and_ignores_the_padding_after_it() {
        // A link delivers whole frames, so the last fragment of a table is padded; nothing in
        // it may be taken for another section.
        let bytes = table_bytes(
            &[0x0080_0200, 4, 0x0080_0000, 0xa5a5_5a5a, 0],
            Endian::Little,
        );
        let mut padded = bytes.clone();
        padded.extend_from_slice(&table_bytes(&[4, 0x0080_0010, 0xffff_ffff], Endian::Little));
        let mut memory = Memory::default();
        let mut stream = BootStream::new(Endian::Little);
        assert_eq!(stream.feed(&padded, &mut memory), Ok(true));
        assert_eq!(stream.entry(), Some(0x0080_0200));
        assert_eq!(memory.boot_read_word(0x0080_0000), 0xa5a5_5a5a);
        assert_eq!(
            memory.boot_read_word(0x0080_0010),
            0,
            "the padding is not a section"
        );
        // Anything fed afterwards is padding too.
        assert_eq!(stream.feed(&[0xff; 16], &mut memory), Ok(true));
        assert_eq!(memory.boot_read_word(0x0080_0010), 0);
    }

    #[test]
    fn a_stream_reads_its_header_words_in_the_devices_byte_order() {
        let bytes = table_bytes(&[0x0080_0300, 4, 0x0080_0000, 0x1122_3344, 0], Endian::Big);
        let mut memory = Memory::default();
        let mut stream = BootStream::new(Endian::Big);
        assert_eq!(stream.feed(&bytes, &mut memory), Ok(true));
        assert_eq!(stream.entry(), Some(0x0080_0300));
        // The section data is a byte stream whichever way the words are read.
        assert_eq!(
            memory.bytes_at(0x0080_0000, 4),
            vec![0x11, 0x22, 0x33, 0x44]
        );
    }

    #[test]
    fn a_stream_that_never_terminates_is_an_error_rather_than_a_loop() {
        let mut memory = Memory::default();
        let mut stream = BootStream::new(Endian::Little);
        let entry = table_bytes(&[0x0080_0000], Endian::Little);
        assert_eq!(stream.feed(&entry, &mut memory), Ok(false));
        let section = table_bytes(&[4, 0x0080_0000, 1], Endian::Little);
        let mut result = Ok(false);
        for _ in 0..=MAX_SECTIONS {
            result = stream.feed(&section, &mut memory);
        }
        assert!(
            result.is_err(),
            "a stream of sections that never ends is refused"
        );
    }
}
