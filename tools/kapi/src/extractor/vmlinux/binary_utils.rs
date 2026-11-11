// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

// Array-bound constants matching `include/linux/kernel_api_spec.h`.
// String fields are `const char *`; call `DataReader::ptr_size()` for
// the per-target pointer width.
pub mod sizes {
    pub const MAX_PARAMS: usize = 16;
    pub const MAX_ERRORS: usize = 32;
    pub const MAX_CONSTRAINTS: usize = 32;
    pub const MAX_LOCKS: usize = 16;
    pub const MAX_CAPABILITIES: usize = 8;
    pub const MAX_SIGNALS: usize = 32;
    pub const MAX_STRUCT_SPECS: usize = 8;
    pub const MAX_SIDE_EFFECTS: usize = 32;
    pub const MAX_STATE_TRANS: usize = 8;

    pub const NAME: usize = 0;
    pub const DESC: usize = 0;
}

// Section markers of `struct kernel_api_spec`. The kernel only sets a
// marker when the matching *_COUNT()/KAPI_EXAMPLES() macro is used, so
// a zero marker is as valid as the expected one.
pub mod magic {
    pub const PARAMS: u32 = 0x4B415031; // 'KAP1'
    pub const RETURN: u32 = 0x4B415232; // 'KAR2'
    pub const ERRORS: u32 = 0x4B414533; // 'KAE3'
    pub const LOCKS: u32 = 0x4B414C34; // 'KAL4'
    pub const CONSTRAINTS: u32 = 0x4B414335; // 'KAC5'
    pub const INFO: u32 = 0x4B414936; // 'KAI6'
    pub const SIGNALS: u32 = 0x4B415337; // 'KAS7'
    pub const SIGMASK: u32 = 0x4B414D38; // 'KAM8'
    pub const STRUCTS: u32 = 0x4B415439; // 'KAT9'
    pub const EFFECTS: u32 = 0x4B414641; // 'KAFA'
    pub const TRANS: u32 = 0x4B415442; // 'KATB'
    pub const CAPS: u32 = 0x4B414343; // 'KACC'
}

/// Resolve a virtual-address string pointer against the vmlinux ELF
/// and return the NUL-terminated C string it points at.
pub fn resolve_vaddr_string(elf: &goblin::elf::Elf, data: &[u8], vaddr: u64) -> Option<String> {
    if vaddr == 0 {
        return None;
    }
    for sh in &elf.section_headers {
        let start = sh.sh_addr;
        let end = start.checked_add(sh.sh_size)?;
        if vaddr < start || vaddr >= end {
            continue;
        }
        // File-backed sections only (skip SHT_NOBITS etc.)
        if sh.sh_type == goblin::elf::section_header::SHT_NOBITS {
            return None;
        }
        let rel = (vaddr - start) as usize;
        let file_start = sh.sh_offset as usize + rel;
        if file_start >= data.len() {
            return None;
        }
        let tail = &data[file_start..];
        let nul = tail.iter().position(|&b| b == 0)?;
        return std::str::from_utf8(&tail[..nul]).ok().map(str::to_string);
    }
    None
}

/// Endianness of the target ELF binary
#[derive(Clone, Copy, PartialEq)]
pub enum Endian {
    Little,
    Big,
}

/// Resolves string pointers read from `.kapi_specs` back to their
/// underlying C strings in the vmlinux rodata.
pub struct StringResolver<'a> {
    pub elf: &'a goblin::elf::Elf<'a>,
    pub vmlinux: &'a [u8],
}

// Helper for reading data at specific offsets
pub struct DataReader<'a> {
    pub data: &'a [u8],
    pub pos: usize,
    pub endian: Endian,
    /// true for 64-bit ELF, false for 32-bit
    pub is_64bit: bool,
    /// Used to follow `const char *` fields into rodata.
    pub resolver: Option<StringResolver<'a>>,
}

impl<'a> DataReader<'a> {
    pub fn new(data: &'a [u8], offset: usize, endian: Endian, is_64bit: bool) -> Self {
        Self {
            data,
            pos: offset,
            endian,
            is_64bit,
            resolver: None,
        }
    }

    pub fn with_resolver(mut self, resolver: StringResolver<'a>) -> Self {
        self.resolver = Some(resolver);
        self
    }

    /// Pointer width of the target in bytes (4 or 8).
    pub fn ptr_size(&self) -> usize {
        if self.is_64bit {
            8
        } else {
            4
        }
    }

    /// Advance the read position to the next multiple of `align`.
    /// Needed before every naturally-aligned field when the containing
    /// struct is not `__packed`.
    pub fn align_to(&mut self, align: usize) {
        if align > 1 {
            let rem = self.pos % align;
            if rem != 0 {
                self.pos = (self.pos + (align - rem)).min(self.data.len());
            }
        }
    }

    /// Read a target-sized pointer slot. Returns the virtual address
    /// stored in the slot, or `None` if there isn't enough data. The
    /// caller is expected to align the reader first if the containing
    /// struct demands natural alignment.
    pub fn read_ptr(&mut self) -> Option<u64> {
        self.align_to(self.ptr_size());
        if self.is_64bit {
            self.read_u64()
        } else {
            self.read_u32().map(|v| v as u64)
        }
    }

    pub fn read_bytes(&mut self, len: usize) -> Option<&'a [u8]> {
        if self.pos + len <= self.data.len() {
            let bytes = &self.data[self.pos..self.pos + len];
            self.pos += len;
            Some(bytes)
        } else {
            None
        }
    }

    pub fn read_u32(&mut self) -> Option<u32> {
        self.align_to(4);
        let b: [u8; 4] = self.read_bytes(4)?.try_into().unwrap();
        Some(match self.endian {
            Endian::Little => u32::from_le_bytes(b),
            Endian::Big => u32::from_be_bytes(b),
        })
    }

    pub fn read_u8(&mut self) -> Option<u8> {
        self.read_bytes(1).map(|b| b[0])
    }

    pub fn read_i32(&mut self) -> Option<i32> {
        self.align_to(4);
        let b: [u8; 4] = self.read_bytes(4)?.try_into().unwrap();
        Some(match self.endian {
            Endian::Little => i32::from_le_bytes(b),
            Endian::Big => i32::from_be_bytes(b),
        })
    }

    pub fn read_u64(&mut self) -> Option<u64> {
        self.align_to(8);
        let b: [u8; 8] = self.read_bytes(8)?.try_into().unwrap();
        Some(match self.endian {
            Endian::Little => u64::from_le_bytes(b),
            Endian::Big => u64::from_be_bytes(b),
        })
    }

    pub fn read_i64(&mut self) -> Option<i64> {
        self.align_to(8);
        let b: [u8; 8] = self.read_bytes(8)?.try_into().unwrap();
        Some(match self.endian {
            Endian::Little => i64::from_le_bytes(b),
            Endian::Big => i64::from_be_bytes(b),
        })
    }

    /// Read a target-sized unsigned value (4 bytes for 32-bit, 8 bytes for 64-bit).
    pub fn read_usize(&mut self) -> Option<usize> {
        self.align_to(self.ptr_size());
        if self.is_64bit {
            // No double-align: read_u64 would re-align, but we just
            // did that with ptr_size() which is 8 on 64-bit.
            let b: [u8; 8] = self.read_bytes(8)?.try_into().unwrap();
            Some(match self.endian {
                Endian::Little => u64::from_le_bytes(b) as usize,
                Endian::Big => u64::from_be_bytes(b) as usize,
            })
        } else {
            let b: [u8; 4] = self.read_bytes(4)?.try_into().unwrap();
            Some(match self.endian {
                Endian::Little => u32::from_le_bytes(b) as usize,
                Endian::Big => u32::from_be_bytes(b) as usize,
            })
        }
    }

    // Helper methods for common patterns
    pub fn read_bool(&mut self) -> Option<bool> {
        self.read_u8().map(|v| v != 0)
    }

    /// Read a `const char *` slot using the target pointer width
    /// (4 bytes on 32-bit, 8 bytes on 64-bit) and, if a resolver is
    /// attached, follow the address into the vmlinux to recover the
    /// C string. The `_max_len` argument is ignored.
    pub fn read_optional_string(&mut self, _max_len: usize) -> Option<String> {
        let vaddr = self.read_ptr()?;
        let resolver = self.resolver.as_ref()?;
        resolve_vaddr_string(resolver.elf, resolver.vmlinux, vaddr).filter(|s| !s.is_empty())
    }

    pub fn read_string_or_default(&mut self, max_len: usize) -> String {
        self.read_optional_string(max_len).unwrap_or_default()
    }

    /// Follow a `const s64 *` slot (already read with `read_ptr`) to the
    /// `count` values it points at. A NULL or unresolvable pointer yields
    /// an empty list.
    pub fn resolve_s64_array(&self, vaddr: u64, count: u32) -> Vec<i64> {
        let Some(resolver) = self.resolver.as_ref() else {
            return Vec::new();
        };
        if vaddr == 0 {
            return Vec::new();
        }
        let Some(start) = super::vaddr_to_file_offset(resolver.elf, vaddr) else {
            return Vec::new();
        };
        let raw = (count as usize)
            .checked_mul(8)
            .and_then(|len| start.checked_add(len))
            .and_then(|end| resolver.vmlinux.get(start..end));
        raw.map_or_else(Vec::new, |raw| {
            raw.chunks_exact(8)
                .map(|chunk| {
                    let b: [u8; 8] = chunk.try_into().unwrap();
                    match self.endian {
                        Endian::Little => i64::from_le_bytes(b),
                        Endian::Big => i64::from_be_bytes(b),
                    }
                })
                .collect()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- DataReader little-endian tests ----

    #[test]
    fn read_u32_little_endian() {
        let data = [0x78, 0x56, 0x34, 0x12];
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_u32(), Some(0x12345678));
    }

    #[test]
    fn read_u32_big_endian() {
        let data = [0x12, 0x34, 0x56, 0x78];
        let mut reader = DataReader::new(&data, 0, Endian::Big, true);
        assert_eq!(reader.read_u32(), Some(0x12345678));
    }

    #[test]
    fn read_u64_little_endian() {
        let data = 0xDEADBEEFCAFEBABEu64.to_le_bytes();
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_u64(), Some(0xDEADBEEFCAFEBABE));
    }

    #[test]
    fn read_u64_big_endian() {
        let data = 0xDEADBEEFCAFEBABEu64.to_be_bytes();
        let mut reader = DataReader::new(&data, 0, Endian::Big, true);
        assert_eq!(reader.read_u64(), Some(0xDEADBEEFCAFEBABE));
    }

    #[test]
    fn read_i32_little_endian_negative() {
        let val: i32 = -42;
        let data = val.to_le_bytes();
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_i32(), Some(-42));
    }

    #[test]
    fn read_i32_big_endian_negative() {
        let val: i32 = -1;
        let data = val.to_be_bytes();
        let mut reader = DataReader::new(&data, 0, Endian::Big, true);
        assert_eq!(reader.read_i32(), Some(-1));
    }

    #[test]
    fn read_i64_little_endian() {
        let val: i64 = -9999999999;
        let data = val.to_le_bytes();
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_i64(), Some(-9999999999));
    }

    #[test]
    fn read_i64_big_endian() {
        let val: i64 = i64::MIN;
        let data = val.to_be_bytes();
        let mut reader = DataReader::new(&data, 0, Endian::Big, true);
        assert_eq!(reader.read_i64(), Some(i64::MIN));
    }

    // ---- read_usize tests ----

    #[test]
    fn read_usize_64bit() {
        let val: u64 = 0x00000000FFFFFFFF;
        let data = val.to_le_bytes();
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_usize(), Some(0xFFFFFFFF));
    }

    #[test]
    fn read_usize_32bit() {
        let val: u32 = 0xABCD1234;
        let data = val.to_le_bytes();
        let mut reader = DataReader::new(&data, 0, Endian::Little, false);
        assert_eq!(reader.read_usize(), Some(0xABCD1234));
    }

    #[test]
    fn read_usize_32bit_does_not_consume_8_bytes() {
        // In 32-bit mode, read_usize should only consume 4 bytes
        let mut data = [0u8; 8];
        data[..4].copy_from_slice(&42u32.to_le_bytes());
        data[4..8].copy_from_slice(&99u32.to_le_bytes());
        let mut reader = DataReader::new(&data, 0, Endian::Little, false);
        assert_eq!(reader.read_usize(), Some(42));
        // After reading 4 bytes, pos should be at 4
        assert_eq!(reader.pos, 4);
        assert_eq!(reader.read_usize(), Some(99));
    }

    // ---- Bounds checking ----

    #[test]
    fn read_u32_past_end_returns_none() {
        let data = [0x01, 0x02, 0x03]; // only 3 bytes, need 4
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_u32(), None);
    }

    #[test]
    fn read_u64_past_end_returns_none() {
        let data = [0u8; 7]; // only 7 bytes, need 8
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_u64(), None);
    }

    #[test]
    fn read_bytes_past_end_returns_none() {
        let data = [0u8; 4];
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_bytes(5), None);
    }

    #[test]
    fn read_at_offset() {
        // read_u32 auto-aligns to a 4-byte boundary, so the starting
        // offset must itself be 4-aligned for the value to be read
        // from its declared position.
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0x78, 0x56, 0x34, 0x12];
        let mut reader = DataReader::new(&data, 4, Endian::Little, true);
        assert_eq!(reader.read_u32(), Some(0x12345678));
    }

    #[test]
    fn read_u32_auto_aligns() {
        // Starting mid-word, read_u32 snaps to the next 4-byte boundary.
        let data = [0xDE, 0xAD, 0xBE, 0xEF, 0x78, 0x56, 0x34, 0x12];
        let mut reader = DataReader::new(&data, 1, Endian::Little, true);
        assert_eq!(reader.read_u32(), Some(0x12345678));
        assert_eq!(reader.pos, 8);
    }

    #[test]
    fn read_ptr_32bit_uses_4_bytes() {
        let data = [0x78, 0x56, 0x34, 0x12];
        let mut reader = DataReader::new(&data, 0, Endian::Little, false);
        assert_eq!(reader.read_ptr(), Some(0x12345678));
        assert_eq!(reader.pos, 4);
    }

    #[test]
    fn read_ptr_64bit_uses_8_bytes() {
        let data = [0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00];
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_ptr(), Some(0x12345678));
        assert_eq!(reader.pos, 8);
    }

    #[test]
    fn read_bool_values() {
        let data = [0, 1, 255];
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_bool(), Some(false));
        assert_eq!(reader.read_bool(), Some(true));
        assert_eq!(reader.read_bool(), Some(true)); // any non-zero is true
    }

    #[test]
    fn sequential_reads_advance_position() {
        let mut data = [0u8; 12];
        data[..4].copy_from_slice(&1u32.to_le_bytes());
        data[4..8].copy_from_slice(&2u32.to_le_bytes());
        data[8..12].copy_from_slice(&3u32.to_le_bytes());
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_u32(), Some(1));
        assert_eq!(reader.read_u32(), Some(2));
        assert_eq!(reader.read_u32(), Some(3));
        assert_eq!(reader.pos, 12);
    }

    #[test]
    fn read_optional_string_empty_returns_none() {
        // A string buffer that is just NUL
        let data = [0u8; 10];
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_optional_string(10), None);
    }

    #[test]
    fn read_string_or_default_with_empty() {
        let data = [0u8; 10];
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_string_or_default(10), "");
    }

    #[test]
    fn read_u8_value() {
        let data = [0x42];
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);
        assert_eq!(reader.read_u8(), Some(0x42));
    }
}
