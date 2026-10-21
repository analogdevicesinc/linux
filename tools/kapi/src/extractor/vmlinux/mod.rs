// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

use super::{
    ApiExtractor, ApiSpec, CapabilitySpec, ConstraintSpec, ErrorSpec, LockSpec, ParamSpec,
    ReturnSpec, SideEffectSpec, SignalMaskSpec, SignalSpec, StateTransitionSpec, StructFieldSpec,
    StructSpec,
};
use crate::formatter::OutputFormatter;
use anyhow::{Context, Result};
use goblin::elf::Elf;
use std::fs;
use std::io::Write;

mod binary_utils;
use binary_utils::{magic, sizes, DataReader, Endian};

// Helper to convert empty strings to None
fn opt_string(s: String) -> Option<String> {
    if s.is_empty() {
        None
    } else {
        Some(s)
    }
}

pub struct VmlinuxExtractor {
    vmlinux: Vec<u8>,
    specs: Vec<KapiSpec>,
    endian: Endian,
    is_64bit: bool,
}

#[derive(Debug)]
struct KapiSpec {
    name: String,
    api_type: String,
    /// File offset in the vmlinux buffer where this spec's
    /// `struct kernel_api_spec` begins.
    file_offset: usize,
}

impl VmlinuxExtractor {
    pub fn new(vmlinux_path: &str) -> Result<Self> {
        let vmlinux = fs::read(vmlinux_path)
            .with_context(|| format!("Failed to read vmlinux file: {vmlinux_path}"))?;

        let elf = Elf::parse(&vmlinux).context("Failed to parse ELF file")?;
        let endian = if elf.little_endian {
            Endian::Little
        } else {
            Endian::Big
        };
        let is_64bit = elf.is_64;

        // Locate the .kapi_specs section boundaries.
        let mut start_addr = None;
        let mut stop_addr = None;
        for sym in &elf.syms {
            if let Some(name) = elf.strtab.get_at(sym.st_name) {
                match name {
                    "__start_kapi_specs" => start_addr = Some(sym.st_value),
                    "__stop_kapi_specs" => stop_addr = Some(sym.st_value),
                    _ => {}
                }
            }
        }
        let start = start_addr.context("Could not find __start_kapi_specs symbol")?;
        let stop = stop_addr.context("Could not find __stop_kapi_specs symbol")?;
        if stop <= start {
            anyhow::bail!("No kernel API specifications found in vmlinux");
        }

        // `.kapi_specs` is a tightly-packed array of `struct kernel_api_spec *`
        // pointers; walk them to find each real spec's vaddr, then resolve to
        // a file offset inside `vmlinux`. Pointer width tracks the target
        // (4 bytes for 32-bit, 8 bytes for 64-bit).
        let ptr_size = if is_64bit { 8usize } else { 4 };
        let ptr_count = ((stop - start) as usize) / ptr_size;
        let ptr_file_off =
            vaddr_to_file_offset(&elf, start).context("Could not locate .kapi_specs in file")?;

        let read_ptr = |raw: &[u8]| -> u64 {
            match (endian, is_64bit) {
                (Endian::Little, true) => u64::from_le_bytes(raw.try_into().unwrap()),
                (Endian::Big, true) => u64::from_be_bytes(raw.try_into().unwrap()),
                (Endian::Little, false) => u32::from_le_bytes(raw.try_into().unwrap()) as u64,
                (Endian::Big, false) => u32::from_be_bytes(raw.try_into().unwrap()) as u64,
            }
        };

        let mut specs = Vec::with_capacity(ptr_count);
        for i in 0..ptr_count {
            let p = ptr_file_off + i * ptr_size;
            if p + ptr_size > vmlinux.len() {
                break;
            }
            let spec_vaddr = read_ptr(&vmlinux[p..p + ptr_size]);
            if spec_vaddr == 0 {
                continue;
            }
            let Some(spec_file_off) = vaddr_to_file_offset(&elf, spec_vaddr) else {
                continue;
            };
            // The first field of `struct kernel_api_spec` is `const char *name`.
            if spec_file_off + ptr_size > vmlinux.len() {
                continue;
            }
            let name_vaddr = read_ptr(&vmlinux[spec_file_off..spec_file_off + ptr_size]);
            let name =
                binary_utils::resolve_vaddr_string(&elf, &vmlinux, name_vaddr).unwrap_or_default();
            if name.is_empty() {
                continue;
            }
            let api_type = if name.starts_with("sys_") {
                "syscall"
            } else if name.ends_with("_ioctl") {
                "ioctl"
            } else {
                "function"
            }
            .to_string();
            specs.push(KapiSpec {
                name,
                api_type,
                file_offset: spec_file_off,
            });
        }

        Ok(VmlinuxExtractor {
            vmlinux,
            specs,
            endian,
            is_64bit,
        })
    }
}

/// Map a virtual address to a file offset inside the raw vmlinux bytes.
fn vaddr_to_file_offset(elf: &Elf, vaddr: u64) -> Option<usize> {
    for sh in &elf.section_headers {
        let start = sh.sh_addr;
        let end = start.checked_add(sh.sh_size)?;
        if vaddr >= start && vaddr < end {
            if sh.sh_type == goblin::elf::section_header::SHT_NOBITS {
                return None;
            }
            return Some((sh.sh_offset + (vaddr - start)) as usize);
        }
    }
    None
}

impl VmlinuxExtractor {
    fn parse_at(&self, file_offset: usize) -> Result<ApiSpec> {
        parse_binary_to_api_spec(&self.vmlinux, file_offset, self.endian, self.is_64bit)
    }
}

impl ApiExtractor for VmlinuxExtractor {
    fn extract_all(&self) -> Result<Vec<ApiSpec>> {
        Ok(self
            .specs
            .iter()
            .map(|spec| {
                self.parse_at(spec.file_offset).unwrap_or_else(|e| {
                    eprintln!("Warning: cannot parse the spec of {}: {e:#}", spec.name);
                    ApiSpec {
                        name: spec.name.clone(),
                        api_type: spec.api_type.clone(),
                        ..Default::default()
                    }
                })
            })
            .collect())
    }

    fn extract_by_name(&self, api_name: &str) -> Result<Option<ApiSpec>> {
        if let Some(spec) = self.specs.iter().find(|s| s.name == api_name) {
            Ok(Some(self.parse_at(spec.file_offset)?))
        } else {
            Ok(None)
        }
    }

    fn display_api_details(
        &self,
        api_name: &str,
        formatter: &mut dyn OutputFormatter,
        writer: &mut dyn Write,
    ) -> Result<()> {
        if let Some(spec) = self.specs.iter().find(|s| s.name == api_name) {
            let api_spec = self.parse_at(spec.file_offset)?;
            super::display_api_spec(&api_spec, formatter, writer)?;
        }
        Ok(())
    }
}

const TRUNCATED: &str = "kernel_api_spec runs past the end of the file";

/// Consume a section marker. The kernel leaves it zero when the matching
/// macro is not used, so only a different non-zero value means that the
/// reader has lost sync with the struct layout.
fn read_magic(reader: &mut DataReader, expected: u32) -> Result<()> {
    let found = reader.read_u32().context(TRUNCATED)?;
    if found != 0 && found != expected {
        anyhow::bail!(
            "unexpected section marker {found:#x} at offset {:#x}, expected {expected:#x}",
            reader.pos - 4
        );
    }
    Ok(())
}

/// Parse a `{ u32 magic; u32 count; T items[MAX]; }` section. Every one of
/// the `max_items` slots is consumed, so the reader ends up behind the
/// whole array; only the first `count` are returned.
fn parse_array<T, F>(
    reader: &mut DataReader,
    expected_magic: u32,
    max_items: usize,
    parse_fn: F,
) -> Result<Vec<T>>
where
    F: Fn(&mut DataReader, usize) -> Option<T>,
{
    read_magic(reader, expected_magic)?;
    let count = reader.read_u32().context(TRUNCATED)? as usize;
    let align = reader.ptr_size();
    reader.align_to(align);

    let mut items = Vec::new();
    for i in 0..max_items {
        let item = parse_fn(reader, i).context(TRUNCATED)?;
        reader.align_to(align);
        if i < count {
            items.push(item);
        }
    }
    Ok(items)
}

fn parse_binary_to_api_spec(
    data: &[u8],
    offset: usize,
    endian: Endian,
    is_64bit: bool,
) -> Result<ApiSpec> {
    let elf = Elf::parse(data).context("Failed to re-parse ELF for string resolution")?;
    let resolver = binary_utils::StringResolver {
        elf: &elf,
        vmlinux: data,
    };
    let mut reader = DataReader::new(data, offset, endian, is_64bit).with_resolver(resolver);

    // Read fields in exact order of struct kernel_api_spec.
    // Every string field is a `const char *` pointer resolved via the
    // StringResolver attached to the DataReader.
    let name = reader
        .read_optional_string(sizes::NAME)
        .ok_or_else(|| anyhow::anyhow!("Failed to read API name"))?;

    // Determine API type
    let api_type = if name.starts_with("sys_") {
        "syscall"
    } else if name.ends_with("_ioctl") {
        "ioctl"
    } else if name.contains("sysfs") {
        "sysfs"
    } else {
        "function"
    }
    .to_string();

    let version = reader.read_u32().map(|v| v.to_string());

    let description = reader
        .read_optional_string(sizes::DESC)
        .filter(|s| !s.is_empty());

    let long_description = reader
        .read_optional_string(sizes::DESC)
        .filter(|s| !s.is_empty());

    let context_flags = parse_context_flags(&mut reader);

    let parameters = parse_array(&mut reader, magic::PARAMS, sizes::MAX_PARAMS, parse_param)?;

    read_magic(&mut reader, magic::RETURN)?;
    let return_spec = parse_return_spec(&mut reader);

    let errors = parse_array(&mut reader, magic::ERRORS, sizes::MAX_ERRORS, |r, _| {
        parse_error(r)
    })?;

    let locks = parse_array(&mut reader, magic::LOCKS, sizes::MAX_LOCKS, |r, _| {
        parse_lock(r)
    })?;

    let constraints = parse_array(
        &mut reader,
        magic::CONSTRAINTS,
        sizes::MAX_CONSTRAINTS,
        |r, _| parse_constraint(r),
    )?;

    // Only KAPI_EXAMPLES() sets info_magic, so it says nothing about
    // whether notes are present.
    read_magic(&mut reader, magic::INFO)?;
    let examples = reader
        .read_optional_string(sizes::DESC)
        .filter(|s| !s.is_empty());
    let notes = reader
        .read_optional_string(sizes::DESC)
        .filter(|s| !s.is_empty());

    let signals = parse_array(&mut reader, magic::SIGNALS, sizes::MAX_SIGNALS, |r, _| {
        parse_signal(r)
    })?;

    let signal_masks = parse_array(&mut reader, magic::SIGMASK, sizes::MAX_SIGNALS, |r, _| {
        parse_signal_mask(r)
    })?;

    let struct_specs = parse_array(
        &mut reader,
        magic::STRUCTS,
        sizes::MAX_STRUCT_SPECS,
        |r, _| parse_struct_spec(r),
    )?;

    let side_effects = parse_array(
        &mut reader,
        magic::EFFECTS,
        sizes::MAX_SIDE_EFFECTS,
        |r, _| parse_side_effect(r),
    )?;

    let state_transitions =
        parse_array(&mut reader, magic::TRANS, sizes::MAX_STATE_TRANS, |r, _| {
            parse_state_transition(r)
        })?;

    let capabilities = parse_array(&mut reader, magic::CAPS, sizes::MAX_CAPABILITIES, |r, _| {
        parse_capability(r)
    })?;

    Ok(ApiSpec {
        name,
        api_type,
        description,
        long_description,
        version,
        context_flags,
        param_count: if parameters.is_empty() {
            None
        } else {
            Some(parameters.len() as u32)
        },
        error_count: if errors.is_empty() {
            None
        } else {
            Some(errors.len() as u32)
        },
        examples,
        notes,
        subsystem: None,
        sysfs_path: None,
        permissions: None,
        capabilities,
        parameters,
        return_spec,
        errors,
        signals,
        signal_masks,
        side_effects,
        state_transitions,
        constraints,
        locks,
        struct_specs,
    })
}

// Helper parsing functions

fn parse_context_flags(reader: &mut DataReader) -> Vec<String> {
    const KAPI_CTX_PROCESS: u32 = 1 << 0;
    const KAPI_CTX_SOFTIRQ: u32 = 1 << 1;
    const KAPI_CTX_HARDIRQ: u32 = 1 << 2;
    const KAPI_CTX_NMI: u32 = 1 << 3;
    const KAPI_CTX_ATOMIC: u32 = 1 << 4;
    const KAPI_CTX_SLEEPABLE: u32 = 1 << 5;
    const KAPI_CTX_PREEMPT_DISABLED: u32 = 1 << 6;
    const KAPI_CTX_IRQ_DISABLED: u32 = 1 << 7;

    if let Some(flags) = reader.read_u32() {
        let mut parts = Vec::new();

        if flags & KAPI_CTX_PROCESS != 0 {
            parts.push("KAPI_CTX_PROCESS");
        }
        if flags & KAPI_CTX_SOFTIRQ != 0 {
            parts.push("KAPI_CTX_SOFTIRQ");
        }
        if flags & KAPI_CTX_HARDIRQ != 0 {
            parts.push("KAPI_CTX_HARDIRQ");
        }
        if flags & KAPI_CTX_NMI != 0 {
            parts.push("KAPI_CTX_NMI");
        }
        if flags & KAPI_CTX_ATOMIC != 0 {
            parts.push("KAPI_CTX_ATOMIC");
        }
        if flags & KAPI_CTX_SLEEPABLE != 0 {
            parts.push("KAPI_CTX_SLEEPABLE");
        }
        if flags & KAPI_CTX_PREEMPT_DISABLED != 0 {
            parts.push("KAPI_CTX_PREEMPT_DISABLED");
        }
        if flags & KAPI_CTX_IRQ_DISABLED != 0 {
            parts.push("KAPI_CTX_IRQ_DISABLED");
        }

        parts.into_iter().map(|s| s.to_string()).collect()
    } else {
        vec![]
    }
}

fn parse_param(reader: &mut DataReader, index: usize) -> Option<ParamSpec> {
    let name = reader.read_string_or_default(sizes::NAME);
    let type_name = reader.read_string_or_default(sizes::NAME);
    let param_type = reader.read_u32()?;
    let flags = reader.read_u32()?;
    let size = reader.read_usize()?;
    let alignment = reader.read_usize()?;
    let min_value = reader.read_i64()?;
    let max_value = reader.read_i64()?;
    let valid_mask = reader.read_u64()?;

    let enum_values_ptr = reader.read_ptr()?;
    let enum_count = reader.read_u32()?;
    let constraint_type = reader.read_u32()?;
    // Skip validate function pointer
    reader.read_ptr()?;

    let description = reader.read_string_or_default(sizes::DESC);
    let constraint = reader.read_optional_string(sizes::DESC);
    let size_param_idx_raw = reader.read_i32()?; // Must use ? to propagate errors
    let _size_multiplier = reader.read_usize()?; // Must use ? to propagate errors

    // In the C struct, size_param_idx is stored 1-based; 0 means
    // "no size-carrying param". Surface the real (0-based) index as
    // `Option<u32>`.
    let size_param_idx = if size_param_idx_raw > 0 {
        Some((size_param_idx_raw - 1) as u32)
    } else {
        None
    };

    let mut param = ParamSpec {
        index: index as u32,
        name,
        type_name,
        description,
        flags,
        param_type,
        constraint_type,
        constraint,
        min_value: Some(min_value),
        max_value: Some(max_value),
        valid_mask: Some(valid_mask),
        enum_values: reader
            .resolve_s64_array(enum_values_ptr, enum_count)
            .iter()
            .map(i64::to_string)
            .collect(),
        size: Some(size as u32),
        alignment: Some(alignment as u32),
        size_param_idx,
    };
    param.keep_used_numbers();
    Some(param)
}

fn parse_return_spec(reader: &mut DataReader) -> Option<ReturnSpec> {
    // Read type_name, but treat empty as valid (will be empty string)
    let type_name = reader.read_string_or_default(sizes::NAME);

    // Read return_type and check_type
    let return_type = reader.read_u32().unwrap_or(0);
    let check_type = reader.read_u32().unwrap_or(0);
    let success_value = reader.read_i64().unwrap_or(0);
    let success_min = reader.read_i64().unwrap_or(0);
    let success_max = reader.read_i64().unwrap_or(0);

    let error_values_ptr = reader.read_ptr().unwrap_or(0);
    let error_count = reader.read_u32().unwrap_or(0);
    // Skip is_success function pointer
    let _ = reader.read_ptr();

    let description = reader.read_string_or_default(sizes::DESC);

    // Return a spec even if type_name is empty, as long as we have some data
    // The type_name might be a string like "KAPI_TYPE_INT" that gets stored literally
    if type_name.is_empty() && return_type == 0 && check_type == 0 && success_value == 0 {
        // No return spec at all
        return None;
    }

    let mut ret = ReturnSpec {
        type_name,
        description,
        return_type,
        check_type,
        success_value: Some(success_value),
        success_min: Some(success_min),
        success_max: Some(success_max),
        error_values: reader
            .resolve_s64_array(error_values_ptr, error_count)
            .into_iter()
            .filter_map(|v| i32::try_from(v).ok())
            .collect(),
    };
    ret.keep_used_success_fields();
    Some(ret)
}

fn parse_error(reader: &mut DataReader) -> Option<ErrorSpec> {
    let error_code = reader.read_i32()?;
    let name = reader.read_string_or_default(sizes::NAME);
    let condition = reader.read_string_or_default(sizes::DESC);
    let description = reader.read_string_or_default(sizes::DESC);

    Some(ErrorSpec {
        error_code,
        name,
        condition,
        description,
    })
}

fn parse_lock(reader: &mut DataReader) -> Option<LockSpec> {
    let lock_name = reader.read_string_or_default(sizes::NAME);
    let lock_type = reader.read_u32()?;
    let scope = reader.read_u32()?;
    let description = reader.read_string_or_default(sizes::DESC);

    Some(LockSpec {
        lock_name,
        lock_type,
        scope,
        description,
    })
}

fn parse_constraint(reader: &mut DataReader) -> Option<ConstraintSpec> {
    let name = reader.read_string_or_default(sizes::NAME);
    let description = reader.read_string_or_default(sizes::DESC);
    let expression = reader.read_string_or_default(sizes::DESC);

    Some(ConstraintSpec {
        name,
        description,
        expression: opt_string(expression),
    })
}

fn parse_signal(reader: &mut DataReader) -> Option<SignalSpec> {
    // Matches `struct kapi_signal_spec`. All string fields are pointers.
    let signal_num = reader.read_i32()?;
    let signal_name = reader.read_optional_string(sizes::NAME).unwrap_or_default();
    let direction = reader.read_u32()?;
    let action = reader.read_u32()?;
    let target = reader.read_optional_string(sizes::DESC);
    let condition = reader.read_optional_string(sizes::DESC);
    let description = reader.read_optional_string(sizes::DESC);
    let restartable = reader.read_bool()?;
    let sa_flags_required = reader.read_u32()?;
    let sa_flags_forbidden = reader.read_u32()?;
    let error_on_signal = reader.read_i32()?;
    let transform_to = reader.read_i32()?;
    // Read the symbolic timing token (const char *) and map it to the
    // numeric timing code used by downstream consumers.
    let timing_str = reader.read_optional_string(sizes::NAME).unwrap_or_default();
    let timing = match timing_str.as_str() {
        "KAPI_SIGNAL_TIME_BEFORE" | "before" => 0u32,
        "KAPI_SIGNAL_TIME_DURING" | "during" => 1,
        "KAPI_SIGNAL_TIME_AFTER" | "after" => 2,
        _ => 0,
    };
    let priority = reader.read_u8()?;
    let interruptible = reader.read_bool()?;
    let queue_behavior = reader.read_optional_string(sizes::NAME);
    let state_required = reader.read_u32()?;
    let state_forbidden = reader.read_u32()?;

    Some(SignalSpec {
        signal_num,
        signal_name,
        direction,
        action,
        target,
        condition,
        description,
        timing,
        priority: priority as u32,
        restartable,
        interruptible,
        queue: queue_behavior,
        sa_flags: 0, // Not a field of struct kapi_signal_spec
        sa_flags_required,
        sa_flags_forbidden,
        state_required,
        state_forbidden,
        // `error_on_signal` of 0 means "no errno returned"; surface
        // that as None to match the source-parser convention.
        error_on_signal: if error_on_signal != 0 {
            Some(error_on_signal)
        } else {
            None
        },
        transform_to: if transform_to != 0 {
            // The compiled struct holds the numeric value; the C
            // preprocessor already resolved any signal symbol.
            Some(transform_to)
        } else {
            None
        },
    })
}

fn parse_signal_mask(reader: &mut DataReader) -> Option<SignalMaskSpec> {
    let name = reader.read_string_or_default(sizes::NAME);

    let mut signals = Vec::with_capacity(sizes::MAX_SIGNALS);
    for _ in 0..sizes::MAX_SIGNALS {
        signals.push(reader.read_i32()?);
    }
    let signal_count = reader.read_u32()?;
    signals.truncate(signal_count as usize);

    let description = reader.read_string_or_default(sizes::DESC);

    Some(SignalMaskSpec {
        name,
        description,
        signals,
    })
}

fn parse_struct_field(reader: &mut DataReader) -> Option<StructFieldSpec> {
    let name = reader.read_string_or_default(sizes::NAME);
    let field_type = reader.read_u32()?;
    let type_name = reader.read_string_or_default(sizes::NAME);
    let offset = reader.read_usize()?;
    let size = reader.read_usize()?;
    let flags = reader.read_u32()?;
    let constraint_type = reader.read_u32()?;
    let min_value = reader.read_i64()?;
    let max_value = reader.read_i64()?;
    let valid_mask = reader.read_u64()?;
    // enum_values is a `const char *` that StructFieldSpec has no slot for
    reader.read_ptr()?;
    let description = reader.read_string_or_default(sizes::DESC);

    Some(StructFieldSpec {
        name,
        field_type,
        type_name,
        offset,
        size,
        flags,
        constraint_type,
        min_value,
        max_value,
        valid_mask,
        description,
    })
}

fn parse_struct_spec(reader: &mut DataReader) -> Option<StructSpec> {
    let name = reader.read_string_or_default(sizes::NAME);
    let size = reader.read_usize()?;
    let alignment = reader.read_usize()?;
    let field_count = reader.read_u32()?;

    let mut fields = Vec::new();
    for i in 0..sizes::MAX_PARAMS {
        let field = parse_struct_field(reader)?;
        reader.align_to(reader.ptr_size());
        if i < field_count as usize {
            fields.push(field);
        }
    }

    let description = reader.read_string_or_default(sizes::DESC);

    Some(StructSpec {
        name,
        size,
        alignment,
        field_count: fields.len() as u32,
        fields,
        description,
    })
}

fn parse_side_effect(reader: &mut DataReader) -> Option<SideEffectSpec> {
    let effect_type = reader.read_u32()?;
    let target = reader.read_string_or_default(sizes::NAME);
    let condition = reader.read_string_or_default(sizes::DESC);
    let description = reader.read_string_or_default(sizes::DESC);
    let reversible = reader.read_bool()?;

    Some(SideEffectSpec {
        effect_type,
        target,
        condition: opt_string(condition),
        description,
        reversible,
    })
}

fn parse_state_transition(reader: &mut DataReader) -> Option<StateTransitionSpec> {
    let from_state = reader.read_string_or_default(sizes::NAME);
    let to_state = reader.read_string_or_default(sizes::NAME);
    let condition = reader.read_string_or_default(sizes::DESC);
    let object = reader.read_string_or_default(sizes::NAME);
    let description = reader.read_string_or_default(sizes::DESC);

    Some(StateTransitionSpec {
        object,
        from_state,
        to_state,
        condition: opt_string(condition),
        description,
    })
}

fn parse_capability(reader: &mut DataReader) -> Option<CapabilitySpec> {
    // Struct layout matches `struct kapi_capability_spec`:
    //   int capability; const char *cap_name; enum action;
    //   const char *allows; const char *without_cap;
    //   const char *check_condition; u8 priority;
    //   int alternative[KAPI_MAX_CAPABILITIES]; u32 alternative_count;
    let capability = reader.read_i32()?;
    let cap_name = reader.read_string_or_default(sizes::NAME);
    let action = reader.read_u32()?;
    let allows = reader.read_string_or_default(sizes::DESC);
    let without_cap = reader.read_string_or_default(sizes::DESC);
    let check_condition = reader.read_optional_string(sizes::DESC);
    let priority = reader.read_u8()?;

    let mut alternatives = Vec::with_capacity(sizes::MAX_CAPABILITIES);
    for _ in 0..sizes::MAX_CAPABILITIES {
        alternatives.push(reader.read_i32()?);
    }
    let alternative_count = reader.read_u32()?;
    alternatives.truncate(alternative_count as usize);

    Some(CapabilitySpec {
        capability,
        name: cap_name,
        action: capability_action_to_string(action),
        allows,
        without_cap,
        check_condition,
        priority: Some(priority),
        alternatives,
    })
}

/// Map the `enum kapi_capability_action` numeric value to its symbolic
/// spelling, matching `include/linux/kernel_api_spec.h`.
fn capability_action_to_string(n: u32) -> String {
    match n {
        0 => "KAPI_CAP_BYPASS_CHECK",
        1 => "KAPI_CAP_INCREASE_LIMIT",
        2 => "KAPI_CAP_OVERRIDE_RESTRICTION",
        3 => "KAPI_CAP_GRANT_PERMISSION",
        4 => "KAPI_CAP_MODIFY_BEHAVIOR",
        5 => "KAPI_CAP_ACCESS_RESOURCE",
        6 => "KAPI_CAP_PERFORM_OPERATION",
        _ => return n.to_string(),
    }
    .to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_transition_with_empty_strings_consumes_the_whole_struct() {
        // five `const char *` slots: from_state, to_state, condition,
        // object, description
        let data = [0u8; 5 * 8];
        let mut reader = DataReader::new(&data, 0, Endian::Little, true);

        let trans = parse_state_transition(&mut reader).unwrap();

        assert_eq!(reader.pos, data.len());
        assert_eq!(trans.from_state, "");
        assert_eq!(trans.condition, None);
        assert_eq!(trans.description, "");
    }

    // Offsets inside `struct kernel_api_spec` and its element structs as
    // laid out by gcc on x86-64, taken from offsetof()/sizeof().
    const SPEC_SIZE: usize = 26400;
    const PARAM_MAGIC: usize = 36;
    const PARAMS: usize = 48;
    const PARAM_SIZE: usize = 120;
    const RETURN_MAGIC: usize = 1968;
    const RETURN_SPEC: usize = 1976;
    const ERROR_MAGIC: usize = 2048;
    const ERRORS: usize = 2056;
    const ERROR_SIZE: usize = 32;
    const LOCK_MAGIC: usize = 3080;
    const LOCKS: usize = 3088;
    const LOCK_SIZE: usize = 24;
    const CONSTRAINT_MAGIC: usize = 3472;
    const CONSTRAINTS: usize = 3480;
    const CONSTRAINT_SIZE: usize = 24;
    const INFO_MAGIC: usize = 4248;
    const NOTES: usize = 4264;
    const SIGNAL_MAGIC: usize = 4272;
    const SIGNALS: usize = 4280;
    const SIGNAL_SIZE: usize = 104;
    const SIGMASK_MAGIC: usize = 7608;
    const SIGNAL_MASKS: usize = 7616;
    const SIGNAL_MASK_SIZE: usize = 152;
    const STRUCT_MAGIC: usize = 12480;
    const STRUCT_SPECS: usize = 12488;
    const STRUCT_SPEC_SIZE: usize = 1448;
    const STRUCT_FIELD_SIZE: usize = 88;
    const EFFECT_MAGIC: usize = 24072;
    const SIDE_EFFECTS: usize = 24080;
    const SIDE_EFFECT_SIZE: usize = 40;
    const TRANS_MAGIC: usize = 25360;
    const STATE_TRANSITIONS: usize = 25368;
    const STATE_TRANSITION_SIZE: usize = 40;
    const CAP_MAGIC: usize = 25688;
    const CAPABILITIES: usize = 25696;
    const CAPABILITY_SIZE: usize = 88;

    const BASE: u64 = 0xffff_ffff_8100_0000;
    const CONTENT_OFFSET: usize = 0x1000;

    /// A `struct kernel_api_spec` image preceded by the strings and
    /// `s64` arrays it points at, wrapped in a minimal ELF file.
    struct Image {
        spec: Vec<u8>,
        tail: Vec<u8>,
    }

    impl Image {
        fn new() -> Self {
            Image {
                spec: vec![0; SPEC_SIZE],
                tail: Vec::new(),
            }
        }

        fn u32(&mut self, at: usize, v: u32) {
            self.spec[at..at + 4].copy_from_slice(&v.to_le_bytes());
        }

        fn u64(&mut self, at: usize, v: u64) {
            self.spec[at..at + 8].copy_from_slice(&v.to_le_bytes());
        }

        fn tail_vaddr(&self) -> u64 {
            BASE + self.tail.len() as u64
        }

        fn string(&mut self, at: usize, s: &str) {
            let vaddr = self.tail_vaddr();
            self.tail.extend_from_slice(s.as_bytes());
            self.tail.push(0);
            self.u64(at, vaddr);
        }

        fn s64s(&mut self, at: usize, vals: &[i64]) {
            while self.tail.len() % 8 != 0 {
                self.tail.push(0);
            }
            let vaddr = self.tail_vaddr();
            for v in vals {
                self.tail.extend_from_slice(&v.to_le_bytes());
            }
            self.u64(at, vaddr);
        }

        fn elf(&self) -> Vec<u8> {
            let mut content = self.tail.clone();
            content.extend_from_slice(&self.spec);

            let mut img = vec![0u8; CONTENT_OFFSET];
            img[..4].copy_from_slice(b"\x7fELF");
            img[4] = 2; // ELFCLASS64
            img[5] = 1; // ELFDATA2LSB
            img[6] = 1; // EV_CURRENT
            img[16..18].copy_from_slice(&2u16.to_le_bytes()); // ET_EXEC
            img[18..20].copy_from_slice(&62u16.to_le_bytes()); // EM_X86_64
            img[20..24].copy_from_slice(&1u32.to_le_bytes());
            img[40..48].copy_from_slice(&64u64.to_le_bytes()); // e_shoff
            img[52..54].copy_from_slice(&64u16.to_le_bytes()); // e_ehsize
            img[58..60].copy_from_slice(&64u16.to_le_bytes()); // e_shentsize
            img[60..62].copy_from_slice(&2u16.to_le_bytes()); // e_shnum
            let sh = 64 + 64; // section 1 follows the null section header at 64
            img[sh + 4..sh + 8].copy_from_slice(&1u32.to_le_bytes()); // SHT_PROGBITS
            img[sh + 8..sh + 16].copy_from_slice(&2u64.to_le_bytes()); // SHF_ALLOC
            img[sh + 16..sh + 24].copy_from_slice(&BASE.to_le_bytes());
            img[sh + 24..sh + 32].copy_from_slice(&(CONTENT_OFFSET as u64).to_le_bytes());
            img[sh + 32..sh + 40].copy_from_slice(&(content.len() as u64).to_le_bytes());
            img[sh + 48..sh + 56].copy_from_slice(&8u64.to_le_bytes());
            img.extend_from_slice(&content);
            img
        }

        fn parse(&mut self) -> Result<ApiSpec> {
            self.string(0, "kapi_fixture");
            while self.tail.len() % 8 != 0 {
                self.tail.push(0);
            }
            let img = self.elf();
            parse_binary_to_api_spec(&img, CONTENT_OFFSET + self.tail.len(), Endian::Little, true)
        }
    }

    #[test]
    fn every_array_slot_is_consumed() {
        let mut img = Image::new();
        img.u32(PARAM_MAGIC, magic::PARAMS);
        img.u32(PARAM_MAGIC + 4, 16);
        img.u32(RETURN_MAGIC, magic::RETURN);
        img.u32(ERROR_MAGIC, magic::ERRORS);
        img.u32(ERROR_MAGIC + 4, 32);
        img.u32(LOCK_MAGIC, magic::LOCKS);
        img.u32(LOCK_MAGIC + 4, 16);
        img.u32(CONSTRAINT_MAGIC, magic::CONSTRAINTS);
        img.u32(CONSTRAINT_MAGIC + 4, 32);
        img.u32(SIGNAL_MAGIC, magic::SIGNALS);
        img.u32(SIGNAL_MAGIC + 4, 32);
        img.u32(SIGMASK_MAGIC, magic::SIGMASK);
        img.u32(SIGMASK_MAGIC + 4, 32);
        img.u32(STRUCT_MAGIC, magic::STRUCTS);
        img.u32(STRUCT_MAGIC + 4, 8);
        img.u32(EFFECT_MAGIC, magic::EFFECTS);
        img.u32(EFFECT_MAGIC + 4, 32);
        img.u32(TRANS_MAGIC, magic::TRANS);
        img.u32(TRANS_MAGIC + 4, 8);
        img.u32(CAP_MAGIC, magic::CAPS);
        img.u32(CAP_MAGIC + 4, 8);

        img.string(PARAMS + 15 * PARAM_SIZE, "last_param");
        img.u32(ERRORS + 31 * ERROR_SIZE, -7i32 as u32);
        img.string(ERRORS + 31 * ERROR_SIZE + 8, "ELAST");
        img.string(LOCKS + 15 * LOCK_SIZE, "last_lock");
        img.string(CONSTRAINTS + 31 * CONSTRAINT_SIZE, "last_constraint");
        img.u32(SIGNALS + 31 * SIGNAL_SIZE, 31);
        img.u32(SIGNAL_MASKS + 31 * SIGNAL_MASK_SIZE + 8, 64);
        img.u32(SIGNAL_MASKS + 31 * SIGNAL_MASK_SIZE + 136, 1);
        img.string(SIGNAL_MASKS + 31 * SIGNAL_MASK_SIZE, "last mask");
        img.string(SIGNAL_MASKS + 31 * SIGNAL_MASK_SIZE + 144, "mask desc");

        let last_struct = STRUCT_SPECS + 7 * STRUCT_SPEC_SIZE;
        img.string(last_struct, "last_struct");
        img.u32(last_struct + 24, 16);
        img.string(last_struct + 32 + 15 * STRUCT_FIELD_SIZE, "last_field");
        img.string(last_struct + 32 + 15 * STRUCT_FIELD_SIZE + 80, "field desc");
        img.string(last_struct + 1440, "struct desc");

        img.string(SIDE_EFFECTS + 31 * SIDE_EFFECT_SIZE + 8, "last_effect");
        img.string(
            STATE_TRANSITIONS + 7 * STATE_TRANSITION_SIZE + 24,
            "last_object",
        );
        let last_cap = CAPABILITIES + 7 * CAPABILITY_SIZE;
        img.u32(last_cap, 40);
        img.u32(last_cap + 52 + 7 * 4, 99);
        img.u32(last_cap + 84, 8);

        let spec = img.parse().unwrap();

        assert_eq!(spec.parameters.len(), 16);
        assert_eq!(spec.parameters[15].name, "last_param");
        assert_eq!(spec.errors.len(), 32);
        assert_eq!(spec.errors[31].error_code, -7);
        assert_eq!(spec.errors[31].name, "ELAST");
        assert_eq!(spec.locks[15].lock_name, "last_lock");
        assert_eq!(spec.constraints[31].name, "last_constraint");
        assert_eq!(spec.signals[31].signal_num, 31);
        assert_eq!(spec.signal_masks.len(), 32);
        assert_eq!(spec.signal_masks[31].name, "last mask");
        assert_eq!(spec.signal_masks[31].signals, [64]);
        assert_eq!(spec.signal_masks[31].description, "mask desc");

        let st = &spec.struct_specs[7];
        assert_eq!(st.name, "last_struct");
        assert_eq!(st.description, "struct desc");
        assert_eq!(st.field_count, 16);
        assert_eq!(st.fields[15].name, "last_field");
        assert_eq!(st.fields[15].description, "field desc");

        assert_eq!(spec.side_effects[31].target, "last_effect");
        assert_eq!(spec.state_transitions[7].object, "last_object");
        let cap = &spec.capabilities[7];
        assert_eq!(cap.capability, 40);
        assert_eq!(cap.alternatives, [0, 0, 0, 0, 0, 0, 0, 99]);
    }

    #[test]
    fn notes_and_signal_masks_are_read_without_their_markers() {
        let mut img = Image::new();
        img.u32(SIGMASK_MAGIC + 4, 2);
        img.string(NOTES, "only notes");
        img.string(SIGNAL_MASKS, "first");
        img.u32(SIGNAL_MASKS + 8, 2);
        img.u32(SIGNAL_MASKS + 12, 15);
        img.u32(SIGNAL_MASKS + 136, 2);
        img.string(SIGNAL_MASKS + 144, "first desc");
        img.string(SIGNAL_MASKS + SIGNAL_MASK_SIZE, "second");

        let spec = img.parse().unwrap();

        assert_eq!(spec.examples, None);
        assert_eq!(spec.notes.as_deref(), Some("only notes"));
        assert_eq!(spec.signal_masks.len(), 2);
        assert_eq!(spec.signal_masks[0].name, "first");
        assert_eq!(spec.signal_masks[0].description, "first desc");
        assert_eq!(spec.signal_masks[0].signals, [2, 15]);
        assert_eq!(spec.signal_masks[1].name, "second");
        assert!(spec.signal_masks[1].signals.is_empty());
    }

    #[test]
    fn examples_are_read_next_to_notes() {
        let mut img = Image::new();
        img.u32(INFO_MAGIC, magic::INFO);
        img.string(INFO_MAGIC + 8, "an example");
        img.string(NOTES, "a note");

        let spec = img.parse().unwrap();

        assert_eq!(spec.examples.as_deref(), Some("an example"));
        assert_eq!(spec.notes.as_deref(), Some("a note"));
    }

    #[test]
    fn enum_and_error_values_are_followed_through_their_pointers() {
        let mut img = Image::new();
        img.u32(PARAM_MAGIC, magic::PARAMS);
        img.u32(PARAM_MAGIC + 4, 2);
        img.string(PARAMS, "mode");
        img.s64s(PARAMS + 64, &[0, 1, -7, 1 << 40]);
        img.u32(PARAMS + 72, 4);
        img.string(PARAMS + PARAM_SIZE, "plain");
        img.u32(RETURN_MAGIC, magic::RETURN);
        img.string(RETURN_SPEC, "long");
        img.u32(RETURN_SPEC + 12, 2);
        img.s64s(RETURN_SPEC + 40, &[-22, -2, -(1 << 40)]);
        img.u32(RETURN_SPEC + 48, 3);

        let spec = img.parse().unwrap();

        assert_eq!(
            spec.parameters[0].enum_values,
            ["0", "1", "-7", "1099511627776"]
        );
        assert!(spec.parameters[1].enum_values.is_empty());
        let ret = spec.return_spec.unwrap();
        assert_eq!(ret.check_type, 2);
        assert_eq!(ret.error_values, [-22, -2]);
    }

    #[test]
    fn numbers_the_constraint_does_not_use_are_unset() {
        let mut img = Image::new();
        img.u32(PARAM_MAGIC, magic::PARAMS);
        img.u32(PARAM_MAGIC + 4, 4);
        img.string(PARAMS, "plain");
        let ranged = PARAMS + PARAM_SIZE;
        img.string(ranged, "ranged");
        img.u64(ranged + 24, 16);
        img.u64(ranged + 40, -5i64 as u64);
        img.u64(ranged + 48, 9);
        img.u64(ranged + 56, 0xff);
        img.u32(ranged + 76, 1);
        let string = PARAMS + 2 * PARAM_SIZE;
        img.string(string, "string");
        img.u64(string + 40, 1);
        img.u64(string + 48, 255);
        img.u32(string + 76, 8);
        let unlimited = PARAMS + 3 * PARAM_SIZE;
        img.string(unlimited, "unlimited");
        img.u32(unlimited + 76, 8);
        img.u32(RETURN_MAGIC, magic::RETURN);
        img.string(RETURN_SPEC, "long");
        img.u32(RETURN_SPEC + 12, 1);
        img.u64(RETURN_SPEC + 16, 7);
        img.u64(RETURN_SPEC + 32, 100);

        let spec = img.parse().unwrap();

        let plain = &spec.parameters[0];
        assert_eq!((plain.min_value, plain.max_value), (None, None));
        assert_eq!(
            (plain.valid_mask, plain.size, plain.alignment),
            (None, None, None)
        );
        let ranged = &spec.parameters[1];
        assert_eq!((ranged.min_value, ranged.max_value), (Some(-5), Some(9)));
        assert_eq!((ranged.valid_mask, ranged.size), (None, Some(16)));
        let string = &spec.parameters[2];
        assert_eq!((string.min_value, string.max_value), (Some(1), Some(255)));
        let unlimited = &spec.parameters[3];
        assert_eq!((unlimited.min_value, unlimited.max_value), (None, None));
        let ret = spec.return_spec.unwrap();
        assert_eq!(ret.success_value, None);
        assert_eq!((ret.success_min, ret.success_max), (Some(0), Some(100)));
    }

    #[test]
    fn enum_values_behind_a_dangling_pointer_are_dropped() {
        let mut img = Image::new();
        img.u32(PARAM_MAGIC, magic::PARAMS);
        img.u32(PARAM_MAGIC + 4, 1);
        img.string(PARAMS, "mode");
        img.u64(PARAMS + 64, 0x1000);
        img.u32(PARAMS + 72, 4);

        let spec = img.parse().unwrap();

        assert!(spec.parameters[0].enum_values.is_empty());
    }

    #[test]
    fn unexpected_section_marker_is_rejected() {
        let mut img = Image::new();
        img.u32(ERROR_MAGIC, 0xdead_beef);

        let err = img.parse().unwrap_err();

        assert!(err.to_string().contains("0xdeadbeef"), "{err}");
    }

    #[test]
    fn truncated_spec_is_rejected() {
        let mut img = Image::new();
        img.spec.truncate(SIGNAL_MASKS);

        let err = img.parse().unwrap_err();

        assert!(err.to_string().contains("past the end"), "{err}");
    }
}
