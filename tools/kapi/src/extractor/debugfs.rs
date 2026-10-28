// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

use crate::formatter::OutputFormatter;
use anyhow::{bail, Context, Result};
use serde::Deserialize;
use std::fs;
use std::io::Write;
use std::path::PathBuf;

use super::{
    display_api_spec, ApiExtractor, ApiSpec, CapabilitySpec, ConstraintSpec, ErrorSpec, LockSpec,
    ParamSpec, ReturnSpec, SignalMaskSpec, StateTransitionSpec, StructFieldSpec, StructSpec,
};

// Schema matching what kapi_export_json() in the kernel emits.  The kernel
// serialises several enum-like fields as hex strings ("0x%x") or token
// strings ("exact", "process"); we keep them as Option<String> here and
// interpret them during conversion.
#[derive(Deserialize)]
struct KernelApiJson {
    name: String,
    #[serde(default)]
    api_type: Option<String>,
    #[serde(default)]
    version: Option<u32>,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    long_description: Option<String>,
    #[serde(default)]
    context_flags: Option<String>,
    #[serde(default)]
    examples: Option<String>,
    #[serde(default)]
    notes: Option<String>,
    #[serde(default)]
    capabilities: Option<Vec<KernelCapabilityJson>>,
    #[serde(default)]
    parameters: Option<Vec<KernelParamJson>>,
    #[serde(default)]
    errors: Option<Vec<KernelErrorJson>>,
    #[serde(default, rename = "return")]
    return_spec: Option<KernelReturnJson>,
    #[serde(default)]
    locks: Option<Vec<KernelLockJson>>,
    #[serde(default)]
    constraints: Option<Vec<KernelConstraintJson>>,
    #[serde(default)]
    signals: Option<Vec<KernelSignalJson>>,
    #[serde(default)]
    side_effects: Option<Vec<KernelSideEffectJson>>,
    #[serde(default)]
    state_transitions: Option<Vec<KernelStateTransitionJson>>,
    #[serde(default)]
    signal_masks: Option<Vec<KernelSignalMaskJson>>,
    #[serde(default)]
    struct_specs: Option<Vec<KernelStructJson>>,
}

#[derive(Deserialize)]
struct KernelStateTransitionJson {
    #[serde(default)]
    object: Option<String>,
    #[serde(default)]
    from_state: Option<String>,
    #[serde(default)]
    to_state: Option<String>,
    #[serde(default)]
    condition: Option<String>,
    #[serde(default)]
    description: Option<String>,
}

#[derive(Deserialize)]
struct KernelSignalMaskJson {
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    signals: Vec<i32>,
}

#[derive(Deserialize)]
struct KernelStructFieldJson {
    #[serde(default)]
    name: Option<String>,
    #[serde(rename = "type", default)]
    type_name: Option<String>,
    #[serde(default)]
    type_class: Option<String>,
    #[serde(default)]
    offset: usize,
    #[serde(default)]
    size: usize,
    #[serde(default)]
    flags: Option<String>,
    #[serde(default)]
    constraint_type: Option<String>,
    #[serde(default)]
    min_value: i64,
    #[serde(default)]
    max_value: i64,
    #[serde(default)]
    valid_mask: Option<String>,
    #[serde(default)]
    description: Option<String>,
}

#[derive(Deserialize)]
struct KernelStructJson {
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    size: usize,
    #[serde(default)]
    alignment: usize,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    fields: Vec<KernelStructFieldJson>,
}

#[derive(Deserialize)]
struct KernelConstraintJson {
    name: String,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    expression: Option<String>,
}

#[derive(Deserialize)]
struct KernelSignalJson {
    #[serde(default)]
    signal_num: i32,
    #[serde(default)]
    signal_name: Option<String>,
    #[serde(default)]
    direction: Option<String>,
    #[serde(default)]
    action: u32,
    #[serde(default)]
    target: Option<String>,
    #[serde(default)]
    condition: Option<String>,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    restartable: bool,
    #[serde(default)]
    sa_flags_required: Option<String>,
    #[serde(default)]
    sa_flags_forbidden: Option<String>,
    #[serde(default)]
    error_on_signal: i32,
    #[serde(default)]
    transform_to: i32,
    #[serde(default)]
    timing: Option<String>,
    #[serde(default)]
    priority: u32,
    #[serde(default)]
    interruptible: bool,
    #[serde(default)]
    queue_behavior: Option<String>,
    #[serde(default)]
    state_required: Option<String>,
    #[serde(default)]
    state_forbidden: Option<String>,
}

#[derive(Deserialize)]
struct KernelSideEffectJson {
    #[serde(rename = "type", default)]
    type_hex: Option<String>,
    #[serde(default)]
    target: Option<String>,
    #[serde(default)]
    condition: Option<String>,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    reversible: bool,
}

#[derive(Deserialize)]
struct KernelParamJson {
    name: String,
    #[serde(rename = "type", default)]
    type_name: Option<String>,
    #[serde(default)]
    type_class: Option<String>,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    flags: Option<String>,
    #[serde(default)]
    constraint_type: Option<String>,
    #[serde(default)]
    constraint_desc: Option<String>,
    #[serde(default)]
    min_value: Option<i64>,
    #[serde(default)]
    max_value: Option<i64>,
    #[serde(default)]
    valid_mask: Option<String>,
    #[serde(default)]
    enum_values: Vec<i64>,
    #[serde(default)]
    size: Option<u64>,
    #[serde(default)]
    alignment: Option<u64>,
    #[serde(default)]
    size_param_idx: Option<u32>,
}

#[derive(Deserialize)]
struct KernelErrorJson {
    #[serde(rename = "code")]
    error_code: i32,
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    condition: Option<String>,
    #[serde(default)]
    description: Option<String>,
}

#[derive(Deserialize)]
struct KernelReturnJson {
    #[serde(rename = "type", default)]
    type_name: Option<String>,
    #[serde(default)]
    type_class: Option<String>,
    #[serde(default)]
    check_type: Option<String>,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    success_value: Option<i64>,
    #[serde(default)]
    success_min: Option<i64>,
    #[serde(default)]
    success_max: Option<i64>,
    #[serde(default)]
    error_values: Vec<i64>,
}

#[derive(Deserialize)]
struct KernelLockJson {
    name: String,
    #[serde(rename = "type", default)]
    lock_type: Option<String>,
    #[serde(default)]
    scope: Option<String>,
    #[serde(default)]
    description: Option<String>,
}

#[derive(Deserialize)]
struct KernelCapabilityJson {
    capability: i32,
    name: String,
    action: String,
    allows: String,
    without_cap: String,
    check_condition: Option<String>,
    priority: Option<u8>,
    alternatives: Option<Vec<i32>>,
}

/// Free-text fields of the text dump that may span several lines.
#[derive(Clone, Copy)]
enum TextField {
    Description,
    LongDescription,
    Examples,
    Notes,
}

impl TextField {
    /// Recognise the header line of a text field, returning the field and
    /// whatever follows the colon on the same line.
    fn start(line: &str) -> Option<(Self, &str)> {
        [
            ("Description:", Self::Description),
            ("Long description:", Self::LongDescription),
            ("Examples:", Self::Examples),
            ("Notes:", Self::Notes),
        ]
        .into_iter()
        .find_map(|(header, field)| {
            line.strip_prefix(header)
                .map(|rest| (field, rest.trim_start()))
        })
    }

    fn store(self, spec: &mut ApiSpec, lines: &[String]) {
        let text = lines.join("\n");
        let text = text.trim_matches('\n').trim_end().to_string();
        let slot = match self {
            Self::Description => &mut spec.description,
            Self::LongDescription => &mut spec.long_description,
            Self::Examples => &mut spec.examples,
            Self::Notes => &mut spec.notes,
        };
        *slot = Some(text);
    }
}

/// Extractor for kernel API specifications from debugfs
pub struct DebugfsExtractor {
    debugfs_path: PathBuf,
}

impl DebugfsExtractor {
    /// Create a new debugfs extractor with the specified debugfs path
    pub fn new(debugfs_path: Option<String>) -> Result<Self> {
        let path = match debugfs_path {
            Some(p) => PathBuf::from(p),
            None => PathBuf::from("/sys/kernel/debug"),
        };

        // Check if the debugfs path exists
        if !path.exists() {
            bail!("Debugfs path does not exist: {}", path.display());
        }

        // Check if kapi directory exists
        let kapi_path = path.join("kapi");
        if !kapi_path.exists() {
            bail!(
                "Kernel API debugfs interface not found at: {}",
                kapi_path.display()
            );
        }

        Ok(Self { debugfs_path: path })
    }

    /// Parse the list file to get all available API names
    fn parse_list_file(&self) -> Result<Vec<String>> {
        let list_path = self.debugfs_path.join("kapi/list");
        let content = fs::read_to_string(&list_path)
            .with_context(|| format!("Failed to read {}", list_path.display()))?;

        let mut apis = Vec::new();
        let mut in_list = false;

        for line in content.lines() {
            if line.contains("===") {
                in_list = true;
                continue;
            }

            if in_list && line.starts_with("Total:") {
                break;
            }

            if in_list && !line.trim().is_empty() {
                // Extract API name from lines like "sys_read - Read from a file descriptor"
                if let Some(name) = line.split(" - ").next() {
                    apis.push(name.trim().to_string());
                }
            }
        }

        Ok(apis)
    }

    /// Convert context flags (emitted by the kernel as a hex string like
    /// "0x21") into the token list consumed by the formatter.
    fn parse_context_flags(flags: &str) -> Vec<String> {
        let mut result = Vec::new();
        let bits = flags
            .strip_prefix("0x")
            .or_else(|| flags.strip_prefix("0X"))
            .unwrap_or(flags);
        let Ok(flags) = u32::from_str_radix(bits, 16) else {
            return result;
        };

        // These values should match KAPI_CTX_* flags from kernel
        if flags & (1 << 0) != 0 {
            result.push("KAPI_CTX_PROCESS".to_string());
        }
        if flags & (1 << 1) != 0 {
            result.push("KAPI_CTX_SOFTIRQ".to_string());
        }
        if flags & (1 << 2) != 0 {
            result.push("KAPI_CTX_HARDIRQ".to_string());
        }
        if flags & (1 << 3) != 0 {
            result.push("KAPI_CTX_NMI".to_string());
        }
        if flags & (1 << 4) != 0 {
            result.push("KAPI_CTX_ATOMIC".to_string());
        }
        if flags & (1 << 5) != 0 {
            result.push("KAPI_CTX_SLEEPABLE".to_string());
        }
        if flags & (1 << 6) != 0 {
            result.push("KAPI_CTX_PREEMPT_DISABLED".to_string());
        }
        if flags & (1 << 7) != 0 {
            result.push("KAPI_CTX_IRQ_DISABLED".to_string());
        }

        result
    }

    /// Parse a hex-string like "0x123" into u64, returning 0 on failure.
    fn parse_hex_u64(value: &str) -> u64 {
        let bits = value
            .strip_prefix("0x")
            .or_else(|| value.strip_prefix("0X"))
            .unwrap_or(value);
        u64::from_str_radix(bits, 16).unwrap_or(0)
    }

    /// Parse a hex-string like "0x123" into u32, returning 0 on failure.
    fn parse_hex_u32(value: &str) -> u32 {
        u32::try_from(Self::parse_hex_u64(value)).unwrap_or(0)
    }

    /// Map the type-class token emitted by the kernel (param_type_to_string)
    /// back to the numeric enum kapi_param_type.
    fn parse_type_class(token: &str) -> u32 {
        match token {
            "void" => 0,
            "int" => 1,
            "uint" => 2,
            "pointer" => 3,
            "struct" => 4,
            "union" => 5,
            "enum" => 6,
            "function_pointer" => 7,
            "array" => 8,
            "file_descriptor" => 9,
            "user_pointer" => 10,
            "pathname" => 11,
            "custom" => 12,
            _ => 0,
        }
    }

    /// Map the constraint-type token emitted by the kernel
    /// (constraint_type_to_string) back to the numeric enum
    /// kapi_constraint_type.
    fn parse_constraint_type(token: &str) -> u32 {
        match token {
            "none" => 0,
            "range" => 1,
            "mask" => 2,
            "enum" => 3,
            "alignment" => 4,
            "power_of_two" => 5,
            "page_aligned" => 6,
            "nonzero" => 7,
            "user_string" => 8,
            "user_path" => 9,
            "user_ptr" => 10,
            "buffer" => 11,
            "custom" => 12,
            _ => 0,
        }
    }

    /// Derive the API type from the symbol name, the same way the vmlinux
    /// extractor does.
    fn api_type_from_name(name: &str) -> &'static str {
        if name.starts_with("sys_") {
            "syscall"
        } else if name.ends_with("_ioctl") {
            "ioctl"
        } else if name.contains("sysfs") {
            "sysfs"
        } else {
            "function"
        }
    }

    /// Map the check-type token emitted by the kernel
    /// (return_check_type_to_string) back to the u32 enum the formatter wants.
    fn parse_check_type(token: &str) -> u32 {
        match token {
            "exact" => 0,
            "range" => 1,
            "error_check" => 2,
            "file_descriptor" => 3,
            "custom" => 4,
            "no_return" => 5,
            _ => 0,
        }
    }

    /// Map the lock-type token emitted by the kernel (lock_type_to_string).
    fn parse_lock_type(token: &str) -> u32 {
        match token {
            "none" => 0,
            "mutex" => 1,
            "spinlock" => 2,
            "rwlock" => 3,
            "seqlock" => 4,
            "rcu" => 5,
            "semaphore" => 6,
            "custom" => 7,
            _ => 0,
        }
    }

    /// Map the lock-scope token emitted by the kernel (lock_scope_to_string).
    fn parse_lock_scope(token: &str) -> u32 {
        match token {
            "internal" => 0,
            "acquires" => 1,
            "releases" => 2,
            "caller_held" => 3,
            _ => 0,
        }
    }

    /// Map the signal-timing token (e.g. "during") back to the u32 enum that
    /// the source-side parser produces. Mirrors
    /// KerneldocParser::parse_signal_timing in kerneldoc_parser.rs so the
    /// debugfs path and the source path agree.
    fn parse_signal_timing(token: &str) -> u32 {
        match token.trim().to_ascii_lowercase().as_str() {
            "before" => 0,
            "during" => 1,
            "after" => 2,
            _ => 0,
        }
    }

    /// Convert the capability action token emitted by the kernel
    /// (capability_action_to_string) to the KAPI_CAP_* spelling used by the
    /// other extractors.
    fn parse_capability_action(action: &str) -> String {
        match action {
            "bypass_check" => "KAPI_CAP_BYPASS_CHECK".to_string(),
            "increase_limit" => "KAPI_CAP_INCREASE_LIMIT".to_string(),
            "override_restriction" => "KAPI_CAP_OVERRIDE_RESTRICTION".to_string(),
            "grant_permission" => "KAPI_CAP_GRANT_PERMISSION".to_string(),
            "modify_behavior" => "KAPI_CAP_MODIFY_BEHAVIOR".to_string(),
            "access_resource" => "KAPI_CAP_ACCESS_RESOURCE".to_string(),
            "perform_operation" => "KAPI_CAP_PERFORM_OPERATION".to_string(),
            _ => action.to_string(),
        }
    }

    /// Try to parse as JSON first
    fn try_parse_json(&self, content: &str) -> Result<ApiSpec, serde_json::Error> {
        let json_data: KernelApiJson = serde_json::from_str(content)?;
        // The kernel-side kapi_json_str() emits NULL char * as the empty
        // string "", so normalise empty -> None to match the ApiSpec
        // convention used by --source / --vmlinux.
        fn opt_str(s: Option<String>) -> Option<String> {
            s.filter(|v| !v.is_empty())
        }
        let api_type = json_data
            .api_type
            .unwrap_or_else(|| Self::api_type_from_name(&json_data.name).to_string());

        let mut spec = ApiSpec {
            name: json_data.name,
            api_type,
            description: opt_str(json_data.description),
            long_description: opt_str(json_data.long_description),
            version: json_data.version.map(|v| v.to_string()),
            context_flags: json_data
                .context_flags
                .as_deref()
                .map_or_else(Vec::new, Self::parse_context_flags),
            param_count: None,
            error_count: None,
            examples: opt_str(json_data.examples),
            notes: opt_str(json_data.notes),
            subsystem: None,   // Not in the JSON format
            sysfs_path: None,  // Not in the JSON format
            permissions: None, // Not in the JSON format
            capabilities: vec![],
            parameters: vec![],
            return_spec: None,
            errors: vec![],
            signals: vec![],
            signal_masks: vec![],
            side_effects: vec![],
            state_transitions: vec![],
            constraints: vec![],
            locks: vec![],
            struct_specs: vec![],
        };

        // Convert capabilities
        if let Some(caps) = json_data.capabilities {
            for cap in caps {
                spec.capabilities.push(CapabilitySpec {
                    capability: cap.capability,
                    name: cap.name,
                    action: Self::parse_capability_action(&cap.action),
                    allows: cap.allows,
                    without_cap: cap.without_cap,
                    check_condition: opt_str(cap.check_condition),
                    priority: cap.priority,
                    alternatives: cap.alternatives.unwrap_or_default(),
                });
            }
        }

        // Convert parameters.
        if let Some(params) = json_data.parameters {
            for (i, p) in params.into_iter().enumerate() {
                let flags = p.flags.as_deref().map_or(0, Self::parse_hex_u32);
                let mut param = ParamSpec {
                    index: i as u32,
                    name: p.name,
                    type_name: p.type_name.unwrap_or_default(),
                    description: p.description.unwrap_or_default(),
                    flags,
                    param_type: p.type_class.as_deref().map_or(0, Self::parse_type_class),
                    constraint_type: p
                        .constraint_type
                        .as_deref()
                        .map_or(0, Self::parse_constraint_type),
                    constraint: opt_str(p.constraint_desc),
                    min_value: p.min_value,
                    max_value: p.max_value,
                    valid_mask: p.valid_mask.as_deref().map(Self::parse_hex_u64),
                    enum_values: p.enum_values.iter().map(i64::to_string).collect(),
                    size: p.size.map(|v| v as u32),
                    alignment: p.alignment.map(|v| v as u32),
                    size_param_idx: p.size_param_idx,
                };
                param.keep_used_numbers();
                spec.parameters.push(param);
            }
            if !spec.parameters.is_empty() {
                spec.param_count = Some(spec.parameters.len() as u32);
            }
        }

        // Convert errors
        if let Some(errors) = json_data.errors {
            for e in errors {
                spec.errors.push(ErrorSpec {
                    error_code: e.error_code,
                    name: e.name.unwrap_or_default(),
                    condition: e.condition.unwrap_or_default(),
                    description: e.description.unwrap_or_default(),
                });
            }
            if !spec.errors.is_empty() {
                spec.error_count = Some(spec.errors.len() as u32);
            }
        }

        // Convert return spec
        if let Some(ret) = json_data.return_spec {
            let check_type = ret.check_type.as_deref().map_or(0, Self::parse_check_type);
            let return_type = ret.type_class.as_deref().map_or(0, Self::parse_type_class);
            let type_name = ret.type_name.unwrap_or_default();
            let success_value = ret.success_value.unwrap_or(0);
            // A spec without a return block exports as an all-zero return
            // spec; report it as absent like the vmlinux extractor does.
            if !(type_name.is_empty() && return_type == 0 && check_type == 0 && success_value == 0)
            {
                let mut ret_spec = ReturnSpec {
                    type_name,
                    description: ret.description.unwrap_or_default(),
                    return_type,
                    check_type,
                    success_value: ret.success_value,
                    success_min: ret.success_min,
                    success_max: ret.success_max,
                    error_values: ret
                        .error_values
                        .into_iter()
                        .filter_map(|v| i32::try_from(v).ok())
                        .collect(),
                };
                ret_spec.keep_used_success_fields();
                spec.return_spec = Some(ret_spec);
            }
        }

        // Convert locks
        if let Some(locks) = json_data.locks {
            for l in locks {
                let lock_type = l.lock_type.as_deref().map_or(0, Self::parse_lock_type);
                let scope = l.scope.as_deref().map_or(0, Self::parse_lock_scope);
                spec.locks.push(LockSpec {
                    lock_name: l.name,
                    lock_type,
                    scope,
                    description: l.description.unwrap_or_default(),
                });
            }
        }

        // Convert constraints.  Empty strings emitted from kapi_json_str()
        // for NULL char * fields normalise back to None to match --source.
        if let Some(constraints) = json_data.constraints {
            for c in constraints {
                spec.constraints.push(ConstraintSpec {
                    name: c.name,
                    description: c.description.unwrap_or_default(),
                    expression: c.expression.filter(|v| !v.is_empty()),
                });
            }
        }

        // Convert signals.
        if let Some(signals) = json_data.signals {
            for s in signals {
                let direction = s.direction.as_deref().map_or(0, Self::parse_hex_u32);
                let sa_flags_required = s
                    .sa_flags_required
                    .as_deref()
                    .map_or(0, Self::parse_hex_u32);
                let sa_flags_forbidden = s
                    .sa_flags_forbidden
                    .as_deref()
                    .map_or(0, Self::parse_hex_u32);
                let state_required = s.state_required.as_deref().map_or(0, Self::parse_hex_u32);
                let state_forbidden = s.state_forbidden.as_deref().map_or(0, Self::parse_hex_u32);
                let timing = s.timing.as_deref().map_or(0, Self::parse_signal_timing);
                spec.signals.push(super::SignalSpec {
                    signal_num: s.signal_num,
                    signal_name: s.signal_name.unwrap_or_default(),
                    direction,
                    action: s.action,
                    target: opt_str(s.target),
                    condition: opt_str(s.condition),
                    description: opt_str(s.description),
                    timing,
                    priority: s.priority,
                    restartable: s.restartable,
                    interruptible: s.interruptible,
                    queue: opt_str(s.queue_behavior),
                    sa_flags: 0,
                    sa_flags_required,
                    sa_flags_forbidden,
                    state_required,
                    state_forbidden,
                    error_on_signal: if s.error_on_signal != 0 {
                        Some(s.error_on_signal)
                    } else {
                        None
                    },
                    transform_to: if s.transform_to != 0 {
                        // Kernel JSON already carries the numeric value.
                        Some(s.transform_to)
                    } else {
                        None
                    },
                });
            }
        }

        // Convert side effects.
        if let Some(effects) = json_data.side_effects {
            for e in effects {
                let effect_type = e.type_hex.as_deref().map_or(0, Self::parse_hex_u32);
                spec.side_effects.push(super::SideEffectSpec {
                    effect_type,
                    target: e.target.unwrap_or_default(),
                    condition: e.condition.filter(|v| !v.is_empty()),
                    description: e.description.unwrap_or_default(),
                    reversible: e.reversible,
                });
            }
        }

        if let Some(transitions) = json_data.state_transitions {
            for t in transitions {
                spec.state_transitions.push(StateTransitionSpec {
                    object: t.object.unwrap_or_default(),
                    from_state: t.from_state.unwrap_or_default(),
                    to_state: t.to_state.unwrap_or_default(),
                    condition: opt_str(t.condition),
                    description: t.description.unwrap_or_default(),
                });
            }
        }

        if let Some(masks) = json_data.signal_masks {
            for m in masks {
                spec.signal_masks.push(SignalMaskSpec {
                    name: m.name.unwrap_or_default(),
                    description: m.description.unwrap_or_default(),
                    signals: m.signals,
                });
            }
        }

        if let Some(structs) = json_data.struct_specs {
            for st in structs {
                let fields: Vec<StructFieldSpec> = st
                    .fields
                    .into_iter()
                    .map(|f| StructFieldSpec {
                        name: f.name.unwrap_or_default(),
                        field_type: f.type_class.as_deref().map_or(0, Self::parse_type_class),
                        type_name: f.type_name.unwrap_or_default(),
                        offset: f.offset,
                        size: f.size,
                        flags: f.flags.as_deref().map_or(0, Self::parse_hex_u32),
                        constraint_type: f
                            .constraint_type
                            .as_deref()
                            .map_or(0, Self::parse_constraint_type),
                        min_value: f.min_value,
                        max_value: f.max_value,
                        valid_mask: f.valid_mask.as_deref().map_or(0, Self::parse_hex_u64),
                        description: f.description.unwrap_or_default(),
                    })
                    .collect();
                spec.struct_specs.push(StructSpec {
                    name: st.name.unwrap_or_default(),
                    size: st.size,
                    alignment: st.alignment,
                    field_count: fields.len() as u32,
                    fields,
                    description: st.description.unwrap_or_default(),
                });
            }
        }

        Ok(spec)
    }

    /// Parse a single API specification file
    fn parse_spec_file(&self, api_name: &str) -> Result<ApiSpec> {
        // Prefer the JSON endpoint; fall back to the plain-text dump under
        // kapi/specs/ if it is missing or does not parse.
        let json_path = self
            .debugfs_path
            .join(format!("kapi/specs-json/{}", api_name));
        match fs::read_to_string(&json_path) {
            Ok(content) => match self.try_parse_json(&content) {
                Ok(spec) => return Ok(spec),
                Err(e) => eprintln!(
                    "Warning: invalid JSON in {}: {}; using the text dump",
                    json_path.display(),
                    e
                ),
            },
            Err(e) if e.kind() != std::io::ErrorKind::NotFound => eprintln!(
                "Warning: cannot read {}: {}; using the text dump",
                json_path.display(),
                e
            ),
            Err(_) => {}
        }

        let spec_path = self.debugfs_path.join(format!("kapi/specs/{}", api_name));
        let content = fs::read_to_string(&spec_path)
            .with_context(|| format!("Failed to read {}", spec_path.display()))?;

        // The specs/ file may hold JSON as well.
        if let Ok(spec) = self.try_parse_json(&content) {
            return Ok(spec);
        }

        // Fall back to plain text parsing
        let mut spec = ApiSpec {
            name: api_name.to_string(),
            api_type: "unknown".to_string(),
            description: None,
            long_description: None,
            version: None,
            context_flags: Vec::new(),
            param_count: None,
            error_count: None,
            examples: None,
            notes: None,
            subsystem: None,
            sysfs_path: None,
            permissions: None,
            capabilities: vec![],
            parameters: vec![],
            return_spec: None,
            errors: vec![],
            signals: vec![],
            signal_masks: vec![],
            side_effects: vec![],
            state_transitions: vec![],
            constraints: vec![],
            locks: vec![],
            struct_specs: vec![],
        };

        // Parse the content
        let mut text_field: Option<TextField> = None;
        let mut text_lines: Vec<String> = Vec::new();
        let mut parsing_capability = false;
        let mut in_capabilities_section = false;
        let mut current_capability: Option<CapabilitySpec> = None;

        for line in content.lines() {
            // The kernel indents every line of a multi-line value, so a value
            // ends at the first non-blank line that is not indented.
            if let Some(field) = text_field {
                if line.is_empty() {
                    text_lines.push(String::new());
                    continue;
                }
                if let Some(rest) = line.strip_prefix("  ") {
                    text_lines.push(rest.to_string());
                    continue;
                }
                field.store(&mut spec, &text_lines);
                text_field = None;
            }

            // Handle capability sections
            if line.starts_with("Capabilities (") {
                in_capabilities_section = true;
                continue;
            }
            // Any other top-level section header ends the capabilities section
            // so that "  pending_signals (0):" inside "Signal handling (1):"
            // isn't mis-parsed as a capability entry.
            if !line.starts_with(' ') && !line.is_empty() && line.ends_with(':') {
                in_capabilities_section = false;
            }
            if in_capabilities_section
                && line.starts_with("  ")
                && line.contains(" (")
                && line.ends_with("):")
            {
                // Start of a capability entry like "  CAP_IPC_LOCK (14):"
                if let Some(cap) = current_capability.take() {
                    spec.capabilities.push(cap);
                }

                let parts: Vec<&str> = line.trim().split(" (").collect();
                if parts.len() == 2 {
                    let cap_name = parts[0].to_string();
                    let cap_id = parts[1].trim_end_matches("):").parse().unwrap_or(0);
                    current_capability = Some(CapabilitySpec {
                        capability: cap_id,
                        name: cap_name,
                        action: String::new(),
                        allows: String::new(),
                        without_cap: String::new(),
                        check_condition: None,
                        priority: None,
                        alternatives: Vec::new(),
                    });
                    parsing_capability = true;
                }
                continue;
            }
            if parsing_capability && line.starts_with("    ") {
                // Parse capability fields
                if let Some(ref mut cap) = current_capability {
                    if let Some(action) = line.strip_prefix("    Action: ") {
                        cap.action = action.to_string();
                    } else if let Some(allows) = line.strip_prefix("    Allows: ") {
                        cap.allows = allows.to_string();
                    } else if let Some(without) = line.strip_prefix("    Without: ") {
                        cap.without_cap = without.to_string();
                    } else if let Some(cond) = line.strip_prefix("    Condition: ") {
                        cap.check_condition = Some(cond.to_string());
                    } else if let Some(prio) = line.strip_prefix("    Priority: ") {
                        cap.priority = prio.parse().ok();
                    } else if let Some(alts) = line.strip_prefix("    Alternatives: ") {
                        cap.alternatives =
                            alts.split(", ").filter_map(|s| s.parse().ok()).collect();
                    }
                }
                continue;
            }
            if parsing_capability && !line.starts_with("  ") {
                // End of capabilities section
                if let Some(cap) = current_capability.take() {
                    spec.capabilities.push(cap);
                }
                parsing_capability = false;
            }

            // Handle section headers
            if line.starts_with("Parameters (") {
                if let Some(count_str) = line
                    .strip_prefix("Parameters (")
                    .and_then(|s| s.strip_suffix("):"))
                {
                    spec.param_count = count_str.parse().ok();
                }
                continue;
            } else if line.starts_with("Errors (") {
                if let Some(count_str) = line
                    .strip_prefix("Errors (")
                    .and_then(|s| s.strip_suffix("):"))
                {
                    spec.error_count = count_str.parse().ok();
                }
                continue;
            }

            // Parse regular fields
            if let Some((field, first)) = TextField::start(line) {
                text_field = Some(field);
                text_lines.clear();
                if !first.is_empty() {
                    text_lines.push(first.to_string());
                }
            } else if let Some(version) = line.strip_prefix("Version: ") {
                spec.version = Some(version.to_string());
            } else if let Some(flags) = line.strip_prefix("Context flags: ") {
                spec.context_flags = flags
                    .split_whitespace()
                    .map(|f| format!("KAPI_CTX_{f}"))
                    .collect();
            } else if let Some(subsys) = line.strip_prefix("Subsystem: ") {
                spec.subsystem = Some(subsys.to_string());
            } else if let Some(path) = line.strip_prefix("Sysfs Path: ") {
                spec.sysfs_path = Some(path.to_string());
            } else if let Some(perms) = line.strip_prefix("Permissions: ") {
                spec.permissions = Some(perms.to_string());
            }
        }

        if let Some(field) = text_field {
            field.store(&mut spec, &text_lines);
        }

        // Handle any remaining capability
        if let Some(cap) = current_capability.take() {
            spec.capabilities.push(cap);
        }

        // Determine API type based on name
        if api_name.starts_with("sys_") {
            spec.api_type = "syscall".to_string();
        } else if api_name.contains("_ioctl") || api_name.starts_with("ioctl_") {
            spec.api_type = "ioctl".to_string();
        } else if api_name.contains("sysfs")
            || api_name.ends_with("_show")
            || api_name.ends_with("_store")
        {
            spec.api_type = "sysfs".to_string();
        } else {
            spec.api_type = "function".to_string();
        }

        Ok(spec)
    }
}

impl ApiExtractor for DebugfsExtractor {
    fn extract_all(&self) -> Result<Vec<ApiSpec>> {
        let api_names = self.parse_list_file()?;
        let mut specs = Vec::new();

        for name in api_names {
            match self.parse_spec_file(&name) {
                Ok(spec) => specs.push(spec),
                Err(e) => {
                    eprintln!("Warning: failed to parse API spec '{}': {}", name, e);
                }
            }
        }

        Ok(specs)
    }

    fn extract_by_name(&self, name: &str) -> Result<Option<ApiSpec>> {
        let api_names = self.parse_list_file()?;

        if api_names.contains(&name.to_string()) {
            Ok(Some(self.parse_spec_file(name)?))
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
        if let Some(spec) = self.extract_by_name(api_name)? {
            display_api_spec(&spec, formatter, writer)?;
        } else {
            writeln!(writer, "API '{api_name}' not found in debugfs")?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Shortened sys_read dump as emitted by kapi_export_json().
    const SYS_READ_JSON: &str = r#"{
  "name": "sys_read",
  "version": 1,
  "description": "Read data from a file descriptor",
  "long_description": "",
  "context_flags": "0x21",
  "parameters": [
    {
      "name": "fd",
      "type": "unsigned int fd",
      "type_class": "file_descriptor",
      "flags": "0x1",
      "description": "File descriptor to read from ",
      "constraint_type": "range",
      "constraint_desc": "Must be a valid, open file descriptor",
      "min_value": 0,
      "max_value": 2147483647,
      "valid_mask": "0x0",
      "enum_values": [],
      "size": 0,
      "alignment": 0,
      "size_param_idx": null,
      "size_multiplier": 0
    },
    {
      "name": "buf",
      "type": "char __user * buf",
      "type_class": "user_pointer",
      "flags": "0x42",
      "description": "User-space buffer to read data into ",
      "constraint_type": "buffer",
      "constraint_desc": "Must point to a writable user-space region",
      "min_value": 0,
      "max_value": 0,
      "valid_mask": "0x0",
      "enum_values": [],
      "size": 0,
      "alignment": 0,
      "size_param_idx": 2,
      "size_multiplier": 0
    },
    {
      "name": "count",
      "type": "size_t count",
      "type_class": "uint",
      "flags": "0x1",
      "description": "Maximum number of bytes to read ",
      "constraint_type": "none",
      "constraint_desc": "",
      "min_value": 0,
      "max_value": 0,
      "valid_mask": "0x0",
      "enum_values": [],
      "size": 0,
      "alignment": 0,
      "size_param_idx": null,
      "size_multiplier": 0
    }
  ],
  "return": {
    "type": "KAPI_TYPE_INT",
    "type_class": "int",
    "check_type": "range",
    "success_value": 0,
    "success_min": 0,
    "success_max": 9223372036854775807,
    "error_values": [],
    "description": "Number of bytes read"
  },
  "errors": [
    { "code": -9, "name": "EBADF", "condition": "Bad file descriptor",
      "description": "fd is not valid" }
  ],
  "locks": [
    { "name": "file->f_pos_lock", "type": "mutex", "scope": "internal",
      "description": "Position lock" }
  ],
  "capabilities": [
    {
      "capability": 1,
      "name": "CAP_DAC_OVERRIDE",
      "action": "bypass_check",
      "allows": "Bypass read permission checks",
      "without_cap": "Standard DAC checks are enforced",
      "check_condition": "",
      "priority": 0
    }
  ],
  "constraints": [
    { "name": "MAX_RW_COUNT",
      "description": "Count is clamped", "expression": "min(count, MAX_RW_COUNT)" }
  ],
  "signals": [
    {
      "signal_num": 0,
      "signal_name": "Any signal",
      "direction": "0x1",
      "action": 6,
      "target": "",
      "condition": "When blocked waiting for data",
      "description": "May be interrupted",
      "restartable": true,
      "sa_flags_required": "0x0",
      "sa_flags_forbidden": "0x0",
      "error_on_signal": -4,
      "transform_to": 0,
      "timing": "during",
      "priority": 0,
      "interruptible": false,
      "queue_behavior": "",
      "state_required": "0x0",
      "state_forbidden": "0x0"
    }
  ],
  "side_effects": [
    { "type": "0x10", "target": "file->f_pos", "condition": "",
      "description": "Offset advances", "reversible": false }
  ],
  "state_transitions": [],
  "signal_masks": [],
  "struct_specs": [],
  "examples": "",
  "notes": ""
}
"#;

    // Synthetic dump exercising the fields sys_read leaves empty: enum
    // values, error values, struct specs, signal masks, state transitions.
    const FULL_JSON: &str = r#"{
  "name": "tmp_full",
  "version": 1,
  "description": "Full \"feature\" test\tline\nbreak\u0001",
  "long_description": "",
  "context_flags": "0x91",
  "parameters": [
    {
      "name": "mode",
      "type": "int mode",
      "type_class": "enum",
      "flags": "0x1",
      "description": "Enumerated parameter",
      "constraint_type": "enum",
      "constraint_desc": "",
      "min_value": 0,
      "max_value": 0,
      "valid_mask": "0x0",
      "enum_values": [1, 2, -3, 1000000000000],
      "size": 4096,
      "alignment": 64,
      "size_param_idx": null,
      "size_multiplier": 0
    },
    {
      "name": "bits",
      "type": "u64 bits",
      "type_class": "uint",
      "flags": "0x1",
      "description": "Masked parameter",
      "constraint_type": "mask",
      "constraint_desc": "",
      "min_value": 0,
      "max_value": 0,
      "valid_mask": "0xffffffffffffffff",
      "enum_values": [],
      "size": 0,
      "alignment": 0,
      "size_param_idx": null,
      "size_multiplier": 0
    }
  ],
  "return": {
    "type": "long",
    "type_class": "int",
    "check_type": "error_check",
    "success_value": 0,
    "success_min": 0,
    "success_max": 0,
    "error_values": [-1, -22, -4095],
    "description": "Error-checked return"
  },
  "errors": [],
  "locks": [
    { "name": "tmp_lock", "type": "seqlock", "scope": "acquires", "description": "a seqlock" }
  ],
  "capabilities": [
    {
      "capability": 21,
      "name": "CAP_SYS_ADMIN",
      "action": "perform_operation",
      "allows": "do it",
      "without_cap": "cannot",
      "check_condition": "always checked",
      "priority": 3,
      "alternatives": [21, 17]
    }
  ],
  "constraints": [],
  "signals": [],
  "side_effects": [],
  "state_transitions": [
    { "object": "obj", "from_state": "a", "to_state": "b", "condition": "when asked",
      "description": "moves a to b" }
  ],
  "signal_masks": [
    { "name": "blocked", "description": "Blocked while running", "signals": [2, 15, 3] }
  ],
  "struct_specs": [
    {
      "name": "tmp_struct",
      "size": 16,
      "alignment": 8,
      "description": "Test structure",
      "fields": [
        {
          "name": "flags",
          "type": "u32",
          "type_class": "uint",
          "offset": 0,
          "size": 4,
          "flags": "0x1",
          "constraint_type": "mask",
          "min_value": 0,
          "max_value": 0,
          "valid_mask": "0xff",
          "enum_values": "",
          "description": "Flag bits"
        },
        {
          "name": "kind",
          "type": "enum k",
          "type_class": "enum",
          "offset": 8,
          "size": 4,
          "flags": "0x0",
          "constraint_type": "range",
          "min_value": -5,
          "max_value": 5,
          "valid_mask": "0x0",
          "enum_values": "",
          "description": "Kind"
        }
      ]
    }
  ],
  "examples": "",
  "notes": "note"
}
"#;

    // Dump without the optional per-parameter constraint fields.
    const MINIMAL_JSON: &str = r#"{
  "name": "sys_read",
  "version": 1,
  "description": "Read data from a file descriptor",
  "long_description": "Reads.",
  "context_flags": "0x21",
  "parameters": [
    {
      "name": "buf",
      "type": "char __user * buf",
      "type_class": "user_pointer",
      "flags": "0x42",
      "description": "User-space buffer to read data into "
    }
  ],
  "return": {
    "type": "KAPI_TYPE_INT",
    "type_class": "int",
    "check_type": "range",
    "success_min": 0,
    "success_max": 9223372036854775807,
    "description": "Bytes read"
  },
  "errors": [],
  "locks": [],
  "capabilities": [
    {
      "capability": 1,
      "name": "CAP_DAC_OVERRIDE",
      "action": "bypass_check",
      "allows": "a",
      "without_cap": "b",
      "check_condition": "c",
      "priority": 0
    }
  ],
  "constraints": [],
  "signals": [],
  "side_effects": [],
  "examples": "",
  "notes": ""
}
"#;

    fn extractor() -> DebugfsExtractor {
        DebugfsExtractor {
            debugfs_path: PathBuf::new(),
        }
    }

    #[test]
    fn json_sys_read_carries_type_and_constraint_data() {
        let spec = extractor().try_parse_json(SYS_READ_JSON).unwrap();

        assert_eq!(spec.api_type, "syscall");
        assert_eq!(spec.version.as_deref(), Some("1"));
        assert_eq!(spec.long_description, None);
        assert_eq!(
            spec.context_flags,
            ["KAPI_CTX_PROCESS", "KAPI_CTX_SLEEPABLE"]
        );
        assert_eq!(spec.param_count, Some(3));

        let fd = &spec.parameters[0];
        assert_eq!((fd.param_type, fd.constraint_type), (9, 1));
        assert_eq!((fd.min_value, fd.max_value), (Some(0), Some(2147483647)));
        assert_eq!(fd.valid_mask, None);
        assert_eq!(fd.size_param_idx, None);
        assert_eq!(
            fd.constraint.as_deref(),
            Some("Must be a valid, open file descriptor")
        );

        let buf = &spec.parameters[1];
        assert_eq!((buf.param_type, buf.constraint_type), (10, 11));
        assert_eq!(buf.flags, 0x42);
        assert_eq!(buf.size_param_idx, Some(2));
        assert_eq!(
            (buf.min_value, buf.max_value, buf.valid_mask),
            (None, None, None)
        );
        assert_eq!((buf.size, buf.alignment), (None, None));
        assert_eq!(
            spec.parameters[buf.size_param_idx.unwrap() as usize].name,
            "count"
        );

        let count = &spec.parameters[2];
        assert_eq!((count.param_type, count.constraint_type), (2, 0));
        assert_eq!(count.constraint, None);
        assert_eq!(count.description, "Maximum number of bytes to read ");

        let ret = spec.return_spec.as_ref().unwrap();
        assert_eq!((ret.return_type, ret.check_type), (1, 1));
        assert_eq!(ret.success_value, None);
        assert_eq!(ret.success_min, Some(0));
        assert_eq!(ret.success_max, Some(i64::MAX));
        assert!(ret.error_values.is_empty());

        assert_eq!(spec.errors[0].error_code, -9);
        assert_eq!((spec.locks[0].lock_type, spec.locks[0].scope), (1, 0));
        assert_eq!(spec.capabilities[0].action, "KAPI_CAP_BYPASS_CHECK");
        assert_eq!(spec.capabilities[0].check_condition, None);
        assert_eq!(spec.signals[0].timing, 1);
        assert_eq!(spec.signals[0].error_on_signal, Some(-4));
        assert_eq!(spec.signals[0].target, None);
        assert_eq!(spec.side_effects[0].effect_type, 0x10);
        assert_eq!(spec.side_effects[0].condition, None);
        assert_eq!(spec.constraints.len(), 1);
    }

    #[test]
    fn json_full_carries_enum_error_struct_and_transition_data() {
        let spec = extractor().try_parse_json(FULL_JSON).unwrap();

        assert_eq!(
            spec.description.as_deref(),
            Some("Full \"feature\" test\tline\nbreak\u{1}")
        );
        assert_eq!(
            spec.context_flags,
            [
                "KAPI_CTX_PROCESS",
                "KAPI_CTX_ATOMIC",
                "KAPI_CTX_IRQ_DISABLED"
            ]
        );

        let mode = &spec.parameters[0];
        assert_eq!((mode.param_type, mode.constraint_type), (6, 3));
        assert_eq!(mode.enum_values, ["1", "2", "-3", "1000000000000"]);
        assert_eq!((mode.size, mode.alignment), (Some(4096), Some(64)));
        assert_eq!(
            (mode.min_value, mode.max_value, mode.valid_mask),
            (None, None, None)
        );

        let bits = &spec.parameters[1];
        assert_eq!(bits.constraint_type, 2);
        assert_eq!(bits.valid_mask, Some(u64::MAX));
        assert_eq!((bits.min_value, bits.max_value), (None, None));
        assert_eq!((bits.size, bits.alignment), (None, None));

        let ret = spec.return_spec.as_ref().unwrap();
        assert_eq!(ret.check_type, 2);
        assert_eq!(ret.error_values, [-1, -22, -4095]);

        assert_eq!(spec.capabilities[0].alternatives, [21, 17]);
        assert_eq!(spec.capabilities[0].priority, Some(3));
        assert_eq!((spec.locks[0].lock_type, spec.locks[0].scope), (4, 1));

        let trans = &spec.state_transitions[0];
        assert_eq!(
            (trans.object.as_str(), trans.from_state.as_str()),
            ("obj", "a")
        );
        assert_eq!(trans.to_state, "b");
        assert_eq!(trans.condition.as_deref(), Some("when asked"));

        assert_eq!(spec.signal_masks[0].name, "blocked");
        assert_eq!(spec.signal_masks[0].signals, [2, 15, 3]);

        let st = &spec.struct_specs[0];
        assert_eq!(
            (st.name.as_str(), st.size, st.alignment),
            ("tmp_struct", 16, 8)
        );
        assert_eq!(st.field_count, 2);
        assert_eq!(st.description, "Test structure");
        assert_eq!(
            (st.fields[0].field_type, st.fields[0].constraint_type),
            (2, 2)
        );
        assert_eq!(st.fields[0].valid_mask, 0xff);
        assert_eq!(st.fields[1].offset, 8);
        assert_eq!((st.fields[1].min_value, st.fields[1].max_value), (-5, 5));
    }

    #[test]
    fn json_without_constraint_fields_still_parses() {
        let spec = extractor().try_parse_json(MINIMAL_JSON).unwrap();

        let buf = &spec.parameters[0];
        assert_eq!(buf.param_type, 10);
        assert_eq!(buf.constraint_type, 0);
        assert_eq!(buf.min_value, None);
        assert_eq!(buf.valid_mask, None);
        assert_eq!(buf.size_param_idx, None);
        assert!(buf.enum_values.is_empty());

        let ret = spec.return_spec.as_ref().unwrap();
        assert_eq!((ret.return_type, ret.check_type), (1, 1));
        assert_eq!(ret.success_value, None);
        assert_eq!(ret.success_max, Some(i64::MAX));
        assert_eq!(spec.capabilities[0].action, "KAPI_CAP_BYPASS_CHECK");
        assert!(spec.state_transitions.is_empty());
        assert!(spec.struct_specs.is_empty());
    }

    #[test]
    fn enum_tokens_match_kernel_numbering() {
        let classes = [
            "void",
            "int",
            "uint",
            "pointer",
            "struct",
            "union",
            "enum",
            "function_pointer",
            "array",
            "file_descriptor",
            "user_pointer",
            "pathname",
            "custom",
        ];
        for (n, token) in classes.iter().enumerate() {
            assert_eq!(
                DebugfsExtractor::parse_type_class(token),
                n as u32,
                "{token}"
            );
        }

        let constraints = [
            "none",
            "range",
            "mask",
            "enum",
            "alignment",
            "power_of_two",
            "page_aligned",
            "nonzero",
            "user_string",
            "user_path",
            "user_ptr",
            "buffer",
            "custom",
        ];
        for (n, token) in constraints.iter().enumerate() {
            assert_eq!(
                DebugfsExtractor::parse_constraint_type(token),
                n as u32,
                "{token}"
            );
        }
    }

    #[test]
    fn api_type_follows_symbol_name() {
        assert_eq!(DebugfsExtractor::api_type_from_name("sys_read"), "syscall");
        assert_eq!(DebugfsExtractor::api_type_from_name("foo_ioctl"), "ioctl");
        assert_eq!(DebugfsExtractor::api_type_from_name("kmalloc"), "function");
    }

    #[test]
    fn truncated_json_is_rejected() {
        let cut = &SYS_READ_JSON[..SYS_READ_JSON.len() / 2];
        assert!(extractor().try_parse_json(cut).is_err());
    }

    fn debugfs_with(json: &str, text: &str) -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        let kapi = dir.path().join("kapi");
        fs::create_dir_all(kapi.join("specs")).unwrap();
        fs::create_dir_all(kapi.join("specs-json")).unwrap();
        fs::write(
            kapi.join("list"),
            "Available Kernel API Specifications\n\
             ===================================\n\n\
             sys_read - Read data from a file descriptor\n\n\
             Total: 1 specifications\n",
        )
        .unwrap();
        fs::write(kapi.join("specs-json/sys_read"), json).unwrap();
        fs::write(kapi.join("specs/sys_read"), text).unwrap();
        dir
    }

    #[test]
    fn reads_json_endpoint_from_debugfs_tree() {
        let dir = debugfs_with(SYS_READ_JSON, "Name: sys_read\n");
        let ex = DebugfsExtractor::new(Some(dir.path().to_string_lossy().into_owned())).unwrap();
        let spec = ex.extract_by_name("sys_read").unwrap().unwrap();

        assert_eq!(spec.parameters[1].constraint_type, 11);
        assert_eq!(spec.parameters[1].size_param_idx, Some(2));
    }

    #[test]
    fn truncated_json_endpoint_falls_back_to_text_dump() {
        let text = "Name: sys_read\n\
                    Version: 1\n\
                    Description: Read data from a file descriptor\n\
                    Context flags: PROCESS SLEEPABLE \n";
        let dir = debugfs_with(&SYS_READ_JSON[..SYS_READ_JSON.len() / 2], text);
        let ex = DebugfsExtractor::new(Some(dir.path().to_string_lossy().into_owned())).unwrap();
        let spec = ex.extract_by_name("sys_read").unwrap().unwrap();

        assert!(spec.parameters.is_empty());
        assert_eq!(
            spec.context_flags,
            ["KAPI_CTX_PROCESS", "KAPI_CTX_SLEEPABLE"]
        );
    }

    #[test]
    fn text_dump_multi_line_values_stay_in_their_field() {
        let text = "Kernel API Specification\n\
                    ========================\n\n\
                    Name: sys_read\n\
                    Version: 1\n\
                    Description: Read data from a file descriptor\n\
                    Long description:\n  \
                    First paragraph.\n\n  \
                    Notes: not a header\n  \
                    Version: 9\n\
                    Context flags: PROCESS SLEEPABLE \n\n\
                    Parameters (1):\n  \
                    [0] fd:\n    \
                    description: File descriptor\n\n\
                    Examples:\n  \
                    read(fd, buf, n);\n  \
                    close(fd);\n\n\
                    Notes:\n  \
                    one\n\n  \
                    two\n\n";
        let dir = debugfs_with("", text);
        let ex = DebugfsExtractor::new(Some(dir.path().to_string_lossy().into_owned())).unwrap();
        let spec = ex.extract_by_name("sys_read").unwrap().unwrap();

        assert_eq!(spec.version.as_deref(), Some("1"));
        assert_eq!(
            spec.description.as_deref(),
            Some("Read data from a file descriptor")
        );
        assert_eq!(
            spec.long_description.as_deref(),
            Some("First paragraph.\n\nNotes: not a header\nVersion: 9")
        );
        assert_eq!(
            spec.context_flags,
            ["KAPI_CTX_PROCESS", "KAPI_CTX_SLEEPABLE"]
        );
        assert_eq!(spec.param_count, Some(1));
        assert_eq!(
            spec.examples.as_deref(),
            Some("read(fd, buf, n);\nclose(fd);")
        );
        assert_eq!(spec.notes.as_deref(), Some("one\n\ntwo"));
    }
}
