// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

use super::{
    ApiSpec, CapabilitySpec, ConstraintSpec, ErrorSpec, LockSpec, ParamSpec, ReturnSpec,
    SideEffectSpec, SignalSpec, StateTransitionSpec,
};
use anyhow::Result;
use std::collections::HashMap;

/// Kerneldoc parser that extracts KAPI annotations
pub struct KerneldocParser;

/// What block are we currently inside?
#[derive(Debug, Clone, PartialEq)]
enum BlockContext {
    None,
    Param(String),   // param: <name>
    Error(String),   // error: <name>
    Signal,          // signal: <name>
    Capability,      // capability: <name>
    SideEffect,      // side-effect: <type>
    StateTransition, // state-trans: ...
    Constraint,      // constraint: <name>
    Lock,            // lock: <name>
    Return,          // return:
}

/// Evaluate an integer literal exactly as the compiler does on LP64 (int is
/// 32 bits, long and long long are 64): decimal, 0x/0X hex, 0b/0B binary or
/// leading-0 octal, an optional u/U/l/L suffix and an optional leading '-'.
/// The literal's C type (C11 6.4.4.1) decides how '-' wraps, so `-1U` is
/// 4294967295 and `-0x80000000` is 2147483648. The result is the 64-bit
/// two's complement pattern the value has once stored into an s64/u64
/// field. Returns `None` for anything that requires cpp-level constant
/// resolution (e.g. symbolic masks like `O_RDONLY | O_WRONLY`). Callers must
/// treat that case as "value unknown" and leave the downstream slot unset,
/// not store it as 0, which would wrongly assert that zero bits are valid.
fn eval_c_int_literal(s: &str) -> Option<u64> {
    let t = s.trim();
    let (neg, t) = match t.strip_prefix('-') {
        Some(rest) => (true, rest.trim_start()),
        None => (false, t),
    };
    let body = t.trim_end_matches(['u', 'U', 'l', 'L']);
    let suffix = &t[body.len()..];
    let unsigned = match suffix.matches(['u', 'U']).count() {
        0 => false,
        1 => true,
        _ => return None,
    };
    let longs = suffix.matches(['l', 'L']).count();
    if longs > 2 {
        return None;
    }

    let (digits, radix) =
        if let Some(h) = body.strip_prefix("0x").or_else(|| body.strip_prefix("0X")) {
            (h, 16)
        } else if let Some(b) = body.strip_prefix("0b").or_else(|| body.strip_prefix("0B")) {
            (b, 2)
        } else if body.len() > 1 && body.starts_with('0') {
            (&body[1..], 8)
        } else {
            (body, 10)
        };
    if digits.is_empty() || !digits.chars().all(|c| c.is_digit(radix)) {
        return None;
    }
    let v = u64::from_str_radix(digits, radix).ok()?;

    // Decimal literals without 'u' only take signed types; GCC treats one
    // too large for long long as unsigned.
    let value = if longs == 0 && !unsigned && v <= i32::MAX as u64 {
        let x = v as i32;
        (if neg { x.wrapping_neg() } else { x }) as i64 as u64
    } else if longs == 0 && (unsigned || radix != 10) && v <= u32::MAX as u64 {
        let x = v as u32;
        u64::from(if neg { x.wrapping_neg() } else { x })
    } else if !unsigned && v <= i64::MAX as u64 {
        let x = v as i64;
        (if neg { x.wrapping_neg() } else { x }) as u64
    } else if neg {
        v.wrapping_neg()
    } else {
        v
    };
    Some(value)
}

fn parse_u64_literal(s: &str) -> Option<u64> {
    eval_c_int_literal(s)
}

fn parse_i64_literal(s: &str) -> Option<i64> {
    eval_c_int_literal(s).map(|v| v as i64)
}

/// Split a return `success:` value into (exact value, range minimum,
/// range maximum). `>= N` is the range [N, i64::MAX]; `N`, `= N` and
/// `== N` are an exact value, and any value that is not `>= N` leaves
/// the default range [0, i64::MAX] a range check falls back to. Mirrors
/// `_return_success_macro()` in kdoc_apispec.py.
fn parse_return_success(text: &str) -> (Option<i64>, Option<i64>, Option<i64>) {
    let t = text.trim();
    if let Some(min) = t.strip_prefix(">=") {
        return (
            None,
            Some(parse_i64_literal(min).unwrap_or(0)),
            Some(i64::MAX),
        );
    }
    let exact = t
        .strip_prefix("==")
        .or_else(|| t.strip_prefix('='))
        .unwrap_or(t);
    (parse_i64_literal(exact), Some(0), Some(i64::MAX))
}

/// Canonicalise a capability `type:` value to its KAPI_CAP_* spelling.
fn canon_kapi_cap_action(s: &str) -> String {
    let t = s.trim();
    if t.starts_with("KAPI_CAP_") {
        return t.to_string();
    }
    match t.to_ascii_lowercase().as_str() {
        "bypass_check" => "KAPI_CAP_BYPASS_CHECK".to_string(),
        "increase_limit" => "KAPI_CAP_INCREASE_LIMIT".to_string(),
        "override_restriction" => "KAPI_CAP_OVERRIDE_RESTRICTION".to_string(),
        "grant_permission" => "KAPI_CAP_GRANT_PERMISSION".to_string(),
        "modify_behavior" => "KAPI_CAP_MODIFY_BEHAVIOR".to_string(),
        "access_resource" => "KAPI_CAP_ACCESS_RESOURCE".to_string(),
        "perform_operation" => "KAPI_CAP_PERFORM_OPERATION".to_string(),
        _ => t.to_string(),
    }
}

/// Types whose semantics imply `KAPI_PARAM_USER` on the param, so
/// `type: user_ptr, input` doesn't need a separate `user` flag.
fn type_implies_user_flag(tok: &str) -> bool {
    matches!(tok.trim(), "KAPI_TYPE_USER_PTR" | "KAPI_TYPE_PATH")
        || matches!(
            tok.trim().to_ascii_lowercase().as_str(),
            "user_ptr" | "uptr" | "path"
        )
}

/// Subfield names each block type consumes. An indented line opens a
/// new subfield only when it starts with one of these followed by ':';
/// any other line continues the previous subfield. Must stay in sync
/// with the `_*_SUBFIELDS` sets in tools/lib/python/kdoc/kdoc_apispec.py.
fn block_subfield_keys(block: &BlockContext) -> &'static [&'static str] {
    match block {
        BlockContext::Param(_) => &[
            "type",
            "flags",
            "size",
            "constraint-type",
            "constraint",
            "cdesc",
            "range",
            "mask",
            "valid-mask",
            "valid-values",
            "alignment",
            "size-param",
            "struct-type",
            "arch-mask",
            "desc",
            "description",
        ],
        BlockContext::Error(_) => &["desc", "condition"],
        BlockContext::Signal => &[
            "direction",
            "action",
            "condition",
            "desc",
            "errno",
            "timing",
            "priority",
            "restartable",
            "interruptible",
            "number",
            "target",
            "queue",
            "queue_behavior",
            "transform",
            "transform_to",
            "transform-to",
            "sa_flags_required",
            "sa-flags-required",
            "sa_flags_forbidden",
            "sa-flags-forbidden",
            "state_required",
            "state-required",
            "state_forbidden",
            "state-forbidden",
        ],
        BlockContext::Capability => &["type", "allows", "without", "condition", "priority", "desc"],
        BlockContext::SideEffect => &["target", "desc", "condition", "reversible"],
        BlockContext::StateTransition => &["object", "from", "to", "condition", "desc"],
        BlockContext::Constraint => &["desc", "expr"],
        BlockContext::Lock => &[
            "type",
            "scope",
            "acquired",
            "released",
            "held-on-entry",
            "held-on-exit",
            "desc",
        ],
        BlockContext::Return => &[
            "type",
            "check-type",
            "success",
            "success-range",
            "error-values",
            "desc",
        ],
        BlockContext::None => &[],
    }
}

/// True if `line` starts with one of `keys` followed by ':'.
fn starts_subfield(line: &str, keys: &[&str]) -> bool {
    line.split_once(':')
        .is_some_and(|(key, _)| keys.contains(&key.trim()))
}

fn is_indented_line(line: &str) -> bool {
    line.starts_with("  ") || line.starts_with('\t')
}

/// Fold free-form prose into paragraphs. Blank lines separate
/// paragraphs ("\n\n"); wrapped lines inside a paragraph are joined
/// with spaces, except that a line starting with "- " always begins a
/// new line so bullet lists survive. Mirrors `_fold_paragraphs()` in
/// kdoc_apispec.py.
fn fold_paragraphs(lines: &[&str]) -> String {
    let mut paragraphs: Vec<Vec<String>> = Vec::new();
    let mut current: Vec<String> = Vec::new();
    for line in lines {
        let line = line.trim();
        if line.is_empty() {
            if !current.is_empty() {
                paragraphs.push(std::mem::take(&mut current));
            }
        } else if line.starts_with("- ") || current.is_empty() {
            current.push(line.to_string());
        } else if let Some(last) = current.last_mut() {
            last.push(' ');
            last.push_str(line);
        }
    }
    if !current.is_empty() {
        paragraphs.push(current);
    }
    paragraphs
        .iter()
        .map(|p| p.join("\n"))
        .collect::<Vec<_>>()
        .join("\n\n")
        .replace('\t', " ")
}

fn expand_tabs(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
    let mut col = 0;
    for c in line.chars() {
        if c == '\t' {
            let pad = 8 - col % 8;
            out.extend(std::iter::repeat_n(' ', pad));
            col += pad;
        } else {
            out.push(c);
            col += 1;
        }
    }
    out
}

/// Keep every line of a block on its own line. The indentation shared
/// by the continuation lines is removed so relative indentation
/// (nested code) is preserved; runs of blank lines collapse into one.
/// Mirrors `_fold_lines()` in kdoc_apispec.py.
fn fold_lines(lines: &[&str]) -> String {
    let mut lines: Vec<String> = lines
        .iter()
        .map(|l| expand_tabs(l).trim_end().to_string())
        .collect();
    while lines.first().is_some_and(|l| l.is_empty()) {
        lines.remove(0);
    }
    while lines.last().is_some_and(|l| l.is_empty()) {
        lines.pop();
    }

    let indent = |l: &str| l.len() - l.trim_start().len();
    let base = lines
        .iter()
        .skip(1)
        .filter(|l| !l.is_empty())
        .map(|l| indent(l))
        .min()
        .unwrap_or(0);

    let mut out: Vec<&str> = Vec::new();
    for line in &lines {
        if line.is_empty() {
            if out.last().is_some_and(|l| l.is_empty()) {
                continue;
            }
            out.push("");
        } else {
            out.push(&line[base.min(indent(line))..]);
        }
    }
    out.join("\n")
}

impl KerneldocParser {
    pub fn new() -> Self {
        KerneldocParser
    }

    pub fn parse_kerneldoc(
        &self,
        doc: &str,
        name: &str,
        api_type: &str,
        signature: Option<&str>,
    ) -> Result<ApiSpec> {
        let mut spec = ApiSpec {
            name: name.to_string(),
            api_type: api_type.to_string(),
            ..Default::default()
        };

        let lines: Vec<&str> = doc.lines().collect();

        // Extract main description from function name line
        if let Some(first_line) = lines.first() {
            if let Some((_, desc)) = first_line.split_once(" - ") {
                spec.description = Some(desc.trim().to_string());
            }
        }

        // Extract type names from SYSCALL_DEFINE signature
        let type_map = if let Some(sig) = signature {
            self.extract_types_from_signature(sig)
        } else {
            HashMap::new()
        };

        // Keep track of parameters we've seen (from @param lines)
        let mut param_map: HashMap<String, ParamSpec> = HashMap::new();

        // Current block being parsed
        let mut block = BlockContext::None;

        // Temporary storage for current block items
        let mut current_lock: Option<LockSpec> = None;
        let mut current_signal: Option<SignalSpec> = None;
        // Pending symbolic `transform-to:` token. Captured when the parser
        // sees a non-numeric value, but only reported if the final
        // `transform_to` after all lines in the signal block is still
        // unresolved. A later numeric `transform-to:` clears this so we
        // don't warn about a value that was subsequently overridden.
        let mut pending_transform_warning: Option<String> = None;
        let mut current_capability: Option<CapabilitySpec> = None;
        let mut current_side_effect: Option<SideEffectSpec> = None;
        let mut current_constraint: Option<ConstraintSpec> = None;
        let mut current_error: Option<ErrorSpec> = None;
        let mut current_return: Option<ReturnSpec> = None;
        let mut current_state_trans: Option<StateTransitionSpec> = None;

        let mut i = 0;

        while i < lines.len() {
            let line = lines[i];
            let trimmed = line.trim();

            // Skip empty lines
            if trimmed.is_empty() {
                i += 1;
                continue;
            }

            // Check if this is an indented continuation line (part of current block)
            let is_indented = is_indented_line(line);

            // If indented and we're in a block, parse as block attribute.
            // A subfield line absorbs the lines that follow it until the
            // next known subfield key, so a value such as
            // `constraint-type: mask(FOO | BAR |` ... `| BAZ)` or a
            // wrapped `condition:` arrives as a single logical line.
            if is_indented && block != BlockContext::None {
                let keys = block_subfield_keys(&block);
                let mut logical = trimmed.to_string();
                if starts_subfield(trimmed, keys) {
                    let mut j = i + 1;
                    while j < lines.len() {
                        let next = lines[j];
                        let next_trim = next.trim();
                        if next_trim.is_empty() {
                            j += 1;
                            continue;
                        }
                        if !is_indented_line(next) || starts_subfield(next_trim, keys) {
                            break;
                        }
                        logical.push(' ');
                        logical.push_str(next_trim);
                        i = j;
                        j += 1;
                    }
                }
                self.parse_block_attribute(
                    &logical,
                    &block,
                    &mut param_map,
                    &mut current_error,
                    &mut current_signal,
                    &mut pending_transform_warning,
                    &mut current_capability,
                    &mut current_side_effect,
                    &mut current_constraint,
                    &mut current_lock,
                    &mut current_return,
                    &mut current_state_trans,
                );
                i += 1;
                continue;
            }

            // Not indented or not in block: flush current block if any.
            // If a symbolic `transform-to:` was captured and no later
            // numeric line cleared it, surface the warning now; by
            // construction `transform_to` is None in that case.
            if matches!(block, BlockContext::Signal) {
                if let Some(raw) = pending_transform_warning.take() {
                    eprintln!(
                        "kapi: warning: transform-to: {raw:?} is symbolic; \
                         source-mode cannot resolve signal numbers portably. \
                         Use --vmlinux or --debugfs to get the resolved value.",
                    );
                }
            }
            self.flush_block(
                &mut block,
                &mut spec,
                &mut current_error,
                &mut current_signal,
                &mut current_capability,
                &mut current_side_effect,
                &mut current_constraint,
                &mut current_lock,
                &mut current_return,
                &mut current_state_trans,
            );

            // Parse top-level annotations
            if let Some(rest) = trimmed.strip_prefix("@") {
                // @param: description (standard kerneldoc parameter)
                if let Some((param_name, desc)) = rest.split_once(':') {
                    let param_name = param_name.trim();
                    let desc = desc.trim();
                    if !param_name.contains('-') {
                        let idx = param_map.len() as u32;
                        let type_name = type_map.get(param_name).cloned().unwrap_or_default();
                        param_map.insert(
                            param_name.to_string(),
                            ParamSpec {
                                index: idx,
                                name: param_name.to_string(),
                                type_name,
                                description: desc.to_string(),
                                flags: 0,
                                param_type: 0,
                                constraint_type: 0,
                                constraint: None,
                                min_value: None,
                                max_value: None,
                                valid_mask: None,
                                enum_values: vec![],
                                size: None,
                                alignment: None,
                                size_param_idx: None,
                            },
                        );
                    }
                }
            } else if let Some(rest) = trimmed.strip_prefix("long-desc:") {
                let (section, next_i) = self.collect_section(&lines, i, rest);
                let val = fold_paragraphs(&section);
                spec.long_description = Some(val).filter(|v| !v.is_empty());
                i = next_i;
                continue;
            } else if let Some(rest) = trimmed.strip_prefix("context-flags:") {
                spec.context_flags = self.parse_context_flags(rest.trim());
            } else if let Some(rest) = trimmed.strip_prefix("contexts:") {
                // Short form: "contexts: process, sleepable"
                spec.context_flags = self.parse_context_list(rest.trim());
            } else if let Some(rest) = trimmed.strip_prefix("param-count:") {
                spec.param_count = rest.trim().parse().ok();
            }
            // Block-start annotations
            else if let Some(rest) = trimmed.strip_prefix("param:") {
                let param_name = rest.trim().to_string();
                block = BlockContext::Param(param_name.clone());
                // Ensure param exists in map
                if !param_map.contains_key(&param_name) {
                    let idx = param_map.len() as u32;
                    let type_name = type_map
                        .get(param_name.as_str())
                        .cloned()
                        .unwrap_or_default();
                    param_map.insert(
                        param_name.clone(),
                        ParamSpec {
                            index: idx,
                            name: param_name,
                            type_name,
                            description: String::new(),
                            flags: 0,
                            param_type: 0,
                            constraint_type: 0,
                            constraint: None,
                            min_value: None,
                            max_value: None,
                            valid_mask: None,
                            enum_values: vec![],
                            size: None,
                            alignment: None,
                            size_param_idx: None,
                        },
                    );
                }
            } else if let Some(rest) = trimmed.strip_prefix("error:") {
                // error: NAME, condition
                let parts: Vec<&str> = rest.splitn(2, ',').map(|s| s.trim()).collect();
                if !parts.is_empty() {
                    let error_name = parts[0].to_string();
                    let condition = if parts.len() >= 2 {
                        parts[1].to_string()
                    } else {
                        String::new()
                    };
                    let error_code = self.error_name_to_code(&error_name);
                    current_error = Some(ErrorSpec {
                        error_code,
                        name: error_name.clone(),
                        condition,
                        description: String::new(),
                    });
                    block = BlockContext::Error(error_name);
                }
            } else if let Some(rest) = trimmed.strip_prefix("signal:") {
                let signal_name = rest.trim().to_string();
                current_signal = Some(SignalSpec {
                    signal_num: 0,
                    signal_name,
                    direction: 1,
                    action: 0,
                    target: None,
                    condition: None,
                    description: None,
                    restartable: false,
                    timing: 0,
                    priority: 0,
                    interruptible: false,
                    queue: None,
                    sa_flags: 0,
                    sa_flags_required: 0,
                    sa_flags_forbidden: 0,
                    state_required: 0,
                    state_forbidden: 0,
                    error_on_signal: None,
                    transform_to: None,
                });
                block = BlockContext::Signal;
            } else if let Some(rest) = trimmed.strip_prefix("capability:") {
                let parts: Vec<&str> = rest.split(',').map(|s| s.trim()).collect();
                if !parts.is_empty() {
                    let cap_name = parts[0].to_string();
                    let cap_value = self.parse_capability_value(&cap_name);
                    // If we have 3 parts, it's flat format: capability: CAP, action, name
                    let (action, name) = if parts.len() >= 3 {
                        (parts[1].to_string(), parts[2].to_string())
                    } else {
                        (String::new(), cap_name.clone())
                    };
                    current_capability = Some(CapabilitySpec {
                        capability: cap_value,
                        name,
                        action,
                        allows: String::new(),
                        without_cap: String::new(),
                        check_condition: None,
                        priority: Some(0),
                        alternatives: vec![],
                    });
                    block = BlockContext::Capability;
                }
            } else if let Some(rest) = trimmed.strip_prefix("side-effect:") {
                // Could be flat format (comma-separated) or block start
                let rest = rest.trim();
                // Check if it's the flat format with commas
                let comma_parts: Vec<&str> = rest.splitn(3, ',').map(|s| s.trim()).collect();
                if comma_parts.len() >= 3 {
                    // Flat format: side-effect: TYPE, target, desc
                    let mut effect = SideEffectSpec {
                        effect_type: self.parse_effect_type(comma_parts[0]),
                        target: comma_parts[1].to_string(),
                        condition: None,
                        description: comma_parts[2].to_string(),
                        reversible: false,
                    };
                    if comma_parts[2].contains("reversible=yes") {
                        effect.reversible = true;
                    }
                    spec.side_effects.push(effect);
                } else {
                    // Block format: side-effect: TYPE
                    current_side_effect = Some(SideEffectSpec {
                        effect_type: self.parse_effect_type(rest),
                        target: String::new(),
                        condition: None,
                        description: String::new(),
                        reversible: false,
                    });
                    block = BlockContext::SideEffect;
                }
            } else if let Some(rest) = trimmed.strip_prefix("state-trans:") {
                // Flat form: state-trans: OBJECT, FROM, TO, DESCRIPTION
                // (the description may itself contain commas). Block
                // form: a bare OBJECT followed by from:/to:/condition:/
                // desc: subfields.
                if rest.contains(',') {
                    let mut parts = rest.splitn(4, ',').map(|s| s.trim());
                    let mut next_part = || parts.next().unwrap_or_default().to_string();
                    spec.state_transitions.push(StateTransitionSpec {
                        object: next_part(),
                        from_state: next_part(),
                        to_state: next_part(),
                        condition: None,
                        description: next_part(),
                    });
                } else {
                    current_state_trans = Some(StateTransitionSpec {
                        object: rest.trim().to_string(),
                        from_state: String::new(),
                        to_state: String::new(),
                        condition: None,
                        description: String::new(),
                    });
                }
                block = BlockContext::StateTransition;
            } else if let Some(rest) = trimmed.strip_prefix("constraint:") {
                let rest = rest.trim();
                // Could be flat format: constraint: name, desc
                // Or block format: constraint: name
                let parts: Vec<&str> = rest.splitn(2, ',').map(|s| s.trim()).collect();
                if parts.len() >= 2 {
                    // Flat format
                    current_constraint = Some(ConstraintSpec {
                        name: parts[0].to_string(),
                        description: parts[1].to_string(),
                        expression: None,
                    });
                } else {
                    // Block format
                    current_constraint = Some(ConstraintSpec {
                        name: rest.to_string(),
                        description: String::new(),
                        expression: None,
                    });
                }
                block = BlockContext::Constraint;
            } else if let Some(rest) = trimmed.strip_prefix("lock:") {
                let rest = rest.trim();
                // Could be flat: lock: name, type
                // Or block: lock: name
                let parts: Vec<&str> = rest.split(',').map(|s| s.trim()).collect();
                if parts.len() >= 2 {
                    current_lock = Some(LockSpec {
                        lock_name: parts[0].to_string(),
                        lock_type: self.parse_lock_type(parts[1]),
                        scope: super::KAPI_LOCK_INTERNAL,
                        description: String::new(),
                    });
                } else {
                    current_lock = Some(LockSpec {
                        lock_name: rest.to_string(),
                        lock_type: 0,
                        scope: super::KAPI_LOCK_INTERNAL,
                        description: String::new(),
                    });
                }
                block = BlockContext::Lock;
            }
            // Other top-level annotations
            else if let Some(rest) = trimmed.strip_prefix("return:") {
                let rest = rest.trim();
                if rest.is_empty() {
                    // Block format
                    current_return = Some(ReturnSpec {
                        type_name: String::new(),
                        description: String::new(),
                        return_type: 0,
                        check_type: 0,
                        success_value: None,
                        success_min: None,
                        success_max: None,
                        error_values: vec![],
                    });
                    block = BlockContext::Return;
                }
            } else if let Some(rest) = trimmed.strip_prefix("examples:") {
                let (section, next_i) = self.collect_section(&lines, i, rest);
                let val = fold_lines(&section);
                spec.examples = Some(val).filter(|v| !v.is_empty());
                i = next_i;
                continue;
            } else if let Some(rest) = trimmed.strip_prefix("notes:") {
                let (section, next_i) = self.collect_section(&lines, i, rest);
                let val = fold_paragraphs(&section);
                spec.notes = Some(val).filter(|v| !v.is_empty());
                i = next_i;
                continue;
            }

            i += 1;
        }

        // Flush any remaining block. Emit a pending symbolic
        // `transform-to:` warning if the final state still has no
        // resolved numeric value (see per-line loop for rationale).
        if matches!(block, BlockContext::Signal) {
            if let Some(raw) = pending_transform_warning.take() {
                eprintln!(
                    "kapi: warning: transform-to: {raw:?} is symbolic; \
                     source-mode cannot resolve signal numbers portably. \
                     Use --vmlinux or --debugfs to get the resolved value.",
                );
            }
        }
        self.flush_block(
            &mut block,
            &mut spec,
            &mut current_error,
            &mut current_signal,
            &mut current_capability,
            &mut current_side_effect,
            &mut current_constraint,
            &mut current_lock,
            &mut current_return,
            &mut current_state_trans,
        );

        // Convert param_map to vec preserving order
        let mut params: Vec<ParamSpec> = param_map.into_values().collect();
        params.sort_by_key(|p| p.index);

        // If the spec carries an explicit param-count, warn when it
        // disagrees with the number of param: blocks we actually saw.
        if let Some(claimed) = spec.param_count {
            if claimed as usize != params.len() {
                eprintln!(
                    "kapi: {}: param-count: {} disagrees with {} param: block(s)",
                    name,
                    claimed,
                    params.len(),
                );
            }
        }

        for param in &mut params {
            param.drop_unset_string_limits();
        }
        spec.parameters = params;

        Ok(spec)
    }

    /// Parse an indented attribute line within a block
    #[allow(clippy::too_many_arguments)]
    fn parse_block_attribute(
        &self,
        trimmed: &str,
        block: &BlockContext,
        param_map: &mut HashMap<String, ParamSpec>,
        current_error: &mut Option<ErrorSpec>,
        current_signal: &mut Option<SignalSpec>,
        pending_transform_warning: &mut Option<String>,
        current_capability: &mut Option<CapabilitySpec>,
        current_side_effect: &mut Option<SideEffectSpec>,
        current_constraint: &mut Option<ConstraintSpec>,
        current_lock: &mut Option<LockSpec>,
        current_return: &mut Option<ReturnSpec>,
        current_state_trans: &mut Option<StateTransitionSpec>,
    ) {
        match block {
            BlockContext::Param(param_name) => {
                if let Some(param) = param_map.get_mut(param_name) {
                    if let Some(rest) = trimmed.strip_prefix("type:") {
                        // Accept either:
                        //   type: KAPI_TYPE_UINT              (long, single token)
                        //   type: uint                        (short, single token)
                        //   type: uint, input                 (short, type + flags)
                        //   type: path, input                 (short, type + flags)
                        // Single-token inputs leave flags alone; they are
                        // set by a separate `flags:` line.
                        //
                        // User-space pointer types (user_ptr, path) imply
                        // KAPI_PARAM_USER, so specs don't need to repeat
                        // `user` after the type.
                        let mut parts = rest.split(',').map(str::trim);
                        let type_token = parts.next();
                        if let Some(ty) = type_token {
                            param.param_type = self.parse_param_type(ty);
                        }
                        for flag in parts {
                            param.flags |= self.parse_param_flag_token(flag);
                        }
                        if type_token.map(type_implies_user_flag).unwrap_or(false) {
                            param.flags |= 1 << 6; // KAPI_PARAM_USER
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("flags:") {
                        param.flags = self.parse_param_flags(rest.trim());
                    } else if let Some(rest) = trimmed.strip_prefix("constraint-type:") {
                        // Accepts `KAPI_CONSTRAINT_*` enum tokens or
                        // function-call expressions like `range(0, 4096)`
                        // / `mask(0xff)` / `buffer(2)` that also populate
                        // the matching numeric fields on `param`.
                        let text = rest.trim();
                        if !self.apply_constraint_expr(param, text) {
                            param.constraint_type = self.parse_constraint_type(text);
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("valid-mask:") {
                        // Symbolic mask values need cpp-level resolution;
                        // leave that to the binary reader.
                        let _ = rest;
                    } else if let Some(rest) = trimmed
                        .strip_prefix("cdesc:")
                        .or_else(|| trimmed.strip_prefix("constraint:"))
                    {
                        // Free-text constraint description.
                        param.constraint = Some(rest.trim().to_string());
                    } else if let Some(rest) = trimmed.strip_prefix("range:") {
                        let parts: Vec<&str> = rest.split(',').map(|s| s.trim()).collect();
                        if parts.len() >= 2 {
                            param.min_value = parts[0].parse().ok();
                            param.max_value = parts[1].parse().ok();
                            param.constraint_type = 1; // KAPI_CONSTRAINT_RANGE
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("size-param:") {
                        param.size_param_idx = rest.trim().parse().ok();
                    } else if let Some(rest) = trimmed.strip_prefix("description:") {
                        param.description = rest.trim().to_string();
                    } else if let Some(rest) = trimmed.strip_prefix("desc:") {
                        param.description = rest.trim().to_string();
                    } else if !trimmed.contains(':') || trimmed.starts_with("  ") {
                        // Continuation of the previous attribute's value.
                        if let Some(c) = param.constraint.as_mut() {
                            c.push(' ');
                            c.push_str(trimmed);
                        }
                    }
                }
            }
            BlockContext::Error(_) => {
                if let Some(error) = current_error.as_mut() {
                    if let Some(rest) = trimmed.strip_prefix("desc:") {
                        let text = rest.trim().to_string();
                        if error.description.is_empty() {
                            error.description = text;
                        } else {
                            error.description.push(' ');
                            error.description.push_str(&text);
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("condition:") {
                        error.condition = rest.trim().to_string();
                    } else {
                        // Continuation of description
                        if !error.description.is_empty() {
                            error.description.push(' ');
                            error.description.push_str(trimmed);
                        }
                    }
                }
            }
            BlockContext::Signal => {
                if let Some(signal) = current_signal.as_mut() {
                    if let Some(rest) = trimmed.strip_prefix("direction:") {
                        signal.direction = self.parse_signal_direction(rest.trim());
                    } else if let Some(rest) = trimmed.strip_prefix("action:") {
                        signal.action = self.parse_signal_action(rest.trim());
                    } else if let Some(rest) = trimmed.strip_prefix("condition:") {
                        signal.condition = Some(rest.trim().to_string());
                    } else if let Some(rest) = trimmed.strip_prefix("desc:") {
                        let text = rest.trim().to_string();
                        if signal.description.is_none() {
                            signal.description = Some(text);
                        } else if let Some(d) = signal.description.as_mut() {
                            d.push(' ');
                            d.push_str(&text);
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("errno:") {
                        // `error:` cannot be used here because kerneldoc
                        // promotes it to a top-level section header.
                        //
                        // Accepted forms:
                        //   errno: -4         -> numeric literal, stored as-is
                        //   errno: -EINTR     -> kernel convention; resolve
                        //                       the symbol and negate
                        //   errno: EINTR      -> bare symbol; resolved value
                        //                       is already negative
                        let value = rest.trim();
                        signal.error_on_signal = if let Ok(code) = value.parse::<i32>() {
                            Some(code)
                        } else if let Some(name) = value.strip_prefix('-') {
                            // `error_name_to_code` already returns the negated
                            // code (e.g. "EINTR" -> -4), so `-EINTR` resolves
                            // to -4 too; the leading `-` on the symbolic form
                            // is kernel-source convention, not a second negation.
                            Some(self.error_name_to_code(name))
                        } else {
                            Some(self.error_name_to_code(value))
                        };
                    } else if let Some(rest) = trimmed.strip_prefix("timing:") {
                        signal.timing = self.parse_signal_timing(rest.trim());
                    } else if let Some(rest) = trimmed.strip_prefix("restartable:") {
                        let val = rest.trim().to_lowercase();
                        signal.restartable = matches!(val.as_str(), "yes" | "true" | "1");
                    } else if let Some(rest) = trimmed.strip_prefix("interruptible:") {
                        let val = rest.trim().to_lowercase();
                        signal.interruptible = matches!(val.as_str(), "yes" | "true" | "1");
                    } else if let Some(rest) = trimmed.strip_prefix("priority:") {
                        signal.priority = rest.trim().parse().unwrap_or(0);
                    } else if let Some(rest) = trimmed.strip_prefix("target:") {
                        signal.target = Some(rest.trim().to_string());
                    } else if let Some(rest) = trimmed
                        .strip_prefix("queue:")
                        .or_else(|| trimmed.strip_prefix("queue_behavior:"))
                    {
                        signal.queue = Some(rest.trim().to_string());
                    } else if let Some(rest) = trimmed.strip_prefix("number:") {
                        signal.signal_num = rest.trim().parse().unwrap_or(0);
                    } else if let Some(rest) = trimmed
                        .strip_prefix("transform-to:")
                        .or_else(|| trimmed.strip_prefix("transform_to:"))
                        .or_else(|| trimmed.strip_prefix("transform:"))
                    {
                        // transform-to: takes a signal constant (e.g.
                        // SIGKILL) or a numeric literal. Only a numeric
                        // literal fills `transform_to`; symbolic values
                        // cannot be resolved portably in userspace
                        // because signal numbers are arch-dependent and
                        // we have no access to the target arch's
                        // <asm/signal.h>. Report such cases to stderr so
                        // they are not silently lost, and point the user
                        // at --vmlinux / --debugfs, which consult the
                        // compiled struct where the C preprocessor has
                        // already baked in the correct value.
                        //
                        // Assign unconditionally so the last line in
                        // the kerneldoc wins and an intended symbolic
                        // override doesn't silently leave a stale
                        // numeric value from an earlier line. The
                        // warning is deferred until flush_block() so a
                        // subsequent numeric line can cancel it; if the
                        // last line was still symbolic we report it
                        // then.
                        let v = rest.trim();
                        let parsed = v.parse::<i32>().ok();
                        signal.transform_to = parsed;
                        if parsed.is_some() {
                            *pending_transform_warning = None;
                        } else if !v.is_empty() {
                            *pending_transform_warning = Some(v.to_string());
                        }
                    } else if let Some(rest) = trimmed
                        .strip_prefix("sa-flags-required:")
                        .or_else(|| trimmed.strip_prefix("sa_flags_required:"))
                    {
                        signal.sa_flags_required = self.parse_hex_or_bitmask(rest.trim());
                    } else if let Some(rest) = trimmed
                        .strip_prefix("sa-flags-forbidden:")
                        .or_else(|| trimmed.strip_prefix("sa_flags_forbidden:"))
                    {
                        signal.sa_flags_forbidden = self.parse_hex_or_bitmask(rest.trim());
                    } else if let Some(rest) = trimmed
                        .strip_prefix("state-required:")
                        .or_else(|| trimmed.strip_prefix("state_required:"))
                    {
                        signal.state_required = self.parse_signal_state_mask(rest.trim());
                    } else if let Some(rest) = trimmed
                        .strip_prefix("state-forbidden:")
                        .or_else(|| trimmed.strip_prefix("state_forbidden:"))
                    {
                        signal.state_forbidden = self.parse_signal_state_mask(rest.trim());
                    } else {
                        // Continuation of description
                        if let Some(d) = signal.description.as_mut() {
                            d.push(' ');
                            d.push_str(trimmed);
                        }
                    }
                }
            }
            BlockContext::Capability => {
                if let Some(cap) = current_capability.as_mut() {
                    if let Some(rest) = trimmed.strip_prefix("type:") {
                        cap.action = canon_kapi_cap_action(rest.trim());
                    } else if let Some(rest) = trimmed.strip_prefix("allows:") {
                        cap.allows = rest.trim().to_string();
                    } else if let Some(rest) = trimmed.strip_prefix("without:") {
                        cap.without_cap = rest.trim().to_string();
                    } else if let Some(rest) = trimmed.strip_prefix("condition:") {
                        cap.check_condition = Some(rest.trim().to_string());
                    } else if let Some(rest) = trimmed.strip_prefix("priority:") {
                        cap.priority = rest.trim().parse().ok();
                    }
                }
            }
            BlockContext::SideEffect => {
                if let Some(effect) = current_side_effect.as_mut() {
                    if let Some(rest) = trimmed.strip_prefix("target:") {
                        effect.target = rest.trim().to_string();
                    } else if let Some(rest) = trimmed.strip_prefix("condition:") {
                        effect.condition = Some(rest.trim().to_string());
                    } else if let Some(rest) = trimmed.strip_prefix("desc:") {
                        let text = rest.trim().to_string();
                        if effect.description.is_empty() {
                            effect.description = text;
                        } else {
                            effect.description.push(' ');
                            effect.description.push_str(&text);
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("reversible:") {
                        let val = rest.trim().to_lowercase();
                        effect.reversible = matches!(val.as_str(), "yes" | "true" | "1");
                    } else {
                        // Continuation of description
                        if !effect.description.is_empty() {
                            effect.description.push(' ');
                            effect.description.push_str(trimmed);
                        }
                    }
                }
            }
            BlockContext::Constraint => {
                if let Some(constraint) = current_constraint.as_mut() {
                    if let Some(rest) = trimmed.strip_prefix("desc:") {
                        let text = rest.trim().to_string();
                        if constraint.description.is_empty() {
                            constraint.description = text;
                        } else {
                            constraint.description.push(' ');
                            constraint.description.push_str(&text);
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("expr:") {
                        constraint.expression = Some(rest.trim().to_string());
                    } else {
                        // Continuation of description
                        if !constraint.description.is_empty() {
                            constraint.description.push(' ');
                            constraint.description.push_str(trimmed);
                        }
                    }
                }
            }
            BlockContext::Lock => {
                if let Some(lock) = current_lock.as_mut() {
                    if let Some(rest) = trimmed.strip_prefix("type:") {
                        lock.lock_type = self.parse_lock_type(rest.trim());
                    } else if let Some(rest) = trimmed.strip_prefix("scope:") {
                        lock.scope = match rest.trim() {
                            "internal" => super::KAPI_LOCK_INTERNAL,
                            "acquires" => super::KAPI_LOCK_ACQUIRES,
                            "releases" => super::KAPI_LOCK_RELEASES,
                            "caller_held" => super::KAPI_LOCK_CALLER_HELD,
                            _ => super::KAPI_LOCK_INTERNAL,
                        };
                    } else if let Some(rest) = trimmed.strip_prefix("desc:") {
                        let text = rest.trim().to_string();
                        if lock.description.is_empty() {
                            lock.description = text;
                        } else {
                            lock.description.push(' ');
                            lock.description.push_str(&text);
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("acquired:") {
                        // Same rule as kdoc_apispec.py: a lock that is both
                        // acquired and released stays KAPI_LOCK_INTERNAL.
                        if matches!(rest.trim().to_lowercase().as_str(), "true" | "yes") {
                            lock.scope = if lock.scope == super::KAPI_LOCK_RELEASES {
                                super::KAPI_LOCK_INTERNAL
                            } else {
                                super::KAPI_LOCK_ACQUIRES
                            };
                        }
                    } else if let Some(rest) = trimmed.strip_prefix("released:") {
                        if matches!(rest.trim().to_lowercase().as_str(), "true" | "yes") {
                            lock.scope = if lock.scope == super::KAPI_LOCK_ACQUIRES {
                                super::KAPI_LOCK_INTERNAL
                            } else {
                                super::KAPI_LOCK_RELEASES
                            };
                        }
                    } else {
                        // Continuation of description
                        if !lock.description.is_empty() {
                            lock.description.push(' ');
                            lock.description.push_str(trimmed);
                        }
                    }
                }
            }
            BlockContext::Return => {
                if let Some(ret) = current_return.as_mut() {
                    if let Some(rest) = trimmed.strip_prefix("type:") {
                        let raw = rest.trim();
                        ret.type_name = raw.to_string();
                        ret.return_type = self.parse_param_type(raw);
                    } else if let Some(rest) = trimmed.strip_prefix("check-type:") {
                        ret.check_type = self.parse_return_check_type(rest.trim());
                    } else if let Some(rest) = trimmed.strip_prefix("success:") {
                        (ret.success_value, ret.success_min, ret.success_max) =
                            parse_return_success(rest.trim());
                    } else if let Some(rest) = trimmed.strip_prefix("desc:") {
                        let text = rest.trim().to_string();
                        if ret.description.is_empty() {
                            ret.description = text;
                        } else {
                            ret.description.push(' ');
                            ret.description.push_str(&text);
                        }
                    } else {
                        // Continuation of description
                        if !ret.description.is_empty() {
                            ret.description.push(' ');
                            ret.description.push_str(trimmed);
                        }
                    }
                }
            }
            BlockContext::StateTransition => {
                if let Some(trans) = current_state_trans.as_mut() {
                    if let Some(rest) = trimmed.strip_prefix("object:") {
                        trans.object = rest.trim().to_string();
                    } else if let Some(rest) = trimmed.strip_prefix("from:") {
                        trans.from_state = rest.trim().to_string();
                    } else if let Some(rest) = trimmed.strip_prefix("to:") {
                        trans.to_state = rest.trim().to_string();
                    } else if let Some(rest) = trimmed.strip_prefix("condition:") {
                        trans.condition = Some(rest.trim().to_string());
                    } else if let Some(rest) = trimmed.strip_prefix("desc:") {
                        trans.description = rest.trim().to_string();
                    }
                }
            }
            BlockContext::None => {}
        }
    }

    /// Flush the current block, pushing items into the spec
    #[allow(clippy::too_many_arguments)]
    fn flush_block(
        &self,
        block: &mut BlockContext,
        spec: &mut ApiSpec,
        current_error: &mut Option<ErrorSpec>,
        current_signal: &mut Option<SignalSpec>,
        current_capability: &mut Option<CapabilitySpec>,
        current_side_effect: &mut Option<SideEffectSpec>,
        current_constraint: &mut Option<ConstraintSpec>,
        current_lock: &mut Option<LockSpec>,
        current_return: &mut Option<ReturnSpec>,
        current_state_trans: &mut Option<StateTransitionSpec>,
    ) {
        match block {
            BlockContext::Error(_) => {
                if let Some(error) = current_error.take() {
                    spec.errors.push(error);
                }
            }
            BlockContext::Signal => {
                if let Some(signal) = current_signal.take() {
                    spec.signals.push(signal);
                }
            }
            BlockContext::Capability => {
                if let Some(cap) = current_capability.take() {
                    spec.capabilities.push(cap);
                }
            }
            BlockContext::SideEffect => {
                if let Some(effect) = current_side_effect.take() {
                    spec.side_effects.push(effect);
                }
            }
            BlockContext::Constraint => {
                if let Some(constraint) = current_constraint.take() {
                    spec.constraints.push(constraint);
                }
            }
            BlockContext::Lock => {
                if let Some(lock) = current_lock.take() {
                    spec.locks.push(lock);
                }
            }
            BlockContext::Return => {
                if let Some(mut ret) = current_return.take() {
                    ret.keep_used_success_fields();
                    spec.return_spec = Some(ret);
                }
            }
            BlockContext::StateTransition => {
                if let Some(trans) = current_state_trans.take() {
                    spec.state_transitions.push(trans);
                }
            }
            _ => {}
        }
        *block = BlockContext::None;
    }

    /// Extract parameter type names from SYSCALL_DEFINE signature
    fn extract_types_from_signature(&self, sig: &str) -> HashMap<String, String> {
        let mut types = HashMap::new();

        // Find content between outermost parens
        let content = if let Some(start) = sig.find('(') {
            let end = sig.rfind(')').unwrap_or(sig.len());
            &sig[start + 1..end]
        } else {
            return types;
        };

        // Split by comma and process type/name pairs
        // SYSCALL_DEFINE format: (syscall_name, type1, name1, type2, name2, ...)
        let parts: Vec<&str> = content.split(',').map(|s| s.trim()).collect();

        // Skip first part (syscall name), then process pairs
        let mut i = 1;
        while i + 1 < parts.len() {
            let type_part = parts[i].trim();
            let name_part = parts[i + 1].trim();

            // Build the type_name string: "type name"
            let type_name = format!("{} {}", type_part, name_part);
            types.insert(name_part.to_string(), type_name);

            i += 2;
        }

        types
    }

    /// Collect the raw lines of a free-form section: the text after the
    /// key plus every following indented or blank line, up to the next
    /// annotation line.
    fn collect_section<'a>(
        &self,
        lines: &[&'a str],
        start_idx: usize,
        first_part: &'a str,
    ) -> (Vec<&'a str>, usize) {
        let mut section = vec![first_part];
        let mut i = start_idx + 1;

        while i < lines.len() {
            let line = lines[i];
            if self.is_annotation_line(line) {
                break;
            }
            if !line.trim().is_empty() && !is_indented_line(line) {
                break;
            }
            section.push(line);
            i += 1;
        }

        (section, i)
    }

    fn is_annotation_line(&self, line: &str) -> bool {
        let trimmed = line.trim_start();
        if !trimmed.contains(':') {
            return false;
        }
        let annotations = [
            "param:",
            "param-count:",
            "error:",
            "lock:",
            "signal:",
            "side-effect:",
            "state-trans:",
            "capability:",
            "constraint:",
            "return:",
            "examples:",
            "notes:",
            "context-",
            "long-desc:",
            "api-type:",
        ];

        for ann in &annotations {
            if trimmed.starts_with(ann) {
                return true;
            }
        }
        false
    }

    /// Parse a constraint expression and apply it to `param`.
    /// Shapes:
    ///   NAME                         (e.g. "user_path", "nonzero")
    ///   NAME ( ARG (, ARG)* )        (e.g. "range(0, 4096)", "buffer(2)")
    /// Returns true if the expression matched a known constraint kind,
    /// populating `param`'s numeric fields. Returns false if the text
    /// is free-form, leaving `param` untouched.
    fn apply_constraint_expr(&self, param: &mut ParamSpec, text: &str) -> bool {
        let t = text.trim();
        if t.is_empty() {
            return false;
        }
        // Split NAME ( ARGS ); no nesting, no escaping.
        let (name, args): (&str, Option<&str>) = match (t.find('('), t.rfind(')')) {
            (Some(lp), Some(rp)) if rp > lp => (t[..lp].trim(), Some(t[lp + 1..rp].trim())),
            _ => (t, None),
        };
        // Bail out on anything that looks like free text (spaces inside the
        // name part).
        if name.contains(char::is_whitespace) || name.is_empty() {
            return false;
        }
        let name_lc = name.to_ascii_lowercase();
        let split_args = || -> Vec<String> {
            args.map(|a| a.split(',').map(|s| s.trim().to_string()).collect())
                .unwrap_or_default()
        };
        match name_lc.as_str() {
            "range" => {
                let a = split_args();
                if a.len() != 2 {
                    return false;
                }
                param.min_value = parse_i64_literal(&a[0]);
                param.max_value = parse_i64_literal(&a[1]);
                param.constraint_type = 1; // KAPI_CONSTRAINT_RANGE
                true
            }
            "mask" => {
                let a = split_args();
                if a.len() != 1 {
                    return false;
                }
                // Symbolic masks (e.g. "O_RDONLY | O_WRONLY | ...") can't
                // be resolved at parse time; leave valid_mask as None so
                // downstream consumers treat the mask as unknown, matching
                // the long-form `valid-mask:` handler (which also leaves
                // the slot untouched when the value isn't a literal).
                param.valid_mask = parse_u64_literal(&a[0]);
                param.constraint_type = 2; // KAPI_CONSTRAINT_MASK
                true
            }
            "enum" => {
                let a = split_args();
                if a.is_empty() {
                    return false;
                }
                // --vmlinux and --debugfs report each value in decimal, so
                // numeric literals are normalised; symbolic names stay.
                param.enum_values = a
                    .into_iter()
                    .map(|v| parse_i64_literal(&v).map_or(v, |n| n.to_string()))
                    .collect();
                param.constraint_type = 3; // KAPI_CONSTRAINT_ENUM
                true
            }
            "alignment" | "align" => {
                let a = split_args();
                if a.len() != 1 {
                    return false;
                }
                param.alignment = parse_u64_literal(&a[0]).and_then(|n| u32::try_from(n).ok());
                param.constraint_type = 4; // KAPI_CONSTRAINT_ALIGNMENT
                true
            }
            "power_of_two" => {
                if args.is_some() {
                    return false;
                }
                param.constraint_type = 5; // KAPI_CONSTRAINT_POWER_OF_TWO
                true
            }
            "page_aligned" => {
                if args.is_some() {
                    return false;
                }
                param.constraint_type = 6; // KAPI_CONSTRAINT_PAGE_ALIGNED
                true
            }
            "nonzero" => {
                if args.is_some() {
                    return false;
                }
                param.constraint_type = 7; // KAPI_CONSTRAINT_NONZERO
                true
            }
            "user_string" => {
                // Optional size argument: user_string(N)
                if let Some(arg) = args {
                    if let Some(n) = parse_u64_literal(arg).and_then(|n| u32::try_from(n).ok()) {
                        param.size = Some(n);
                    }
                }
                param.constraint_type = 8; // KAPI_CONSTRAINT_USER_STRING
                true
            }
            "user_path" => {
                if args.is_some() {
                    return false;
                }
                param.constraint_type = 9; // KAPI_CONSTRAINT_USER_PATH
                true
            }
            "user_ptr" => {
                if args.is_some() {
                    return false;
                }
                param.constraint_type = 10; // KAPI_CONSTRAINT_USER_PTR
                true
            }
            "buffer" => {
                // buffer(size_param_idx): capture the index into
                // param.size_param_idx so it matches the long-form
                // `size-param: N` handler below (and the C struct
                // field populated by KAPI_PARAM_SIZE_PARAM()).
                let a = split_args();
                if a.len() != 1 {
                    return false;
                }
                param.size_param_idx = a[0].parse().ok();
                param.constraint_type = 11; // KAPI_CONSTRAINT_BUFFER
                true
            }
            "custom" => {
                // custom(fn_name): record function name as free-text constraint
                // so downstream tooling can wire it up.
                if let Some(arg) = args {
                    param.constraint = Some(arg.trim().to_string());
                }
                param.constraint_type = 12; // KAPI_CONSTRAINT_CUSTOM
                true
            }
            _ => false,
        }
    }

    fn parse_context_flags(&self, flags: &str) -> Vec<String> {
        flags
            .split('|')
            .map(|f| self.ctx_alias(f.trim()).to_string())
            .filter(|f| !f.is_empty())
            .collect()
    }

    /// Parse a comma-separated short-form context list
    /// (e.g. "process, sleepable" -> ["KAPI_CTX_PROCESS", "KAPI_CTX_SLEEPABLE"]).
    /// Tokens that already look like KAPI_CTX_* are passed through.
    fn parse_context_list(&self, flags: &str) -> Vec<String> {
        flags
            .split(',')
            .map(|f| self.ctx_alias(f.trim()).to_string())
            .filter(|f| !f.is_empty())
            .collect()
    }

    /// Canonicalise a single context token to its KAPI_CTX_* spelling.
    /// Short aliases are case-insensitive. Unknown tokens pass through
    /// verbatim so mixed/long-form input keeps working.
    fn ctx_alias(&self, tok: &str) -> String {
        let t = tok.trim();
        if t.is_empty() {
            return String::new();
        }
        match t.to_ascii_lowercase().as_str() {
            "process" => "KAPI_CTX_PROCESS".to_string(),
            "softirq" => "KAPI_CTX_SOFTIRQ".to_string(),
            "hardirq" => "KAPI_CTX_HARDIRQ".to_string(),
            "nmi" => "KAPI_CTX_NMI".to_string(),
            "atomic" => "KAPI_CTX_ATOMIC".to_string(),
            "sleepable" => "KAPI_CTX_SLEEPABLE".to_string(),
            "preempt_disabled" => "KAPI_CTX_PREEMPT_DISABLED".to_string(),
            "irq_disabled" => "KAPI_CTX_IRQ_DISABLED".to_string(),
            _ => t.to_string(),
        }
    }

    fn error_name_to_code(&self, name: &str) -> i32 {
        match name {
            "EPERM" => -1,
            "ENOENT" => -2,
            "ESRCH" => -3,
            "EINTR" => -4,
            "EIO" => -5,
            "ENXIO" => -6,
            "E2BIG" => -7,
            "ENOEXEC" => -8,
            "EBADF" => -9,
            "ECHILD" => -10,
            "EAGAIN" | "EWOULDBLOCK" => -11,
            "ENOMEM" => -12,
            "EACCES" => -13,
            "EFAULT" => -14,
            "ENOTBLK" => -15,
            "EBUSY" => -16,
            "EEXIST" => -17,
            "EXDEV" => -18,
            "ENODEV" => -19,
            "ENOTDIR" => -20,
            "EISDIR" => -21,
            "EINVAL" => -22,
            "ENFILE" => -23,
            "EMFILE" => -24,
            "ENOTTY" => -25,
            "ETXTBSY" => -26,
            "EFBIG" => -27,
            "ENOSPC" => -28,
            "ESPIPE" => -29,
            "EROFS" => -30,
            "EMLINK" => -31,
            "EPIPE" => -32,
            "EDOM" => -33,
            "ERANGE" => -34,
            "EDEADLK" => -35,
            "ENAMETOOLONG" => -36,
            "ENOLCK" => -37,
            "ENOSYS" => -38,
            "ENOTEMPTY" => -39,
            "ELOOP" => -40,
            "ENOMSG" => -42,
            "ENODATA" => -61,
            "ENOLINK" => -67,
            "EPROTO" => -71,
            "EOVERFLOW" => -75,
            "ELIBBAD" => -80,
            "EILSEQ" => -84,
            "ENOTSOCK" => -88,
            "EDESTADDRREQ" => -89,
            "EMSGSIZE" => -90,
            "EPROTOTYPE" => -91,
            "ENOPROTOOPT" => -92,
            "EPROTONOSUPPORT" => -93,
            "EOPNOTSUPP" | "ENOTSUP" => -95,
            "EADDRINUSE" => -98,
            "EADDRNOTAVAIL" => -99,
            "ENETDOWN" => -100,
            "ENETUNREACH" => -101,
            "ENETRESET" => -102,
            "ECONNABORTED" => -103,
            "ECONNRESET" => -104,
            "ENOBUFS" => -105,
            "EISCONN" => -106,
            "ENOTCONN" => -107,
            "ETIMEDOUT" => -110,
            "ECONNREFUSED" => -111,
            "EALREADY" => -114,
            "EINPROGRESS" => -115,
            "ESTALE" => -116,
            "EDQUOT" => -122,
            "ENOMEDIUM" => -123,
            "ENOKEY" => -126,
            "EHWPOISON" => -133,
            "ERESTARTSYS" => -512,
            _ => 0,
        }
    }

    /// Map a KAPI_TYPE_* token (or its short-form alias) to the numeric
    /// value declared in `enum kapi_param_type` in
    /// `include/linux/kernel_api_spec.h`.
    fn parse_param_type(&self, type_str: &str) -> u32 {
        let s = type_str.trim();
        match s {
            "KAPI_TYPE_VOID" => 0,
            "KAPI_TYPE_INT" => 1,
            "KAPI_TYPE_UINT" => 2,
            "KAPI_TYPE_PTR" => 3,
            "KAPI_TYPE_STRUCT" => 4,
            "KAPI_TYPE_UNION" => 5,
            "KAPI_TYPE_ENUM" => 6,
            "KAPI_TYPE_FUNC_PTR" => 7,
            "KAPI_TYPE_ARRAY" => 8,
            "KAPI_TYPE_FD" => 9,
            "KAPI_TYPE_USER_PTR" => 10,
            "KAPI_TYPE_PATH" => 11,
            "KAPI_TYPE_CUSTOM" => 12,
            _ => match s.to_ascii_lowercase().as_str() {
                "void" => 0,
                "int" => 1,
                "uint" => 2,
                "ptr" => 3,
                "struct" => 4,
                "union" => 5,
                "enum" => 6,
                "func_ptr" => 7,
                "array" => 8,
                "fd" => 9,
                "user_ptr" | "uptr" => 10,
                "path" => 11,
                "custom" => 12,
                _ => 0,
            },
        }
    }

    /// Map a KAPI_CONSTRAINT_* token to the numeric value declared in
    /// `enum kapi_constraint_type` in `include/linux/kernel_api_spec.h`.
    fn parse_constraint_type(&self, type_str: &str) -> u32 {
        let s = type_str.trim();
        match s {
            "KAPI_CONSTRAINT_NONE" => 0,
            "KAPI_CONSTRAINT_RANGE" => 1,
            "KAPI_CONSTRAINT_MASK" => 2,
            "KAPI_CONSTRAINT_ENUM" => 3,
            "KAPI_CONSTRAINT_ALIGNMENT" => 4,
            "KAPI_CONSTRAINT_POWER_OF_TWO" => 5,
            "KAPI_CONSTRAINT_PAGE_ALIGNED" => 6,
            "KAPI_CONSTRAINT_NONZERO" => 7,
            "KAPI_CONSTRAINT_USER_STRING" => 8,
            "KAPI_CONSTRAINT_USER_PATH" => 9,
            "KAPI_CONSTRAINT_USER_PTR" => 10,
            "KAPI_CONSTRAINT_BUFFER" => 11,
            "KAPI_CONSTRAINT_CUSTOM" => 12,
            _ => 0,
        }
    }

    fn parse_param_flags(&self, flags: &str) -> u32 {
        flags
            .split('|')
            .map(|f| self.parse_param_flag_token(f.trim()))
            .fold(0, |acc, bit| acc | bit)
    }

    /// Parse one flag token (long or short form, case-insensitive for
    /// short form). Returns 0 for unknown tokens.
    fn parse_param_flag_token(&self, tok: &str) -> u32 {
        let t = tok.trim();
        // KAPI_PARAM_* names and upper-case short forms first.
        match t {
            "KAPI_PARAM_IN" | "IN" => return 1,
            "KAPI_PARAM_OUT" | "OUT" => return 2,
            "KAPI_PARAM_INOUT" | "INOUT" => return 3,
            "KAPI_PARAM_OPTIONAL" | "OPTIONAL" => return 1 << 3,
            "KAPI_PARAM_CONST" | "CONST" => return 1 << 4,
            "KAPI_PARAM_VOLATILE" | "VOLATILE" => return 1 << 5,
            "KAPI_PARAM_USER" | "USER" => return 1 << 6,
            "KAPI_PARAM_DMA" | "DMA" => return 1 << 7,
            "KAPI_PARAM_ALIGNED" | "ALIGNED" => return 1 << 8,
            _ => {}
        }
        // English short aliases (case-insensitive).
        match t.to_ascii_lowercase().as_str() {
            "input" => 1,
            "output" => 2,
            "inout" => 3,
            "optional" => 1 << 3,
            "const" => 1 << 4,
            "volatile" => 1 << 5,
            "user" => 1 << 6,
            "dma" => 1 << 7,
            "aligned" => 1 << 8,
            _ => 0,
        }
    }

    /// Map a KAPI_LOCK_* token to the numeric value declared in
    /// `enum kapi_lock_type` in `include/linux/kernel_api_spec.h`.
    fn parse_lock_type(&self, type_str: &str) -> u32 {
        let s = type_str.trim();
        match s {
            "KAPI_LOCK_NONE" => 0,
            "KAPI_LOCK_MUTEX" => 1,
            "KAPI_LOCK_SPINLOCK" => 2,
            "KAPI_LOCK_RWLOCK" => 3,
            "KAPI_LOCK_SEQLOCK" => 4,
            "KAPI_LOCK_RCU" => 5,
            "KAPI_LOCK_SEMAPHORE" => 6,
            "KAPI_LOCK_CUSTOM" => 7,
            _ => match s.to_ascii_lowercase().as_str() {
                "none" => 0,
                "mutex" => 1,
                "spinlock" => 2,
                "rwlock" => 3,
                "seqlock" => 4,
                "rcu" => 5,
                "semaphore" => 6,
                "custom" => 7,
                _ => 0,
            },
        }
    }

    fn parse_signal_direction(&self, dir: &str) -> u32 {
        let s = dir.trim();
        match s {
            "KAPI_SIGNAL_RECEIVE" => 1,
            "KAPI_SIGNAL_SEND" => 2,
            "KAPI_SIGNAL_HANDLE" => 4,
            "KAPI_SIGNAL_BLOCK" => 8,
            "KAPI_SIGNAL_IGNORE" => 16,
            _ => match s.to_ascii_lowercase().as_str() {
                "receive" => 1,
                "send" => 2,
                "handle" => 4,
                "block" => 8,
                "ignore" => 16,
                _ => 0,
            },
        }
    }

    fn parse_signal_action(&self, action: &str) -> u32 {
        let s = action.trim();
        match s {
            "KAPI_SIGNAL_ACTION_DEFAULT" => 0,
            "KAPI_SIGNAL_ACTION_TERMINATE" => 1,
            "KAPI_SIGNAL_ACTION_COREDUMP" => 2,
            "KAPI_SIGNAL_ACTION_STOP" => 3,
            "KAPI_SIGNAL_ACTION_CONTINUE" => 4,
            "KAPI_SIGNAL_ACTION_CUSTOM" => 5,
            "KAPI_SIGNAL_ACTION_RETURN" => 6,
            "KAPI_SIGNAL_ACTION_RESTART" => 7,
            "KAPI_SIGNAL_ACTION_QUEUE" => 8,
            "KAPI_SIGNAL_ACTION_DISCARD" => 9,
            "KAPI_SIGNAL_ACTION_TRANSFORM" => 10,
            _ => match s.to_ascii_lowercase().as_str() {
                "default" => 0,
                "terminate" => 1,
                "coredump" => 2,
                "stop" => 3,
                "continue" => 4,
                "custom" => 5,
                "return" => 6,
                "restart" => 7,
                "queue" => 8,
                "discard" => 9,
                "transform" => 10,
                _ => 0,
            },
        }
    }

    fn parse_signal_timing(&self, timing: &str) -> u32 {
        let s = timing.trim();
        match s {
            "KAPI_SIGNAL_TIME_BEFORE" => 0,
            "KAPI_SIGNAL_TIME_DURING" => 1,
            "KAPI_SIGNAL_TIME_AFTER" => 2,
            _ => match s.to_ascii_lowercase().as_str() {
                "before" => 0,
                "during" => 1,
                "after" => 2,
                _ => 0,
            },
        }
    }

    /// Accept a hex literal ("0x4"), a decimal literal ("4"), or a '|'-separated
    /// bitmask expression. Unknown tokens contribute 0.
    fn parse_hex_or_bitmask(&self, value: &str) -> u32 {
        let v = value.trim();
        if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
            if let Ok(n) = u32::from_str_radix(hex, 16) {
                return n;
            }
        }
        if let Ok(n) = v.parse::<u32>() {
            return n;
        }
        let mut acc = 0u32;
        for part in v.split(['|', ',']) {
            let t = part.trim();
            if t.is_empty() {
                continue;
            }
            if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
                if let Ok(n) = u32::from_str_radix(hex, 16) {
                    acc |= n;
                    continue;
                }
            }
            if let Ok(n) = t.parse::<u32>() {
                acc |= n;
            }
        }
        acc
    }

    /// Parse a '|'-separated list of KAPI_SIGNAL_STATE_* tokens (or short
    /// names like "RUNNING") and OR their bit values together. Matches the
    /// BIT(N) definitions in kernel_api_spec.h.
    fn parse_signal_state_mask(&self, value: &str) -> u32 {
        let mut acc = 0u32;
        for part in value.split(['|', ',']) {
            let t = part.trim().trim_start_matches("KAPI_SIGNAL_STATE_");
            let bit = match t.to_ascii_uppercase().as_str() {
                "RUNNING" => 1 << 0,
                "SLEEPING" => 1 << 1,
                "STOPPED" => 1 << 2,
                "TRACED" => 1 << 3,
                "ZOMBIE" => 1 << 4,
                "DEAD" => 1 << 5,
                _ => 0,
            };
            acc |= bit;
        }
        acc
    }

    /// Bitmask of `KAPI_EFFECT_*` values joined by '|' or ','.
    /// Values match `enum kapi_side_effect_type` in
    /// `include/linux/kernel_api_spec.h`.
    fn parse_effect_type(&self, type_str: &str) -> u32 {
        let sep = if type_str.contains('|') || !type_str.contains(',') {
            '|'
        } else {
            ','
        };
        let mut result = 0;
        for flag in type_str.split(sep) {
            let t = flag.trim();
            let bit = match t {
                "KAPI_EFFECT_NONE" => 0,
                "KAPI_EFFECT_ALLOC_MEMORY" => 1 << 0,
                "KAPI_EFFECT_FREE_MEMORY" => 1 << 1,
                "KAPI_EFFECT_MODIFY_STATE" => 1 << 2,
                "KAPI_EFFECT_SIGNAL_SEND" => 1 << 3,
                "KAPI_EFFECT_FILE_POSITION" => 1 << 4,
                "KAPI_EFFECT_LOCK_ACQUIRE" => 1 << 5,
                "KAPI_EFFECT_LOCK_RELEASE" => 1 << 6,
                "KAPI_EFFECT_RESOURCE_CREATE" => 1 << 7,
                "KAPI_EFFECT_RESOURCE_DESTROY" => 1 << 8,
                "KAPI_EFFECT_SCHEDULE" => 1 << 9,
                "KAPI_EFFECT_HARDWARE" => 1 << 10,
                "KAPI_EFFECT_NETWORK" => 1 << 11,
                "KAPI_EFFECT_FILESYSTEM" => 1 << 12,
                "KAPI_EFFECT_PROCESS_STATE" => 1 << 13,
                "KAPI_EFFECT_IRREVERSIBLE" => 1 << 14,
                _ => match t.to_ascii_lowercase().as_str() {
                    "none" => 0,
                    "alloc_memory" => 1 << 0,
                    "free_memory" => 1 << 1,
                    "modify_state" => 1 << 2,
                    "signal_send" => 1 << 3,
                    "file_position" => 1 << 4,
                    "lock_acquire" => 1 << 5,
                    "lock_release" => 1 << 6,
                    "resource_create" => 1 << 7,
                    "resource_destroy" => 1 << 8,
                    "schedule" => 1 << 9,
                    "hardware" => 1 << 10,
                    "network" => 1 << 11,
                    "filesystem" => 1 << 12,
                    "process_state" => 1 << 13,
                    "irreversible" => 1 << 14,
                    _ => 0,
                },
            };
            result |= bit;
        }
        result
    }

    fn parse_capability_value(&self, cap: &str) -> i32 {
        match cap {
            "CAP_CHOWN" => 0,
            "CAP_DAC_OVERRIDE" => 1,
            "CAP_DAC_READ_SEARCH" => 2,
            "CAP_FOWNER" => 3,
            "CAP_FSETID" => 4,
            "CAP_KILL" => 5,
            "CAP_SETGID" => 6,
            "CAP_SETUID" => 7,
            "CAP_SETPCAP" => 8,
            "CAP_LINUX_IMMUTABLE" => 9,
            "CAP_NET_BIND_SERVICE" => 10,
            "CAP_NET_BROADCAST" => 11,
            "CAP_NET_ADMIN" => 12,
            "CAP_NET_RAW" => 13,
            "CAP_IPC_LOCK" => 14,
            "CAP_IPC_OWNER" => 15,
            "CAP_SYS_MODULE" => 16,
            "CAP_SYS_RAWIO" => 17,
            "CAP_SYS_CHROOT" => 18,
            "CAP_SYS_PTRACE" => 19,
            "CAP_SYS_PACCT" => 20,
            "CAP_SYS_ADMIN" => 21,
            "CAP_SYS_BOOT" => 22,
            "CAP_SYS_NICE" => 23,
            "CAP_SYS_RESOURCE" => 24,
            "CAP_SYS_TIME" => 25,
            "CAP_SYS_TTY_CONFIG" => 26,
            "CAP_MKNOD" => 27,
            "CAP_LEASE" => 28,
            "CAP_AUDIT_WRITE" => 29,
            "CAP_AUDIT_CONTROL" => 30,
            "CAP_SETFCAP" => 31,
            "CAP_MAC_OVERRIDE" => 32,
            "CAP_MAC_ADMIN" => 33,
            "CAP_SYSLOG" => 34,
            "CAP_WAKE_ALARM" => 35,
            "CAP_BLOCK_SUSPEND" => 36,
            "CAP_AUDIT_READ" => 37,
            "CAP_PERFMON" => 38,
            "CAP_BPF" => 39,
            "CAP_CHECKPOINT_RESTORE" => 40,
            _ => 0,
        }
    }

    /// Map a KAPI_RETURN_* token to the numeric value declared in
    /// `enum kapi_return_check_type` in `include/linux/kernel_api_spec.h`.
    fn parse_return_check_type(&self, check: &str) -> u32 {
        let s = check.trim();
        match s {
            "KAPI_RETURN_EXACT" => 0,
            "KAPI_RETURN_RANGE" => 1,
            "KAPI_RETURN_ERROR_CHECK" => 2,
            "KAPI_RETURN_FD" => 3,
            "KAPI_RETURN_CUSTOM" => 4,
            "KAPI_RETURN_NO_RETURN" => 5,
            _ => match s.to_ascii_lowercase().as_str() {
                "exact" => 0,
                "range" => 1,
                "error_check" => 2,
                "fd" => 3,
                "custom" => 4,
                "no_return" => 5,
                _ => 0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parser() -> KerneldocParser {
        KerneldocParser::new()
    }

    #[test]
    fn parse_minimal_kerneldoc() {
        let doc = "\
sys_foo - Do something useful
context-flags: KAPI_CTX_PROCESS
param-count: 1
@fd: The file descriptor
error: EBADF, Bad file descriptor
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_foo", "syscall", None)
            .unwrap();

        assert_eq!(spec.name, "sys_foo");
        assert_eq!(spec.api_type, "syscall");
        assert_eq!(spec.description.as_deref(), Some("Do something useful"));
        assert_eq!(spec.param_count, Some(1));
        assert_eq!(spec.parameters.len(), 1);
        assert_eq!(spec.parameters[0].name, "fd");
        assert_eq!(spec.parameters[0].description, "The file descriptor");
        assert_eq!(spec.errors.len(), 1);
        assert_eq!(spec.errors[0].name, "EBADF");
        assert_eq!(spec.errors[0].error_code, -9);
    }

    #[test]
    fn parse_multiple_param_types() {
        let doc = "\
sys_bar - Multiple params
@fd: file descriptor arg
@buf: user buffer
@count: byte count
@flags: option flags
param: fd
  type: KAPI_TYPE_FD
param: buf
  type: KAPI_TYPE_USER_PTR
param: count
  type: KAPI_TYPE_UINT
param: flags
  type: KAPI_TYPE_UINT
";
        let sig = "(bar, int, fd, char __user *, buf, size_t, count, unsigned long, flags)";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_bar", "syscall", Some(sig))
            .unwrap();

        assert_eq!(spec.parameters.len(), 4);

        let fd_param = spec.parameters.iter().find(|p| p.name == "fd").unwrap();
        assert_eq!(fd_param.param_type, 9); // FD (kernel enum)

        let buf_param = spec.parameters.iter().find(|p| p.name == "buf").unwrap();
        assert_eq!(buf_param.param_type, 10); // USER_PTR (kernel enum)
        assert_eq!(buf_param.type_name, "char __user * buf");

        let count_param = spec.parameters.iter().find(|p| p.name == "count").unwrap();
        assert_eq!(count_param.param_type, 2); // UINT

        let flags_param = spec.parameters.iter().find(|p| p.name == "flags").unwrap();
        assert_eq!(flags_param.param_type, 2); // UINT
    }

    #[test]
    fn parse_error_codes_with_descriptions() {
        let doc = "\
sys_err - Error test
error: EBADF
  desc: Bad file descriptor
  condition: fd < 0
error: EFAULT
  desc: Bad user pointer
  condition: buf is NULL
error: EINVAL
  desc: Invalid argument
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_err", "syscall", None)
            .unwrap();

        assert_eq!(spec.errors.len(), 3);

        assert_eq!(spec.errors[0].name, "EBADF");
        assert_eq!(spec.errors[0].error_code, -9);
        assert_eq!(spec.errors[0].description, "Bad file descriptor");
        assert_eq!(spec.errors[0].condition, "fd < 0");

        assert_eq!(spec.errors[1].name, "EFAULT");
        assert_eq!(spec.errors[1].error_code, -14);
        assert_eq!(spec.errors[1].description, "Bad user pointer");

        assert_eq!(spec.errors[2].name, "EINVAL");
        assert_eq!(spec.errors[2].error_code, -22);
        assert_eq!(spec.errors[2].description, "Invalid argument");
    }

    #[test]
    fn parse_context_flags() {
        let doc = "\
sys_ctx - Context test
context-flags: KAPI_CTX_PROCESS|KAPI_CTX_SLEEPABLE
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_ctx", "syscall", None)
            .unwrap();

        assert_eq!(spec.context_flags.len(), 2);
        assert_eq!(spec.context_flags[0], "KAPI_CTX_PROCESS");
        assert_eq!(spec.context_flags[1], "KAPI_CTX_SLEEPABLE");
    }

    #[test]
    fn parse_context_list_short() {
        // "contexts: process, sleepable" -> KAPI_CTX_PROCESS | SLEEPABLE
        let doc = "\
sys_ctx - Context test
contexts: process, sleepable
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_ctx", "syscall", None)
            .unwrap();

        assert_eq!(
            spec.context_flags,
            vec![
                "KAPI_CTX_PROCESS".to_string(),
                "KAPI_CTX_SLEEPABLE".to_string(),
            ]
        );
    }

    #[test]
    fn parse_context_list_mixed() {
        // Short tokens intermixed with explicit KAPI_CTX_* still work.
        let doc = "\
sys_ctx - Context test
contexts: process, KAPI_CTX_SLEEPABLE, softirq
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_ctx", "syscall", None)
            .unwrap();

        assert_eq!(
            spec.context_flags,
            vec![
                "KAPI_CTX_PROCESS".to_string(),
                "KAPI_CTX_SLEEPABLE".to_string(),
                "KAPI_CTX_SOFTIRQ".to_string(),
            ]
        );
    }

    #[test]
    fn parse_context_flags_long_with_short_token() {
        // Long-form "context-flags:" accepts "|"-joined short aliases.
        let doc = "\
sys_ctx - Context test
context-flags: process | KAPI_CTX_SLEEPABLE
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_ctx", "syscall", None)
            .unwrap();

        assert_eq!(
            spec.context_flags,
            vec![
                "KAPI_CTX_PROCESS".to_string(),
                "KAPI_CTX_SLEEPABLE".to_string(),
            ]
        );
    }

    #[test]
    fn parse_param_type_short_combined() {
        // "type: uint, input" combines the type and flag aliases.
        let doc = "\
sys_t - Short type test
param: size
  type: uint, input
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_t", "syscall", None)
            .unwrap();

        assert_eq!(spec.parameters.len(), 1);
        assert_eq!(spec.parameters[0].param_type, 2); // KAPI_TYPE_UINT
        assert_eq!(spec.parameters[0].flags, 1); // KAPI_PARAM_IN
    }

    #[test]
    fn parse_param_type_short_multi_flag() {
        // "type: path, input, user" sets both the IN and USER flags.
        let doc = "\
sys_t - Short type test
param: filename
  type: path, input, user
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_t", "syscall", None)
            .unwrap();

        assert_eq!(spec.parameters.len(), 1);
        assert_eq!(spec.parameters[0].param_type, 11); // PATH (kernel enum)
        assert_eq!(spec.parameters[0].flags, 1 | (1 << 6)); // IN | USER
    }

    #[test]
    fn parse_constraint_type_range_expr() {
        // Short form: "constraint-type: range(0, 4096)" replaces the
        // two-line long form "constraint-type: KAPI_CONSTRAINT_RANGE"
        // + "range: 0, 4096".
        let doc = "\
sys_c - Constraint test
param: count
  type: uint, input
  constraint-type: range(0, 4096)
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_c", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.constraint_type, 1); // KAPI_CONSTRAINT_RANGE
        assert_eq!(p.min_value, Some(0));
        assert_eq!(p.max_value, Some(4096));
    }

    #[test]
    fn parse_constraint_type_mask_expr() {
        let doc = "\
sys_c - Constraint test
param: flags
  type: uint, input
  constraint-type: mask(0xff)
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_c", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.constraint_type, 2); // KAPI_CONSTRAINT_MASK
        assert_eq!(p.valid_mask, Some(0xff));
    }

    #[test]
    fn parse_constraint_type_enum_expr() {
        let doc = "\
sys_c - Constraint test
param: mode
  type: int, input
  constraint-type: enum(0, 0x10, -3, MODE_FAST)
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_c", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.constraint_type, 3); // KAPI_CONSTRAINT_ENUM
        assert_eq!(p.enum_values, ["0", "16", "-3", "MODE_FAST"]);
    }

    #[test]
    fn c_integer_literals() {
        assert_eq!(parse_u64_literal("0"), Some(0));
        assert_eq!(parse_u64_literal("010"), Some(8));
        assert_eq!(parse_u64_literal("0755"), Some(0o755));
        assert_eq!(parse_u64_literal("0x1F"), Some(31));
        assert_eq!(parse_u64_literal("0b101"), Some(5));
        assert_eq!(parse_u64_literal("4096UL"), Some(4096));
        assert_eq!(parse_u64_literal("0xffULL"), Some(255));
        assert_eq!(parse_u64_literal("08"), None);
        assert_eq!(parse_u64_literal("0x"), None);
        assert_eq!(parse_u64_literal("+5"), None);
        assert_eq!(parse_u64_literal("PAGE_SIZE"), None);
        assert_eq!(parse_i64_literal("-010"), Some(-8));
        assert_eq!(parse_i64_literal("-1U"), Some(4294967295));
        assert_eq!(parse_i64_literal("-0x80000000"), Some(2147483648));
        assert_eq!(parse_i64_literal("-2147483648"), Some(-2147483648));
        assert_eq!(parse_i64_literal("-1UL"), Some(-1));
        assert_eq!(parse_i64_literal("0xFFFFFFFFFFFFFFFF"), Some(-1));
        assert_eq!(parse_i64_literal("18446744073709551615"), Some(-1));
        assert_eq!(parse_i64_literal("-0x8000000000000000"), Some(i64::MIN));
        assert_eq!(parse_u64_literal("-1"), Some(u64::MAX));
        assert_eq!(parse_u64_literal("1uu"), None);
        assert_eq!(parse_u64_literal("1lll"), None);
    }

    #[test]
    fn parse_constraint_c_literals() {
        let doc = "\
sys_c - Constraint test
param: mode
  type: int, input
  constraint-type: enum(010, 0x10, -07, 0b11, 5U)
param: perms
  type: uint, input
  constraint-type: mask(0755)
param: len
  type: uint, input
  constraint-type: range(01, 0x1000UL)
param: align
  type: uint, input
  constraint-type: alignment(0x10)
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_c", "syscall", None)
            .unwrap();

        let p = &spec.parameters;
        assert_eq!(p[0].enum_values, ["8", "16", "-7", "3", "5"]);
        assert_eq!(p[1].valid_mask, Some(0o755));
        assert_eq!((p[2].min_value, p[2].max_value), (Some(1), Some(4096)));
        assert_eq!(p[3].alignment, Some(16));
    }

    #[test]
    fn zero_user_string_limits_are_unset() {
        let doc = "\
sys_s - String test
param: name
  type: user_ptr, input
  range: 0, 255
  constraint-type: user_string
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_s", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.constraint_type, 8);
        assert_eq!((p.min_value, p.max_value), (None, Some(255)));
    }

    #[test]
    fn user_ptr_type_implies_user_flag() {
        let doc = "\
sys_u - Implicit user flag test
param: buf
  type: user_ptr, output
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_u", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.param_type, 10); // KAPI_TYPE_USER_PTR
        assert_eq!(
            p.flags,
            (1 << 1) | (1 << 6), // OUT | USER
            "user_ptr type must imply KAPI_PARAM_USER"
        );
    }

    #[test]
    fn fd_type_does_not_imply_user_flag() {
        // Only user_ptr / path imply KAPI_PARAM_USER. fd, int, uint,
        // and every other non-user-space type must leave flags alone.
        let doc = "\
sys_fd - fd has no implicit user flag
param: fd
  type: fd, input
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_fd", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.param_type, 9);
        assert_eq!(p.flags, 1, "fd must not auto-set KAPI_PARAM_USER");
    }

    #[test]
    fn path_type_implies_user_flag() {
        let doc = "\
sys_p - Path implicit user flag
param: filename
  type: path, input
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_p", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.param_type, 11);
        assert_eq!(
            p.flags,
            1 | (1 << 6), // IN | USER
            "path type must imply KAPI_PARAM_USER"
        );
    }

    #[test]
    fn short_form_enum_equivalence() {
        // Short-form and long-form renderings of the same spec must
        // produce identical ApiSpec output across every enum family:
        // context flags, param type+flags, constraint type, lock type,
        // signal direction/action/timing, capability action, side-effect
        // bitmask, return check type.
        let long = "\
sys_x - Enum short form test
context-flags: KAPI_CTX_PROCESS | KAPI_CTX_SLEEPABLE

param: fd
  type: KAPI_TYPE_FD
  flags: KAPI_PARAM_IN

lock: files->file_lock
  type: KAPI_LOCK_SPINLOCK
  scope: acquires
  desc: table lock

signal: pending_signals
  direction: KAPI_SIGNAL_RECEIVE
  action: KAPI_SIGNAL_ACTION_RETURN
  timing: KAPI_SIGNAL_TIME_DURING
  desc: sig

capability: CAP_SYS_ADMIN
  type: KAPI_CAP_BYPASS_CHECK

return:
  type: KAPI_TYPE_INT
  check-type: KAPI_RETURN_FD
  desc: fd or errno

side-effect: KAPI_EFFECT_RESOURCE_CREATE | KAPI_EFFECT_ALLOC_MEMORY
  target: t
  desc: d
";
        let short = "\
sys_x - Enum short form test
contexts: process, sleepable

param: fd
  type: fd, input

lock: files->file_lock
  type: spinlock
  scope: acquires
  desc: table lock

signal: pending_signals
  direction: receive
  action: return
  timing: during
  desc: sig

capability: CAP_SYS_ADMIN
  type: bypass_check

return:
  type: int
  check-type: fd
  desc: fd or errno

side-effect: resource_create | alloc_memory
  target: t
  desc: d
";
        let mut sp_l = parser()
            .parse_kerneldoc(long, "sys_x", "syscall", None)
            .unwrap();
        let mut sp_s = parser()
            .parse_kerneldoc(short, "sys_x", "syscall", None)
            .unwrap();
        // The return type name is the human spelling, kept as written.
        for sp in [&mut sp_l, &mut sp_s] {
            sp.return_spec.as_mut().unwrap().type_name.clear();
        }
        assert_eq!(
            format!("{:#?}", sp_l),
            format!("{:#?}", sp_s),
            "long-form and short-form of every enum family must normalise identically"
        );
    }

    #[test]
    fn parse_buffer_short_captures_size_param_idx() {
        let doc = "\
sys_b - Buffer test
param: buf
  type: user_ptr, output, user
  constraint-type: buffer(2)
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_b", "syscall", None)
            .unwrap();

        assert_eq!(spec.parameters[0].constraint_type, 11);
        assert_eq!(spec.parameters[0].size_param_idx, Some(2));
    }

    #[test]
    fn buffer_short_and_size_param_long_are_symmetric() {
        let short = "\
sys_b - Symmetric buffer test
param: buf
  type: user_ptr, output, user
  constraint-type: buffer(2)
";
        let long = "\
sys_b - Symmetric buffer test
param: buf
  type: KAPI_TYPE_USER_PTR
  flags: KAPI_PARAM_OUT | KAPI_PARAM_USER
  constraint-type: KAPI_CONSTRAINT_BUFFER
  size-param: 2
";
        let sp_s = parser()
            .parse_kerneldoc(short, "sys_b", "syscall", None)
            .unwrap();
        let sp_l = parser()
            .parse_kerneldoc(long, "sys_b", "syscall", None)
            .unwrap();
        assert_eq!(format!("{:#?}", sp_s), format!("{:#?}", sp_l));
    }

    #[test]
    fn parse_constraint_type_bare_user_path() {
        let doc = "\
sys_c - Constraint test
param: filename
  type: path, input, user
  constraint-type: user_path
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_c", "syscall", None)
            .unwrap();

        assert_eq!(spec.parameters[0].constraint_type, 9); // USER_PATH
    }

    #[test]
    fn starts_subfield_requires_known_key() {
        let keys = block_subfield_keys(&BlockContext::SideEffect);
        assert!(super::starts_subfield("target: file table", keys));
        assert!(super::starts_subfield("condition:", keys));
        assert!(super::starts_subfield("reversible: yes", keys));
        // A colon inside prose does not open a subfield.
        assert!(!super::starts_subfield(
            "this lock mid-operation: the handler",
            keys
        ));
        assert!(!super::starts_subfield("Careful: it is dangerous", keys));
        // A key that belongs to another block type does not count.
        assert!(!super::starts_subfield("direction: receive", keys));
        assert!(!super::starts_subfield("O_RDONLY | O_WRONLY |", keys));
        assert!(!super::starts_subfield(")", keys));
    }

    #[test]
    fn multiline_fold_stops_at_sibling_block_attribute() {
        // The constraint-type's continuation must not greedily eat the
        // next subfield of the same block.
        let doc = "\
sys_y - Fold stop test
param: f
  type: int, input
  constraint-type: mask(FOO |
                        BAR)
  cdesc: something about f
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_y", "syscall", None)
            .unwrap();

        assert_eq!(spec.parameters.len(), 1);
        // If the fold over-consumed, `cdesc:` would have been swallowed
        // into the mask expression and param.constraint_type would be 0.
        assert_eq!(spec.parameters[0].constraint_type, 2);
        assert_eq!(
            spec.parameters[0].constraint.as_deref(),
            Some("something about f")
        );
    }

    #[test]
    fn parse_constraint_type_mask_expr_multiline() {
        // Real-world sys_open/flags case: a symbolic mask split across
        // four continuation lines. The parser must fold the continuation
        // lines before running the function-call match, otherwise the
        // constraint type silently decays to 0.
        let doc = "\
sys_x - Multi-line mask test
param: f
  type: int, input
  constraint-type: mask(O_RDONLY | O_WRONLY | O_RDWR | O_CREAT | O_EXCL | O_NOCTTY |
                        O_TRUNC | O_APPEND | O_NONBLOCK | O_DSYNC | O_SYNC | FASYNC |
                        O_DIRECT | O_LARGEFILE | O_DIRECTORY | O_NOFOLLOW | O_NOATIME |
                        O_CLOEXEC | O_PATH | O_TMPFILE)
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_x", "syscall", None)
            .unwrap();

        assert_eq!(spec.parameters.len(), 1);
        let p = &spec.parameters[0];
        assert_eq!(p.constraint_type, 2, "multi-line mask must set MASK type");
        // Symbolic mask: must stay unresolved rather than becoming Some(0).
        assert_eq!(
            p.valid_mask, None,
            "symbolic mask values must remain None, not Some(0)"
        );
    }

    #[test]
    fn parse_constraint_long_form() {
        let doc = "\
sys_c - Constraint test
param: foo
  type: uint, input
  constraint-type: KAPI_CONSTRAINT_MASK
  valid-mask: 0xff
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_c", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.constraint_type, 2); // KAPI_CONSTRAINT_MASK
    }

    #[test]
    fn parse_constraint_free_text() {
        // `constraint:` carries free-text constraint description;
        // function-call short form lives on `constraint-type:`.
        let doc = "\
sys_c - Constraint test
param: foo
  type: uint, input
  constraint: must be a valid page descriptor
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_c", "syscall", None)
            .unwrap();

        let p = &spec.parameters[0];
        assert_eq!(p.constraint_type, 0);
        assert_eq!(
            p.constraint.as_deref(),
            Some("must be a valid page descriptor")
        );
    }

    #[test]
    fn parse_description_alias_overrides_kerneldoc() {
        // `description:` inside a `param:` block is an alias for `desc:`
        // and overrides the @param description.
        let doc = "\
sys_d - Description alias test
@size: kerneldoc short description
param: size
  type: uint, input
  description: The new long form description.
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_d", "syscall", None)
            .unwrap();

        assert_eq!(spec.parameters.len(), 1);
        assert_eq!(
            spec.parameters[0].description,
            "The new long form description."
        );
    }

    #[test]
    fn canonical_equivalence_short_vs_long() {
        // Two spellings of the same spec must produce identical ApiSpec
        // JSON.
        let long = "\
sys_open - open a file
context-flags: KAPI_CTX_PROCESS | KAPI_CTX_SLEEPABLE

param: filename
  type: KAPI_TYPE_PATH
  flags: KAPI_PARAM_IN | KAPI_PARAM_USER
  constraint-type: KAPI_CONSTRAINT_USER_PATH
  desc: Pathname to open

param: count
  type: KAPI_TYPE_UINT
  flags: KAPI_PARAM_IN
  constraint-type: KAPI_CONSTRAINT_RANGE
  range: 0, 4096
  desc: Byte count
";
        let short = "\
sys_open - open a file
contexts: process, sleepable

param: filename
  type: path, input, user
  constraint-type: user_path
  description: Pathname to open

param: count
  type: uint, input
  constraint-type: range(0, 4096)
  description: Byte count
";
        let long_spec = parser()
            .parse_kerneldoc(long, "sys_open", "syscall", None)
            .unwrap();
        let short_spec = parser()
            .parse_kerneldoc(short, "sys_open", "syscall", None)
            .unwrap();

        // ApiSpec isn't Serialize as a whole, so compare the Debug
        // rendering, which still proves every field canonicalises
        // identically.
        let d_long = format!("{:#?}", long_spec);
        let d_short = format!("{:#?}", short_spec);
        assert_eq!(
            d_long, d_short,
            "short-form and long-form specs must normalise identically"
        );
    }

    #[test]
    fn parse_capability_block() {
        let doc = "\
sys_cap - Capability test
capability: CAP_SYS_ADMIN
  type: required
  allows: Full system administration
  without: Operation not permitted
  condition: always
  priority: 5
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_cap", "syscall", None)
            .unwrap();

        assert_eq!(spec.capabilities.len(), 1);
        let cap = &spec.capabilities[0];
        assert_eq!(cap.capability, 21); // CAP_SYS_ADMIN
        assert_eq!(cap.action, "required");
        assert_eq!(cap.allows, "Full system administration");
        assert_eq!(cap.without_cap, "Operation not permitted");
        assert_eq!(cap.check_condition.as_deref(), Some("always"));
        assert_eq!(cap.priority, Some(5));
    }

    #[test]
    fn parse_lock_block() {
        let doc = "\
sys_lock - Lock test
lock: files_lock, KAPI_LOCK_MUTEX
  scope: acquires
  desc: Protects file table
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_lock", "syscall", None)
            .unwrap();

        assert_eq!(spec.locks.len(), 1);
        let lock = &spec.locks[0];
        assert_eq!(lock.lock_name, "files_lock");
        assert_eq!(lock.lock_type, 1); // MUTEX
        assert_eq!(lock.scope, super::super::KAPI_LOCK_ACQUIRES);
        assert_eq!(lock.description, "Protects file table");
    }

    #[test]
    fn parse_lock_acquired_released_flags() {
        let doc = "\
sys_lock - Lock test
lock: a_lock, KAPI_LOCK_MUTEX
  acquired: true
  released: true
lock: b_lock, KAPI_LOCK_MUTEX
  acquired: true
lock: c_lock, KAPI_LOCK_MUTEX
  released: yes
lock: d_lock, KAPI_LOCK_MUTEX
  acquired: conditional
  released: false
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_lock", "syscall", None)
            .unwrap();

        let scopes: Vec<u32> = spec.locks.iter().map(|l| l.scope).collect();
        assert_eq!(
            scopes,
            vec![
                super::super::KAPI_LOCK_INTERNAL,
                super::super::KAPI_LOCK_ACQUIRES,
                super::super::KAPI_LOCK_RELEASES,
                super::super::KAPI_LOCK_INTERNAL,
            ]
        );
    }

    #[test]
    fn parse_signal_block() {
        let doc = "\
sys_sig - Signal test
signal: SIGKILL
  direction: KAPI_SIGNAL_RECEIVE
  action: KAPI_SIGNAL_ACTION_TERMINATE
  timing: KAPI_SIGNAL_TIME_DURING
  priority: 3
  restartable: yes
  interruptible: yes
  desc: Process termination signal
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_sig", "syscall", None)
            .unwrap();

        assert_eq!(spec.signals.len(), 1);
        let sig = &spec.signals[0];
        assert_eq!(sig.signal_name, "SIGKILL");
        assert_eq!(sig.direction, 1); // RECEIVE
        assert_eq!(sig.action, 1); // TERMINATE
        assert_eq!(sig.timing, 1); // DURING
        assert_eq!(sig.priority, 3);
        assert!(sig.restartable);
        assert!(sig.interruptible);
        assert_eq!(
            sig.description.as_deref(),
            Some("Process termination signal")
        );
    }

    #[test]
    fn parse_signal_errno_shapes() {
        // All three accepted spellings of the signal errno field must
        // produce the same negative kernel return code.
        for (form, label) in [
            ("errno: -EINTR", "-EINTR symbolic"),
            ("errno: EINTR", "bare symbolic"),
            ("errno: -4", "numeric literal"),
        ] {
            let doc = format!(
                "sys_s - Signal errno test\n\
                 signal: SIGINT\n\
                 \x20 direction: receive\n\
                 \x20 action: return\n\
                 \x20 {}\n",
                form,
            );
            let spec = parser()
                .parse_kerneldoc(&doc, "sys_s", "syscall", None)
                .unwrap();
            assert_eq!(spec.signals.len(), 1, "{label}");
            assert_eq!(
                spec.signals[0].error_on_signal,
                Some(-4),
                "errno form {label:?} must resolve to -EINTR (-4)",
            );
        }
    }

    #[test]
    fn parse_side_effect_flat() {
        let doc = "\
sys_se - Side effect test
side-effect: KAPI_EFFECT_MODIFY_STATE, file_table, Allocates a new file descriptor
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_se", "syscall", None)
            .unwrap();

        assert_eq!(spec.side_effects.len(), 1);
        let se = &spec.side_effects[0];
        assert_eq!(se.effect_type, 1 << 2); // KAPI_EFFECT_MODIFY_STATE
        assert_eq!(se.target, "file_table");
        assert_eq!(se.description, "Allocates a new file descriptor");
    }

    #[test]
    fn parse_side_effect_block() {
        let doc = "\
sys_se2 - Side effect block test
side-effect: KAPI_EFFECT_ALLOC_MEMORY
  target: kernel_heap
  desc: Allocates kernel memory
  reversible: yes
  condition: size > 0
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_se2", "syscall", None)
            .unwrap();

        assert_eq!(spec.side_effects.len(), 1);
        let se = &spec.side_effects[0];
        assert_eq!(se.effect_type, 1 << 0); // KAPI_EFFECT_ALLOC_MEMORY
        assert_eq!(se.target, "kernel_heap");
        assert_eq!(se.description, "Allocates kernel memory");
        assert!(se.reversible);
        assert_eq!(se.condition.as_deref(), Some("size > 0"));
    }

    #[test]
    fn parse_empty_doc_no_error() {
        let doc = "";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_empty", "syscall", None)
            .unwrap();

        assert_eq!(spec.name, "sys_empty");
        assert!(spec.description.is_none());
        assert!(spec.parameters.is_empty());
        assert!(spec.errors.is_empty());
        assert!(spec.signals.is_empty());
        assert!(spec.capabilities.is_empty());
        assert!(spec.locks.is_empty());
        assert!(spec.side_effects.is_empty());
        assert!(spec.context_flags.is_empty());
    }

    #[test]
    fn parse_missing_sections_no_error() {
        // Only has a description, no KAPI annotations
        let doc = "\
sys_simple - Just a simple syscall
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_simple", "syscall", None)
            .unwrap();

        assert_eq!(spec.description.as_deref(), Some("Just a simple syscall"));
        assert!(spec.parameters.is_empty());
        assert!(spec.errors.is_empty());
        assert!(spec.context_flags.is_empty());
    }

    #[test]
    fn parse_constraint_block() {
        let doc = "\
sys_cst - Constraint test
constraint: valid_fd
  desc: File descriptor must be valid and open
  expr: fd >= 0 && fd < NR_OPEN
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_cst", "syscall", None)
            .unwrap();

        assert_eq!(spec.constraints.len(), 1);
        let cst = &spec.constraints[0];
        assert_eq!(cst.name, "valid_fd");
        assert_eq!(cst.description, "File descriptor must be valid and open");
        assert_eq!(cst.expression.as_deref(), Some("fd >= 0 && fd < NR_OPEN"));
    }

    #[test]
    fn parse_state_transition_flat() {
        let doc = "\
sys_st - State transition test
state-trans: fd, open, closed, File descriptor is closed
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_st", "syscall", None)
            .unwrap();

        assert_eq!(spec.state_transitions.len(), 1);
        let st = &spec.state_transitions[0];
        assert_eq!(st.object, "fd");
        assert_eq!(st.from_state, "open");
        assert_eq!(st.to_state, "closed");
        assert_eq!(st.description, "File descriptor is closed");
    }

    #[test]
    fn parse_param_block_with_range() {
        let doc = "\
sys_rng - Range test
@count: byte count
param: count
  type: KAPI_TYPE_UINT
  flags: IN
  range: 0, 4096
  constraint-type: KAPI_CONSTRAINT_RANGE
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_rng", "syscall", None)
            .unwrap();

        assert_eq!(spec.parameters.len(), 1);
        let p = &spec.parameters[0];
        assert_eq!(p.name, "count");
        assert_eq!(p.param_type, 2); // UINT
        assert_eq!(p.flags, 1); // IN
        assert_eq!(p.min_value, Some(0));
        assert_eq!(p.max_value, Some(4096));
        assert_eq!(p.constraint_type, 1); // RANGE
    }

    #[test]
    fn parse_return_block() {
        let doc = "\
sys_ret - Return test
return:
  type: KAPI_TYPE_INT
  check-type: KAPI_RETURN_FD
  success: 0
  desc: Returns file descriptor on success
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_ret", "syscall", None)
            .unwrap();

        let ret = spec.return_spec.as_ref().unwrap();
        assert_eq!(ret.type_name, "KAPI_TYPE_INT");
        assert_eq!(ret.return_type, 1); // INT
        assert_eq!(ret.check_type, 3); // FD
        assert_eq!(ret.success_value, None); // FD checks carry no success value
        assert_eq!(ret.description, "Returns file descriptor on success");
    }

    #[test]
    fn colon_in_continuation_line_does_not_start_subfield() {
        let doc = "\
sys_col - Colon continuation test
param: x
  type: int
  cdesc: Must be sane. Note: this is a continuation
    with a colon: in the middle.

lock: mylock
  type: mutex
  desc: Taken at entry; if the caller drops
    this lock mid-operation: the handler waits
    until released.

side-effect: modify_state
  target: stuff
  condition: only when a
    special case applies: for example
    on Tuesdays
  desc: Does things.
    Careful: it is dangerous.
  reversible: no

signal: SIGINT
  direction: receive
  condition: while blocked, unless
    flagged: see below
  desc: Interrupts the wait.

capability: CAP_SYS_ADMIN
  type: bypass_check
  allows: Things
    that need: privilege
  condition: Always

constraint: limit
  desc: A limit
    of note: nothing.
  expr: a &&
    b
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_col", "syscall", None)
            .unwrap();

        assert_eq!(
            spec.parameters[0].constraint.as_deref(),
            Some("Must be sane. Note: this is a continuation with a colon: in the middle.")
        );
        assert_eq!(
            spec.locks[0].description,
            "Taken at entry; if the caller drops this lock mid-operation: the handler waits until released."
        );
        let se = &spec.side_effects[0];
        assert_eq!(
            se.condition.as_deref(),
            Some("only when a special case applies: for example on Tuesdays")
        );
        assert_eq!(se.description, "Does things. Careful: it is dangerous.");
        assert!(!se.reversible);
        let sig = &spec.signals[0];
        assert_eq!(
            sig.condition.as_deref(),
            Some("while blocked, unless flagged: see below")
        );
        assert_eq!(sig.description.as_deref(), Some("Interrupts the wait."));
        let cap = &spec.capabilities[0];
        assert_eq!(cap.allows, "Things that need: privilege");
        assert_eq!(cap.check_condition.as_deref(), Some("Always"));
        let cst = &spec.constraints[0];
        assert_eq!(cst.description, "A limit of note: nothing.");
        assert_eq!(cst.expression.as_deref(), Some("a && b"));
    }

    #[test]
    fn multiline_side_effect_condition_is_kept_whole() {
        let doc = "\
sys_mc - Multi-line condition test
side-effect: modify_state
  target: userfaultfd event queue
  condition: MADV_DONTNEED, MADV_FREE
    on a VMA whose userfaultfd context negotiated
    UFFD_FEATURE_EVENT_REMOVE
  desc: Generates a notification.
    The monitor cannot veto it.
  reversible: no
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_mc", "syscall", None)
            .unwrap();

        let se = &spec.side_effects[0];
        assert_eq!(
            se.condition.as_deref(),
            Some(
                "MADV_DONTNEED, MADV_FREE on a VMA whose userfaultfd context \
                 negotiated UFFD_FEATURE_EVENT_REMOVE"
            )
        );
        assert_eq!(
            se.description,
            "Generates a notification. The monitor cannot veto it."
        );
    }

    #[test]
    fn blank_line_inside_block_continues_subfield() {
        let doc = "\
sys_bl - Blank line test
lock: l
  type: mutex
  desc: First half

    second half.
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_bl", "syscall", None)
            .unwrap();

        assert_eq!(spec.locks[0].description, "First half second half.");
    }

    #[test]
    fn param_cdesc_is_the_constraint_text() {
        let doc = "\
sys_cd - cdesc test
param: fd
  type: int, input
  cdesc: Must be open.
    Zero is allowed.
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_cd", "syscall", None)
            .unwrap();

        assert_eq!(
            spec.parameters[0].constraint.as_deref(),
            Some("Must be open. Zero is allowed.")
        );
    }

    #[test]
    fn state_transition_block_keeps_condition_separate() {
        let doc = "\
sys_stb - State transition block test
state-trans: file_descriptor
  from: open
  to: closed/free
  condition: Valid fd passed to close
  desc: The fd becomes unusable.
    It may be reused.

state-trans: refcount
  from: n
  to: n-1
  desc: Decremented.
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_stb", "syscall", None)
            .unwrap();

        assert_eq!(spec.state_transitions.len(), 2);
        let st = &spec.state_transitions[0];
        assert_eq!(st.object, "file_descriptor");
        assert_eq!(st.from_state, "open");
        assert_eq!(st.to_state, "closed/free");
        assert_eq!(st.condition.as_deref(), Some("Valid fd passed to close"));
        assert_eq!(st.description, "The fd becomes unusable. It may be reused.");
        let st = &spec.state_transitions[1];
        assert_eq!(st.object, "refcount");
        assert_eq!(st.condition, None);
        assert_eq!(st.description, "Decremented.");
    }

    #[test]
    fn state_transition_flat_description_keeps_commas() {
        let doc = "\
sys_stc - Flat state transition test
state-trans: fd, open, closed, File is closed, and the number is free
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_stc", "syscall", None)
            .unwrap();

        let st = &spec.state_transitions[0];
        assert_eq!(st.description, "File is closed, and the number is free");
        assert_eq!(st.condition, None);
    }

    #[test]
    fn return_type_name_is_kept_as_written() {
        for (written, ty) in [("int", 1), ("long", 0), ("KAPI_TYPE_UINT", 2)] {
            let doc = format!("sys_rt - Return type test\nreturn:\n  type: {written}\n");
            let spec = parser()
                .parse_kerneldoc(&doc, "sys_rt", "syscall", None)
                .unwrap();
            let ret = spec.return_spec.as_ref().unwrap();
            assert_eq!(ret.type_name, written);
            assert_eq!(ret.return_type, ty);
        }
    }

    #[test]
    fn return_success_follows_check_type() {
        let cases = [
            // (check-type, success, value, min, max)
            ("exact", "0", Some(0), None, None),
            ("exact", "= 0", Some(0), None, None),
            ("exact", "== -1", Some(-1), None, None),
            ("exact", "0x10", Some(16), None, None),
            ("exact", ">= 0", None, None, None),
            ("range", ">= 0", None, Some(0), Some(i64::MAX)),
            ("range", ">= 1", None, Some(1), Some(i64::MAX)),
            ("range", "0", None, Some(0), Some(i64::MAX)),
            ("fd", ">= 0", None, None, None),
            ("error_check", "0", None, None, None),
        ];
        for (check, success, value, min, max) in cases {
            let doc = format!(
                "sys_rs - Return success test\nreturn:\n  type: int\n  \
                 success: {success}\n  check-type: {check}\n  desc: ok\n"
            );
            let spec = parser()
                .parse_kerneldoc(&doc, "sys_rs", "syscall", None)
                .unwrap();
            let ret = spec.return_spec.as_ref().unwrap();
            assert_eq!(ret.success_value, value, "{check} {success}");
            assert_eq!(ret.success_min, min, "{check} {success}");
            assert_eq!(ret.success_max, max, "{check} {success}");
        }
    }

    #[test]
    fn return_description_continuation_with_colon() {
        let doc = "\
sys_rd - Return description test
return:
  type: int
  check-type: exact
  success: 0
  desc: Returns zero on success. Note: this is
    a continuation line with a colon.
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_rd", "syscall", None)
            .unwrap();

        let ret = spec.return_spec.as_ref().unwrap();
        assert_eq!(
            ret.description,
            "Returns zero on success. Note: this is a continuation line with a colon."
        );
        assert_eq!(ret.success_value, Some(0));
    }

    #[test]
    fn examples_keep_one_example_per_line() {
        let doc = "\
sys_ex - Examples test
examples: fd = open(\"/etc/passwd\", O_RDONLY);  // Read existing file
  fd = open(\"/tmp/new\", O_WRONLY | O_CREAT, 0644);  // Create
  // Handle short writes:
  while (total < len) {
    n = write(fd, buf + total, len - total);
    if (n < 0) break;
  }

  close(fd);

notes: After the examples.
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_ex", "syscall", None)
            .unwrap();

        assert_eq!(
            spec.examples.as_deref(),
            Some(
                "fd = open(\"/etc/passwd\", O_RDONLY);  // Read existing file\n\
                 fd = open(\"/tmp/new\", O_WRONLY | O_CREAT, 0644);  // Create\n\
                 // Handle short writes:\n\
                 while (total < len) {\n  \
                 n = write(fd, buf + total, len - total);\n  \
                 if (n < 0) break;\n\
                 }\n\
                 \n\
                 close(fd);"
            )
        );
        assert_eq!(spec.notes.as_deref(), Some("After the examples."));
    }

    #[test]
    fn notes_and_long_desc_keep_paragraphs_and_bullets() {
        let doc = "\
sys_nt - Notes test
long-desc: First paragraph wraps
  over two lines.

  Second paragraph introduces a list:
  - item one wraps
    onto a second line
  - item two

  Last paragraph.
notes: The behavior varies:

  - Regular files: reads
    from the current position.

  - Pipes: block.

  Trailing: text.
";
        let spec = parser()
            .parse_kerneldoc(doc, "sys_nt", "syscall", None)
            .unwrap();

        assert_eq!(
            spec.long_description.as_deref(),
            Some(
                "First paragraph wraps over two lines.\n\n\
                 Second paragraph introduces a list:\n\
                 - item one wraps onto a second line\n\
                 - item two\n\n\
                 Last paragraph."
            )
        );
        assert_eq!(
            spec.notes.as_deref(),
            Some(
                "The behavior varies:\n\n\
                 - Regular files: reads from the current position.\n\n\
                 - Pipes: block.\n\n\
                 Trailing: text."
            )
        );
    }

    #[test]
    fn fold_lines_handles_leading_blank_and_tabs() {
        assert_eq!(
            super::fold_lines(&["", "  a();", "  b();", "    c();", ""]),
            "a();\nb();\n  c();"
        );
        assert_eq!(super::fold_lines(&["x();", "\ty();"]), "x();\ny();");
        assert_eq!(super::fold_lines(&[""]), "");
        assert_eq!(super::fold_lines(&["a", "  b", "", "", "  c"]), "a\nb\n\nc");
    }

    #[test]
    fn fold_paragraphs_joins_wrapped_lines_only() {
        assert_eq!(
            super::fold_paragraphs(&["one", "  two", "", "", "  three", "- four", "- five"]),
            "one two\n\nthree\n- four\n- five"
        );
        assert_eq!(super::fold_paragraphs(&["", "  "]), "");
    }
}
