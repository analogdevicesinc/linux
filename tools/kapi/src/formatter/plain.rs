// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

use super::OutputFormatter;
use crate::extractor::{
    CapabilitySpec, ConstraintSpec, ErrorSpec, LockSpec, ParamSpec, ReturnSpec, SideEffectSpec,
    SignalMaskSpec, SignalSpec, StateTransitionSpec,
};
use std::io::Write;

pub struct PlainFormatter;

impl PlainFormatter {
    pub fn new() -> Self {
        PlainFormatter
    }
}

impl OutputFormatter for PlainFormatter {
    fn begin_document(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn end_document(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_api_list(&mut self, w: &mut dyn Write, title: &str) -> std::io::Result<()> {
        writeln!(w, "\n{title}:")?;
        writeln!(w, "{}", "-".repeat(title.len() + 1))
    }

    fn api_item(&mut self, w: &mut dyn Write, name: &str, _api_type: &str) -> std::io::Result<()> {
        writeln!(w, "  {name}")
    }

    fn end_api_list(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn total_specs(&mut self, w: &mut dyn Write, count: usize) -> std::io::Result<()> {
        writeln!(w, "\nTotal specifications found: {count}")
    }

    fn begin_api_details(&mut self, w: &mut dyn Write, name: &str) -> std::io::Result<()> {
        writeln!(w, "\nDetailed information for {name}:")?;
        writeln!(w, "{}=", "=".repeat(25 + name.len()))
    }

    fn end_api_details(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn description(&mut self, w: &mut dyn Write, desc: &str) -> std::io::Result<()> {
        writeln!(w, "Description: {desc}")
    }

    fn long_description(&mut self, w: &mut dyn Write, desc: &str) -> std::io::Result<()> {
        writeln!(w, "\nDetailed Description:")?;
        super::write_indented(w, "  ", desc)
    }

    fn begin_context_flags(&mut self, w: &mut dyn Write) -> std::io::Result<()> {
        writeln!(w, "\nExecution Context:")
    }

    fn context_flag(&mut self, w: &mut dyn Write, flag: &str) -> std::io::Result<()> {
        writeln!(w, "  - {flag}")
    }

    fn end_context_flags(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_parameters(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nParameters ({count}):")
    }

    fn parameter(&mut self, w: &mut dyn Write, param: &ParamSpec) -> std::io::Result<()> {
        writeln!(
            w,
            "  [{}] {} ({})",
            param.index, param.name, param.type_name
        )?;
        if !param.description.is_empty() {
            writeln!(w, "      {}", param.description)?;
        }

        // Display flags
        let flags = super::param_flag_names(param.flags);
        if !flags.is_empty() {
            writeln!(w, "      Flags: {}", flags.join(" | "))?;
        }

        // Display constraints
        if let Some(constraint) = &param.constraint {
            writeln!(w, "      Constraint: {constraint}")?;
        }
        if let Some(range) = super::range_text(param.min_value, param.max_value) {
            writeln!(w, "      Range: {range}")?;
        }
        if let Some(mask) = param.valid_mask {
            writeln!(w, "      Valid mask: 0x{mask:x}")?;
        }
        Ok(())
    }

    fn end_parameters(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn return_spec(&mut self, w: &mut dyn Write, ret: &ReturnSpec) -> std::io::Result<()> {
        writeln!(w, "\nReturn Value:")?;
        writeln!(w, "  Type: {}", ret.type_name)?;
        writeln!(w, "  {}", ret.description)?;
        if let Some(val) = ret.success_value {
            writeln!(w, "  Success value: {val}")?;
        }
        if let (Some(min), Some(max)) = (ret.success_min, ret.success_max) {
            writeln!(w, "  Success range: {min} to {max}")?;
        }
        Ok(())
    }

    fn begin_errors(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nPossible Errors ({count}):")
    }

    fn error(&mut self, w: &mut dyn Write, error: &ErrorSpec) -> std::io::Result<()> {
        writeln!(w, "  {} ({})", error.name, error.error_code)?;
        if !error.condition.is_empty() {
            writeln!(w, "      Condition: {}", error.condition)?;
        }
        if !error.description.is_empty() {
            writeln!(w, "      {}", error.description)?;
        }
        Ok(())
    }

    fn end_errors(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn examples(&mut self, w: &mut dyn Write, examples: &str) -> std::io::Result<()> {
        writeln!(w, "\nExamples:")?;
        super::write_indented(w, "  ", examples)
    }

    fn notes(&mut self, w: &mut dyn Write, notes: &str) -> std::io::Result<()> {
        writeln!(w, "\nNotes:")?;
        super::write_indented(w, "  ", notes)
    }

    fn sysfs_subsystem(&mut self, w: &mut dyn Write, subsystem: &str) -> std::io::Result<()> {
        writeln!(w, "Subsystem: {subsystem}")
    }

    fn sysfs_path(&mut self, w: &mut dyn Write, path: &str) -> std::io::Result<()> {
        writeln!(w, "Sysfs Path: {path}")
    }

    fn sysfs_permissions(&mut self, w: &mut dyn Write, perms: &str) -> std::io::Result<()> {
        writeln!(w, "Permissions: {perms}")
    }

    fn begin_capabilities(&mut self, w: &mut dyn Write) -> std::io::Result<()> {
        writeln!(w, "\nRequired Capabilities:")
    }

    fn capability(&mut self, w: &mut dyn Write, cap: &CapabilitySpec) -> std::io::Result<()> {
        writeln!(w, "  {} ({}) - {}", cap.name, cap.capability, cap.action)?;
        if !cap.allows.is_empty() {
            writeln!(w, "    Allows: {}", cap.allows)?;
        }
        if !cap.without_cap.is_empty() {
            writeln!(w, "    Without capability: {}", cap.without_cap)?;
        }
        if let Some(cond) = &cap.check_condition {
            writeln!(w, "    Condition: {cond}")?;
        }
        Ok(())
    }

    fn end_capabilities(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    // Signal-related methods
    fn begin_signals(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nSignal Specifications ({count}):")
    }

    fn signal(&mut self, w: &mut dyn Write, signal: &SignalSpec) -> std::io::Result<()> {
        write!(w, "  {} ({})", signal.signal_name, signal.signal_num)?;

        // Display direction (bitmask matching C enum kapi_signal_direction)
        let mut dirs = Vec::new();
        if signal.direction & 1 != 0 {
            dirs.push("RECEIVE");
        }
        if signal.direction & 2 != 0 {
            dirs.push("SEND");
        }
        if signal.direction & 4 != 0 {
            dirs.push("HANDLE");
        }
        if signal.direction & 8 != 0 {
            dirs.push("BLOCK");
        }
        if signal.direction & 16 != 0 {
            dirs.push("IGNORE");
        }
        let direction = if dirs.is_empty() {
            "UNKNOWN".to_string()
        } else {
            dirs.join("|")
        };
        write!(w, " - {direction}")?;

        // Display action (matching C enum kapi_signal_action)
        let action = match signal.action {
            0 => "DEFAULT",
            1 => "TERMINATE",
            2 => "COREDUMP",
            3 => "STOP",
            4 => "CONTINUE",
            5 => "CUSTOM",
            6 => "RETURN",
            7 => "RESTART",
            8 => "QUEUE",
            9 => "DISCARD",
            10 => "TRANSFORM",
            _ => "UNKNOWN",
        };
        writeln!(w, " - {action}")?;

        if let Some(target) = &signal.target {
            writeln!(w, "      Target: {target}")?;
        }
        if let Some(condition) = &signal.condition {
            writeln!(w, "      Condition: {condition}")?;
        }
        if let Some(desc) = &signal.description {
            writeln!(w, "      {desc}")?;
        }

        // Display timing
        let timing = match signal.timing {
            0 => "BEFORE",
            1 => "DURING",
            2 => "AFTER",
            3 => "EXIT",
            _ => "UNKNOWN",
        };
        writeln!(w, "      Timing: {timing}")?;
        writeln!(w, "      Priority: {}", signal.priority)?;

        if signal.restartable {
            writeln!(w, "      Restartable: yes")?;
        }
        if signal.interruptible {
            writeln!(w, "      Interruptible: yes")?;
        }
        if let Some(queue) = &signal.queue {
            writeln!(w, "      Queue: {queue}")?;
        }
        if signal.sa_flags_required != 0 {
            writeln!(
                w,
                "      SA flags required: {:#x}",
                signal.sa_flags_required
            )?;
        }
        if signal.sa_flags_forbidden != 0 {
            writeln!(
                w,
                "      SA flags forbidden: {:#x}",
                signal.sa_flags_forbidden
            )?;
        }
        if signal.state_required != 0 {
            writeln!(w, "      State required: {:#x}", signal.state_required)?;
        }
        if signal.state_forbidden != 0 {
            writeln!(w, "      State forbidden: {:#x}", signal.state_forbidden)?;
        }
        if let Some(error) = signal.error_on_signal {
            writeln!(w, "      Error on signal: {error}")?;
        }
        if let Some(transform) = signal.transform_to {
            writeln!(w, "      Transform to: {transform}")?;
        }
        Ok(())
    }

    fn end_signals(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_signal_masks(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nSignal Masks ({count}):")
    }

    fn signal_mask(&mut self, w: &mut dyn Write, mask: &SignalMaskSpec) -> std::io::Result<()> {
        writeln!(w, "  {}", mask.name)?;
        if !mask.description.is_empty() {
            writeln!(w, "      {}", mask.description)?;
        }
        if !mask.signals.is_empty() {
            let signals: Vec<String> = mask.signals.iter().map(i32::to_string).collect();
            writeln!(w, "      Signals: {}", signals.join(", "))?;
        }
        Ok(())
    }

    fn end_signal_masks(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    // Side effects and state transitions
    fn begin_side_effects(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nSide Effects ({count}):")
    }

    fn side_effect(&mut self, w: &mut dyn Write, effect: &SideEffectSpec) -> std::io::Result<()> {
        writeln!(w, "  {} - {}", effect.target, effect.description)?;
        if let Some(condition) = &effect.condition {
            writeln!(w, "      Condition: {condition}")?;
        }
        if effect.reversible {
            writeln!(w, "      Reversible: yes")?;
        }
        Ok(())
    }

    fn end_side_effects(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_state_transitions(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nState Transitions ({count}):")
    }

    fn state_transition(
        &mut self,
        w: &mut dyn Write,
        trans: &StateTransitionSpec,
    ) -> std::io::Result<()> {
        writeln!(
            w,
            "  {} : {} -> {}",
            trans.object, trans.from_state, trans.to_state
        )?;
        if let Some(condition) = &trans.condition {
            writeln!(w, "      Condition: {condition}")?;
        }
        if !trans.description.is_empty() {
            writeln!(w, "      {}", trans.description)?;
        }
        Ok(())
    }

    fn end_state_transitions(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    // Constraints and locks
    fn begin_constraints(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nAdditional Constraints ({count}):")
    }

    fn constraint(
        &mut self,
        w: &mut dyn Write,
        constraint: &ConstraintSpec,
    ) -> std::io::Result<()> {
        writeln!(w, "  {}", constraint.name)?;
        if !constraint.description.is_empty() {
            writeln!(w, "      {}", constraint.description)?;
        }
        if let Some(expr) = &constraint.expression {
            writeln!(w, "      Expression: {expr}")?;
        }
        Ok(())
    }

    fn end_constraints(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_locks(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nLocking Requirements ({count}):")
    }

    fn lock(&mut self, w: &mut dyn Write, lock: &LockSpec) -> std::io::Result<()> {
        write!(w, "  {}", lock.lock_name)?;

        // Display lock type
        let lock_type = match lock.lock_type {
            0 => "NONE",
            1 => "MUTEX",
            2 => "SPINLOCK",
            3 => "RWLOCK",
            4 => "SEQLOCK",
            5 => "RCU",
            6 => "SEMAPHORE",
            7 => "CUSTOM",
            _ => "UNKNOWN",
        };
        writeln!(w, " ({lock_type})")?;

        let scope_str = match lock.scope {
            0 => "acquired and released",
            1 => "acquired (not released)",
            2 => "released (held on entry)",
            3 => "held by caller",
            _ => "unknown",
        };
        writeln!(w, "      Scope: {scope_str}")?;

        if !lock.description.is_empty() {
            writeln!(w, "      {}", lock.description)?;
        }
        Ok(())
    }

    fn end_locks(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_struct_specs(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()> {
        writeln!(w, "\nStructure Specifications ({count}):")
    }

    fn struct_spec(
        &mut self,
        w: &mut dyn Write,
        spec: &crate::extractor::StructSpec,
    ) -> std::io::Result<()> {
        writeln!(
            w,
            "  {} (size={}, align={}):",
            spec.name, spec.size, spec.alignment
        )?;
        if !spec.description.is_empty() {
            writeln!(w, "      {}", spec.description)?;
        }

        if !spec.fields.is_empty() {
            writeln!(w, "      Fields ({}):", spec.field_count)?;
            for field in &spec.fields {
                write!(w, "        - {} ({}):", field.name, field.type_name)?;
                if !field.description.is_empty() {
                    write!(w, " {}", field.description)?;
                }
                writeln!(w)?;

                // Show constraints if present
                if field.min_value != 0 || field.max_value != 0 {
                    writeln!(
                        w,
                        "          Range: [{}, {}]",
                        field.min_value, field.max_value
                    )?;
                }
                if field.valid_mask != 0 {
                    writeln!(w, "          Mask: {:#x}", field.valid_mask)?;
                }
            }
        }
        Ok(())
    }

    fn end_struct_specs(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extractor::{ErrorSpec, ParamSpec, ReturnSpec};

    fn render_plain(f: &mut PlainFormatter, sink: &mut Vec<u8>) -> String {
        f.end_document(sink).unwrap();
        String::from_utf8(sink.clone()).unwrap()
    }

    #[test]
    fn plain_api_list() {
        let mut f = PlainFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_list(&mut sink, "System Calls").unwrap();
        f.api_item(&mut sink, "sys_open", "syscall").unwrap();
        f.api_item(&mut sink, "sys_read", "syscall").unwrap();
        f.end_api_list(&mut sink).unwrap();
        f.total_specs(&mut sink, 2).unwrap();

        let out = render_plain(&mut f, &mut sink);
        assert!(out.contains("sys_open"));
        assert!(out.contains("sys_read"));
        assert!(out.contains("Total specifications found: 2"));
    }

    #[test]
    fn plain_api_details() {
        let mut f = PlainFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.description(&mut sink, "A test syscall").unwrap();
        f.long_description(&mut sink, "Detailed description here")
            .unwrap();
        f.end_api_details(&mut sink).unwrap();

        let out = render_plain(&mut f, &mut sink);
        assert!(out.contains("sys_test"));
        assert!(out.contains("A test syscall"));
        assert!(out.contains("Detailed description here"));
    }

    #[test]
    fn plain_parameters() {
        let mut f = PlainFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_write").unwrap();
        f.begin_parameters(&mut sink, 1).unwrap();
        f.parameter(
            &mut sink,
            &ParamSpec {
                index: 0,
                name: "fd".to_string(),
                type_name: "unsigned int".to_string(),
                description: "file descriptor".to_string(),
                flags: 1,
                param_type: 2,
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
        )
        .unwrap();
        f.end_parameters(&mut sink).unwrap();
        f.end_api_details(&mut sink).unwrap();

        let out = render_plain(&mut f, &mut sink);
        assert!(out.contains("fd"));
        assert!(out.contains("unsigned int"));
        assert!(out.contains("file descriptor"));
    }

    #[test]
    fn plain_errors() {
        let mut f = PlainFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.begin_errors(&mut sink, 1).unwrap();
        f.error(
            &mut sink,
            &ErrorSpec {
                error_code: -2,
                name: "ENOENT".to_string(),
                condition: "File not found".to_string(),
                description: "The file does not exist".to_string(),
            },
        )
        .unwrap();
        f.end_errors(&mut sink).unwrap();
        f.end_api_details(&mut sink).unwrap();

        let out = render_plain(&mut f, &mut sink);
        assert!(out.contains("ENOENT"));
        assert!(out.contains("-2"));
        assert!(out.contains("File not found"));
    }

    #[test]
    fn plain_return_spec() {
        let mut f = PlainFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.return_spec(
            &mut sink,
            &ReturnSpec {
                type_name: "KAPI_TYPE_INT".to_string(),
                description: "Returns 0 on success".to_string(),
                return_type: 1,
                check_type: 0,
                success_value: Some(0),
                success_min: None,
                success_max: None,
                error_values: vec![],
            },
        )
        .unwrap();
        f.end_api_details(&mut sink).unwrap();

        let out = render_plain(&mut f, &mut sink);
        assert!(out.contains("KAPI_TYPE_INT"));
        assert!(out.contains("Returns 0 on success"));
    }

    #[test]
    fn plain_context_flags() {
        let mut f = PlainFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.begin_context_flags(&mut sink).unwrap();
        f.context_flag(&mut sink, "KAPI_CTX_PROCESS").unwrap();
        f.context_flag(&mut sink, "KAPI_CTX_SLEEPABLE").unwrap();
        f.end_context_flags(&mut sink).unwrap();
        f.end_api_details(&mut sink).unwrap();

        let out = render_plain(&mut f, &mut sink);
        assert!(out.contains("KAPI_CTX_PROCESS"));
        assert!(out.contains("KAPI_CTX_SLEEPABLE"));
    }

    #[test]
    fn plain_signal_mask_lists_its_signals() {
        let mut f = PlainFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.begin_signal_masks(&mut sink, 1).unwrap();
        f.signal_mask(
            &mut sink,
            &SignalMaskSpec {
                name: "blocked".to_string(),
                description: "Blocked while running".to_string(),
                signals: vec![2, 15],
            },
        )
        .unwrap();
        f.end_signal_masks(&mut sink).unwrap();
        f.end_api_details(&mut sink).unwrap();

        let out = render_plain(&mut f, &mut sink);
        assert!(out.contains("  blocked\n      Blocked while running\n      Signals: 2, 15\n"));
    }

    #[test]
    fn plain_multiline_blocks_are_indented() {
        let mut f = PlainFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.long_description(&mut sink, "First paragraph.\n\n- item one\n- item two")
            .unwrap();
        f.examples(&mut sink, "a();\nif (x) {\n  b();\n}").unwrap();
        f.notes(&mut sink, "Note one.\n\nNote two.").unwrap();
        f.end_api_details(&mut sink).unwrap();

        let out = render_plain(&mut f, &mut sink);
        assert!(out
            .contains("Detailed Description:\n  First paragraph.\n\n  - item one\n  - item two\n"));
        assert!(out.contains("Examples:\n  a();\n  if (x) {\n    b();\n  }\n"));
        assert!(out.contains("Notes:\n  Note one.\n\n  Note two.\n"));
    }
}
