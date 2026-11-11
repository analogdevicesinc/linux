// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

use crate::extractor::{
    CapabilitySpec, ConstraintSpec, ErrorSpec, LockSpec, ParamSpec, ReturnSpec, SideEffectSpec,
    SignalMaskSpec, SignalSpec, StateTransitionSpec, StructSpec,
};
use std::io::Write;

mod json;
mod plain;
mod rst;

pub use json::JsonFormatter;
pub use plain::PlainFormatter;
pub use rst::RstFormatter;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OutputFormat {
    Plain,
    Json,
    Rst,
}

impl std::str::FromStr for OutputFormat {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "plain" => Ok(OutputFormat::Plain),
            "json" => Ok(OutputFormat::Json),
            "rst" => Ok(OutputFormat::Rst),
            _ => Err(format!("Unknown output format: {}", s)),
        }
    }
}

/// Names of the KAPI_PARAM_* flags set in `flags`.
fn param_flag_names(flags: u32) -> Vec<&'static str> {
    const IN: u32 = 1 << 0;
    const OUT: u32 = 1 << 1;
    const OTHER: [(u32, &str); 6] = [
        (1 << 3, "OPTIONAL"),
        (1 << 4, "CONST"),
        (1 << 5, "VOLATILE"),
        (1 << 6, "USER"),
        (1 << 7, "DMA"),
        (1 << 8, "ALIGNED"),
    ];

    let mut names = Vec::new();
    if flags & (IN | OUT) == IN | OUT {
        names.push("INOUT");
    } else if flags & IN != 0 {
        names.push("IN");
    } else if flags & OUT != 0 {
        names.push("OUT");
    }
    names.extend(
        OTHER
            .iter()
            .filter(|(bit, _)| flags & bit != 0)
            .map(|(_, name)| *name),
    );
    names
}

/// Text for a parameter's bounds; either end may be unset.
fn range_text(min: Option<i64>, max: Option<i64>) -> Option<String> {
    match (min, max) {
        (Some(min), Some(max)) => Some(format!("{min} to {max}")),
        (Some(min), None) => Some(format!(">= {min}")),
        (None, Some(max)) => Some(format!("<= {max}")),
        (None, None) => None,
    }
}

/// Write `text` line by line, indenting every non-blank line by `indent`.
fn write_indented(w: &mut dyn Write, indent: &str, text: &str) -> std::io::Result<()> {
    for line in text.lines() {
        if line.is_empty() {
            writeln!(w)?;
        } else {
            writeln!(w, "{indent}{line}")?;
        }
    }
    Ok(())
}

/// Prepare text with embedded newlines for reStructuredText. Wrapped
/// prose stays in its paragraph, but a bullet list needs a blank line
/// before its first item and after its last one to be recognised.
fn rst_text(text: &str) -> String {
    let mut out: Vec<&str> = Vec::new();
    let mut prev_bullet = false;
    for line in text.lines() {
        let bullet = line.starts_with("- ");
        if !line.is_empty() && bullet != prev_bullet && out.last().is_some_and(|l| !l.is_empty()) {
            out.push("");
        }
        out.push(line);
        if !line.is_empty() {
            prev_bullet = bullet;
        }
    }
    out.join("\n")
}

pub trait OutputFormatter {
    fn begin_document(&mut self, w: &mut dyn Write) -> std::io::Result<()>;
    fn end_document(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn begin_api_list(&mut self, w: &mut dyn Write, title: &str) -> std::io::Result<()>;
    fn api_item(&mut self, w: &mut dyn Write, name: &str, api_type: &str) -> std::io::Result<()>;
    fn end_api_list(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn total_specs(&mut self, w: &mut dyn Write, count: usize) -> std::io::Result<()>;

    fn begin_api_details(&mut self, w: &mut dyn Write, name: &str) -> std::io::Result<()>;
    fn end_api_details(&mut self, w: &mut dyn Write) -> std::io::Result<()>;
    fn description(&mut self, w: &mut dyn Write, desc: &str) -> std::io::Result<()>;
    fn long_description(&mut self, w: &mut dyn Write, desc: &str) -> std::io::Result<()>;

    fn begin_context_flags(&mut self, w: &mut dyn Write) -> std::io::Result<()>;
    fn context_flag(&mut self, w: &mut dyn Write, flag: &str) -> std::io::Result<()>;
    fn end_context_flags(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn begin_parameters(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn parameter(&mut self, w: &mut dyn Write, param: &ParamSpec) -> std::io::Result<()>;
    fn end_parameters(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn return_spec(&mut self, w: &mut dyn Write, ret: &ReturnSpec) -> std::io::Result<()>;

    fn begin_errors(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn error(&mut self, w: &mut dyn Write, error: &ErrorSpec) -> std::io::Result<()>;
    fn end_errors(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn examples(&mut self, w: &mut dyn Write, examples: &str) -> std::io::Result<()>;
    fn notes(&mut self, w: &mut dyn Write, notes: &str) -> std::io::Result<()>;

    // Sysfs-specific methods
    fn sysfs_subsystem(&mut self, w: &mut dyn Write, subsystem: &str) -> std::io::Result<()>;
    fn sysfs_path(&mut self, w: &mut dyn Write, path: &str) -> std::io::Result<()>;
    fn sysfs_permissions(&mut self, w: &mut dyn Write, perms: &str) -> std::io::Result<()>;

    fn begin_capabilities(&mut self, w: &mut dyn Write) -> std::io::Result<()>;
    fn capability(&mut self, w: &mut dyn Write, cap: &CapabilitySpec) -> std::io::Result<()>;
    fn end_capabilities(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    // Signal-related methods
    fn begin_signals(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn signal(&mut self, w: &mut dyn Write, signal: &SignalSpec) -> std::io::Result<()>;
    fn end_signals(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn begin_signal_masks(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn signal_mask(&mut self, w: &mut dyn Write, mask: &SignalMaskSpec) -> std::io::Result<()>;
    fn end_signal_masks(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    // Side effects and state transitions
    fn begin_side_effects(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn side_effect(&mut self, w: &mut dyn Write, effect: &SideEffectSpec) -> std::io::Result<()>;
    fn end_side_effects(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn begin_state_transitions(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn state_transition(
        &mut self,
        w: &mut dyn Write,
        trans: &StateTransitionSpec,
    ) -> std::io::Result<()>;
    fn end_state_transitions(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    // Constraints and locks
    fn begin_constraints(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn constraint(&mut self, w: &mut dyn Write, constraint: &ConstraintSpec)
        -> std::io::Result<()>;
    fn end_constraints(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn begin_locks(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn lock(&mut self, w: &mut dyn Write, lock: &LockSpec) -> std::io::Result<()>;
    fn end_locks(&mut self, w: &mut dyn Write) -> std::io::Result<()>;

    fn begin_struct_specs(&mut self, w: &mut dyn Write, count: u32) -> std::io::Result<()>;
    fn struct_spec(&mut self, w: &mut dyn Write, spec: &StructSpec) -> std::io::Result<()>;
    fn end_struct_specs(&mut self, w: &mut dyn Write) -> std::io::Result<()>;
}

pub fn create_formatter(format: OutputFormat) -> Box<dyn OutputFormatter> {
    match format {
        OutputFormat::Plain => Box::new(PlainFormatter::new()),
        OutputFormat::Json => Box::new(JsonFormatter::new()),
        OutputFormat::Rst => Box::new(RstFormatter::new()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_flags_use_the_header_bit_values() {
        assert!(param_flag_names(0).is_empty());
        assert_eq!(param_flag_names(0x1), ["IN"]);
        assert_eq!(param_flag_names(0x2), ["OUT"]);
        assert_eq!(param_flag_names(0x3), ["INOUT"]);
        assert_eq!(param_flag_names(0x42), ["OUT", "USER"]);
        assert_eq!(param_flag_names(0x9), ["IN", "OPTIONAL"]);
        assert_eq!(
            param_flag_names(0x1f8),
            ["OPTIONAL", "CONST", "VOLATILE", "USER", "DMA", "ALIGNED"]
        );
    }

    #[test]
    fn range_text_shows_one_sided_bounds() {
        assert_eq!(range_text(Some(0), Some(9)).as_deref(), Some("0 to 9"));
        assert_eq!(range_text(Some(1), None).as_deref(), Some(">= 1"));
        assert_eq!(range_text(None, Some(255)).as_deref(), Some("<= 255"));
        assert_eq!(range_text(None, None), None);
    }
}
