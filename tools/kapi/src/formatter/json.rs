// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

use super::OutputFormatter;
use crate::extractor::{
    CapabilitySpec, ConstraintSpec, ErrorSpec, LockSpec, ParamSpec, ReturnSpec, SideEffectSpec,
    SignalMaskSpec, SignalSpec, StateTransitionSpec, StructSpec,
};
use serde::Serialize;
use std::io::Write;

pub struct JsonFormatter {
    data: JsonData,
}

#[derive(Serialize)]
struct JsonData {
    #[serde(skip_serializing_if = "Option::is_none")]
    apis: Option<Vec<JsonApi>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    api_details: Option<JsonApiDetails>,
}

#[derive(Serialize)]
struct JsonApi {
    name: String,
    api_type: String,
}

#[derive(Serialize)]
struct JsonApiDetails {
    name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    long_description: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    context_flags: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    examples: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    notes: Option<String>,
    // Sysfs-specific fields
    #[serde(skip_serializing_if = "Option::is_none")]
    subsystem: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    sysfs_path: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    permissions: Option<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    capabilities: Vec<CapabilitySpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    state_transitions: Vec<StateTransitionSpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    side_effects: Vec<SideEffectSpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    parameters: Vec<ParamSpec>,
    #[serde(skip_serializing_if = "Option::is_none")]
    return_spec: Option<ReturnSpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    errors: Vec<ErrorSpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    locks: Vec<LockSpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    struct_specs: Vec<StructSpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    signals: Vec<SignalSpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    signal_masks: Vec<SignalMaskSpec>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    constraints: Vec<ConstraintSpec>,
}

impl JsonFormatter {
    pub fn new() -> Self {
        JsonFormatter {
            data: JsonData {
                apis: None,
                api_details: None,
            },
        }
    }
}

impl OutputFormatter for JsonFormatter {
    fn begin_document(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn end_document(&mut self, w: &mut dyn Write) -> std::io::Result<()> {
        let json = serde_json::to_string_pretty(&self.data)?;
        writeln!(w, "{json}")?;
        Ok(())
    }

    fn begin_api_list(&mut self, _w: &mut dyn Write, _title: &str) -> std::io::Result<()> {
        if self.data.apis.is_none() {
            self.data.apis = Some(Vec::new());
        }
        Ok(())
    }

    fn api_item(&mut self, _w: &mut dyn Write, name: &str, api_type: &str) -> std::io::Result<()> {
        if let Some(apis) = &mut self.data.apis {
            apis.push(JsonApi {
                name: name.to_string(),
                api_type: api_type.to_string(),
            });
        }
        Ok(())
    }

    fn end_api_list(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn total_specs(&mut self, _w: &mut dyn Write, _count: usize) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_api_details(&mut self, _w: &mut dyn Write, name: &str) -> std::io::Result<()> {
        self.data.api_details = Some(JsonApiDetails {
            name: name.to_string(),
            description: None,
            long_description: None,
            context_flags: Vec::new(),
            examples: None,
            notes: None,
            subsystem: None,
            sysfs_path: None,
            permissions: None,
            capabilities: Vec::new(),
            state_transitions: Vec::new(),
            side_effects: Vec::new(),
            parameters: Vec::new(),
            return_spec: None,
            errors: Vec::new(),
            locks: Vec::new(),
            struct_specs: Vec::new(),
            signals: Vec::new(),
            signal_masks: Vec::new(),
            constraints: Vec::new(),
        });
        Ok(())
    }

    fn end_api_details(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn description(&mut self, _w: &mut dyn Write, desc: &str) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.description = Some(desc.to_string());
        }
        Ok(())
    }

    fn long_description(&mut self, _w: &mut dyn Write, desc: &str) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.long_description = Some(desc.to_string());
        }
        Ok(())
    }

    fn begin_context_flags(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn context_flag(&mut self, _w: &mut dyn Write, flag: &str) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.context_flags.push(flag.to_string());
        }
        Ok(())
    }

    fn end_context_flags(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_parameters(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn end_parameters(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_errors(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn end_errors(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn examples(&mut self, _w: &mut dyn Write, examples: &str) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.examples = Some(examples.to_string());
        }
        Ok(())
    }

    fn notes(&mut self, _w: &mut dyn Write, notes: &str) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.notes = Some(notes.to_string());
        }
        Ok(())
    }

    fn sysfs_subsystem(&mut self, _w: &mut dyn Write, subsystem: &str) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.subsystem = Some(subsystem.to_string());
        }
        Ok(())
    }

    fn sysfs_path(&mut self, _w: &mut dyn Write, path: &str) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.sysfs_path = Some(path.to_string());
        }
        Ok(())
    }

    fn sysfs_permissions(&mut self, _w: &mut dyn Write, perms: &str) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.permissions = Some(perms.to_string());
        }
        Ok(())
    }

    fn begin_capabilities(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn capability(&mut self, _w: &mut dyn Write, cap: &CapabilitySpec) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.capabilities.push(cap.clone());
        }
        Ok(())
    }

    fn end_capabilities(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn parameter(&mut self, _w: &mut dyn Write, param: &ParamSpec) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.parameters.push(param.clone());
        }
        Ok(())
    }

    fn return_spec(&mut self, _w: &mut dyn Write, ret: &ReturnSpec) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.return_spec = Some(ret.clone());
        }
        Ok(())
    }

    fn error(&mut self, _w: &mut dyn Write, error: &ErrorSpec) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.errors.push(error.clone());
        }
        Ok(())
    }

    fn begin_signals(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn signal(&mut self, _w: &mut dyn Write, signal: &SignalSpec) -> std::io::Result<()> {
        if let Some(api_details) = &mut self.data.api_details {
            api_details.signals.push(signal.clone());
        }
        Ok(())
    }

    fn end_signals(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_signal_masks(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn signal_mask(&mut self, _w: &mut dyn Write, mask: &SignalMaskSpec) -> std::io::Result<()> {
        if let Some(api_details) = &mut self.data.api_details {
            api_details.signal_masks.push(mask.clone());
        }
        Ok(())
    }

    fn end_signal_masks(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_side_effects(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn side_effect(&mut self, _w: &mut dyn Write, effect: &SideEffectSpec) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.side_effects.push(effect.clone());
        }
        Ok(())
    }

    fn end_side_effects(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_state_transitions(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn state_transition(
        &mut self,
        _w: &mut dyn Write,
        trans: &StateTransitionSpec,
    ) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.state_transitions.push(trans.clone());
        }
        Ok(())
    }

    fn end_state_transitions(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_constraints(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn constraint(
        &mut self,
        _w: &mut dyn Write,
        constraint: &ConstraintSpec,
    ) -> std::io::Result<()> {
        if let Some(api_details) = &mut self.data.api_details {
            api_details.constraints.push(constraint.clone());
        }
        Ok(())
    }

    fn end_constraints(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_locks(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn lock(&mut self, _w: &mut dyn Write, lock: &LockSpec) -> std::io::Result<()> {
        if let Some(details) = &mut self.data.api_details {
            details.locks.push(lock.clone());
        }
        Ok(())
    }

    fn end_locks(&mut self, _w: &mut dyn Write) -> std::io::Result<()> {
        Ok(())
    }

    fn begin_struct_specs(&mut self, _w: &mut dyn Write, _count: u32) -> std::io::Result<()> {
        Ok(())
    }

    fn struct_spec(&mut self, _w: &mut dyn Write, spec: &StructSpec) -> std::io::Result<()> {
        if let Some(ref mut details) = self.data.api_details {
            details.struct_specs.push(spec.clone());
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

    fn render_json(f: &mut JsonFormatter) -> String {
        let mut buf = Vec::new();
        f.end_document(&mut buf).unwrap();
        String::from_utf8(buf).unwrap()
    }

    #[test]
    fn json_output_is_valid() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.description(&mut sink, "A test syscall").unwrap();
        f.end_api_details(&mut sink).unwrap();

        let json = render_json(&mut f);

        // Verify it parses as valid JSON
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed["api_details"]["name"].as_str(), Some("sys_test"));
        assert_eq!(
            parsed["api_details"]["description"].as_str(),
            Some("A test syscall")
        );
    }

    #[test]
    fn json_api_list() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_list(&mut sink, "Syscalls").unwrap();
        f.api_item(&mut sink, "sys_open", "syscall").unwrap();
        f.api_item(&mut sink, "sys_read", "syscall").unwrap();
        f.end_api_list(&mut sink).unwrap();

        let json = render_json(&mut f);
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();

        let apis = parsed["apis"].as_array().unwrap();
        assert_eq!(apis.len(), 2);
        assert_eq!(apis[0]["name"].as_str(), Some("sys_open"));
        assert_eq!(apis[0]["api_type"].as_str(), Some("syscall"));
        assert_eq!(apis[1]["name"].as_str(), Some("sys_read"));
    }

    #[test]
    fn json_special_characters_in_description() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.description(&mut sink, "Contains \"quotes\" and \\backslashes\\")
            .unwrap();
        f.end_api_details(&mut sink).unwrap();

        let json = render_json(&mut f);

        // Must be valid JSON despite special characters
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(
            parsed["api_details"]["description"].as_str(),
            Some("Contains \"quotes\" and \\backslashes\\")
        );
    }

    #[test]
    fn json_special_characters_in_name() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_list(&mut sink, "APIs").unwrap();
        // Names with underscores (common in kernel) and unusual strings
        f.api_item(&mut sink, "sys_new\tline", "syscall").unwrap();
        f.end_api_list(&mut sink).unwrap();

        let json = render_json(&mut f);

        // Must parse correctly; serde_json handles escaping for us
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed["apis"][0]["name"].as_str(), Some("sys_new\tline"));
    }

    #[test]
    fn json_parameters_serialized() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_write").unwrap();
        f.begin_parameters(&mut sink, 2).unwrap();
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
                min_value: Some(0),
                max_value: Some(1024),
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

        let json = render_json(&mut f);
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();

        let params = parsed["api_details"]["parameters"].as_array().unwrap();
        assert_eq!(params.len(), 1);
        assert_eq!(params[0]["name"].as_str(), Some("fd"));
        assert_eq!(params[0]["param_type"].as_u64(), Some(2));
    }

    #[test]
    fn json_errors_serialized() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_read").unwrap();
        f.begin_errors(&mut sink, 1).unwrap();
        f.error(
            &mut sink,
            &ErrorSpec {
                error_code: -9,
                name: "EBADF".to_string(),
                condition: "fd is not valid".to_string(),
                description: "Bad file descriptor".to_string(),
            },
        )
        .unwrap();
        f.end_errors(&mut sink).unwrap();
        f.end_api_details(&mut sink).unwrap();

        let json = render_json(&mut f);
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();

        let errors = parsed["api_details"]["errors"].as_array().unwrap();
        assert_eq!(errors.len(), 1);
        assert_eq!(errors[0]["name"].as_str(), Some("EBADF"));
        assert_eq!(errors[0]["error_code"].as_i64(), Some(-9));
    }

    #[test]
    fn json_empty_details_omits_empty_fields() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_empty").unwrap();
        f.end_api_details(&mut sink).unwrap();

        let json = render_json(&mut f);
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();

        // description should not be present (skip_serializing_if = Option::is_none)
        assert!(parsed["api_details"]["description"].is_null());
        // parameters empty array should not be present (skip_serializing_if = Vec::is_empty)
        assert!(parsed["api_details"]["parameters"].is_null());
        // errors empty array should not be present
        assert!(parsed["api_details"]["errors"].is_null());
    }

    #[test]
    fn json_braces_balance() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_balanced").unwrap();
        f.description(&mut sink, "Test braces balance").unwrap();
        f.end_api_details(&mut sink).unwrap();

        let json = render_json(&mut f);

        let open_braces = json.chars().filter(|&c| c == '{').count();
        let close_braces = json.chars().filter(|&c| c == '}').count();
        assert_eq!(open_braces, close_braces, "Braces are unbalanced");

        let open_brackets = json.chars().filter(|&c| c == '[').count();
        let close_brackets = json.chars().filter(|&c| c == ']').count();
        assert_eq!(open_brackets, close_brackets, "Brackets are unbalanced");
    }

    #[test]
    fn json_return_spec_serialized() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_open").unwrap();
        f.return_spec(
            &mut sink,
            &ReturnSpec {
                type_name: "int".to_string(),
                description: "file descriptor on success".to_string(),
                return_type: 1,
                check_type: 3,
                success_value: Some(0),
                success_min: None,
                success_max: None,
                error_values: vec![-1],
            },
        )
        .unwrap();
        f.end_api_details(&mut sink).unwrap();

        let json = render_json(&mut f);
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();

        let ret = &parsed["api_details"]["return_spec"];
        assert_eq!(ret["type_name"].as_str(), Some("int"));
        assert_eq!(ret["check_type"].as_u64(), Some(3));
    }

    #[test]
    fn json_unicode_in_description() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_uni").unwrap();
        f.description(&mut sink, "Supports unicode: \u{00e9}\u{00e8}\u{00ea}")
            .unwrap();
        f.end_api_details(&mut sink).unwrap();

        let json = render_json(&mut f);
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert!(parsed["api_details"]["description"]
            .as_str()
            .unwrap()
            .contains('\u{00e9}'));
    }

    #[test]
    fn json_escapes_embedded_newlines() {
        let mut f = JsonFormatter::new();
        let mut sink = Vec::new();

        f.begin_document(&mut sink).unwrap();
        f.begin_api_details(&mut sink, "sys_test").unwrap();
        f.long_description(&mut sink, "One.\n\n- a\n- b").unwrap();
        f.examples(&mut sink, "a();\nb();").unwrap();
        f.notes(&mut sink, "Note one.\n\nNote two.").unwrap();
        f.end_api_details(&mut sink).unwrap();

        let json = render_json(&mut f);
        assert!(json.contains(r#""examples": "a();\nb();""#));
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(
            parsed["api_details"]["long_description"].as_str(),
            Some("One.\n\n- a\n- b")
        );
        assert_eq!(
            parsed["api_details"]["notes"].as_str(),
            Some("Note one.\n\nNote two.")
        );
    }
}
