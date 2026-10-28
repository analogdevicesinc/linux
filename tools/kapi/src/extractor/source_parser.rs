// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

use super::kerneldoc_parser::KerneldocParser;
use super::{display_api_spec, ApiExtractor, ApiSpec};
use crate::formatter::OutputFormatter;
use anyhow::{bail, Context, Result};
use regex::Regex;
use std::fs;
use std::io::Write;
use std::path::Path;
use walkdir::WalkDir;

/// Extractor for kernel source files with KAPI-annotated kerneldoc
pub struct SourceExtractor {
    path: String,
    parser: KerneldocParser,
    syscall_regex: Regex,
    ioctl_regex: Regex,
    function_regex: Regex,
}

impl SourceExtractor {
    pub fn new(path: &str) -> Result<Self> {
        if !Path::new(path).exists() {
            bail!("Source path does not exist: {path}");
        }

        Ok(SourceExtractor {
            path: path.to_string(),
            parser: KerneldocParser::new(),
            syscall_regex: Regex::new(r"SYSCALL_DEFINE\d+\((\w+)")?,
            ioctl_regex: Regex::new(r"(?:static\s+)?long\s+(\w+_ioctl)\s*\(")?,
            function_regex: Regex::new(concat!(
                r"(?m)^(?:static\s+)?(?:inline\s+)?",
                r"(?:(?:unsigned\s+)?",
                r"(?:long|int|void|char|short",
                r"|struct\s+\w+\s*\*?",
                r"|[\w_]+_t)",
                r"\s*\*?\s+)?",
                r"(\w+)\s*\([^)]*\)",
            ))?,
        })
    }

    fn extract_from_file(&self, path: &Path) -> Result<Vec<ApiSpec>> {
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read file: {}", path.display()))?;

        self.extract_from_content(&content)
    }

    /// Same rule as has-apispec in scripts/Makefile.build: a contexts: (or
    /// context-flags:) line plus one more KAPI section.
    fn has_context_line(doc: &str) -> bool {
        const SECTIONS: &[&str] = &[
            "api-type:",
            "param:",
            "error:",
            "capability:",
            "signal:",
            "lock:",
            "state-trans:",
            "constraint:",
            "side-effect:",
            "long-desc:",
        ];
        let has_line = |keys: &[&str]| {
            doc.lines()
                .map(str::trim_start)
                .any(|l| keys.iter().any(|k| l.starts_with(k)))
        };

        has_line(&["contexts:", "context-flags:"]) && has_line(SECTIONS)
    }

    fn extract_from_content(&self, content: &str) -> Result<Vec<ApiSpec>> {
        let mut specs = Vec::new();
        let mut in_kerneldoc = false;
        let mut current_doc = String::new();
        let lines: Vec<&str> = content.lines().collect();
        let mut i = 0;

        while i < lines.len() {
            let line = lines[i];

            // Start of kerneldoc comment
            if line.trim_start().starts_with("/**") {
                in_kerneldoc = true;
                current_doc.clear();
                i += 1;
                continue;
            }

            // Inside kerneldoc comment
            if in_kerneldoc {
                if line.contains("*/") {
                    in_kerneldoc = false;

                    // Check if this kerneldoc has KAPI annotations
                    if Self::has_context_line(&current_doc) {
                        // Look ahead for the function declaration
                        if let Some((name, api_type, signature)) =
                            self.find_function_after(&lines, i + 1)
                        {
                            if let Ok(spec) = self.parser.parse_kerneldoc(
                                &current_doc,
                                &name,
                                &api_type,
                                Some(&signature),
                            ) {
                                specs.push(spec);
                            }
                        }
                    }
                } else {
                    // Remove leading asterisk and preserve content
                    let cleaned = if let Some(stripped) = line.trim_start().strip_prefix("*") {
                        if let Some(no_space) = stripped.strip_prefix(' ') {
                            no_space
                        } else {
                            stripped
                        }
                    } else {
                        line.trim_start()
                    };
                    current_doc.push_str(cleaned);
                    current_doc.push('\n');
                }
            }

            i += 1;
        }

        Ok(specs)
    }

    fn find_function_after(
        &self,
        lines: &[&str],
        start: usize,
    ) -> Option<(String, String, String)> {
        for i in start..lines.len().min(start + 10) {
            let line = lines[i];

            // Skip blank lines and a plain comment before the function
            let trimmed = line.trim_start();
            if trimmed.is_empty() || trimmed.starts_with("/*") || trimmed.starts_with('*') {
                continue;
            }

            // Check for SYSCALL_DEFINE
            if let Some(caps) = self.syscall_regex.captures(line) {
                let name = format!("sys_{}", caps.get(1).unwrap().as_str());
                let signature = self.extract_syscall_signature(lines, i);
                return Some((name, "syscall".to_string(), signature));
            }

            // Check for ioctl function
            if let Some(caps) = self.ioctl_regex.captures(line) {
                let name = caps.get(1).unwrap().as_str().to_string();
                return Some((name, "ioctl".to_string(), line.to_string()));
            }

            // Check for regular function
            if let Some(caps) = self.function_regex.captures(line) {
                let name = caps.get(1).unwrap().as_str().to_string();
                return Some((name, "function".to_string(), line.to_string()));
            }

            // Stop if we hit something that's clearly not part of the function declaration
            if !line.starts_with(' ') && !line.starts_with('\t') && !line.trim().is_empty() {
                break;
            }
        }

        None
    }

    fn extract_syscall_signature(&self, lines: &[&str], start: usize) -> String {
        // Extract the full SYSCALL_DEFINE signature
        let mut sig = String::new();
        let mut in_paren = false;
        let mut paren_count = 0;

        for line in lines.iter().skip(start).take(20) {
            let line = *line;

            // Start of SYSCALL_DEFINE
            if line.contains("SYSCALL_DEFINE") {
                if let Some(pos) = line.find('(') {
                    sig.push_str(&line[pos..]);
                    in_paren = true;
                    paren_count = line[pos..].chars().filter(|&c| c == '(').count()
                        - line[pos..].chars().filter(|&c| c == ')').count();
                }
            } else if in_paren {
                sig.push(' ');
                sig.push_str(line.trim());
                paren_count += line.chars().filter(|&c| c == '(').count();
                paren_count =
                    paren_count.saturating_sub(line.chars().filter(|&c| c == ')').count());

                if paren_count == 0 {
                    break;
                }
            }
        }

        sig
    }
}

impl ApiExtractor for SourceExtractor {
    fn extract_all(&self) -> Result<Vec<ApiSpec>> {
        let path = Path::new(&self.path);
        let mut all_specs = Vec::new();

        if path.is_file() {
            // Single file
            all_specs.extend(self.extract_from_file(path)?);
        } else if path.is_dir() {
            // Directory - walk all .c files
            for entry in WalkDir::new(path)
                .into_iter()
                .filter_map(|e| e.ok())
                .filter(|e| {
                    e.path()
                        .extension()
                        .is_some_and(|ext| ext == "c" || ext == "h")
                })
            {
                match self.extract_from_file(entry.path()) {
                    Ok(specs) => all_specs.extend(specs),
                    Err(e) => {
                        eprintln!("Warning: failed to parse {}: {}", entry.path().display(), e);
                    }
                }
            }
        }

        Ok(all_specs)
    }

    fn extract_by_name(&self, name: &str) -> Result<Option<ApiSpec>> {
        let all_specs = self.extract_all()?;
        Ok(all_specs.into_iter().find(|s| s.name == name))
    }

    fn display_api_details(
        &self,
        api_name: &str,
        formatter: &mut dyn OutputFormatter,
        output: &mut dyn Write,
    ) -> Result<()> {
        if let Some(spec) = self.extract_by_name(api_name)? {
            display_api_spec(&spec, formatter, output)?;
        } else {
            writeln!(output, "API '{}' not found", api_name)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_extractor() -> SourceExtractor {
        SourceExtractor::new("/dev/null").unwrap()
    }

    #[test]
    fn detect_contexts_spec() {
        let content = r#"
/**
 * sys_foo - do foo
 * @arg: the argument
 *
 * contexts: process, sleepable
 *
 * long-desc: Does foo.
 */
SYSCALL_DEFINE1(foo, int, arg)
{
    return 0;
}

/**
 * bar - not a spec, even though the prose mentions context: here
 * @x: value
 */
SYSCALL_DEFINE1(bar, int, x)
{
    return 0;
}

/**
 * baz - not a spec either
 * @x: value
 *
 * contexts: task, softirq, hardirq, nmi.
 */
SYSCALL_DEFINE1(baz, int, x)
{
    return 0;
}
"#;
        let specs = make_extractor().extract_from_content(content).unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "sys_foo");
    }

    #[test]
    fn prose_mentioning_a_section_key_is_not_a_spec() {
        let content = r#"
/**
 * __probe - check a device
 * @drv: driver
 *
 * returns 0 on success, else error.
 * side-effect: dev->driver is set to drv when drv claims dev.
 */
static int __probe(struct driver *drv, struct device *dev)
{
    return 0;
}
"#;
        let specs = make_extractor().extract_from_content(content).unwrap();
        assert!(specs.is_empty());
    }

    #[test]
    fn nonexistent_path_is_rejected() {
        let err = SourceExtractor::new("/nonexistent/kapi-test")
            .err()
            .unwrap();

        assert!(err.to_string().contains("does not exist"), "{err}");
    }

    #[test]
    fn has_context_line_needs_two_sections() {
        let rule = SourceExtractor::has_context_line;

        assert!(rule("contexts: process\nlong-desc: foo\n"));
        assert!(rule("context-flags: KAPI_CTX_PROCESS\n  lock: foo\n"));
        assert!(rule("contexts: process\nparam: fd\n"));
        assert!(rule("contexts: process\nerror: EINVAL, bad fd\n"));
        assert!(!rule("contexts: process\n"));
        assert!(!rule("context: process\nlong-desc: foo\n"));
        assert!(!rule("long-desc: foo\nside-effect: bar\n"));
    }

    #[test]
    fn detect_syscall_define3() {
        let content = r#"
/**
 * sys_open - open a file
 * context-flags: KAPI_CTX_PROCESS
 * param-count: 3
 * @filename: pathname to open
 * param: filename
 * error: ENOENT, test
 */
SYSCALL_DEFINE3(open, const char __user *, filename, int, flags, umode_t, mode)
{
    return 0;
}
"#;
        let ext = make_extractor();
        let specs = ext.extract_from_content(content).unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "sys_open");
        assert_eq!(specs[0].api_type, "syscall");
    }

    #[test]
    fn detect_syscall_define1() {
        let content = r#"
/**
 * sys_close - close a file descriptor
 * context-flags: KAPI_CTX_PROCESS
 * @fd: file descriptor to close
 * error: EBADF, test
 */
SYSCALL_DEFINE1(close, unsigned int, fd)
{
    return 0;
}
"#;
        let ext = make_extractor();
        let specs = ext.extract_from_content(content).unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "sys_close");
    }

    #[test]
    fn detect_syscall_define6() {
        let content = r#"
/**
 * sys_mmap - map memory
 * context-flags: KAPI_CTX_PROCESS
 * error: ENOMEM, test
 */
SYSCALL_DEFINE6(mmap, unsigned long, addr, unsigned long, len, unsigned long, prot,
    unsigned long, flags, unsigned long, fd, unsigned long, offset)
{
    return 0;
}
"#;
        let ext = make_extractor();
        let specs = ext.extract_from_content(content).unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "sys_mmap");
    }

    #[test]
    fn detect_ioctl_pattern() {
        let content = r#"
/**
 * my_ioctl - handle ioctl
 * context-flags: KAPI_CTX_PROCESS
 * error: EINVAL, test
 */
static long my_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return 0;
}
"#;
        let ext = make_extractor();
        let specs = ext.extract_from_content(content).unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "my_ioctl");
        assert_eq!(specs[0].api_type, "ioctl");
    }

    #[test]
    fn find_function_after_skips_blanks() {
        // Test that find_function_after looks past blank lines
        let lines = vec!["", "", "SYSCALL_DEFINE2(foo, int, bar, int, baz)", "{"];
        let ext = make_extractor();
        let result = ext.find_function_after(&lines, 0);
        assert!(result.is_some());
        let (name, api_type, _sig) = result.unwrap();
        assert_eq!(name, "sys_foo");
        assert_eq!(api_type, "syscall");
    }

    #[test]
    fn find_function_after_skips_plain_comment() {
        let lines = vec![
            "/*",
            " * Careful here!",
            " */",
            "SYSCALL_DEFINE1(close, unsigned int, fd)",
            "{",
        ];
        let ext = make_extractor();
        let (name, api_type, _sig) = ext.find_function_after(&lines, 0).unwrap();
        assert_eq!(name, "sys_close");
        assert_eq!(api_type, "syscall");
    }

    #[test]
    fn find_function_after_returns_none_for_no_match() {
        // No function declaration within lookahead range
        let lines = vec!["#include <linux/fs.h>", "#define FOO 1", "/* comment */"];
        let ext = make_extractor();
        let result = ext.find_function_after(&lines, 0);
        assert!(result.is_none());
    }

    #[test]
    fn find_function_after_detects_regular_function() {
        let lines = vec!["", "int do_something(struct task_struct *task)", "{"];
        let ext = make_extractor();
        let result = ext.find_function_after(&lines, 0);
        assert!(result.is_some());
        let (name, api_type, _) = result.unwrap();
        assert_eq!(name, "do_something");
        assert_eq!(api_type, "function");
    }

    #[test]
    fn no_kapi_annotations_produces_empty() {
        // kerneldoc without any KAPI annotations should not produce a spec
        let content = r#"
/**
 * my_func - does stuff
 * @arg: an argument
 */
void my_func(int arg)
{
}
"#;
        let ext = make_extractor();
        let specs = ext.extract_from_content(content).unwrap();
        assert!(specs.is_empty());
    }

    #[test]
    fn multiple_syscalls_in_one_file() {
        let content = r#"
/**
 * sys_read - read from fd
 * context-flags: KAPI_CTX_PROCESS
 * error: EBADF, test
 */
SYSCALL_DEFINE3(read, unsigned int, fd, char __user *, buf, size_t, count)
{
    return 0;
}

/**
 * sys_write - write to fd
 * context-flags: KAPI_CTX_PROCESS
 * error: EBADF, test
 */
SYSCALL_DEFINE3(write, unsigned int, fd, const char __user *, buf, size_t, count)
{
    return 0;
}
"#;
        let ext = make_extractor();
        let specs = ext.extract_from_content(content).unwrap();
        assert_eq!(specs.len(), 2);
        assert_eq!(specs[0].name, "sys_read");
        assert_eq!(specs[1].name, "sys_write");
    }
}
