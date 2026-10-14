#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2026 Sasha Levin <sashal@kernel.org>

"""
Generate C macro invocations for kernel API specifications from kernel-doc comments.

This module creates C header files with API specification macros that match
the kernel API specification framework in include/linux/kernel_api_spec.h.
"""

from kdoc.kdoc_output import OutputFormat
import re
import sys


# Valid KAPI effect types
VALID_EFFECT_TYPES = {
    'KAPI_EFFECT_NONE', 'KAPI_EFFECT_MODIFY_STATE', 'KAPI_EFFECT_PROCESS_STATE',
    'KAPI_EFFECT_IRREVERSIBLE', 'KAPI_EFFECT_SCHEDULE', 'KAPI_EFFECT_FILESYSTEM',
    'KAPI_EFFECT_HARDWARE', 'KAPI_EFFECT_ALLOC_MEMORY', 'KAPI_EFFECT_FREE_MEMORY',
    'KAPI_EFFECT_SIGNAL_SEND', 'KAPI_EFFECT_FILE_POSITION', 'KAPI_EFFECT_LOCK_ACQUIRE',
    'KAPI_EFFECT_LOCK_RELEASE', 'KAPI_EFFECT_RESOURCE_CREATE', 'KAPI_EFFECT_RESOURCE_DESTROY',
    'KAPI_EFFECT_NETWORK'
}

# DSL aliases mapping short tokens to their canonical KAPI_* C
# identifier. Unknown tokens pass through unchanged.
_CTX_ALIASES = {
    'process':          'KAPI_CTX_PROCESS',
    'softirq':          'KAPI_CTX_SOFTIRQ',
    'hardirq':          'KAPI_CTX_HARDIRQ',
    'nmi':              'KAPI_CTX_NMI',
    'atomic':           'KAPI_CTX_ATOMIC',
    'sleepable':        'KAPI_CTX_SLEEPABLE',
    'preempt_disabled': 'KAPI_CTX_PREEMPT_DISABLED',
    'irq_disabled':     'KAPI_CTX_IRQ_DISABLED',
}
_TYPE_ALIASES = {
    'int':      'KAPI_TYPE_INT',
    'uint':     'KAPI_TYPE_UINT',
    'ptr':      'KAPI_TYPE_PTR',
    'struct':   'KAPI_TYPE_STRUCT',
    'union':    'KAPI_TYPE_UNION',
    'enum':     'KAPI_TYPE_ENUM',
    'func_ptr': 'KAPI_TYPE_FUNC_PTR',
    'array':    'KAPI_TYPE_ARRAY',
    'fd':       'KAPI_TYPE_FD',
    'user_ptr': 'KAPI_TYPE_USER_PTR',
    'uptr':     'KAPI_TYPE_USER_PTR',
    'path':     'KAPI_TYPE_PATH',
    'custom':   'KAPI_TYPE_CUSTOM',
}
_FLAG_ALIASES = {
    'input':    'KAPI_PARAM_IN',
    'in':       'KAPI_PARAM_IN',
    'output':   'KAPI_PARAM_OUT',
    'out':      'KAPI_PARAM_OUT',
    'inout':    'KAPI_PARAM_INOUT',
    'optional': 'KAPI_PARAM_OPTIONAL',
    'const':    'KAPI_PARAM_CONST',
    'volatile': 'KAPI_PARAM_VOLATILE',
    'user':     'KAPI_PARAM_USER',
    'dma':      'KAPI_PARAM_DMA',
    'aligned':  'KAPI_PARAM_ALIGNED',
}


def _canon_token(tok, table):
    """Look up `tok` (case-insensitive) in `table`. Unknown tokens
    pass through verbatim."""
    t = tok.strip()
    if not t:
        return ''
    return table.get(t.lower(), t)


def _canon_context_expr(expr):
    """Canonicalise a context flag expression. Accepts '|'- or
    ','-joined tokens; returns a '|'-joined string of KAPI_CTX_*
    identifiers ready for KAPI_CONTEXT()."""
    if not expr:
        return expr
    sep = ',' if ',' in expr and '|' not in expr else '|'
    tokens = [_canon_token(t, _CTX_ALIASES) for t in expr.split(sep)]
    return ' | '.join(t for t in tokens if t)


def _canon_flags_expr(expr):
    """Canonicalise a parameter flags expression. Accepts '|'- or
    ','-joined KAPI_PARAM_* tokens or their aliases; returns a
    '|'-joined canonical string."""
    if not expr:
        return expr
    sep = ',' if ',' in expr and '|' not in expr else '|'
    tokens = [_canon_token(t, _FLAG_ALIASES) for t in expr.split(sep)]
    return ' | '.join(t for t in tokens if t)


# Alias tables for enum families used as block-attribute values
# (lock type, signal direction/action/timing, return check type) and
# for the top-level `side-effect:` bitmask.
_LOCK_TYPE_ALIASES = {
    'none':      'KAPI_LOCK_NONE',
    'mutex':     'KAPI_LOCK_MUTEX',
    'spinlock':  'KAPI_LOCK_SPINLOCK',
    'rwlock':    'KAPI_LOCK_RWLOCK',
    'seqlock':   'KAPI_LOCK_SEQLOCK',
    'rcu':       'KAPI_LOCK_RCU',
    'semaphore': 'KAPI_LOCK_SEMAPHORE',
    'custom':    'KAPI_LOCK_CUSTOM',
}
_SIGNAL_DIR_ALIASES = {
    'receive': 'KAPI_SIGNAL_RECEIVE',
    'send':    'KAPI_SIGNAL_SEND',
    'handle':  'KAPI_SIGNAL_HANDLE',
    'block':   'KAPI_SIGNAL_BLOCK',
    'ignore':  'KAPI_SIGNAL_IGNORE',
}
_SIGNAL_ACTION_ALIASES = {
    'default':   'KAPI_SIGNAL_ACTION_DEFAULT',
    'terminate': 'KAPI_SIGNAL_ACTION_TERMINATE',
    'coredump':  'KAPI_SIGNAL_ACTION_COREDUMP',
    'stop':      'KAPI_SIGNAL_ACTION_STOP',
    'continue':  'KAPI_SIGNAL_ACTION_CONTINUE',
    'custom':    'KAPI_SIGNAL_ACTION_CUSTOM',
    'return':    'KAPI_SIGNAL_ACTION_RETURN',
    'restart':   'KAPI_SIGNAL_ACTION_RESTART',
    'queue':     'KAPI_SIGNAL_ACTION_QUEUE',
    'discard':   'KAPI_SIGNAL_ACTION_DISCARD',
    'transform': 'KAPI_SIGNAL_ACTION_TRANSFORM',
}
_SIGNAL_TIMING_ALIASES = {
    'before': 'KAPI_SIGNAL_TIME_BEFORE',
    'during': 'KAPI_SIGNAL_TIME_DURING',
    'after':  'KAPI_SIGNAL_TIME_AFTER',
}
_EFFECT_ALIASES = {
    'none':             'KAPI_EFFECT_NONE',
    'alloc_memory':     'KAPI_EFFECT_ALLOC_MEMORY',
    'free_memory':      'KAPI_EFFECT_FREE_MEMORY',
    'modify_state':     'KAPI_EFFECT_MODIFY_STATE',
    'signal_send':      'KAPI_EFFECT_SIGNAL_SEND',
    'file_position':    'KAPI_EFFECT_FILE_POSITION',
    'lock_acquire':     'KAPI_EFFECT_LOCK_ACQUIRE',
    'lock_release':     'KAPI_EFFECT_LOCK_RELEASE',
    'resource_create':  'KAPI_EFFECT_RESOURCE_CREATE',
    'resource_destroy': 'KAPI_EFFECT_RESOURCE_DESTROY',
    'schedule':         'KAPI_EFFECT_SCHEDULE',
    'hardware':         'KAPI_EFFECT_HARDWARE',
    'network':          'KAPI_EFFECT_NETWORK',
    'filesystem':       'KAPI_EFFECT_FILESYSTEM',
    'process_state':    'KAPI_EFFECT_PROCESS_STATE',
    'irreversible':     'KAPI_EFFECT_IRREVERSIBLE',
}
_RETURN_CHECK_ALIASES = {
    'exact':       'KAPI_RETURN_EXACT',
    'range':       'KAPI_RETURN_RANGE',
    'error_check': 'KAPI_RETURN_ERROR_CHECK',
    'fd':          'KAPI_RETURN_FD',
    'custom':      'KAPI_RETURN_CUSTOM',
    'no_return':   'KAPI_RETURN_NO_RETURN',
}

# Mapping from short architecture name (as used under arch/<name>/) to the
# kernel CONFIG_* symbol that selects that architecture. Used by the
# `arch-mask:` DSL form to wrap arch-specific mask bits in #ifdef so a
# single generated apispec.h compiles on every architecture and folds
# the right bits into the mask at compile time.
_ARCH_CONFIG = {
    'alpha':       'CONFIG_ALPHA',
    'arc':         'CONFIG_ARC',
    'arm':         'CONFIG_ARM',
    'arm64':       'CONFIG_ARM64',
    'csky':        'CONFIG_CSKY',
    'hexagon':     'CONFIG_HEXAGON',
    'loongarch':   'CONFIG_LOONGARCH',
    'm68k':        'CONFIG_M68K',
    'microblaze':  'CONFIG_MICROBLAZE',
    'mips':        'CONFIG_MIPS',
    'nios2':       'CONFIG_NIOS2',
    'openrisc':    'CONFIG_OPENRISC',
    'parisc':      'CONFIG_PARISC',
    'powerpc':     'CONFIG_PPC',
    'riscv':       'CONFIG_RISCV',
    's390':        'CONFIG_S390',
    'sh':          'CONFIG_SUPERH',
    'sparc':       'CONFIG_SPARC',
    'um':          'CONFIG_UML',
    'x86':         'CONFIG_X86',
    'xtensa':      'CONFIG_XTENSA',
}


def _canon_bitmask_expr(expr, table):
    """Canonicalise a bitmask expression (e.g. signal direction/timing,
    side-effect flags). Accepts `|`- or `,`-joined tokens and returns a
    `|`-joined canonical KAPI_* string."""
    if not expr:
        return expr
    sep = ',' if ',' in expr and '|' not in expr else '|'
    tokens = [_canon_token(t, table) for t in expr.split(sep)]
    return ' | '.join(t for t in tokens if t)


# Types that carry user-space pointer semantics. A param with one of
# these types implicitly gets KAPI_PARAM_USER.
_IMPLIES_USER_FLAG = {'KAPI_TYPE_USER_PTR', 'KAPI_TYPE_PATH'}


def _split_type_line(value):
    """Split a 'type:' line into (type, [flags...]).

    Accepts a single-token value (e.g. 'KAPI_TYPE_UINT' or 'uint')
    leaving flags empty, or a comma-separated form
    (e.g. 'uint, input, user') where the first token is the type and
    subsequent tokens are flag aliases.

    When the type is user-space (user_ptr, path), KAPI_PARAM_USER is
    added to the flags list if not already present."""
    parts = [p.strip() for p in value.split(',') if p.strip()]
    if not parts:
        return None, []
    ty = _canon_token(parts[0], _TYPE_ALIASES)
    flags = [_canon_token(f, _FLAG_ALIASES) for f in parts[1:]]
    if ty in _IMPLIES_USER_FLAG and 'KAPI_PARAM_USER' not in flags:
        flags.append('KAPI_PARAM_USER')
    return ty, flags


def _split_constraint_expr(value):
    """Parse a constraint expression into (canonical_type, extras).

    Shapes:
        NAME                              e.g. 'user_path', 'nonzero'
        NAME ( ARG (, ARG)* )             e.g. 'range(0, 4096)', 'buffer(2)'

    Returns None for free text. Otherwise returns
    (constraint_type, {aux_field: value, ...}) where the aux fields map
    onto the matching param-range / param-mask / param-size /
    param-enum-values / param-constraint slots.
    """
    t = value.strip()
    if not t:
        return None
    # Split NAME ( ARGS )
    lp = t.find('(')
    rp = t.rfind(')')
    if lp > 0 and rp > lp:
        name = t[:lp].strip()
        args_raw = t[lp + 1:rp].strip()
    elif lp < 0:
        name = t
        args_raw = None
    else:
        return None
    # Bareword must be a single identifier; multi-word values are free text.
    if not name or any(c.isspace() for c in name):
        return None
    key = name.lower()
    table = {
        'range':          ('KAPI_CONSTRAINT_RANGE',       'param-range'),
        'mask':           ('KAPI_CONSTRAINT_MASK',        'param-mask'),
        'enum':           ('KAPI_CONSTRAINT_ENUM',        'param-enum-values'),
        'alignment':      ('KAPI_CONSTRAINT_ALIGNMENT',   'param-alignment'),
        'align':          ('KAPI_CONSTRAINT_ALIGNMENT',   'param-alignment'),
        'power_of_two':   ('KAPI_CONSTRAINT_POWER_OF_TWO', None),
        'page_aligned':   ('KAPI_CONSTRAINT_PAGE_ALIGNED', None),
        'nonzero':        ('KAPI_CONSTRAINT_NONZERO',      None),
        'user_string':    ('KAPI_CONSTRAINT_USER_STRING',  'param-size'),
        'user_path':      ('KAPI_CONSTRAINT_USER_PATH',    None),
        'user_ptr':       ('KAPI_CONSTRAINT_USER_PTR',     None),
        'buffer':         ('KAPI_CONSTRAINT_BUFFER',       'param-size-param'),
        'custom':         ('KAPI_CONSTRAINT_CUSTOM',       'param-constraint'),
    }
    if key not in table:
        return None
    ctype, aux_key = table[key]
    extras = {}
    if aux_key and args_raw is not None:
        extras[aux_key] = args_raw
    return ctype, extras


# Subfield names consumed per block type. An indented line opens a new
# subfield only when it starts with one of these followed by ':'; any
# other line continues the previous subfield.
_SIGNAL_SUBFIELDS = frozenset({
    'direction', 'action', 'condition', 'desc', 'errno', 'timing',
    'priority', 'restartable', 'interruptible', 'number', 'target',
    'queue', 'queue_behavior', 'transform', 'transform_to', 'transform-to',
    'sa_flags_required', 'sa-flags-required',
    'sa_flags_forbidden', 'sa-flags-forbidden',
    'state_required', 'state-required',
    'state_forbidden', 'state-forbidden',
})
_LOCK_SUBFIELDS = frozenset({
    'type', 'scope', 'acquired', 'released', 'held-on-entry',
    'held-on-exit', 'desc',
})
_CONSTRAINT_SUBFIELDS = frozenset({'desc', 'expr'})
_SIDE_EFFECT_SUBFIELDS = frozenset({'target', 'desc', 'condition', 'reversible'})
_STATE_TRANS_SUBFIELDS = frozenset({'object', 'from', 'to', 'condition', 'desc'})
_CAPABILITY_SUBFIELDS = frozenset({
    'type', 'allows', 'without', 'condition', 'priority', 'desc',
})
_RETURN_SUBFIELDS = frozenset({
    'type', 'check-type', 'success', 'success-range', 'error-values', 'desc',
})

_RETURN_INT = r'-?(?:0[xX][0-9a-fA-F]+|0[bB][01]+|\d+)[uUlL]*'
_RETURN_EXACT_RE = re.compile(rf'^(?:==?\s*)?({_RETURN_INT})$')
_RETURN_RANGE_RE = re.compile(rf'^>=\s*({_RETURN_INT})$')


def _fold_paragraphs(content):
    """Fold free-form prose into paragraphs.

    Blank lines separate paragraphs ("\\n\\n"); wrapped lines inside a
    paragraph are joined with spaces, except that a line starting with
    "- " always begins a new line so bullet lists survive."""
    paragraphs = []
    current = []
    for line in content.split('\n'):
        line = line.strip()
        if not line:
            if current:
                paragraphs.append(current)
                current = []
        elif line.startswith('- ') or not current:
            current.append(line)
        else:
            current[-1] += ' ' + line
    if current:
        paragraphs.append(current)
    return '\n\n'.join('\n'.join(p) for p in paragraphs) or None


def _fold_lines(content):
    """Keep every line of a block on its own line.

    The indentation shared by the continuation lines is removed so that
    relative indentation (nested code) is preserved; runs of blank lines
    collapse into one."""
    lines = [line.rstrip() for line in content.expandtabs().split('\n')]
    while lines and not lines[0]:
        lines.pop(0)
    while lines and not lines[-1]:
        lines.pop()
    if not lines:
        return None

    indents = [len(line) - len(line.lstrip()) for line in lines[1:] if line]
    base = min(indents) if indents else 0

    out = []
    for line in lines:
        if line:
            line = line[min(base, len(line) - len(line.lstrip())):]
        elif out and not out[-1]:
            continue
        out.append(line)
    return '\n'.join(out)


def _return_success_macro(check_type, value):
    """Translate a return `success:` value into the macro matching the
    check type, or None when the check type does not use one (or the
    value is not a plain integer expression)."""
    if check_type in ('', 'KAPI_RETURN_EXACT'):
        m = _RETURN_EXACT_RE.match(value)
        return f"KAPI_RETURN_SUCCESS({m.group(1)})" if m else None
    if check_type == 'KAPI_RETURN_RANGE':
        m = _RETURN_RANGE_RE.match(value)
        return f"KAPI_RETURN_SUCCESS_RANGE({m.group(1) if m else 0}, S64_MAX)"
    return None


class ApiSpecFormat(OutputFormat):
    """Generate C macro invocations for kernel API specifications"""

    def __init__(self):
        super().__init__()
        self.header_written = False

    def msg(self, fname, name, args):
        """Handles a single entry from kernel-doc parser"""
        if not self.header_written:
            header = self._generate_header()
            self.header_written = True
        else:
            header = ""

        self.data = ""
        result = super().msg(fname, name, args)
        return header + (result if result else self.data)

    def _generate_header(self):
        """Generate the file header"""
        return (
            "/* SPDX-License-Identifier: GPL-2.0 */\n"
            "/* Auto-generated from kerneldoc annotations - DO NOT EDIT */\n\n"
            "#include <linux/capability.h>\n"
            "#include <linux/errno.h>\n"
            "#include <linux/fcntl.h>\n"
            "#include <linux/kernel_api_spec.h>\n"
            "#include <linux/signal.h>\n"
            "#include <linux/stat.h>\n\n"
        )

    def _format_macro_param(self, value, multiline=False):
        """Format a value for use in C macro parameter.

        Every string field in the kernel structs is a `const char *`, so
        there is no length limit. Newlines become spaces unless
        `multiline` is set, in which case they are kept as "\\n" escapes.
        """
        if value is None:
            return '""'
        value = str(value).replace('\\', '\\\\').replace('"', '\\"')
        value = value.replace('\t', ' ').replace('\r', '').replace('\0', '')
        value = value.replace('\n', '\\n' if multiline else ' ')
        return f'"{value}"'

    def _get_section(self, sections, key):
        """Get first line from sections, checking with and without @ prefix and case variants"""
        for variant in [key, key.capitalize(), key.title()]:
            for prefix in ['', '@']:
                full_key = prefix + variant
                if full_key in sections:
                    content = sections[full_key].strip()
                    # Return only first line to avoid mixing sections
                    return content.split('\n')[0].strip() if content else ''
        return None

    def _get_raw_section(self, sections, key):
        """Get full section content, checking with and without @ prefix and case variants"""
        for variant in [key, key.capitalize(), key.title()]:
            for prefix in ['', '@']:
                full_key = prefix + variant
                if full_key in sections:
                    return sections[full_key]
        return ''

    def _get_multiline_section(self, sections, key, fold=_fold_paragraphs):
        """Get a multi-line section, structured by `fold`.

        This is used for fields like notes, long-desc, and examples that
        can span multiple lines in the kerneldoc comment.
        """
        content = self._get_raw_section(sections, key)
        return fold(content) if content else None

    def _parse_indented_items(self, section_content, item_parser):
        """Generic parser for indented items.

        Args:
            section_content: Raw section content
            item_parser: Function that takes (lines, start_index) and returns (item, next_index)

        Returns:
            List of parsed items
        """
        if not section_content:
            return []

        items = []
        lines = section_content.strip().split('\n')
        i = 0

        while i < len(lines):
            if not lines[i].strip():
                i += 1
                continue

            # Check if this is a main item (not indented)
            if not lines[i].startswith((' ', '\t')):
                item, i = item_parser(lines, i)
                if item:
                    items.append(item)
            else:
                i += 1

        return items

    def _parse_subfields(self, lines, start_idx, keys):
        """Parse indented subfields starting from start_idx+1.

        A line opens a new subfield only if it starts with one of `keys`
        followed by ':'; every other line continues the previous one.
        Blank lines inside the block are skipped when more indented
        lines follow.

        Returns: (dict of subfields, next index)
        """
        subfields = {}
        i = start_idx + 1

        current_key = None
        while i < len(lines):
            if not lines[i].strip():
                nxt = i + 1
                while nxt < len(lines) and not lines[nxt].strip():
                    nxt += 1
                if nxt == len(lines) or not lines[nxt].startswith((' ', '\t')):
                    break
                i = nxt
                continue
            if not lines[i].startswith((' ', '\t')):
                break
            line = lines[i].strip()
            key, sep, value = line.partition(':')
            key = key.strip()
            if sep and key in keys:
                current_key = key
                subfields[current_key] = value.strip()
            elif current_key:
                subfields[current_key] = (subfields[current_key] + ' ' + line).strip()
            i += 1

        return subfields, i

    def _parse_signal_item(self, lines, i):
        """Parse a single signal specification"""
        signal = {'name': lines[i].strip()}
        subfields, next_i = self._parse_subfields(lines, i, _SIGNAL_SUBFIELDS)

        # `direction` and `timing` are bitmasks of KAPI_SIGNAL_* /
        # KAPI_SIGNAL_TIME_* values; `action` is a single
        # KAPI_SIGNAL_ACTION_* enum. All three canonicalise aliases to
        # their KAPI_* spelling.
        raw_direction = subfields.get('direction', 'KAPI_SIGNAL_RECEIVE')
        raw_action    = subfields.get('action',    'KAPI_SIGNAL_ACTION_RETURN')
        raw_timing    = subfields.get('timing')
        # `errno:` carries the signal's errno-on-return. The plain
        # `error:` spelling cannot be used inside a signal block
        # because kerneldoc promotes it to a top-level `error:` section.
        signal.update({
            'direction':     _canon_bitmask_expr(raw_direction, _SIGNAL_DIR_ALIASES),
            'action':        _canon_token(raw_action, _SIGNAL_ACTION_ALIASES),
            'condition':     subfields.get('condition'),
            'desc':          subfields.get('desc'),
            'error':         subfields.get('errno'),
            'timing':        _canon_bitmask_expr(raw_timing, _SIGNAL_TIMING_ALIASES)
                              if raw_timing else None,
            'priority':      subfields.get('priority'),
            'restartable':   subfields.get('restartable', '').lower() == 'yes',
            'interruptible': subfields.get('interruptible', '').lower() == 'yes',
            'number':        subfields.get('number', '0'),
            # Additional struct fields. These are optional; if absent, no
            # KAPI_SIGNAL_* macro is emitted and the field stays at its
            # zero-initialised default.
            'target':        subfields.get('target'),
            'queue':         subfields.get('queue') or subfields.get('queue_behavior'),
            'transform':     subfields.get('transform') or subfields.get('transform_to')
                              or subfields.get('transform-to'),
            'sa_flags_required':  subfields.get('sa_flags_required')
                                    or subfields.get('sa-flags-required'),
            'sa_flags_forbidden': subfields.get('sa_flags_forbidden')
                                    or subfields.get('sa-flags-forbidden'),
            'state_required':     subfields.get('state_required')
                                    or subfields.get('state-required'),
            'state_forbidden':    subfields.get('state_forbidden')
                                    or subfields.get('state-forbidden'),
        })

        return signal, next_i

    def _parse_error_item(self, lines, i):
        """Parse a single error specification"""
        line = lines[i].strip()

        # Skip desc: lines
        if line.startswith('desc:'):
            return None, i + 1

        # Check for error pattern
        if not re.match(r'^[A-Z][A-Z0-9_]+,', line):
            return None, i + 1

        error = {'line': line, 'desc': ''}

        # Look for desc: and condition: subfields
        i += 1
        desc_lines = []
        while i < len(lines):
            next_line = lines[i].strip()
            if next_line.startswith('desc:'):
                desc_lines.append(next_line[5:].strip())
                i += 1
            elif next_line.startswith('condition:'):
                error['condition'] = next_line[10:].strip()
                i += 1
            elif not next_line:
                break
            elif not desc_lines and re.match(r'^[A-Z][A-Z0-9_]+,', next_line):
                # New error entry, but only if we haven't started a desc block
                break
            else:
                desc_lines.append(next_line)
                i += 1

        if desc_lines:
            error['desc'] = ' '.join(desc_lines)

        return error, i

    def _parse_lock_item(self, lines, i):
        """Parse a single lock specification.

        Two shapes are accepted:
          * inline `NAME, TYPE` on the main line; or
          * `NAME` on the main line with `type:` as an indented
            subfield.
        Lock-type values are canonicalised to KAPI_LOCK_* spellings.
        """
        head = lines[i].strip()
        if not head:
            return None, i + 1

        parts = head.split(',', 1)
        subfields, next_i = self._parse_subfields(lines, i, _LOCK_SUBFIELDS)

        name = parts[0].strip()
        type_raw = (parts[1].strip() if len(parts) >= 2
                    else subfields.get('type', '').strip())
        if not name or not type_raw:
            return None, next_i

        lock = {
            'name': name,
            'type': _canon_token(type_raw, _LOCK_TYPE_ALIASES),
        }

        for field in ['acquired', 'released', 'held-on-entry', 'held-on-exit']:
            if subfields.get(field, '').lower() in ('true', 'yes'):
                lock[field] = True

        lock['desc'] = subfields.get('desc', '')

        return lock, next_i

    def _parse_constraint_item(self, lines, i):
        """Parse a single constraint specification"""
        line = lines[i].strip()

        # NAME, description form
        if ',' in line:
            parts = line.split(',', 1)
            constraint = {
                'name': parts[0].strip(),
                'desc': parts[1].strip() if len(parts) > 1 else '',
                'expr': None
            }
        else:
            constraint = {'name': line, 'desc': '', 'expr': None}

        subfields, next_i = self._parse_subfields(lines, i, _CONSTRAINT_SUBFIELDS)

        if 'desc' in subfields:
            constraint['desc'] = (constraint['desc'] + ' ' + subfields['desc']).strip()
        constraint['expr'] = subfields.get('expr')

        return constraint, next_i

    def _parse_side_effect_item(self, lines, i):
        """Parse a single side effect specification"""
        line = lines[i].strip()

        # Defaults for the subfield form
        effect = {
            'type': line,
            'target': '',
            'desc': '',
            'condition': None,
            'reversible': False
        }

        # Inline comma-separated form
        if ',' in line:
            # Handle condition and reversible flags
            cond_match = re.search(r',\s*condition=([^,]+?)(?:\s*,\s*reversible=(yes|no)\s*)?$', line)
            if cond_match:
                effect['condition'] = cond_match.group(1).strip()
                effect['reversible'] = cond_match.group(2) == 'yes'
                line = line[:cond_match.start()]
            elif ', reversible=yes' in line:
                effect['reversible'] = True
                line = line.replace(', reversible=yes', '')
            elif ', reversible=no' in line:
                line = line.replace(', reversible=no', '')

            parts = line.split(',', 2)
            if len(parts) >= 1:
                effect['type'] = parts[0].strip()
            if len(parts) >= 2:
                effect['target'] = parts[1].strip()
            if len(parts) >= 3:
                effect['desc'] = parts[2].strip()
        else:
            # Multi-line format with subfields
            subfields, next_i = self._parse_subfields(
                lines, i, _SIDE_EFFECT_SUBFIELDS)
            effect.update({
                'target': subfields.get('target', ''),
                'desc': subfields.get('desc', ''),
                'condition': subfields.get('condition'),
                'reversible': subfields.get('reversible', '').lower() == 'yes'
            })
            return effect, next_i

        return effect, i + 1

    def _parse_state_trans_item(self, lines, i):
        """Parse a single state transition specification"""
        line = lines[i].strip()

        trans = {
            'target': line,
            'from': '',
            'to': '',
            'condition': '',
            'desc': ''
        }

        # Inline comma-separated form
        if ',' in line:
            parts = line.split(',', 3)
            if len(parts) >= 1:
                trans['target'] = parts[0].strip()
            if len(parts) >= 2:
                trans['from'] = parts[1].strip()
            if len(parts) >= 3:
                trans['to'] = parts[2].strip()
            if len(parts) >= 4:
                trans['desc'] = parts[3].strip()
            return trans, i + 1
        else:
            # Multi-line format with subfields
            subfields, next_i = self._parse_subfields(
                lines, i, _STATE_TRANS_SUBFIELDS)
            trans.update({
                'target': subfields.get('object', line),
                'from': subfields.get('from', ''),
                'to': subfields.get('to', ''),
                'condition': subfields.get('condition', ''),
                'desc': subfields.get('desc', '')
            })
            return trans, next_i

    def _process_parameters(self, sections, parameterlist, parameterdescs, parametertypes):
        """Process and output parameter specifications"""
        param_count = len(parameterlist)
        if param_count > 0:
            self.data += f"\n\tKAPI_PARAM_COUNT({param_count})\n"

        for param_idx, param in enumerate(parameterlist):
            param_name = param.strip()
            param_desc = parameterdescs.get(param_name, '').strip()
            param_ctype = parametertypes.get(param_name, '')

            # Parse parameter specifications
            param_section = self._get_raw_section(sections, 'param')
            param_specs = {}
            if param_section:
                param_specs = self._parse_param_spec(param_section, param_name)

            self.data += f"\n\tKAPI_PARAM({param_idx}, {self._format_macro_param(param_name)}, "
            self.data += f"{self._format_macro_param(param_ctype)}, {self._format_macro_param(param_desc)})\n"

            # Add parameter attributes
            for key, macro in [
                ('param-type', 'KAPI_PARAM_TYPE'),
                ('param-flags', 'KAPI_PARAM_FLAGS'),
                ('param-size', 'KAPI_PARAM_SIZE'),
                ('param-alignment', 'KAPI_PARAM_ALIGNMENT'),
            ]:
                if key in param_specs:
                    self.data += f"\t\t{macro}({param_specs[key]})\n"

            # Handle constraint type
            if 'param-constraint-type' in param_specs:
                ctype = param_specs['param-constraint-type']
                self.data += f"\t\tKAPI_PARAM_CONSTRAINT_TYPE({ctype})\n"

            # Handle range
            if 'param-range' in param_specs and ',' in param_specs['param-range']:
                min_val, max_val = param_specs['param-range'].split(',', 1)
                self.data += f"\t\tKAPI_PARAM_RANGE({min_val.strip()}, {max_val.strip()})\n"

            # Handle mask. If `arch-mask:` lines are present, fall back
            # to a raw `.valid_mask = (...)` initializer so we can wrap
            # arch-specific bits in #ifdef CONFIG_<ARCH> ... #endif, which
            # is illegal inside a function-like macro argument.
            if 'param-mask' in param_specs:
                arch_masks = param_specs.get('param-arch-mask') or []
                if arch_masks:
                    base = param_specs['param-mask']
                    self.data += f"\t\t.valid_mask = ({base})"
                    for arch, bits in arch_masks:
                        config = _ARCH_CONFIG.get(arch.lower())
                        if config is None:
                            sys.stderr.write(
                                f"kdoc_apispec: unknown arch '{arch}' in "
                                f"arch-mask: line; skipping\n")
                            continue
                        self.data += (f"\n#ifdef {config}\n"
                                      f"\t\t\t| ({bits})\n"
                                      f"#endif\n")
                    self.data += "\t\t,\n"
                else:
                    self.data += (
                        f"\t\tKAPI_PARAM_VALID_MASK("
                        f"{param_specs['param-mask']})\n")

            # Handle enum values
            if 'param-enum-values' in param_specs:
                self.data += f"\t\tKAPI_PARAM_ENUM_VALUES({param_specs['param-enum-values']})\n"

            # Handle size parameter index
            if 'param-size-param' in param_specs:
                self.data += f"\t\tKAPI_PARAM_SIZE_PARAM({param_specs['param-size-param']})\n"

            # Handle constraint description
            if 'param-constraint' in param_specs:
                self.data += f"\t\tKAPI_PARAM_CONSTRAINT({self._format_macro_param(param_specs['param-constraint'])})\n"

            self.data += "\t},\n"

    def _parse_param_spec(self, section_content, param_name):
        """Parse parameter specifications from indented format"""
        specs = {}
        lines = section_content.strip().split('\n')
        current_item = None

        # Map to expected keys
        field_map = {
            'type': 'param-type',
            'flags': 'param-flags',
            'size': 'param-size',
            'constraint-type': 'param-constraint-type',
            'constraint': 'param-constraint',
            'cdesc': 'param-constraint',
            'range': 'param-range',
            'mask': 'param-mask',
            'valid-mask': 'param-mask',
            'valid-values': 'param-enum-values',
            'alignment': 'param-alignment',
            'size-param': 'param-size-param',
            'struct-type': 'param-struct-type',
            'arch-mask': 'param-arch-mask',
        }

        i = 0
        while i < len(lines):
            line = lines[i]
            if not line.strip():
                i += 1
                continue

            # Check if this is our parameter (non-indented line)
            if not line.startswith((' ', '\t')):
                parts = line.strip().split(',', 1)
                current_item = param_name if parts[0].strip() == param_name else None
                if current_item and len(parts) > 1:
                    specs['param-type'] = parts[1].strip()
                i += 1
            elif current_item == param_name:
                # Parse subfield
                stripped = line.strip()
                if ':' in stripped:
                    key, value = stripped.split(':', 1)
                    key = key.strip()
                    value = value.strip()

                    # Collect continuation lines (indented lines without a colon that
                    # defines a new key, i.e., lines that are pure continuations)
                    i += 1
                    while i < len(lines):
                        next_line = lines[i]
                        # Stop if we hit a non-indented line (new param)
                        if next_line.strip() and not next_line.startswith((' ', '\t')):
                            break
                        next_stripped = next_line.strip()
                        # Stop if we hit a new key (contains colon with known key prefix)
                        if next_stripped and ':' in next_stripped:
                            potential_key = next_stripped.split(':', 1)[0].strip()
                            if potential_key in field_map or potential_key in ['type', 'desc']:
                                break
                        # This is a continuation line
                        if next_stripped:
                            value = value + ' ' + next_stripped
                        i += 1

                    if key in field_map:
                        # Clean up the value - remove excessive whitespace
                        value = ' '.join(value.split())
                        mapped = field_map[key]
                        if mapped == 'param-type':
                            # Single token sets the type; additional
                            # comma-separated tokens are flags OR'd
                            # into param-flags.
                            ty, extra_flags = _split_type_line(value)
                            if ty:
                                specs['param-type'] = ty
                            if extra_flags:
                                existing = specs.get('param-flags', '')
                                merged = (existing + ' | ' if existing else '') \
                                         + ' | '.join(extra_flags)
                                specs['param-flags'] = merged
                        elif mapped == 'param-flags':
                            specs['param-flags'] = _canon_flags_expr(value)
                        elif mapped == 'param-constraint-type':
                            # Accepts a KAPI_CONSTRAINT_* token or a
                            # function-call expression like
                            # `range(0, 4096)` / `mask(0xff)` /
                            # `buffer(2)` that also populates the
                            # matching aux field.
                            parsed = _split_constraint_expr(value)
                            if parsed is not None:
                                ctype, extras = parsed
                                specs['param-constraint-type'] = ctype
                                for aux_k, aux_v in extras.items():
                                    specs[aux_k] = aux_v
                            else:
                                specs['param-constraint-type'] = value
                        elif mapped == 'param-arch-mask':
                            # `arch-mask: <arch> = <bits>`: multiple
                            # entries accumulate into a list of
                            # (arch, bits) tuples that the emitter
                            # turns into per-arch #ifdef-guarded mask
                            # contributions.
                            if '=' in value:
                                arch, bits = value.split('=', 1)
                                specs.setdefault(mapped, []).append(
                                    (arch.strip(), bits.strip()))
                        else:
                            specs[mapped] = value
                else:
                    i += 1
            else:
                i += 1

        return specs

    def _validate_effect_type(self, effect_type):
        """Validate and normalize effect type"""
        if 'KAPI_EFFECT_' in effect_type and effect_type not in VALID_EFFECT_TYPES:
            if '|' in effect_type:
                parts = [p.strip() for p in effect_type.split('|')]
                valid_parts = []
                for p in parts:
                    if p in VALID_EFFECT_TYPES:
                        valid_parts.append(p)
                    else:
                        print(f"warning: unrecognized effect type '{p}', "
                              f"defaulting to KAPI_EFFECT_MODIFY_STATE", file=sys.stderr)
                        valid_parts.append('KAPI_EFFECT_MODIFY_STATE')
                return ' | '.join(valid_parts)
            print(f"warning: unrecognized effect type '{effect_type}', "
                  f"defaulting to KAPI_EFFECT_MODIFY_STATE", file=sys.stderr)
            return 'KAPI_EFFECT_MODIFY_STATE'

        return effect_type

    def _has_api_spec(self, sections):
        """Check if this function has an API specification.

        Returns True if a `contexts:` or `context-flags:` section is present
        together with at least one other KAPI section. Regular kernel-doc
        comments that only use a common section name like 'return' or
        'error' do not produce a spec.
        """
        context_keys = ['context-flags', 'contexts']
        indicators = [
            'api-type', 'param', 'error', 'capability', 'signal', 'lock',
            'state-trans', 'constraint', 'side-effect', 'long-desc'
        ]

        def has_section(names):
            return any(key.lower().startswith(name) or
                       key.lower().startswith('@' + name)
                       for key in sections.keys() for name in names)

        return has_section(context_keys) and has_section(indicators)

    def out_function(self, fname, name, args):
        """Generate API spec for a function"""
        function_name = args.get('function', name)
        sections = args.sections if hasattr(args, 'sections') else args.get('sections', {})

        if not self._has_api_spec(sections):
            return

        parameterlist = args.parameterlist if hasattr(args, 'parameterlist') else args.get('parameterlist', [])
        parameterdescs = args.parameterdescs if hasattr(args, 'parameterdescs') else args.get('parameterdescs', {})
        parametertypes = args.parametertypes if hasattr(args, 'parametertypes') else args.get('parametertypes', {})
        purpose = args.get('purpose', '')

        # Start macro invocation
        self.data += f"DEFINE_KERNEL_API_SPEC({function_name})\n"

        # Basic info
        if purpose:
            self.data += f"\tKAPI_DESCRIPTION({self._format_macro_param(purpose)})\n"

        long_desc = self._get_multiline_section(sections, 'long-desc')
        if long_desc:
            self.data += f"\tKAPI_LONG_DESC({self._format_macro_param(long_desc, True)})\n"

        # Context flags. `contexts:` and `context-flags:` both work;
        # tokens canonicalise to KAPI_CTX_* for KAPI_CONTEXT().
        context = (self._get_section(sections, 'contexts')
                   or self._get_section(sections, 'context-flags'))
        if context:
            self.data += f"\tKAPI_CONTEXT({_canon_context_expr(context)})\n"

        # Process parameters
        self._process_parameters(sections, parameterlist, parameterdescs, parametertypes)

        # Process return value
        self._process_return(sections)

        # Process errors
        errors = self._parse_indented_items(
            self._get_raw_section(sections, 'error'),
            self._parse_error_item
        )

        if errors:
            self.data += f"\n\tKAPI_ERROR_COUNT({len(errors)})\n"

            for idx, error in enumerate(errors):
                self._output_error(idx, error)

        # Process signals
        signals = self._parse_indented_items(
            self._get_raw_section(sections, 'signal'),
            self._parse_signal_item
        )

        if signals:
            self.data += f"\n\tKAPI_SIGNAL_COUNT({len(signals)})\n"

            for idx, signal in enumerate(signals):
                self._output_signal(idx, signal)

        # Process other specifications
        self._process_locks(sections)
        self._process_constraints(sections)
        self._process_side_effects(sections)
        self._process_state_transitions(sections)
        self._process_capabilities(sections)

        # Add examples and notes
        for key, macro, fold in [
            ('examples', 'KAPI_EXAMPLES', _fold_lines),
            ('notes', 'KAPI_NOTES', _fold_paragraphs),
        ]:
            value = self._get_multiline_section(sections, key, fold)
            if value:
                self.data += f"\n\t{macro}({self._format_macro_param(value, True)})\n"

        self.data += "\n};\n\n"

    def _process_return(self, sections):
        """Process the return value specification from kerneldoc annotations"""
        raw = self._get_raw_section(sections, 'return')
        if not raw:
            return

        # Parse subfields from the return section, handling continuation lines
        lines = raw.strip().split('\n')
        subfields = {}
        current_key = None
        for line in lines:
            stripped = line.strip()
            key, sep, value = stripped.partition(':')
            key = key.strip()
            if sep and key in _RETURN_SUBFIELDS:
                current_key = key
                subfields[current_key] = value.strip()
            elif current_key and stripped:
                # Continuation line
                subfields[current_key] += ' ' + stripped

        ret_type = subfields.get('type', '')
        check_type = subfields.get('check-type', '')
        desc = subfields.get('desc', '')
        success = subfields.get('success', '')

        if not ret_type and not desc:
            return

        # Canonicalise short aliases:
        #   type: int             -> KAPI_TYPE_INT
        #   check-type: fd        -> KAPI_RETURN_FD
        # The type name itself is kept as written.
        type_name = ret_type
        if ret_type:
            ret_type = _canon_token(ret_type, _TYPE_ALIASES)
        if check_type:
            check_type = _canon_token(check_type, _RETURN_CHECK_ALIASES)

        self.data += f"\n\tKAPI_RETURN({self._format_macro_param(type_name)}, "
        self.data += f"{self._format_macro_param(desc)})\n"

        if ret_type:
            self.data += f"\t\tKAPI_RETURN_TYPE({ret_type})\n"

        if check_type:
            self.data += f"\t\tKAPI_RETURN_CHECK_TYPE({check_type})\n"

        if success:
            macro = _return_success_macro(check_type, success)
            if macro:
                self.data += f"\t\t{macro}\n"
            elif check_type in ('', 'KAPI_RETURN_EXACT'):
                sys.stderr.write(
                    f"kdoc_apispec: ignoring unsupported success value "
                    f"'{success}' for an exact return check\n")

        self.data += "\t},\n"

    def _output_error(self, idx, error):
        """Output a single error specification"""
        # Format: NAME, description
        parts = error['line'].split(',', 1)
        if len(parts) < 2:
            return

        name = parts[0].strip()
        short_desc = parts[1].strip()
        code = f"-{name}"

        condition = error.get('condition') or short_desc
        long_desc = error.get('desc', '') or short_desc

        self.data += f"\n\tKAPI_ERROR({idx}, {code}, {self._format_macro_param(name)}, "
        self.data += f"{self._format_macro_param(condition)},\n\t\t   {self._format_macro_param(long_desc)})\n"

    def _output_signal(self, idx, signal):
        """Output a single signal specification"""
        self.data += f"\n\tKAPI_SIGNAL({idx}, {signal['number']}, "
        self.data += f"{self._format_macro_param(signal['name'])}, "
        self.data += f"{signal['direction']}, {signal['action']})\n"

        # String-valued subfields emitted as KAPI_SIGNAL_* macros.
        if signal.get('condition'):
            self.data += f"\t\tKAPI_SIGNAL_CONDITION({self._format_macro_param(signal['condition'])})\n"
        if signal.get('desc'):
            self.data += f"\t\tKAPI_SIGNAL_DESC({self._format_macro_param(signal['desc'])})\n"
        if signal.get('error'):
            # KAPI_SIGNAL_ERROR expects a numeric/token expression
            # (e.g. -EINTR), not a quoted string.
            self.data += f"\t\tKAPI_SIGNAL_ERROR({signal['error']})\n"

        # Enum-valued subfields emitted as unquoted tokens.
        if signal.get('timing'):
            self.data += f"\t\tKAPI_SIGNAL_TIMING({signal['timing']})\n"
        if signal.get('priority'):
            self.data += f"\t\tKAPI_SIGNAL_PRIORITY({signal['priority']})\n"

        # Boolean flag subfields.
        if signal.get('restartable'):
            self.data += "\t\tKAPI_SIGNAL_RESTARTABLE\n"
        if signal.get('interruptible'):
            self.data += "\t\tKAPI_SIGNAL_INTERRUPTIBLE\n"

        # Additional struct fields, emitted only when present in the
        # kerneldoc.
        if signal.get('target'):
            self.data += f"\t\tKAPI_SIGNAL_TARGET({self._format_macro_param(signal['target'])})\n"
        if signal.get('queue'):
            self.data += f"\t\tKAPI_SIGNAL_QUEUE({self._format_macro_param(signal['queue'])})\n"
        if signal.get('transform'):
            # Numeric/token expression (e.g. SIGKILL), not a quoted string.
            self.data += f"\t\tKAPI_SIGNAL_TRANSFORM({signal['transform']})\n"
        if signal.get('sa_flags_required'):
            self.data += f"\t\tKAPI_SIGNAL_SA_FLAGS_REQ({signal['sa_flags_required']})\n"
        if signal.get('sa_flags_forbidden'):
            self.data += f"\t\tKAPI_SIGNAL_SA_FLAGS_FORBID({signal['sa_flags_forbidden']})\n"
        if signal.get('state_required'):
            self.data += f"\t\tKAPI_SIGNAL_STATE_REQ({signal['state_required']})\n"
        if signal.get('state_forbidden'):
            self.data += f"\t\tKAPI_SIGNAL_STATE_FORBID({signal['state_forbidden']})\n"

        self.data += "\t},\n"

    def _process_locks(self, sections):
        """Process lock specifications"""
        locks = self._parse_indented_items(
            self._get_raw_section(sections, 'lock'),
            self._parse_lock_item
        )

        if locks:
            self.data += f"\n\tKAPI_LOCK_COUNT({len(locks)})\n"

            for idx, lock in enumerate(locks):
                self.data += f"\n\tKAPI_LOCK({idx}, {self._format_macro_param(lock['name'])}, {lock['type']})\n"

                # `.scope` is zero-initialised to KAPI_LOCK_INTERNAL
                # (acquired-and-released). Emit KAPI_LOCK_ACQUIRED /
                # KAPI_LOCK_RELEASED only when exactly one of the flags
                # is true; emitting both would double-initialise `.scope`
                # which breaks `-Werror=override-init` at W=1.
                acquired = bool(lock.get('acquired'))
                released = bool(lock.get('released'))
                if acquired and not released:
                    self.data += "\t\tKAPI_LOCK_ACQUIRED\n"
                elif released and not acquired:
                    self.data += "\t\tKAPI_LOCK_RELEASED\n"

                if lock.get('desc'):
                    self.data += f"\t\tKAPI_LOCK_DESC({self._format_macro_param(lock['desc'])})\n"

                self.data += "\t},\n"

    def _process_constraints(self, sections):
        """Process constraint specifications"""
        constraints = self._parse_indented_items(
            self._get_raw_section(sections, 'constraint'),
            self._parse_constraint_item
        )

        if constraints:
            self.data += f"\n\tKAPI_CONSTRAINT_COUNT({len(constraints)})\n"

            for idx, constraint in enumerate(constraints):
                self.data += f"\n\tKAPI_CONSTRAINT({idx}, {self._format_macro_param(constraint['name'])},\n"
                self.data += f"\t\t\t{self._format_macro_param(constraint['desc'])})\n"

                if constraint.get('expr'):
                    self.data += f"\t\tKAPI_CONSTRAINT_EXPR({self._format_macro_param(constraint['expr'])})\n"

                self.data += "\t},\n"

    def _process_side_effects(self, sections):
        """Process side effect specifications"""
        effects = self._parse_indented_items(
            self._get_raw_section(sections, 'side-effect'),
            self._parse_side_effect_item
        )

        if effects:
            self.data += f"\n\tKAPI_SIDE_EFFECT_COUNT({len(effects)})\n"

            for idx, effect in enumerate(effects):
                # Canonicalise aliases (alloc_memory, modify_state, ...)
                # to KAPI_EFFECT_*. Accepts '|' or ',' as separators.
                effect_type = _canon_bitmask_expr(effect['type'], _EFFECT_ALIASES)
                effect_type = self._validate_effect_type(effect_type)

                self.data += f"\n\tKAPI_SIDE_EFFECT({idx}, {effect_type},\n"
                self.data += f"\t\t\t {self._format_macro_param(effect['target'])},\n"
                self.data += f"\t\t\t {self._format_macro_param(effect['desc'])})\n"

                if effect.get('condition'):
                    self.data += f"\t\tKAPI_EFFECT_CONDITION({self._format_macro_param(effect['condition'])})\n"

                if effect.get('reversible'):
                    self.data += "\t\tKAPI_EFFECT_REVERSIBLE\n"

                self.data += "\t},\n"

    def _process_state_transitions(self, sections):
        """Process state transition specifications"""
        transitions = self._parse_indented_items(
            self._get_raw_section(sections, 'state-trans'),
            self._parse_state_trans_item
        )

        if transitions:
            self.data += f"\n\tKAPI_STATE_TRANS_COUNT({len(transitions)})\n"

            for idx, trans in enumerate(transitions):
                self.data += f"\n\tKAPI_STATE_TRANS({idx}, {self._format_macro_param(trans['target'])}, "
                self.data += f"{self._format_macro_param(trans['from'])}, {self._format_macro_param(trans['to'])},\n"
                self.data += f"\t\t\t {self._format_macro_param(trans['desc'])})\n"

                if trans.get('condition'):
                    self.data += f"\t\tKAPI_STATE_TRANS_COND({self._format_macro_param(trans['condition'])})\n"

                self.data += "\t},\n"

    def _process_capabilities(self, sections):
        """Process capability specifications"""
        cap_section = self._get_raw_section(sections, 'capability')
        if not cap_section:
            return

        lines = cap_section.strip().split('\n')
        capabilities = []
        i = 0

        while i < len(lines):
            line = lines[i].strip()
            # Skip empty lines and subfield lines (they'll be parsed with their parent)
            if not line or line.startswith(('allows:', 'without:', 'condition:', 'priority:', 'type:', 'desc:')):
                i += 1
                continue

            cap_info = {'line': line}

            # Parse subfields
            subfields, next_i = self._parse_subfields(
                lines, i, _CAPABILITY_SUBFIELDS)
            cap_info.update(subfields)
            capabilities.append(cap_info)
            i = next_i

        if capabilities:
            # Filter out "none" capabilities (no capability required)
            valid_caps = [cap for cap in capabilities if cap['line'].strip().lower() != 'none']

            if not valid_caps:
                return

            self.data += f"\n\tKAPI_CAPABILITY_COUNT({len(valid_caps)})\n"

            for idx, cap in enumerate(valid_caps):
                line = cap['line']
                parts = line.split(',', 2)

                # Two forms are accepted:
                # 1. "CAP_NAME" with type/desc as subfields
                # 2. "CAP_NAME, TYPE, description"
                if len(parts) >= 2:
                    # Comma-separated form
                    cap_name = parts[0].strip()
                    cap_type = parts[1].strip()
                    cap_desc = parts[2].strip() if len(parts) > 2 else cap.get('desc', cap_name)
                else:
                    # Subfield form: capability name on main line
                    cap_name = line.strip()
                    cap_type = cap.get('type', 'KAPI_CAP_PERFORM_OPERATION')
                    cap_desc = cap.get('desc', cap_name)

                # Map capability type aliases to KAPI_CAP_* enum values.
                cap_type_map = {
                    'required':             'KAPI_CAP_PERFORM_OPERATION',
                    'bypass':               'KAPI_CAP_BYPASS_CHECK',
                    'grant':                'KAPI_CAP_GRANT_PERMISSION',
                    'override':             'KAPI_CAP_OVERRIDE_RESTRICTION',
                    'access':               'KAPI_CAP_ACCESS_RESOURCE',
                    'modify':               'KAPI_CAP_MODIFY_BEHAVIOR',
                    'limit':                'KAPI_CAP_INCREASE_LIMIT',
                    'bypass_check':         'KAPI_CAP_BYPASS_CHECK',
                    'increase_limit':       'KAPI_CAP_INCREASE_LIMIT',
                    'override_restriction': 'KAPI_CAP_OVERRIDE_RESTRICTION',
                    'grant_permission':     'KAPI_CAP_GRANT_PERMISSION',
                    'modify_behavior':      'KAPI_CAP_MODIFY_BEHAVIOR',
                    'access_resource':      'KAPI_CAP_ACCESS_RESOURCE',
                    'perform_operation':    'KAPI_CAP_PERFORM_OPERATION',
                }
                cap_type = cap_type_map.get(cap_type, cap_type)

                # Any BYPASS variant maps to KAPI_CAP_BYPASS_CHECK
                if 'BYPASS' in cap_type and cap_type != 'KAPI_CAP_BYPASS_CHECK':
                    cap_type = 'KAPI_CAP_BYPASS_CHECK'

                # Ensure cap_type is a valid enum
                valid_types = [
                    'KAPI_CAP_BYPASS_CHECK', 'KAPI_CAP_INCREASE_LIMIT',
                    'KAPI_CAP_OVERRIDE_RESTRICTION', 'KAPI_CAP_GRANT_PERMISSION',
                    'KAPI_CAP_MODIFY_BEHAVIOR', 'KAPI_CAP_ACCESS_RESOURCE',
                    'KAPI_CAP_PERFORM_OPERATION'
                ]
                if cap_type not in valid_types:
                    cap_type = 'KAPI_CAP_PERFORM_OPERATION'

                self.data += f"\n\tKAPI_CAPABILITY({idx}, {cap_name}, {self._format_macro_param(cap_desc)}, {cap_type})\n"

                for key, macro in [
                    ('allows', 'KAPI_CAP_ALLOWS'),
                    ('without', 'KAPI_CAP_WITHOUT'),
                    ('condition', 'KAPI_CAP_CONDITION'),
                    ('priority', 'KAPI_CAP_PRIORITY'),
                ]:
                    if cap.get(key):
                        value = self._format_macro_param(cap[key]) if key != 'priority' else cap[key]
                        self.data += f"\t\t{macro}({value})\n"

                self.data += "\t},\n"

    # Skip output methods for non-function types
    def out_enum(self, fname, name, args): pass
    def out_typedef(self, fname, name, args): pass
    def out_struct(self, fname, name, args): pass
    def out_doc(self, fname, name, args): pass
