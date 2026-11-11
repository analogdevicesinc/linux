// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2025 Meta Platforms, Inc. and affiliates. */
/* arena_strsearch.c and bpf_arena_strsearch.h in plain C */
#include <vmlinux.h>
#include "bpf_experimental.h"
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 100); /* number of pages */
} arena SEC(".maps");

#include <bpf_arena_common.h>

__noinline int bpf_arena_strlen(const char __arena *s)
{
	const char __arena *sc;

	for (sc = s; *sc != '\0'; ++sc)
		;
	return sc - s;
}

/**
 * glob_match - Shell-style pattern matching, like !fnmatch(pat, str, 0)
 * @pat: Shell-style pattern to match, e.g. "*.[ch]".
 * @str: String to match.  The pattern must match the entire string.
 *
 * Perform shell-style glob matching, returning true (1) if the match
 * succeeds, or false (0) if it fails.  Equivalent to !fnmatch(@pat, @str, 0).
 *
 * Pattern metacharacters are ?, *, [ and \.
 * (And, inside character classes, !, - and ].)
 *
 * This is small and simple implementation intended for device blacklists
 * where a string is matched against a number of patterns.  Thus, it
 * does not preprocess the patterns.  It is non-recursive, and run-time
 * is at most quadratic: strlen(@str)*strlen(@pat).
 *
 * An example of the worst case is glob_match("*aaaaa", "aaaaaaaaaa");
 * it takes 6 passes over the pattern before matching the string.
 *
 * Like !fnmatch(@pat, @str, 0) and unlike the shell, this does NOT
 * treat / or leading . specially; it isn't actually used for pathnames.
 *
 * Note that according to glob(7) (and unlike bash), character classes
 * are complemented by a leading !; this does not support the regex-style
 * [^a-z] syntax.
 *
 * An opening bracket without a matching close is matched literally.
 */
__noinline bool glob_match(char const __arena *pat, char const __arena *str)
{
	/*
	 * Backtrack to previous * on mismatch and retry starting one
	 * character later in the string.  Because * matches all characters
	 * (no exception for /), it can be easily proved that there's
	 * never a need to backtrack multiple levels.
	 */
	char const __arena *back_pat = NULL, *back_str;

	/*
	 * Loop over each token (character or class) in pat, matching
	 * it against the remaining unmatched tail of str.  Return false
	 * on mismatch, or true after matching the trailing nul bytes.
	 */
	for (;;) {
		unsigned char c = *str++;
		unsigned char d = *pat++;

		switch (d) {
		case '?':	/* Wildcard: anything but nul */
			if (c == '\0')
				return false;
			break;
		case '*':	/* Any-length wildcard */
			if (*pat == '\0')	/* Optimize trailing * case */
				return true;
			back_pat = pat;
			back_str = --str;	/* Allow zero-length match */
			break;
		case '[': {	/* Character class */
			bool match = false, inverted = (*pat == '!');
			char const __arena *class = pat + inverted;
			unsigned char a = *class++;

			/*
			 * Iterate over each span in the character class.
			 * A span is either a single character a, or a
			 * range a-b.  The first span may begin with ']'.
			 */
			do {
				unsigned char b = a;

				if (a == '\0')	/* Malformed */
					goto literal;

				if (class[0] == '-' && class[1] != ']') {
					b = class[1];

					if (b == '\0')
						goto literal;

					class += 2;
					/* Any special action if a > b? */
				}
				match |= (a <= c && c <= b);
			} while ((a = *class++) != ']');

			if (match == inverted)
				goto backtrack;
			pat = class;
			}
			break;
		case '\\':
			d = *pat++;
			__attribute__((__fallthrough__));
		default:	/* Literal character */
literal:
			if (c == d) {
				if (d == '\0')
					return true;
				break;
			}
backtrack:
			if (c == '\0' || !back_pat)
				return false;	/* No point continuing */
			/* Try again from last *, one character later in str. */
			pat = back_pat;
			str = ++back_str;
			break;
		}
	}
	return false;
}

static bool test(char const __arena *pat, char const __arena *str, bool expected)
{
	bool match = glob_match(pat, str);
	bool success = match == expected;

	/* bpf_printk("glob_match %s %s res %d ok %d", pat, str, match, success); */
	return success;
}

/*
 * The tests are all jammed together in one array to make it simpler
 * to place that array in the .init.rodata section.  The obvious
 * "array of structures containing char *" has no way to force the
 * pointed-to strings to be in a particular section.
 *
 * Anyway, a test consists of:
 * 1. Expected glob_match result: '1' or '0'.
 * 2. Pattern to match: null-terminated string
 * 3. String to match against: null-terminated string
 *
 * The list of tests is terminated with a final '\0' instead of
 * a glob_match result character.
 */
static const char __arena glob_tests[] =
	/* Some basic tests */
	"1" "a\0" "a\0"
	"0" "a\0" "b\0"
	"0" "a\0" "aa\0"
	"0" "a\0" "\0"
	"1" "\0" "\0"
	"0" "\0" "a\0"
	/* Simple character class tests */
	"1" "[a]\0" "a\0"
	"0" "[a]\0" "b\0"
	"0" "[!a]\0" "a\0"
	"1" "[!a]\0" "b\0"
	"1" "[ab]\0" "a\0"
	"1" "[ab]\0" "b\0"
	"0" "[ab]\0" "c\0"
	"1" "[!ab]\0" "c\0"
	"1" "[a-c]\0" "b\0"
	"0" "[a-c]\0" "d\0"
	/* Corner cases in character class parsing */
	"1" "[a-c-e-g]\0" "-\0"
	"0" "[a-c-e-g]\0" "d\0"
	"1" "[a-c-e-g]\0" "f\0"
	"1" "[]a-ceg-ik[]\0" "a\0"
	"1" "[]a-ceg-ik[]\0" "]\0"
	"1" "[]a-ceg-ik[]\0" "[\0"
	"1" "[]a-ceg-ik[]\0" "h\0"
	"0" "[]a-ceg-ik[]\0" "f\0"
	"0" "[!]a-ceg-ik[]\0" "h\0"
	"0" "[!]a-ceg-ik[]\0" "]\0"
	"1" "[!]a-ceg-ik[]\0" "f\0"
	/* Simple wild cards */
	"1" "?\0" "a\0"
	"0" "?\0" "aa\0"
	"0" "??\0" "a\0"
	"1" "?x?\0" "axb\0"
	"0" "?x?\0" "abx\0"
	"0" "?x?\0" "xab\0"
	/* Asterisk wild cards (backtracking) */
	"0" "*??\0" "a\0"
	"1" "*??\0" "ab\0"
	"1" "*??\0" "abc\0"
	"1" "*??\0" "abcd\0"
	"0" "??*\0" "a\0"
	"1" "??*\0" "ab\0"
	"1" "??*\0" "abc\0"
	"1" "??*\0" "abcd\0"
	"0" "?*?\0" "a\0"
	"1" "?*?\0" "ab\0"
	"1" "?*?\0" "abc\0"
	"1" "?*?\0" "abcd\0"
	"1" "*b\0" "b\0"
	"1" "*b\0" "ab\0"
	"0" "*b\0" "ba\0"
	"1" "*b\0" "bb\0"
	"1" "*b\0" "abb\0"
	"1" "*b\0" "bab\0"
	"1" "*bc\0" "abbc\0"
	"1" "*bc\0" "bc\0"
	"1" "*bc\0" "bbc\0"
	"1" "*bc\0" "bcbc\0"
	/* Multiple asterisks (complex backtracking) */
	"1" "*ac*\0" "abacadaeafag\0"
	"1" "*ac*ae*ag*\0" "abacadaeafag\0"
	"1" "*a*b*[bc]*[ef]*g*\0" "abacadaeafag\0"
	"0" "*a*b*[ef]*[cd]*g*\0" "abacadaeafag\0"
	"1" "*abcd*\0" "abcabcabcabcdefg\0"
	"1" "*ab*cd*\0" "abcabcabcabcdefg\0"
	"1" "*abcd*abcdef*\0" "abcabcdabcdeabcdefg\0"
	"0" "*abcd*\0" "abcabcabcabcefg\0"
	"0" "*ab*cd*\0" "abcabcabcabcefg\0";

SEC("syscall")
__success __retval(0) __skip_without_arena
int arena_strsearch(void *ctx)
{
	unsigned successes = 0;
	unsigned n = 0;
	char const __arena *p = glob_tests;

	/*
	 * Tests are jammed together in a string.  The first byte is '1'
	 * or '0' to indicate the expected outcome, or '\0' to indicate the
	 * end of the tests.  Then come two null-terminated strings: the
	 * pattern and the string to match it against.
	 */
	while (*p) {
		bool expected = *p++ & 1;
		char const __arena *pat = p;

		p += bpf_arena_strlen(p) + 1;
		successes += test(pat, p, expected);
		p += bpf_arena_strlen(p) + 1;
		n++;
	}

	n -= successes;
	/* bpf_printk("glob: %u self-tests passed, %u failed\n", successes, n); */

	return n ? -1 : 0;
}

char _license[] SEC("license") = "GPL";
