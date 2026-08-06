#!/usr/bin/env bash
# Install ADSP agent skills into an agent harness's skills directory.
#
# Usage:
#   ./install.sh                  # install all skills (fail if already installed)
#   ./install.sh --force          # overwrite existing installations
#   ./install.sh --uninstall      # remove installed skills
#   ./install.sh sc59x            # install only the SC59x skill
#   ./install.sh sc594            # install only the SC594 skill
#   ./install.sh sc589            # install only the SC589 skill
#   ./install.sh sc846            # install only the SC846 skill
#   ./install.sh --force sc594    # overwrite only SC594
#   ./install.sh -a [harness]     # install for the given agent harness
#        claude (default) -> ~/.claude/skills  ($CLAUDE_SKILLS_DIR)
#        pi               -> ~/.pi/agent/skills ($PI_SKILLS_DIR)
#
# Skills installed:
#   adsp-sc59x  — ADSP-SC595/SC596/SC598 SHARC+ processor family
#   adsp-sc594  — ADSP-SC592/SC594 (and ADSP-21593/21594) SHARC+ processor family
#   adsp-sc589  — ADSP-SC582/SC583/SC584/SC587/SC589 (and ADSP-21583/21584/21587) SHARC+ processor family
#   adsp-sc846  — ADSP-SC844/SC846 (and ADSP-21844/21846) SHARC-FX processor family

set -euo pipefail

# -------- Configuration --------
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILLS_SRC="$REPO_DIR/skills"
AGENT_HARNESS="claude"
SUPPORTED_HARNESSES=("claude" "pi")

AVAILABLE_SKILLS=("adsp-sc59x" "adsp-sc594" "adsp-sc589" "adsp-sc846")

# -------- Helpers --------
color()  { printf '\033[%sm%s\033[0m' "$1" "$2"; }
info()   { echo "$(color '1;34' '==>') $*"; }
ok()     { echo "$(color '1;32' ' ok') $*"; }
warn()   { echo "$(color '1;33' 'warn') $*" >&2; }
fail()   { echo "$(color '1;31' 'fail') $*" >&2; exit 1; }

usage() {
    sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

# -------- Argument parsing --------
FORCE=0
UNINSTALL=0
SELECTED=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)      usage 0 ;;
        -f|--force)     FORCE=1 ;;
        -u|--uninstall) UNINSTALL=1 ;;
        -a|--agent-harness)
            [[ -n "${2:-}" ]] || fail "$1 requires an argument (one of: ${SUPPORTED_HARNESSES[*]})"
            AGENT_HARNESS="$2"; shift ;;
        sc59x|adsp-sc59x) SELECTED+=("adsp-sc59x") ;;
        sc594|adsp-sc594) SELECTED+=("adsp-sc594") ;;
        sc589|adsp-sc589) SELECTED+=("adsp-sc589") ;;
        sc846|adsp-sc846) SELECTED+=("adsp-sc846") ;;
        *) fail "unknown argument: $1 (try --help)" ;;
    esac
    shift
done

# Default to all skills if none selected
if [[ ${#SELECTED[@]} -eq 0 ]]; then
    SELECTED=("${AVAILABLE_SKILLS[@]}")
fi

case "$AGENT_HARNESS" in
    claude) SKILLS_DST="${CLAUDE_SKILLS_DIR:-$HOME/.claude/skills}" ;;
    pi)     SKILLS_DST="${PI_SKILLS_DIR:-$HOME/.pi/agent/skills}" ;;
    *)      fail "unknown agent harness: $AGENT_HARNESS (supported: ${SUPPORTED_HARNESSES[*]})" ;;
esac

# -------- Preflight --------
[[ -d "$SKILLS_SRC" ]] || fail "source directory not found: $SKILLS_SRC"

for skill in "${SELECTED[@]}"; do
    src="$SKILLS_SRC/$skill"
    [[ -d "$src" ]]          || fail "skill source missing: $src"
    [[ -f "$src/SKILL.md" ]] || fail "skill has no SKILL.md: $src"
done

mkdir -p "$SKILLS_DST"

# -------- Uninstall --------
if [[ $UNINSTALL -eq 1 ]]; then
    for skill in "${SELECTED[@]}"; do
        dst="$SKILLS_DST/$skill"
        if [[ -d "$dst" ]]; then
            info "removing $dst"
            rm -rf "$dst"
            ok   "removed $skill"
        else
            warn "$skill not installed at $dst — skipping"
        fi
    done
    exit 0
fi

# -------- Install --------
# Prefer rsync (fast incremental, excludes pdf/ dirs). Fall back to cp -a.
have_rsync=0
command -v rsync >/dev/null 2>&1 && have_rsync=1

for skill in "${SELECTED[@]}"; do
    src="$SKILLS_SRC/$skill/"
    dst="$SKILLS_DST/$skill"

    if [[ -d "$dst" ]]; then
        if [[ $FORCE -eq 1 ]]; then
            info "overwriting existing install at $dst"
            rm -rf "$dst"
        else
            fail "$skill already installed at $dst — re-run with --force to overwrite"
        fi
    fi

    info "installing $skill → $dst"
    mkdir -p "$dst"

    if [[ $have_rsync -eq 1 ]]; then
        rsync -a --exclude='pdf/' --exclude='.DS_Store' "$src" "$dst/"
    else
        warn "rsync not found; falling back to cp (raw PDFs, if any, will be copied)"
        cp -a "$src". "$dst/"
    fi

    ok "installed $skill"
done

echo
info "done. installed skills live under: $SKILLS_DST"
echo "      adsp-sc59x:  ADSP-SC595/SC596/SC598 SHARC+ processors"
echo "      adsp-sc594:  ADSP-SC592/SC594 (+ ADSP-21593/21594) SHARC+ processors"
echo "      adsp-sc589:  ADSP-SC582/SC583/SC584/SC587/SC589 (+ ADSP-21583/21584/21587) SHARC+ processors"
echo "      adsp-sc846:  ADSP-SC844/SC846 (+ ADSP-21844/21846) SHARC-FX processors"
echo
case "$AGENT_HARNESS" in
    claude) echo "claude code will auto-load these on next session. to verify:" ;;
    pi)     echo "pi will discover these on next session. to verify:" ;;
esac
echo "  ls $SKILLS_DST"
