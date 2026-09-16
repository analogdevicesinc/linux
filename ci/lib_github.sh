#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# GitHub Actions artifact utilities
# Shared functions for downloading and extracting workflow artifacts
#

# Source logging utilities if available
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "${SCRIPT_DIR}/lib.sh" ]] && source "${SCRIPT_DIR}/lib.sh"

#######################################
# Get workflow artifacts metadata from GitHub API
# Note: This only retrieves metadata (names, download URLs), not the artifact content.
# The GitHub API paginates this endpoint (30 items/page by default, 100 max), while
# .total_count reports the run total. This function walks every page and re-emits a
# single, complete payload so callers never silently miss artifacts past the first page.
# Arguments:
#   $1 - GitHub token
#   $2 - Repository (owner/repo)
#   $3 - Workflow run ID
# Outputs:
#   JSON object {total_count, artifacts:[...]} with all pages merged
# Returns:
#   0 on success, 1 if any page fails to fetch
#######################################
gh_get_workflow_artifacts() {
    local token="$1"
    local repository="$2"
    local run_id="$3"
    local per_page=100

    local page=1
    local total_count=""
    local collected=0
    local all_artifacts="[]"

    while :; do
        local response
        response=$(curl -sfL \
            -H "Accept: application/vnd.github+json" \
            -H "Authorization: Bearer ${token}" \
            -H "X-GitHub-Api-Version: 2022-11-28" \
            "https://api.github.com/repos/${repository}/actions/runs/${run_id}/artifacts?per_page=${per_page}&page=${page}") \
            || { echo "::error::Failed to fetch artifacts page ${page} for run ${run_id}" >&2; return 1; }

        # total_count is identical on every page; capture it once.
        if [[ -z "${total_count}" ]]; then
            total_count=$(echo "${response}" | jq -r '.total_count // 0')
        fi

        local page_artifacts
        page_artifacts=$(echo "${response}" | jq -c '.artifacts // []')

        local page_len
        page_len=$(echo "${page_artifacts}" | jq 'length')

        all_artifacts=$(jq -c -n \
            --argjson acc "${all_artifacts}" \
            --argjson page "${page_artifacts}" \
            '$acc + $page')
        collected=$((collected + page_len))

        # Stop once every artifact is collected, or a page comes back empty
        # (guards against an over-reported total_count causing an infinite loop).
        if [[ "${collected}" -ge "${total_count}" ]] || [[ "${page_len}" -eq 0 ]]; then
            break
        fi

        page=$((page + 1))
    done

    # Re-emit one merged, complete payload compatible with existing consumers.
    jq -c -n \
        --argjson total_count "${total_count:-0}" \
        --argjson artifacts "${all_artifacts}" \
        '{total_count: $total_count, artifacts: $artifacts}'
}

#######################################
# Download a single artifact from GitHub
# Arguments:
#   $1 - GitHub token
#   $2 - Output file path
#   $3 - Download URL
#######################################
gh_download_artifact() {
    local token="$1"
    local output="$2"
    local url="$3"

    curl -sfL \
        -H "Authorization: Bearer ${token}" \
        -H "Accept: application/vnd.github+json" \
        -o "${output}" \
        "${url}"
}

#######################################
# Download matching artifacts from current workflow run
# Arguments:
#   $1 - GitHub token
#   $2 - Repository (owner/repo)
#   $3 - Workflow run ID
#   $4 - Space-separated patterns to match
#   $5 - Output directory (default: artifacts)
#   $6 - Space-separated patterns to exclude (optional)
# Returns:
#   0 on success, 1 on failure
#######################################
download_matching_artifacts() {
    local token="$1"
    local repository="$2"
    local run_id="$3"
    local patterns="$4"
    local output_dir="${5:-artifacts}"
    local exclude_patterns="${6:-}"

    mkdir -p "${output_dir}"

    local artifacts
    artifacts=$(gh_get_workflow_artifacts "${token}" "${repository}" "${run_id}")

    local total_count
    total_count=$(echo "${artifacts}" | jq '.total_count' -r)

    # When the caller asks for DTB artifacts (e.g. "dtb-*"), a run that yields
    # none is a hard failure: publishing kernels/modules without device trees is
    # worse than failing loudly.
    local expects_dtb=0
    for p in ${patterns}; do
        case "${p}" in
            *dtb*) expects_dtb=1; break ;;
        esac
    done

    if [[ "${total_count}" == "null" ]] || [[ "${total_count}" == "0" ]]; then
        if [[ "${expects_dtb}" == "1" ]]; then
            echo "::error::No artifacts found for run ${run_id}, but DTB artifacts were expected"
            return 1
        fi
        echo "::warning::No artifacts found for run ${run_id}"
        return 0
    fi

    local artifacts_list
    artifacts_list=$(echo "${artifacts}" | jq '[.artifacts[] | [.name, .archive_download_url]]' -r)

    local reviewed=0
    local downloaded=0
    local dtb_downloaded=0
    while IFS=$'\t' read -r name url; do
        reviewed=$((reviewed + 1))

        # Check exclude patterns first
        local excluded=0
        for p in ${exclude_patterns}; do
            if [[ "${name}" == ${p} ]]; then
                excluded=1
                break
            fi
        done

        if [[ "${excluded}" == "1" ]]; then
            echo "  Skipped: ${name} (excluded)"
            continue
        fi

        local matched=0
        for p in ${patterns}; do
            if [[ "${name}" == ${p} ]]; then
                matched=1
                break
            fi
        done

        if [[ "${matched}" == "1" ]]; then
            echo "  Downloading: ${name}"
            gh_download_artifact "${token}" "${output_dir}/${name}.zip" "${url}"
            downloaded=$((downloaded + 1))
            case "${name}" in
                dtb-*) dtb_downloaded=$((dtb_downloaded + 1)) ;;
            esac
        else
            echo "  Skipped: ${name} (no pattern match)"
        fi
    done < <(echo "${artifacts_list}" | jq -r '.[] | @tsv')

    echo "Reviewed ${reviewed} of ${total_count} artifact(s); downloaded ${downloaded} to ${output_dir}/"

    # Every artifact the run reported must have been walked. A mismatch means the
    # metadata was truncated (e.g. an un-paginated fetch) and some artifacts were
    # never even considered for download.
    if [[ "${reviewed}" -ne "${total_count}" ]]; then
        echo "::error::Artifact count mismatch for run ${run_id}: reviewed ${reviewed} but the run reports ${total_count} (pagination/truncation?)"
        return 1
    fi

    # Guard against silently publishing a release without device trees.
    if [[ "${expects_dtb}" == "1" ]] && [[ "${dtb_downloaded}" -eq 0 ]]; then
        echo "::error::No DTB artifacts (dtb-*) were downloaded for run ${run_id}, but they were expected"
        return 1
    fi
}

#######################################
# Extract downloaded artifacts to raw directory
# Arguments:
#   $1 - Source directory (with .zip files)
#   $2 - Target directory (default: raw)
#######################################
extract_artifacts() {
    local source_dir="$1"
    local target_dir="${2:-raw}"

    mkdir -p "${target_dir}"

    local count=0
    for zip in "${source_dir}"/*.zip; do
        [[ ! -f "${zip}" ]] && continue

        local name
        name=$(basename "${zip%.zip}")
        mkdir -p "${target_dir}/${name}"
        unzip -q "${zip}" -d "${target_dir}/${name}"
        rm "${zip}"
        echo "  Extracted: ${name}"
        count=$((count + 1))
    done

    echo "Extracted ${count} artifact(s) to ${target_dir}/"
}

#######################################
# Get version path for Cloudsmith upload
# Arguments:
#   $1 - Artifact type (kuiper or rpi)
#   $2 - Branch name
#   $3 - Timestamp
#   $4 - PR target branch (optional)
#   $5 - PR number (optional)
# Outputs:
#   Version path string
#######################################
get_version_path() {
    local artifact_type="$1"
    local branch="$2"
    local timestamp="$3"
    local pr_target="${4:-}"
    local pr_number="${5:-}"

    if [[ "${artifact_type}" == "rpi" ]]; then
        if [[ -n "${pr_target}" && -n "${pr_number}" ]]; then
            echo "linux_rpi/PRs/${pr_target}/pr_${pr_number}/${timestamp}"
        elif [[ "${branch}" == "main" ]]; then
            echo "linux_rpi/main/${timestamp}"
        else
            echo "linux_rpi/releases/${branch}/${timestamp}"
        fi
    else
        if [[ -n "${pr_target}" && -n "${pr_number}" ]]; then
            echo "linux/PRs/${pr_target}/pr_${pr_number}/${timestamp}"
        elif [[ "${branch}" == "main" ]]; then
            echo "linux/main/${timestamp}"
        else
            echo "linux/releases/${branch}/${timestamp}"
        fi
    fi
}


# Export functions for use in subshells
export -f gh_get_workflow_artifacts
export -f gh_download_artifact
export -f download_matching_artifacts
export -f extract_artifacts
export -f get_version_path
