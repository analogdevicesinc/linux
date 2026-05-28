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
# Note: This only retrieves metadata (names, download URLs), not the artifact content
# This is not used in the main workflow but can be useful in other flows from ci branch
# Arguments:
#   $1 - GitHub token
#   $2 - Repository (owner/repo)
#   $3 - Workflow run ID
# Outputs:
#   JSON response with artifacts list
#######################################
gh_get_workflow_artifacts() {
    local token="$1"
    local repository="$2"
    local run_id="$3"

    curl -sfL \
        -H "Accept: application/vnd.github+json" \
        -H "Authorization: Bearer ${token}" \
        -H "X-GitHub-Api-Version: 2022-11-28" \
        "https://api.github.com/repos/${repository}/actions/runs/${run_id}/artifacts"
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

    if [[ "${total_count}" == "null" ]] || [[ "${total_count}" == "0" ]]; then
        echo "::warning::No artifacts found for run ${run_id}"
        return 0
    fi

    local artifacts_list
    artifacts_list=$(echo "${artifacts}" | jq '[.artifacts[] | [.name, .archive_download_url]]' -r)

    local downloaded=0
    while IFS=$'\t' read -r name url; do
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
        else
            echo "  Skipped: ${name} (no pattern match)"
        fi
    done < <(echo "${artifacts_list}" | jq -r '.[] | @tsv')

    echo "Downloaded ${downloaded} artifact(s) to ${output_dir}/"
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
