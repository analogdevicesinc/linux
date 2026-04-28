#!/bin/bash
#
# Delete stale packages from Cloudsmith
# Deletes packages from closed pull requests (older than N days)
# and branch packages (older than M days)
#
# Usage: cs-delete-stale <cloudsmith_token> <cloudsmith_org_repository> <github_token> <github_org_repository> <branch_days> <pr_days>
#

source ci/lib.sh

_delete_package() {
	local cloudsmith_token="$1"
	local org_repository="$2"
	local slug_perm="$3"
	local uploaded_at="$4"

	while true; do
		local ret
		ret=$(curl -sL \
			-X DELETE \
			-w "%{http_code}" \
			-H "X-Api-Key: $cloudsmith_token" \
			-o /dev/null \
			"https://api.cloudsmith.com/v1/packages/$org_repository/$slug_perm/" 2>/dev/null)

		if [[ "$ret" == "204" ]]; then
			echo "    $slug_perm success"
			return 0
		elif [[ "$ret" == "429" ]]; then
			echo "    $slug_perm too many requests"
			sleep 1
			continue
		fi

		echo "    $slug_perm failed ($ret)"
		return 0
	done
}
export -f _delete_package

_execute_deletions() {
	local cloudsmith_token="$1"
	local org_repository="$2"
	local delete_list="$3"

	[[ ! -f "$delete_list" ]] && return 0

	local line_count
	line_count=$(wc -l < "$delete_list")
	[[ "$line_count" -eq 0 ]] && return 0

	if command -v parallel &>/dev/null; then
		cat "$delete_list" | \
			parallel --jobs 10 \
				--delay 0.15 \
				--colsep '\|' \
				"_delete_package '$cloudsmith_token' '$org_repository' {1} {2}"
	else
		log_info "Tip: install gnu_parallel to fetch in parallel."
		while IFS='|' read -r slug_perm uploaded_at; do
			_delete_package "$cloudsmith_token" "$org_repository" "$slug_perm" "$uploaded_at"
		done < "$delete_list"
	fi
}

_delete_pr_packages() {
	local cloudsmith_token="$1"
	local cloudsmith_org_repository="$2"
	local github_token="$3"
	local github_repository="$4"
	local stale_days="$5"
	local dry_run="$6"

	local cutoff_timestamp
	local delete_list
	local count_stale=0
	local open_prs
	local pr_packages
	local pr_numbers
	local page=1
	local count

	log_step "Search pull request packages"

	cutoff_timestamp=$(date -d "$stale_days days ago" -u +%s)

	echo "  collect artifacts"
	while true; do
		local page_data

		echo "    querying page $page..."
		page_data=$(curl -sL \
			-H "X-Api-Key: $cloudsmith_token" \
			"https://api.cloudsmith.io/v1/packages/$cloudsmith_org_repository/?query=tag:on/pull_request&sort=-date&page_size=100&page=$page")

		if ! echo "$page_data" | jq -e 'type=="array"' &>/dev/null ; then
			# {"detail":"Invalid page."}
			break
		fi

		count=$(echo "$page_data" | jq 'length')
		[[ "$count" -eq 0 ]] && break

		pr_packages+=$(echo "$page_data" | jq -c '.[]')
		pr_packages+=$'\n'
		((page++))
	done

	pr_packages=$(echo "$pr_packages" | jq -s '.')
	pr_numbers=$(echo "$pr_packages" | jq -r '.[] | .tags.info[]? | select(startswith("refs/pull/")) | capture("refs/pull/(?<num>[0-9]+)/") | .num' | sort -u)

	[[ -z "$pr_numbers" ]] && return 0

	echo "  got $(wc -l <<< "$pr_numbers") pull requests (cloudsmith)"

	page=1
	while true; do
		page_data=$(curl -sL \
			-H "Accept: application/vnd.github+json" \
			-H "Authorization: Bearer $github_token" \
			-H "X-GitHub-Api-Version: 2022-11-28" \
			"https://api.github.com/repos/$github_repository/pulls?state=open&per_page=100&page=$page")

		if ! echo "$page_data" | jq -e 'type=="array"' &>/dev/null; then
			log_error "$(echo "$page_data" | jq -r '.message // .' 2>/dev/null)"
			return 1
		fi

		count=$(echo "$page_data" | jq 'length')
		[[ "$count" -eq 0 ]] && break

		open_prs+=$(echo "$page_data" | jq -r '.[] | .number')
		open_prs+=$'\n'
		[[ "$count" -lt 100 ]] && break
		((page++))
	done

	echo "  got $(grep -c . <<< "$open_prs") open pull requests (github)"

	delete_list=$(mktemp)
	trap "rm -f '$delete_list'" RETURN

	while IFS='|' read -r slug_perm uploaded_at pr_number; do
		[[ -z "$slug_perm" ]] && continue
		[[ "$slug_perm" == "null" ]] && continue

		grep -q "^$pr_number$" <<< "$open_prs" && continue || :

		local pkg_timestamp
		pkg_timestamp=$(date -d "$uploaded_at" -u +%s 2>/dev/null || echo "0")

		if [[ "$pkg_timestamp" -lt "$cutoff_timestamp" ]]; then
			echo "$slug_perm|$uploaded_at" >> "$delete_list"
			((count_stale++)) || :
		fi
	done < <(echo "$pr_packages" | jq -r '.[] | . as $pkg | .tags.info[]? | select(startswith("refs/pull/")) | capture("refs/pull/(?<num>[0-9]+)/") | "\($pkg.slug_perm)|\($pkg.uploaded_at)|\(.num)"')

	echo "  got $count_stale artifacts"

	[[ "$count_stale" -eq 0 ]] && return 0 || :
	[[ "$dry_run" == "false" ]] || return 0

	_execute_deletions "$cloudsmith_token" "$cloudsmith_org_repository" "$delete_list"
}

_delete_branch_packages() {
	local cloudsmith_token="$1"
	local cloudsmith_org_repository="$2"
	local stale_days="$3"
	local dry_run="$4"

	local cutoff_timestamp
	local packages
	local count_stale=0
	local delete_list

	log_step "Search branch packages"

	cutoff_timestamp=$(date -d "$stale_days days ago" -u +%s)

	local packages
	local page=1
	echo "  collect artifacts"
	while true; do
		local page_data

		echo "    querying page $page..."
		page_data=$(curl -sL \
			-H "X-Api-Key: $cloudsmith_token" \
			"https://api.cloudsmith.io/v1/packages/$cloudsmith_org_repository/?query=tag:on/push&sort=-date&page_size=100&page=$page")

		if ! echo "$page_data" | jq -e 'type=="array"' &>/dev/null ; then
			break
		fi

		count=$(echo "$page_data" | jq 'length')
		[[ "$count" -eq 0 ]] && break

		packages+=$(echo "$page_data" | jq -c '.[]')
		packages+=$'\n'
		((page++))
	done

	packages=$(echo "$packages" | jq -s '.')

	delete_list=$(mktemp)
	trap "rm -f '$delete_list'" RETURN

	while IFS='|' read -r slug_perm uploaded_at; do
		[[ -z "$slug_perm" ]] && continue
		[[ "$slug_perm" == "null" ]] && continue

		local pkg_timestamp
		pkg_timestamp=$(date -d "$uploaded_at" -u +%s 2>/dev/null || echo "0")

		if [[ "$pkg_timestamp" -lt "$cutoff_timestamp" ]]; then
			echo "$slug_perm|$uploaded_at" >> "$delete_list"
			((count_stale++)) || :
		fi
	done < <(echo "$packages" | jq -r '.[] | "\(.slug_perm)|\(.uploaded_at)"' 2>/dev/null || echo "")

	echo "  got $count_stale artifacts"

	[[ "$count_stale" -eq 0 ]] && return 0 || :
	[[ "$dry_run" == "false" ]] || return 0

	_execute_deletions "$cloudsmith_token" "$cloudsmith_org_repository" "$delete_list"
}

cs-delete-stale() {
	local cloudsmith_token="$1"
	local cloudsmith_org_repository="${2-adi/linux}"
	local github_token="$3"
	local github_org_repository="$4"
	local branch_days="${5-0}"
	local pr_days="${6-0}"
	local dry_run="${7-false}"

	[[ -z "$cloudsmith_token" ]] && cloudsmith_token="$CLOUDSMITH_API_KEY"
	[[ -z "$cloudsmith_token" ]] && { log_warn "CLOUDSMITH_API_KEY is not set" ; return 1 ; } || :

	[ $pr_days -eq 0 ] || _delete_pr_packages "$cloudsmith_token" "$cloudsmith_org_repository" "$github_token" "$github_org_repository" "$pr_days" "$dry_run"
	[ $branch_days -eq 0 ] || _delete_branch_packages "$cloudsmith_token" "$cloudsmith_org_repository" "$branch_days" "$dry_run"
}
