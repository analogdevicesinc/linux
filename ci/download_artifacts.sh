#!/bin/bash

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

_get_artifact () {
	local cloudsmith_token="$1"
	local git_sha="$2"
	local tuple="$3"
	local url=
	local tags=

	IFS='|' read -r url tags <<< "$tuple"

	[[ "$url" == "null" ]] && return
	[[ "$url" =~ $git_sha/adi_ci_defconfig- ]] && return
	[[ "$url" = *-devel ]] && return
	[[ "$url" = *-headers ]] && return

	name=$(basename "$url")
	local zip="$name.zip"
	echo -n "  Artifact $name"

	local ret="200"
	unzip -t "raw/$zip" &>/dev/null && \
		echo -n " (cached zip)" || \
		ret=$(curl -sL \
		-H "X-Api-Key: $cloudsmith_token" \
		-w "%{http_code}\n" \
		-o "raw/$zip" \
		$url)

	echo "tags=$tags" > "raw/$name.metadata.txt"

	[[ "$ret" != "200" ]] && return 1

	[[ ! -d "raw/$name" ]] && \
		unzip -q "raw/$zip" -d "raw/$name" || \
		echo -n " (cached dir)"
	echo ""

	[ "$SKIP_DIST" == "true" ] && return 0 || :

	[[ "$name" == "dtb-gcc" ]] && \
		_unpack_dtb "$name" || \
		_unpack_kernel "$name"
}
export -f _get_artifact

_get_first_result_version () {
	local cloudsmith_token="$1"
	local org_repository="$2"
	local query="$3"

	curl -sL \
	  -H "X-Api-Key: $cloudsmith_token" \
	  "https://api.cloudsmith.io/v1/packages/$org_repository/?query=${query}&sort=-date&page_size=1" \
	  | jq -r '.[].version // empty'
}

download_artifacts() {
	local ref="${1-c894bed472b7}"
	local org_repository="${2-adi/linux}"
	local no_cache="${3-true}"
	local cloudsmith_token=${4}

	local event=

	[ "$no_cache" == "true" ] && command rm -rf dist/ raw/ .get_artifacts

	[ -f '.get_artifacts' ] && { log_step "Get artifacts (checkpoint)" ; return ;} || log_step "Get artifacts"

	[[ -z "$cloudsmith_token" ]] && cloudsmith_token="$CLOUDSMITH_API_KEY"
	[[ $ref == refs/heads/* ]] && event="push"
	[[ $ref == refs/pull/* ]] && event="pull_request"
	[[ $ref == refs/heads/* ]] || [[ $ref == refs/tags/* ]] || [[ $ref == refs/pull/* ]] && \
		git_sha=$(_get_first_result_version "$cloudsmith_token" "$org_repository" "tag:on/${event}+tag:${ref}") || git_sha="$ref"

	[[ -z "$cloudsmith_token" ]] && { log_warn "CLOUDSMITH_API_KEY is not set, only public artifacts will be accessible." ; } || :
	[[ -n "$git_sha" ]] || { log_error "No git sha provided." ; return 1 ;}
	[[ "${#git_sha}" == "40" ]] || git_sha=$(_get_first_result_version "$cloudsmith_token" "$org_repository" "tag:on/${event}+version:$git_sha")
	[[ -n "$git_sha" ]] || { log_error "Failed to get full git sha." ; return 1 ;}

	command -v unzip 1>/dev/null || { log_error "Command unzip not installed." ; return 1 ; }

	local tmpdir=$(mktemp -d)
	mkdir -p dist
	mkdir -p raw

	query="version:^$git_sha\$"
	log_info "Fetching $git_sha"
	ret=$(curl -sL \
		-w "%{http_code}\n" \
		-o "$tmpdir/.query" \
		-H "X-Api-Key: $cloudsmith_token" \
		"https://api.cloudsmith.io/v1/packages/$org_repository/?query=$query&sort=-date&page_size=100")
	[[ "$ret" != "200" ]] && return 1

	# Extract cdn_url and tags
	tuple=$(cat "$tmpdir/.query" | (jq -r '.[] | "\(.cdn_url)|\(.tags.info | join(","))"'))
	[[ -z "$tuple" ]] && { echo "No artifacts found." ; return 1 ; }

	log_info "Got $(echo "$tuple" | wc -l) artifacts"

	if command -v parallel &>/dev/null ; then
		echo "$tuple" |
			parallel --jobs 8 \
				"_get_artifact \"$cloudsmith_token\" \"$git_sha\" {}"
	else
		log_info "Tip: install gnu_parallel to fetch in parallel."
		for url in $tuple; do
			_get_artifact "$cloudsmith_token" "$git_sha" "$url"
		done
	fi

	[ "$SKIP_EXPAND_MICROBLAZE" == "true" ] && log_info "Expand microblaze skipped" || _expand_microblaze
	[ "$SKIP_EXPAND_NIOS2" == "true" ] && log_info "Expand nios2 skipped" || _expand_nios2

	command rm -r $tmpdir
	touch .get_artifacts

	[ "$SKIP_DIST" == "true" ] && log_info "Wrote to raw/" || log_info "Wrote to dist/"
}

