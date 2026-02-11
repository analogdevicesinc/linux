#!/bin/bash
#
# Upload artifacts from dist/ to cloudsmith in the 'artifactory' format.
#
# Expects artifacts to be expanded in dist/
# (run download_artifacts.sh first)
# Open questions:
# - Should we gen [arria10|cyclone5]/extlinux.conf (and other static boot-partition files) here?
# - Are pushing just to sdg-linux, or sdg-linux-rpi as well?
# - Is it ok the new modules format?

source ci/lib.sh

_upload_file() {
	local package_name="$1"
	local version="$2"
	local file_path="$3"
	local tags="$4"
	local org_repository="$5"
	local dry_run="$6"
	local cloudsmith_token="$7"
	local tmptar=

	echo ""
	echo "upload_file"

	local filename=$(basename "$file_path")

	# If a path is given, compress as .tar.gz
	if [[ -d "$file_path" ]]; then
		if [[ "$dry_run" == "true" ]]; then
			echo "  tar: $file_path"
			tmptar="$(mktemp --suffix=.tar.gz -u)"
			file_path=$tmptar
		else
			tmptar="$(mktemp --suffix=.tar.gz)"
			tar -czf "$tmptar" -C "$file_path" .
			file_path=$tmptar
		fi
	fi

	if [[ "$dry_run" == "true" ]]; then
		echo "  https://upload.cloudsmith.io/${org_repository}/${filename}"
		echo "    -T: $file_path"

		echo "  https://api.cloudsmith.com/v1/packages/${org_repository}/upload/raw/"
		echo "    name: $package_name"$'\n'"    tags: $tags"$'\n'"    version: $version"
		return 0
	fi

	package_file=$(curl -sfL \
		-T "$file_path" \
		-H "X-Api-Key: $cloudsmith_token" \
		-H "Content-Sha256: $(shasum -a256 "$file_path" | cut -f1 -d' ')" \
		"https://upload.cloudsmith.io/${org_repository}/${filename}" | jq -r .identifier)

	if [[ -z "$package_file" ]] || [[ "$package_file" == "null" ]]; then
		echo "  failed: $package_name"
		return 1
	fi

	result=$(curl -sfL \
		-X POST \
		-H "Accept: application/json" \
		-H "Content-Type: application/json" \
		-H "X-Api-Key: $cloudsmith_token" \
		-d "{\"package_file\":\"$package_file\", \
			 \"republish\":\"true\", \
			 \"name\":\"$package_name\", \
			 \"tags\": \"$tags\", \
			 \"version\":\"$version\"}" \
		"https://api.cloudsmith.com/v1/packages/${org_repository}/upload/raw/")

	[[ -n "$tmptar" ]] && command rm "$tmptar"
	if echo "$result" | jq -e '.slug' &>/dev/null; then
		echo "  success $package_name"
		return 0
	else
		echo "  failed $package_name"
		return 1
	fi
}
export -f _upload_file

_process_dtb() {
	local type=
	local arch=
	local dtb_file=

	IFS='|' read -r type arch dtb_file <<< "$1"

	local version_prefix="$2"
	local timestamp="$3"
	local tags="$4"
	local org_repository="$5"
	local dry_run="$6"
	local cloudsmith_token="$7"

	local dtb_name=$(basename "$dtb_file")
	local version="${version_prefix}/${timestamp}/${arch}/"

	_upload_file "$dtb_name" "$version" "$dtb_file" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
}
export -f _process_dtb

__defconfig_to_shortname() {
	local defconfig="$1"
	local shortname=

	case "$defconfig" in
		adi_zynqmp_defconfig)
			shortname="zynqmp"
			;;
		adi_versal_defconfig)
			shortname="versal"
			;;
		zynq_xcomm_adv7511_defconfig)
			shortname="zynq"
			;;
		zynq_pluto_defconfig)
			shortname="pluto"
			;;
		zynq_m2k_defconfig)
			shortname="m2k"
			;;
		socfpga_adi_defconfig)
			shortname="socfpga"
			;;
		adi_bcm2709_defconfig)
			shortname="bcm2709"
			;;
		adi_bcmrpi_defconfig)
			shortname="bcmrpi"
			;;
		adi_bcm2711_defconfig)
			shortname="bcm2711"
			;;
		adi_bcm2712_defconfig)
			shortname="bcm2712"
			;;
		*)
			log_info "No shortname for defconfig '$defconfig'"
			shortname="$defconfig"
			;;
	esac

	echo "$shortname"
}
export -f __defconfig_to_shortname

_process_kernel() {
	local type=
	local defconfig=
	local arch=
	local image_file=

	IFS='|' read -r type defconfig arch image_file <<< "$1"

	local version_prefix="$2"
	local timestamp="$3"
	local tags="$4"
	local org_repository="$5"
	local dry_run="$6"
	local cloudsmith_token="$7"

	local shortname=$(__defconfig_to_shortname "$defconfig")
	local package_name=$(basename "$image_file")

	local version="${version_prefix}/${timestamp}/${arch}/${shortname}/"
	_upload_file "$package_name" "$version" "$image_file" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
}
export -f _process_kernel

_process_kernel_modules() {
	local type=
	local defconfig=
	local arch=
	local defconfig_dir=

	IFS='|' read -r type defconfig arch defconfig_dir<<< "$1"

	local version_prefix="$2"
	local timestamp="$3"
	local tags="$4"
	local org_repository="$5"
	local dry_run="$6"
	local cloudsmith_token="$7"

	local shortname=$(__defconfig_to_shortname "$defconfig")
	local package_name="${shortname}_modules.tar.gz"

	local version="${version_prefix}/${timestamp}/${arch}/${shortname}/"
	_upload_file "$package_name" "$version" "$defconfig_dir" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
}
export -f _process_kernel_modules

_process_soft_core() {
	local type=
	local arch=
	local image_file=

	IFS='|' read -r type arch image_file <<< "$1"

	local version_prefix="$2"
	local timestamp="$3"
	local tags="$4"
	local org_repository="$5"
	local dry_run="$6"
	local cloudsmith_token="$7"

	local package_name=$(basename "$image_file")
	local version="${version_prefix}/${timestamp}/${arch}/"

	_upload_file "$package_name" "$version" "$image_file" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
}
export -f _process_soft_core

_process_git_properties() {
	local type=
	local git_branch=
	local git_sha=
	local git_sha_date=

	IFS='|' read -r type git_branch git_sha git_sha_date <<< "$1"

	local version_prefix="$2"
	local timestamp="$3"
	local tags="$4"
	local org_repository="$5"
	local dry_run="$6"
	local cloudsmith_token="$7"

	local tmp_file=$(mktemp)

	echo "git_branch=$git_branch" >> $tmp_file
	echo "git_sha=$git_sha" >> $tmp_file
	echo "git_sha_date=$git_sha_date" >> $tmp_file

	local package_name="git_properties.txt"
	local version="${version_prefix}/${timestamp}/"

	_upload_file "$package_name" "$version" "$tmp_file" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"

	command rm $tmp_file
}

_process_task() {
	local version_prefix="$1"
	local timestamp="$2"
	local tags="$3"
	local org_repository="$4"
	local dry_run="$5"
	local cloudsmith_token="$6"
	local task="$7"

	IFS='|' read -r type args <<< "$task"

	case "$type" in
		dtb)
			_process_dtb "$task" "$version_prefix" "$timestamp" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
			;;
		kernel)
			_process_kernel "$task" "$version_prefix" "$timestamp" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
			;;
		kernel_modules)
			_process_kernel_modules "$task" "$version_prefix" "$timestamp" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
			;;
		softcore)
			_process_soft_core "$task" "$version_prefix" "$timestamp" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
			;;
	esac
}
export -f _process_task

upload_to_cloudsmith_expanded() {
	local _git_sha="${1}"
	local org_repository="${2-adi/sdg-linux}"
	local dry_run="${3-true}"
	local cloudsmith_token="${4}"

	local git_sha=
	local git_sha_ct=
	local git_sha_date=
	local timestamp=
	local git_branch=

	[[ -z "$cloudsmith_token" ]] && { log_error "CLOUDSMITH_API_KEY is required." ; return 1 ; }
	[[ -z "$_git_sha" ]] && { log_error "No git sha provided." ; return 1 ;}
	[[ "${#_git_sha}" != "40" ]] && { log_error "Git sha is not 40 characters long." ; return 1 ; }

	local git_sha_short="${git_sha:0:12}"

	log_info "SHA: $git_sha_short"
	log_info "Target: ${org_repository}"

	# Fill context from any artifact, must be the same for all.
	source $(ls raw/*.metadata.txt | head -1)
	[[ -z "$tags" ]] && { log_error "No tags" ; return 1 ; }
	source $(ls raw/*/context.txt | head -1)
	[[ "$git_sha" != "${_git_sha:0:12}" ]] && { log_error "Mismatch between dist git sha $git_sha and provided ${_git_sha:0:12}" ; return 1 ; }
	git_sha="$_git_sha"
	if [[ -z "$git_sha_ct" ]]; then
		git_sha_ct=$(date +%s)
		log_warn "No git_sha_ct, using current time '$git_sha_ct'"
	fi
	# dubious double timestamp
	timestamp=$(date -d @$git_sha_ct +%Y_%m_%d-%H_%M_%S)
	git_sha_date=$(date -d @$git_sha_ct +%Y-%m-%d-%H-%M)

	log_info "Tags: $tags"

	local version_prefix=""
	if echo "$tags" | grep -q "on/pull_request"; then
		git_branch="merge" # dubious
		local pr_num=$(grep -m1 -oP 'refs/pull/\K[0-9]+' <<< "$tags")
		local base_branch=$(grep -m1 -oP 'refs/base/\K.+' <<< "$tags")
		version_prefix="linux/PRs/${base_branch}/pr_${pr_num}"
		log_info "Branch: ${git_branch} (pr #${pr_num} from ${base_branch})"
	elif echo "$tags" | grep -q 'refs/heads/main'; then
		git_branch="main"
		version_prefix="linux/main"
		log_info "Branch: ${git_branch}"
	elif echo "$tags" | grep -qP 'refs/heads/rpi-'; then
		git_branch=$(grep -m1 -oP 'refs/heads/\Krpi-.+' <<< "$tags")
		version_prefix="linux_rpi/${git_branch}"
		log_info "Branch: ${git_branch} (rpi)"
	elif echo "$tags" | grep -qP 'refs/heads/20[0-9]{2}_'; then
		git_branch=$(grep -m1 -oP 'refs/heads/\K20[0-9]{2}_.+' <<< "$tags")
		version_prefix="linux/releases/${git_branch}"
		log_info "Branch: ${git_branch} (release)"
	else
		git_branch=$(grep -m1 -oP 'refs/heads/\K.+' <<< "$tags")
		log_error "Branch: ${git_branch} (we don't upload that)"
		return 1
	fi

	local tags="git_sha-${git_sha},timestamp_${timestamp}"

	log_info "Version prefix: ${version_prefix}"
	log_info "Upload tags: ${tags}"

	local tasks_file=$(mktemp)

	# git_properties
	_process_git_properties "git_properties|${git_branch}|${git_sha}|${git_sha_date}" "$version_prefix" "$timestamp" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"

	# dtb
	for arch in arm arm64; do
		local dtb_dir="dist/${arch}/boot/dts"
		if [[ -d "$dtb_dir" ]]; then
			find "$dtb_dir" -name "*.dtb" -type f | while read -r dtb_file; do
				echo "dtb|${arch}|${dtb_file}" >> "$tasks_file"
			done
		fi
	done

	# kernel
	for arch in arm arm64; do
		local kernel_base="dist/${arch}/boot/kernel"
		[[ -d "$kernel_base" ]] || continue
		for defconfig_dir in "$kernel_base"/*; do
			[[ -d "$defconfig_dir" ]] || continue
			local defconfig=$(basename "$defconfig_dir")
			local image_file=$(find "$defconfig_dir" -maxdepth 1 -type f \( -name "Image" -o -name "zImage" -o -name "uImage" \) | head -n1)
			[[ -z "$image_file" ]] && continue
			echo "kernel|${defconfig}|${arch}|${image_file}" >> "$tasks_file"
		done
	done

	# kernel_modules
	for arch in arm arm64; do
		local kernel_base="dist/${arch}/lib/modules_set"
		[[ -d "$kernel_base" ]] || continue
		for defconfig_dir in "$kernel_base"/*; do
			[[ -d "$defconfig_dir" ]] || continue
			local defconfig=$(basename "$defconfig_dir")
			echo "kernel_modules|${defconfig}|${arch}|${defconfig_dir}" >> "$tasks_file"
		done
	done

	# softcore
	local mb_dir="dist/microblaze/boot/kernel/adi_mb_defconfig"
	if [[ -d "$mb_dir" ]]; then
		find "$mb_dir" -name "simpleImage.*.strip" -type f | while read -r image_file; do
			echo "softcore|microblaze|${image_file}" >> "$tasks_file"
		done
	fi
	local nios2_dir="dist/nios2/boot/kernel/adi_nios2_defconfig"
	if [[ -d "$nios2_dir" ]]; then
		find "$nios2_dir" -name "zImage.*" -type f | while read -r image_file; do
			echo "softcore|nios2|${image_file}" >> "$tasks_file"
		done
	fi

	local task_count=$(wc -l < "$tasks_file")
	log_info "Uploading $task_count packages"

	if command -v parallel &>/dev/null; then
		cat "$tasks_file" | parallel --jobs 8 \
			_process_task "$version_prefix" "$timestamp" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token" "{}"
	else
		log_info "Tip: install gnu_parallel to upload in parallel."
		while IFS= read -r task; do
			_process_task "$version_prefix" "$timestamp" "$tags" "$org_repository" "$dry_run" "$cloudsmith_token"
		done < "$tasks_file"
	fi

	command rm "$tasks_file"
	log_info "Upload expanded complete"
}
