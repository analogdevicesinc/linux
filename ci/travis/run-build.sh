#!/bin/bash
set -e

# cd to docker build dir if it exists
if [ -d /docker_build_dir ] ; then
	cd /docker_build_dir
fi

[ "${LOCAL_BUILD}" == "y" ] || . ./ci/travis/lib.sh

__echo_green() {
	[ "${LOCAL_BUILD}" == "y" ] && echo $@ || echo_green $@
}

__echo_red() {
	[ "${LOCAL_BUILD}" == "y" ] && echo $@ || echo_red $@
}

MAIN_BRANCH=${MAIN_BRANCH:-main}

if [ -f "${FULL_BUILD_DIR}/env" ] && [ -z "${LOCAL_BUILD}" ]; then
	echo_blue "Loading environment variables"
	cat "${FULL_BUILD_DIR}/env"
	. "${FULL_BUILD_DIR}/env"
fi

# Run once for the entire script
[ "${LOCAL_BUILD}" == "y" ] || sudo apt-get -qq update

apt_install() {
	[ "${LOCAL_BUILD}" == "y" ] || sudo apt-get install -y $@
}

if [ -z "$NUM_JOBS" ] ; then
	NUM_JOBS=$(getconf _NPROCESSORS_ONLN)
	NUM_JOBS=${NUM_JOBS:-1}
fi

KCFLAGS="-Werror"
# FIXME: remove the line below once Talise & Mykonos APIs
#	 dont't use 1024 bytes on stack
KCFLAGS="$KCFLAGS -Wno-error=frame-larger-than="
export KCFLAGS

# FIXME: remove this function once kernel gets upgrade and
#	 GCC doesn't report these warnings anymore
adjust_kcflags_against_gcc() {
	GCC="${CROSS_COMPILE}gcc"
	if [ "$($GCC -dumpversion | cut -d. -f1)" -ge "8" ]; then
		KCFLAGS="$KCFLAGS -Wno-error=stringop-truncation"
		KCFLAGS="$KCFLAGS -Wno-error=packed-not-aligned"
		KCFLAGS="$KCFLAGS -Wno-error=stringop-overflow= -Wno-error=sizeof-pointer-memaccess"
		KCFLAGS="$KCFLAGS -Wno-error=missing-attributes"
	fi

	if [ "$($GCC -dumpversion | cut -d. -f1)" -ge "9" ]; then
		KCFLAGS="$KCFLAGS -Wno-error=address-of-packed-member -Wno-error=attribute-alias="
		KCFLAGS="$KCFLAGS -Wno-error=stringop-truncation"
	fi
	if [ "$($GCC -dumpversion | cut -d. -f1)" -ge "10" ]; then
		KCFLAGS="$KCFLAGS -Wno-error=maybe-uninitialized -Wno-error=restrict"
		KCFLAGS="$KCFLAGS -Wno-error=zero-length-bounds"
	fi
	export KCFLAGS
}

APT_LIST="make bc u-boot-tools flex bison libssl-dev tar kmod xz-utils"

if [ "$ARCH" = "arm64" ] ; then
	if [ -z "$CROSS_COMPILE" ] ; then
		CROSS_COMPILE=aarch64-linux-gnu-
		export CROSS_COMPILE
	fi

	APT_LIST="$APT_LIST gcc-aarch64-linux-gnu"
fi

if [ "$ARCH" = "arm" ] ; then
	if [ -z "$CROSS_COMPILE" ] ; then
		CROSS_COMPILE=arm-linux-gnueabi-
		export CROSS_COMPILE
	fi

	APT_LIST="$APT_LIST gcc-arm-linux-gnueabi"
fi

apt_update_install() {
	apt_install $@
	adjust_kcflags_against_gcc
}

__get_all_c_files() {
	git grep -i "$@" | cut -d: -f1 | sort | uniq  | grep "\.c"
}

__exceptions_file() {
	[ ! -f "$1" ] && return 1
	grep -q "$2" "$1" && return 0 || return 1
}

check_all_adi_files_have_been_built() {
	# Collect all .c files that contain the 'Analog Devices' string/name
	local c_files=$(__get_all_c_files "Analog Devices")
	local ltc_c_files=$(__get_all_c_files "Linear Technology")
	local o_files
	local exceptions_file="ci/travis/${DEFCONFIG}_compile_exceptions"
	local ret=0

	c_files="drivers/misc/mathworks/*.c $c_files $ltc_c_files"

	# Convert them to .o files via sed, and extract only the filenames
	for file in $c_files ; do
		file1=$(echo $file | sed 's/\.c/\.o/g')
		if __exceptions_file "$exceptions_file" "$file1"; then
			continue
		fi

		if [ ! -f "$file1" ] ; then
			if [ "$ret" = "0" ] ; then
				echo
				__echo_red "The following files need to be built OR"
				__echo_green "      added to '$exceptions_file'"

				echo

				__echo_green "  If adding the '$exceptions_file', please make sure"
				__echo_green "  to check if it's better to add the correct Kconfig symbol"
				__echo_green "  to one of the following files:"

				for file in $(find -name Kconfig.adi) ; do
					__echo_green "   $file"
				done

				echo
			fi
			__echo_red "File '$file1' has not been compiled"
			ret=1
		fi
	done

	return $ret
}

get_ref_branch() {
	if [ -n "$TARGET_BRANCH" ] ; then
		echo -n "$TARGET_BRANCH"
	elif [ -n "$TRAVIS_BRANCH" ] ; then
		echo -n "$TRAVIS_BRANCH"
	elif [ -n "$GITHUB_BASE_REF" ] ; then
		echo -n "$GITHUB_BASE_REF"
	else
		echo -n "HEAD~5"
	fi
}

bad_licence_error() {
       __echo_red "File '$1' is being added to Analog Devices Linux tree."
       __echo_red "Analog Devices code is being marked dual-licensed... Make sure this is really intended!"
       __echo_red "If not intended, change MODULE_LICENSE() or the SPDX-License-Identifier accordingly."
       __echo_red "This is not as simple as one thinks and upstream might require a lawyer to sign the patches!"
}

build_check_new_file_license() {
	local ret

	local ref_branch="$(get_ref_branch)"

	if [ -z "$ref_branch" ] ; then
		__echo_red "Could not get a base_ref for checkpatch"
		exit 1
	fi

	__update_git_ref "${ref_branch}" "${ref_branch}"

	COMMIT_RANGE="${ref_branch}.."

	__echo_green "Running check_new_adi_file_license for commit range '$COMMIT_RANGE'"

	ret=0
	# Get list of new files in the commit range
	for file in $(git diff --name-status "$COMMIT_RANGE" | grep ^A | cut -d$'\t' -f2) ; do
		if git diff "$COMMIT_RANGE" "$file" | grep "+MODULE_LICENSE" | grep -q "Dual" ; then
			bad_licence_error "$file"
			ret=1
		elif git diff "$COMMIT_RANGE" "$file" | grep "SPDX-License-Identifier:" | grep -qi " OR " ; then
			# The below might catch bad licenses in header files and also helps to make sure dual licenses are
			# not in driver (eg: sometimes people have MODULE_LICENSE != SPDX-License-Identifier - which is also
			# wrong and maybe someting to improve in this job)
			if echo "$file" | grep -q ".yaml$"; then
				continue
			fi
			bad_licence_error "$file"
			ret=1
		fi
	done

	return $ret
}

__setup_dummy_git_account() {
	[ "${LOCAL_BUILD}" == "y" ] && return 0
	# setup an email account so that we can cherry-pick stuff
	git config user.name "CSE CI"
	git config user.email "cse-ci-notifications@analog.com"
}

build_default() {
	[ -n "$DEFCONFIG" ] || {
		__echo_red "No DEFCONFIG provided"
		return 1
	}

	[ -n "$ARCH" ] || {
		__echo_red "No ARCH provided"
		return 1
	}

	APT_LIST="$APT_LIST git"

	apt_update_install $APT_LIST

	# make sure git does not complain about unsafe repositories when
	# building inside docker.
	[ -d /docker_build_dir ] && git config --global --add safe.directory /docker_build_dir

	make ${DEFCONFIG}
	if [[ "${SYSTEM_PULLREQUEST_TARGETBRANCH}" =~ ^rpi-.* || "${BUILD_SOURCEBRANCH}" =~ ^refs/heads/rpi-.* \
		|| "${BUILD_SOURCEBRANCH}" =~ ^refs/heads/staging-rpi ]]; then
		echo "Rpi build"
    		make -j$NUM_JOBS $IMAGE modules dtbs
		make INSTALL_MOD_PATH="${PWD}/modules" modules_install
	else
    		echo "Normal build"
    		make -j$NUM_JOBS $IMAGE UIMAGE_LOADADDR=0x8000
	fi

	if [ "$CHECK_ALL_ADI_DRIVERS_HAVE_BEEN_BUILT" = "1" ] ; then
		check_all_adi_files_have_been_built
	fi

	make savedefconfig
	mv defconfig arch/$ARCH/configs/$DEFCONFIG

	git diff --exit-code || {
		__echo_red "Defconfig file should be updated: 'arch/$ARCH/configs/$DEFCONFIG'"
		__echo_red "Run 'make savedefconfig', overwrite it and commit it"
		return 1
	}
}

build_allmodconfig() {
	APT_LIST="$APT_LIST git"

	apt_update_install $APT_LIST
	make allmodconfig
	make -j$NUM_JOBS
}

build_checkpatch() {
	local ref_branch="$(get_ref_branch)"

	__echo_green "Running checkpatch for commit range '$ref_branch..'"

	if [ -z "$ref_branch" ] ; then
		__echo_red "Could not get a base_ref for checkpatch"
		exit 1
	fi

	# install checkpatch dependencies
	[ "${LOCAL_BUILD}" == "y" ] || sudo pip install ply GitPython

	# __update_git_ref() does a shallow fetch with depth=50 by default to speed things
	# up. However that could be problematic if the branch in the PR diverged from
	# main/master such that we cannot find a common ancestor. In that case, the job will
	# timeout after 60min even if the branch is able to merge (even if diverged). We
	# could do '$GIT_FETCH_DEPTH="disable"' before calling __update_git_ref() but that
	# would slow things a lot. Instead, let's do a treeless fetch which get's the whole
	# history while being much faster than a typical fetch.
	[ "${LOCAL_BUILD}" == "y" ] || \
                git fetch --filter=tree:0 --no-tags ${ORIGIN} +refs/heads/${ref_branch}:${ref_branch}

	scripts/checkpatch.pl --git "${ref_branch}.." \
		--strict \
		--ignore FILE_PATH_CHANGES \
		--ignore LONG_LINE \
		--ignore LONG_LINE_STRING \
		--ignore LONG_LINE_COMMENT \
		--ignore PARENTHESIS_ALIGNMENT \
		--ignore CAMELCASE \
		--ignore UNDOCUMENTED_DT_STRING
}

build_dt_binding_check() {
	local ref_branch="$(get_ref_branch)"
	local commit="$COMMIT"
	local err=0

	__echo_green "Running dt_binding_check for commit range '$ref_branch..'"

	if [ -z "$ref_branch" ] ; then
		__echo_red "Could not get a base_ref for checkpatch"
		exit 1
	fi

	# install dt_binding_check dependencies
	apt_install pipx
	[ "${LOCAL_BUILD}" == "y" ] || {
		pipx install dtschema
		PATH=$PATH:$HOME/.local/bin/
	}

	__update_git_ref "${ref_branch}" "${ref_branch}"

	local files=$(git diff --name-only "$ref_branch..$commit")

	while read file; do
		case "$file" in
		*.yaml)
			local relative_yaml=${file#Documentation/devicetree/bindings/}

			if [[ "$relative_yaml" = "$file" ]]; then
				echo "$file not a devicetree binding, skip check..."
			else
				echo "Testing devicetree binding $file"

				git checkout -q "$commit" "$file"

				# The dt_binding_check rule won't exit with an error
				# for schema errors, but will exit with an error for
				# dts example errors.
				#
				# All schema files must be validated before exiting,
				# so the script should not exit on error.
				#
				# Disable exit-on-error flag, check the exit code
				# manually, and set err if the exit-code is non-zero,
				# before enabling exit-on-error back.
				set +e
				error_txt=$(make dt_binding_check CONFIG_DTC=y DT_CHECKER_FLAGS=-m DT_SCHEMA_FILES="$relative_yaml" 2>&1)
				if [[ $? -ne 0 ]]; then
					err=1
				fi
				set -e

				echo "$error_txt"

				# file name appears in output if it contains errors
				if echo "$error_txt" | grep -qF "$file"; then
					err=1
				fi
			fi
			;;
		esac
	done <<< "$files"

	return $err
}

build_microblaze() {
	local exceptions_file="ci/travis/dtb_build_test_exceptions"
	local err=0

	# Setup standalone compiler
	wget -q --show-progress "${DOWNLOAD_URL}/microblaze_compiler/microblazeel-xilinx-elf.tar.gz"
	mkdir -p /opt/microblazeel-xilinx-elf
	tar -xvzf microblazeel-xilinx-elf.tar.gz -C /opt/microblazeel-xilinx-elf
	export PATH=$PATH:/opt/microblazeel-xilinx-elf/bin
	sudo apt install u-boot-tools
	microblazeel-xilinx-elf-gcc --version

	for file in $DTS_FILES; do
		if __exceptions_file "$exceptions_file" "$file"; then
			continue
		fi

		if ! grep -q "hdl_project:" $file ; then
			__echo_red "'$file' doesn't contain an 'hdl_project:' tag"
			hdl_project_tag_err=1
		fi
	done

	if [ "$hdl_project_tag_err" = "1" ] ; then
		echo
		echo
		__echo_green "Some DTs have been found that do not contain an 'hdl_project:' tag"
		__echo_green "   Either:"
		__echo_green "     1. Create a 'hdl_project' tag for it"
		__echo_green "     OR"
		__echo_green "     1. add it in file '$exceptions_file'"
	fi

	export CROSS_COMPILE=/opt/microblazeel-xilinx-elf/bin/microblazeel-xilinx-elf-
	ARCH=microblaze make adi_mb_defconfig
	for file in $DTS_FILES; do
		if __exceptions_file "$exceptions_file" "$file"; then
			continue
		fi

		dtb_file="simpleImage."
		dtb_file+=$(echo $file | sed 's/dts\//=/g' | cut -d'=' -f2 | sed 's\.dts\\g')

		echo "######### Building: $dtb_file"
		ARCH=microblaze make ${dtb_file} -j$NUM_JOBS || err=1
	done

	if [ "$err" = "0" ] ; then
		__echo_green "Microblaze build tests passed"
		return 0
	fi

	return $err
}

build_dtb_build_test() {
	local exceptions_file="ci/travis/dtb_build_test_exceptions"
	local err=0
	local defconfig
	local last_defconfig

	for file in $DTS_FILES; do
		arch=$(echo $file |  cut -d'/' -f2)
		# a bit hard-coding for now; only check arm & arm64 DTs;
		# they are shipped via SD-card
		if [ "$arch" != "arm" ] && [ "$arch" != "arm64" ] ; then
			continue
		fi

		if __exceptions_file "$exceptions_file" "$file"; then
			continue
		fi

		if ! grep -q "hdl_project:" $file ; then
			__echo_red "'$file' doesn't contain an 'hdl_project:' tag"
			err=1
			hdl_project_tag_err=1
		fi
	done

	if [ "$hdl_project_tag_err" = "1" ] ; then
		echo
		echo
		__echo_green "Some DTs have been found that do not contain an 'hdl_project:' tag"
		__echo_green "   Either:"
		__echo_green "     1. Create a 'hdl_project' tag for it"
		__echo_green "     OR"
		__echo_green "     1. add it in file '$exceptions_file'"
		return 1
	fi

	for file in $DTS_FILES; do
		if __exceptions_file "$exceptions_file" "$file"; then
			continue
		fi

		dtb_file=$(echo $file | sed 's/dts\//=/g' | cut -d'=' -f2 | sed 's\dts\dtb\g')
		arch=$(echo $file |  cut -d'/' -f2)

		case "$(echo ${file} | grep -Eo 'zynq|zynqmp|socfpga|versal' || echo '')" in
			zynq)
				defconfig="zynq_xcomm_adv7511_defconfig"
				;;
			zynqmp)
				defconfig="adi_zynqmp_defconfig"
				;;
			socfpga)
				defconfig="socfpga_adi_defconfig"
				;;
			versal)
				defconfig="adi_versal_defconfig"
				;;
			*)
				echo "Default defconfig will be used."
				defconfig="defconfig"
				;;
		esac

		# Check if new defconfig nneds to be built
		if [ "$last_defconfig" != "$defconfig" ] ; then
			ARCH=$arch make ${defconfig}
			last_defconfig=$defconfig
		fi
		# XXX: hack for nios2, which doesn't have `arch/nios2/boot/dts/Makefile`
		# but even an empty one is fine
		if [ ! -f arch/$arch/boot/dts/Makefile ] ; then
			touch arch/$arch/boot/dts/Makefile
		fi
		ARCH=$arch make ${dtb_file} -j$NUM_JOBS || err=1
	done

	if [ "$err" = "0" ] ; then
		__echo_green "DTB build tests passed"
		return 0
	fi

	return $err
}

branch_contains_commit() {
	local commit="$1"
	local branch="$2"
	git merge-base --is-ancestor $commit $branch &> /dev/null
}

__update_git_ref() {
	local ref="$1"
	local local_ref="$2"
	local depth

        [ "${LOCAL_BUILD}" == "y" ] && return 0

	[ "$GIT_FETCH_DEPTH" = "disabled" ] || {
		depth="--depth=${GIT_FETCH_DEPTH:-50}"
	}
	if [ -n "$local_ref" ] ; then
		git fetch $depth $ORIGIN +refs/heads/${ref}:${local_ref}
	else
		git fetch $depth $ORIGIN +refs/heads/${ref}
	fi
}

__push_back_to_github() {
	local dst_branch="$1"

	git push --quiet -u $ORIGIN "HEAD:$dst_branch" || {
		__echo_red "Failed to push back '$dst_branch'"
		return 1
	}
}

ORIGIN=${ORIGIN:-origin}

BUILD_TYPE=${BUILD_TYPE:-${1}}
BUILD_TYPE=${BUILD_TYPE:-default}

build_${BUILD_TYPE}
