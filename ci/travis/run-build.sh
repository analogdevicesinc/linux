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

MAIN_BRANCH=${MAIN_BRANCH:-master}

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
		elif git diff "$COMMIT_RANGE" "$file" | grep "SPDX-License-Identifier:" | grep -qi "OR" ; then
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

	# !\FIXME: This is a very hacky way to make sure the 'savedefconfig' mechanism in the
	# end of the function won't trigger a CI build failure. The issue here is that with
	# xilinx toolchain, the build system will detect that GCC_PLUGINS are enabled, as the
	# toolchan "supports" them, but the build will fail to build them... As we want to be able
	# to compile the kernel with xilinx toolchain (do we?) we need to explicitly disable
	# GCC_PLUGINS in our defconfigs (as xilinx is also doing BTW).
	#
	# So far so good... The problem now comes in the CI build because the distro toolchains
	# don't have any GCC_PLUGINS support which means that the kernel build system won't enable
	# GCC_PLUGINS and thus, when running 'make savedefconfig', the line
	# '# CONFIG_GCC_PLUGINS is not set' will be removed causing the CI build to fail because now
	# the new defconfig differs the one in git (see some lines below in the function).
	#
	# All the above said, the solution is just to remove the line where we disable the PLUGINS and
	# locally commit it. Yes, this is very hacky but I'm not seeing any other way unless we:
	#
	#  1) Just remove the 'savedefconfig' mechanism from here.
	#  2) Don't care about xilinx toolchain so that we don't need to change our defconfigs. Anyone
	#     using the toolchain will have to locally handle this...
	#  3) Ideas?
	#
	# Also note that this is only an issue for ARM...
	#
	# We should keep an eye on this (every time we upgrade) so we can remove this as soon as possible...
	[ "$ARCH" = "arm" ] && {
		sed -i  '/CONFIG_GCC_PLUGINS/d' arch/arm/configs/$DEFCONFIG
		__setup_dummy_git_account
		# don't error out if the commit fails as we don't explicitly disable the plugins for
		# some configs (eg: socfga).
		git commit -a -m "dummy commit to remove CONFIG_GCC_PLUGINS" || true
	}

	make ${DEFCONFIG}
	if [[ "${SYSTEM_PULLREQUEST_TARGETBRANCH}" =~ ^rpi-.* || "${BUILD_SOURCEBRANCH}" =~ ^refs/heads/rpi-.* \
		|| "${BUILD_SOURCEBRANCH}" =~ ^refs/heads/staging-rpi ]]; then
		echo "Rpi build"
    		make -j$NUM_JOBS zImage modules dtbs
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
	# master such that we cannot find a common ancestor. In that case, the job will
	# timeout after 60min even if the branch is able to merge (even if diverged). We
	# could do '$GIT_FETCH_DEPTH="disable"' before calling __update_git_ref() but that
	# would slow things a lot. Instead, let's do a treeless fetch which get's the whole
	# history while being much faster than a typical fetch.
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
	[ "${LOCAL_BUILD}" == "y" ] || pip3 install dtschema

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
				error_txt=$(make dt_binding_check DT_CHECKER_FLAGS=-m DT_SCHEMA_FILES="$relative_yaml" 2>&1)
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

build_dtb_build_test() {
	local exceptions_file="ci/travis/dtb_build_test_exceptions"
	local err=0
	local last_arch

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
		if [ "$last_arch" != "$arch" ] ; then
			ARCH=$arch make defconfig
			last_arch=$arch
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

MAIN_MIRROR="xcomm_zynq"

__update_main_mirror() {
	git checkout "$MAIN_MIRROR"
	git merge --ff-only ${ORIGIN}/${MAIN_BRANCH} || {
		__echo_red "Failed while syncing ${ORIGIN}/${MAIN_BRANCH} over '$MAIN_MIRROR'"
		return 1
	}

	__push_back_to_github "$MAIN_MIRROR"
	return $?
}

__handle_sync_with_main() {
	local dst_branch="$1"
	local cm=$(git log --reverse --oneline ${MAIN_MIRROR}..${MAIN_BRANCH} | awk '{print $1}' | head -1)

	[ -n "$cm" ] || {
		__echo_red "No commits to cherry-pick... Was "${MAIN_MIRROR}" manually updated?!"
		return 1
	}

	__update_git_ref "$dst_branch" || {
		__echo_red "Could not fetch branch '$dst_branch'"
		return 1
	}

	tmpfile=$(mktemp)

	[ "$CI" = "true" ] && __setup_dummy_git_account

	git checkout FETCH_HEAD
	# cherry-pick until all commits; if we get a merge-commit, handle it. Note that
	# ~1 is because we also want ${cm} to be cherry-picked!
	git cherry-pick -x "${cm}~1..${ORIGIN}/${MAIN_BRANCH}" 1>/dev/null 2>$tmpfile || {
		was_a_merge=0
		while grep -q "is a merge" $tmpfile ; do
			was_a_merge=1
			# clear file
			cat /dev/null > $tmpfile
			# retry ; we may have a new merge commit
			git cherry-pick --continue 1>/dev/null 2>$tmpfile || {
				was_a_merge=0
				continue
			}
		done
		if [ "$was_a_merge" != "1" ]; then
			__echo_red "Failed to cherry-pick commits '$cm..${ORIGIN}/${MAIN_BRANCH}'"
			__echo_red "$(cat $tmpfile)"
			git cherry-pick --abort
			return 1
		fi
	}
	__push_back_to_github "$dst_branch" || return 1
	return 0
}

build_sync_branches_with_main() {
	GIT_FETCH_DEPTH=50
	BRANCHES="adi-6.1.0 rpi-6.1.y"

	__update_git_ref "$MAIN_BRANCH" "$MAIN_BRANCH" || {
		__echo_red "Could not fetch branch '$MAIN_BRANCH'"
		return 1
	}

	# needed for __handle_sync_with_main() so we can properly get the list
	# of commits to cherry-pick
	__update_git_ref "$MAIN_MIRROR" "$MAIN_MIRROR" || {
		__echo_red "Could not fetch branch '$MAIN_MIRROR'"
		return 1
	}

	for branch in $BRANCHES ; do
		__handle_sync_with_main "$branch" || {
			# In case cherry-picking fails, we need to still make sure that our mirror
			# get's updated. Otherwise in the next time this job is called, we will
			# have commits to cherry-pick that do not belong to this call... Furthermore
			# at this stage the cherry-pick needs to be handled manually. Note that,
			# in theory, the cherry-pick should never fail for the ADI rebased branch
			# (adi-${kernerversion}) but that is not true for the pi branch where it can
		        # fail and where it might actually be acceptable to fail (when touching in
			# xilinx specific code that we do not care in pi platforms).
			__update_main_mirror "$MAIN_MIRROR"
			return 1
		}
	done

	__update_main_mirror "$MAIN_MIRROR"
}

ORIGIN=${ORIGIN:-origin}

BUILD_TYPE=${BUILD_TYPE:-${1}}
BUILD_TYPE=${BUILD_TYPE:-default}

build_${BUILD_TYPE}
