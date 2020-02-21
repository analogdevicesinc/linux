#!/bin/bash
set -e

# cd to docker build dir if it exists
if [ -d /docker_build_dir ] ; then
	cd /docker_build_dir
fi

. ./ci/travis/lib.sh

if [ -f "${FULL_BUILD_DIR}/env" ] ; then
	echo_blue "Loading environment variables"
	cat "${FULL_BUILD_DIR}/env"
	. "${FULL_BUILD_DIR}/env"
fi

# allow this to be configurable; we may want to run it elsewhere
REPO_SLUG=${REPO_SLUG:-analogdevicesinc/linux}

# Run once for the entire script
sudo apt-get -qq update

apt_install() {
	sudo apt-get install -y $@
}

get_pull_requests_urls() {
	wget -q -O- https://api.github.com/repos/${REPO_SLUG}/pulls | jq -r '.[].commits_url'
}

get_pull_request_commits_sha() {
	wget -q -O- $1 | jq -r '.[].sha'
}

branch_has_pull_request() {
	if [ "$TRAVIS_PULL_REQUEST" = "true" ] ; then
		return 1
	fi
	apt_install jq

	for pr_url in $(get_pull_requests_urls) ; do
		for sha in $(get_pull_request_commits_sha $pr_url) ; do
			if [ "$sha" = "$TRAVIS_COMMIT" ] ; then
				TRAVIS_OPEN_PR=$pr_url
				export TRAVIS_OPEN_PR
				return 0
			fi
		done
	done

	return 1
}

# Exit early to save some build time
if branch_has_pull_request ; then
	echo_green "Not running build for branch; there is an open PR @ $TRAVIS_OPEN_PR"
	exit 0
fi

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
	export KCFLAGS
}

APT_LIST="build-essential bc u-boot-tools flex bison libssl-dev"

if [ "$ARCH" == "arm64" ] ; then
	if [ -z "$CROSS_COMPILE" ] ; then
		CROSS_COMPILE=aarch64-linux-gnu-
		export CROSS_COMPILE
	fi

	APT_LIST="$APT_LIST gcc-aarch64-linux-gnu"
fi

if [ "$ARCH" == "arm" ] ; then
	if [ -z "$CROSS_COMPILE" ] ; then
		CROSS_COMPILE=arm-linux-gnueabihf-
		export CROSS_COMPILE
	fi

	APT_LIST="$APT_LIST gcc-arm-linux-gnueabihf"
fi

apt_update_install() {
	apt_install $@
	adjust_kcflags_against_gcc
}

__get_all_c_files() {
	git grep -i "$@" | cut -d: -f1 | sort | uniq  | grep "\.c"
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
		if [ -f "$exceptions_file" ] ; then
			if grep -q "$file1" "$exceptions_file" ; then
				continue
			fi
		fi
		if [ ! -f "$file1" ] ; then
			if [ "$ret" = "0" ] ; then
				echo
				echo_red "The following files need to be built OR"
				echo_green "      added to '$exceptions_file'"

				echo

				echo_green "  If adding the '$exceptions_file', please make sure"
				echo_green "  to check if it's better to add the correct Kconfig symbol"
				echo_green "  to one of the following files:"

				for file in $(find -name Kconfig.adi) ; do
					echo_green "   $file"
				done

				echo
			fi
			echo_red "File '$file1' has not been compiled"
			ret=1
		fi
	done

	return $ret
}

build_default() {
	[ -n "$DEFCONFIG" ] || {
		echo_red "No DEFCONFIG provided"
		return 1
	}

	[ -n "$ARCH" ] || {
		echo_red "No ARCH provided"
		return 1
	}

	APT_LIST="$APT_LIST git"

	apt_update_install $APT_LIST
	make ${DEFCONFIG}
	make -j$NUM_JOBS $IMAGE UIMAGE_LOADADDR=0x8000

	if [ "$CHECK_ALL_ADI_DRIVERS_HAVE_BEEN_BUILT" == "1" ] ; then
		check_all_adi_files_have_been_built
	fi

	make savedefconfig
	mv defconfig arch/$ARCH/configs/$DEFCONFIG

	git diff --exit-code || {
		echo_red "Defconfig file should be updated: 'arch/$ARCH/configs/$DEFCONFIG'"
		echo_red "Run 'make savedefconfig', overwrite it and commit it"
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
	apt_install python-ply
	if [ -n "$TRAVIS_BRANCH" ]; then
		__update_git_ref "${TRAVIS_BRANCH}" "${TRAVIS_BRANCH}"
	fi
	COMMIT_RANGE=$([ "$TRAVIS_PULL_REQUEST" == "false" ] &&  echo HEAD || echo ${TRAVIS_BRANCH}..)
	scripts/checkpatch.pl --git ${COMMIT_RANGE} \
		--ignore FILE_PATH_CHANGES \
		--ignore LONG_LINE \
		--ignore LONG_LINE_STRING \
		--ignore LONG_LINE_COMMENT
}

build_dtb_build_test() {
	if [ "$TRAVIS" = "true" ] ; then
		for patch in $(ls ci/travis/*.patch | sort) ; do
			patch -p1 < $patch
		done
	fi
	local last_arch
	for file in $DTS_FILES; do
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
		ARCH=$arch make ${dtb_file} -j$NUM_JOBS || exit 1
	done
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
	[ "$GIT_FETCH_DEPTH" == "disabled" ] || {
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
		echo_red "Failed to push back '$dst_branch'"
		return 1
	}
}

__handle_sync_with_master() {
	local dst_branch="$1"
	local method="$2"

	__update_git_ref "$dst_branch" || {
		echo_red "Could not fetch branch '$dst_branch'"
		return 1
	}

	if [ "$method" == "fast-forward" ] ; then
		git checkout FETCH_HEAD
		git merge --ff-only ${ORIGIN}/master || {
			echo_red "Failed while syncing ${ORIGIN}/master over '$dst_branch'"
			return 1
		}
		__push_back_to_github "$dst_branch" || return 1
		return 0
	fi

	if [ "$method" == "cherry-pick" ] ; then
		local depth
		if [ "$GIT_FETCH_DEPTH" == "disabled" ] ; then
			depth=50
		else
			GIT_FETCH_DEPTH=${GIT_FETCH_DEPTH:-50}
			depth=$((GIT_FETCH_DEPTH - 1))
		fi
		# FIXME: kind of dumb, the code below; maybe do this a bit neater
		local cm="$(git log "FETCH_HEAD~${depth}..FETCH_HEAD" | grep "cherry picked from commit" | head -1 | awk '{print $5}' | cut -d')' -f1)"
		[ -n "$cm" ] || {
			echo_red "Top commit in branch '${dst_branch}' is not cherry-picked"
			return 1
		}
		branch_contains_commit "$cm" "${ORIGIN}/master" || {
			echo_red "Commit '$cm' is not in branch master"
			return 1
		}
		# Make sure that we are adding something new, or cherry-pick complains
		if git diff --quiet "$cm" "${ORIGIN}/master" ; then
			return 0
		fi

		tmpfile=$(mktemp)

		git checkout FETCH_HEAD
		# cherry-pick until all commits; if we get a merge-commit, handle it
		git cherry-pick -x "${cm}..${ORIGIN}/master" 1>/dev/null 2>$tmpfile || {
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
				echo_red "Failed to cherry-pick commits '$cm..${ORIGIN}/master'"
				echo_red "$(cat $tmpfile)"
				return 1
			fi
		}
		__push_back_to_github "$dst_branch" || return 1
		return 0
	fi
}

build_sync_branches_with_master() {
	GIT_FETCH_DEPTH=50
	BRANCHES="xcomm_zynq:fast-forward adi-4.19.0:cherry-pick"
	BRANCHES="$BRANCHES rpi-4.19.y:cherry-pick altera_4.14:cherry-pick"
	BRANCHES="$BRANCHES adi-iio:cherry-pick"

	for branch in $BRANCHES ; do
		local dst_branch="$(echo $branch | cut -d: -f1)"
		[ -n "$dst_branch" ] || break
		local method="$(echo $branch | cut -d: -f2)"
		[ -n "$method" ] || break
		__handle_sync_with_master "$dst_branch" "$method"
	done
}

build_sync_branches_with_master_travis() {
	# make sure this is on master, and not a PR
	[ -n "$TRAVIS_PULL_REQUEST" ] || return 0
	[ "$TRAVIS_PULL_REQUEST" == "false" ] || return 0
	[ "$TRAVIS_BRANCH" == "master" ] || return 0
	[ "$TRAVIS_REPO_SLUG" == "analogdevicesinc/linux" ] || return 0

	git remote set-url $ORIGIN "git@github.com:analogdevicesinc/linux.git"
	openssl aes-256-cbc -d -in ci/travis/deploy_key.enc -out /tmp/deploy_key -base64 -K $encrypt_key -iv $encrypt_iv
	eval "$(ssh-agent -s)"
	chmod 600 /tmp/deploy_key
	ssh-add /tmp/deploy_key

	build_sync_branches_with_master
}

ORIGIN=${ORIGIN:-origin}

BUILD_TYPE=${BUILD_TYPE:-${1}}
BUILD_TYPE=${BUILD_TYPE:-default}

build_${BUILD_TYPE}
