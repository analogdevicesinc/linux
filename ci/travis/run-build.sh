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
	sudo -s <<-EOF
		apt-get -qq update
		apt-get -y install $@
	EOF
	adjust_kcflags_against_gcc
}

build_default() {
	apt_update_install $APT_LIST
	make ${DEFCONFIG}
	make -j$NUM_JOBS $IMAGE UIMAGE_LOADADDR=0x8000
}

build_compile_test() {
	apt_update_install $APT_LIST
	export COMPILE_TEST=y
	make ${DEFCONFIG}
	make -j$NUM_JOBS
}

build_checkpatch() {
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
	apt_update_install $APT_LIST
	make ${DEFCONFIG:-defconfig}
	for file in $DTS_FILES; do
		dtb_file=$(echo $file | sed 's/dts\//=/g' | cut -d'=' -f2 | sed 's\dts\dtb\g')
		make ${dtb_file} || exit 1
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
