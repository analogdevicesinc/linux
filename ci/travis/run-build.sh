#!/bin/bash
set -e

. ./ci/travis/lib.sh

build_default() {
	make ${DEFCONFIG_NAME}
	make -j`getconf _NPROCESSORS_ONLN` $IMAGE UIMAGE_LOADADDR=0x8000
}

build_compile_test() {
	export COMPILE_TEST=y
	make ${DEFCONFIG_NAME}
	make -j`getconf _NPROCESSORS_ONLN`
}

build_checkpatch() {
	if [ -n "$TRAVIS_BRANCH" ]; then
		__update_git_ref ${TRAVIS_BRANCH}
	fi
	COMMIT_RANGE=$([ "$TRAVIS_PULL_REQUEST" == "false" ] &&  echo HEAD || echo ${TRAVIS_BRANCH}..)
	scripts/checkpatch.pl --git ${COMMIT_RANGE} \
		--ignore FILE_PATH_CHANGES \
		--ignore LONG_LINE \
		--ignore LONG_LINE_STRING \
		--ignore LONG_LINE_COMMENT
}

build_dtb_build_test() {
	for file in $DTS_FILES; do
		make ${DTS_PREFIX}`basename $file | sed  -e 's\dts\dtb\g'` || exit 1
	done
}

branch_contains_commit() {
	local commit="$1"
	local branch="$2"
	git merge-base --is-ancestor $commit $branch &> /dev/null
}

__update_git_ref() {
	local ref="$1"
	git fetch origin +refs/heads/${ref}:${ref}
}

__push_back_to_github() {
	local dst_branch="$1"
	git remote set-url origin "git@github.com:analogdevicesinc/linux.git"

	git push --quiet -u origin "$dst_branch" || {
		echo_red "Failed to push back '$dst_branch'"
		return 1
	}
}

__handle_sync_with_master() {
	local dst_branch="$1"
	local method="$2"

	openssl aes-256-cbc -d -in ci/travis/deploy_key.enc -out /tmp/deploy_key -base64 -K $encrypt_key -iv $encrypt_iv
	eval "$(ssh-agent -s)"
	chmod 600 /tmp/deploy_key
	ssh-add /tmp/deploy_key

	for ref in master ${dst_branch} ; do
		__update_git_ref "$ref" || {
			echo_red "Could not fetch branch '$dst_branch'"
			return 1
		}
	done

	if [ "$method" == "fast-forward" ] ; then
		git checkout ${dst_branch}
		git merge --ff-only master || {
			echo_red "Failed while syncing master over '$dst_branch'"
			return 1
		}
		__push_back_to_github "$dst_branch" || return 1
		return 0
	fi

	if [ "$method" == "cherry-pick" ] ; then
		# FIXME: kind of dumb, the code below; maybe do this a bit neater
		local cm="$(git log "${dst_branch}~1..${dst_branch}" | grep "cherry picked from commit" | awk '{print $5}' | cut -d')' -f1)"
		[ -n "$cm" ] || {
			echo_red "Top commit in branch '${dst_branch}' is not cherry-picked"
			return 1
		}
		branch_contains_commit "$cm" "master" || {
			echo_red "Commit '$cm' is not in branch master"
			return 1
		}

		tmpfile=$(mktemp)

		git checkout ${dst_branch}
		# cherry-pick until all commits; if we get a merge-commit, handle it
		git cherry-pick -x "${cm}..master" 1>/dev/null 2>$tmpfile || {
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
			if [ "$was_a_merge" != "0" ]; then
				echo_red "Failed to cherry-pick commits '$cm..master'"
				echo_red "$(cat $tmpfile)"
				return 1
			fi
		}
		__push_back_to_github "$dst_branch" || return 1
		return 0
	fi
}

build_sync_branches_with_master() {
	# make sure this is on master, and not a PR
	[ -n "$TRAVIS_PULL_REQUEST" ] || return 0
	[ "$TRAVIS_PULL_REQUEST" == "false" ] || return 0
	[ "$TRAVIS_BRANCH" == "master" ] || return 0

	# support sync-ing up to 100 branches; should be enough
	for iter in $(seq 1 100) ; do
		local branch="$(eval "echo \$BRANCH${iter}")"
		[ -n "$branch" ] || break
		local dst_branch="$(echo $branch | cut -d: -f1)"
		[ -n "$dst_branch" ] || break
		local method="$(echo $branch | cut -d: -f2)"
		[ -n "$method" ] || break
		__handle_sync_with_master "$dst_branch" "$method"
	done
}

BUILD_TYPE=${BUILD_TYPE:-default}

build_${BUILD_TYPE}
