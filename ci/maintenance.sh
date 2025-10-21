cherry_pick () {
	# Cherry-pick commits into a branch
	# Takes two parameters, the branch names, and the filters
	local fail_=
	local fail=0
	local fail_push=0
	local initial_branch=
	local failed_commits=()
	local skipped_commits=()
	local files=
	local skip_=
	local skip=

	echo "cherry_pick branch $branch on range $base_sha..$head_sha"

	if [[ "$fetch_depth_fallback" ]]; then
		echo "cannot sync from force push"
		set_step_fail "cherry_pick"
		return
	fi
	initial_branch=$(git rev-parse --abbrev-ref @)
	git switch -d
	git fetch origin $branch:$branch --depth=1 -f || return # on missing ref
	git switch $branch

	range=$(($(git rev-list --count $base_sha..$head_sha) - 1 ))
	for (( iter=$range; iter >= 0; iter-- )); do
		fail_=0
		commit=$(git rev-parse $head_sha~$iter)
		# check if merge commit, and choose parent if so
		git rev-parse $commit^2  &>/dev/null &&
			merge="-m 1" || merge=""
		files=$(git diff --diff-filter=ACM --no-renames --name-only $commit~..$commit)
		skip_=0
		for filter in $2 ; do
			for file in $files ; do
				if [[ "$file" =~ $filter ]]; then
					skip_=1
					break 2
				fi
			done
		done
		if [[ "$skip_" == "1" ]]; then
			echo "Skipped cherry-pick $commit (filtered)."
			skipped_commits+=("$commit")
			skip=1
		else
			echo "Cherry-pick $commit $merge"
		fi
		git --no-pager show --name-only --pretty=format:%s $commit
		[[ "$skip_" == "1" ]] && continue
		git cherry-pick -x --allow-empty --empty=drop $merge $commit 1>/dev/null || fail_=1
		if [[ "$fail_" == "1" ]]; then
			git cherry-pick --abort
			failed_commits+=("$commit")
			printf "Failed to cherry-pick $commit\n"
			fail=1
		fi
	done
	if [[ "$skip" == "1" ]]; then
		printf "\nFor branch '$branch', skipped cherry-pick for:\n"
		for item in "${skipped_commits[@]}"; do
			printf "$(echo $item | cut -c1-12) "
			git --no-pager log -1 --pretty=format:%s $item
			printf "\n"
		done
	fi
	if [[ "$fail" == "1" ]]; then
		printf "\nFor branch '$branch', failed to cherry-pick:\n"
		for item in "${failed_commits[@]}"; do
			printf "$(echo $item | cut -c1-12) "
			git --no-pager log -1 --pretty=format:%s $item
			printf "\n"
		done
		set_step_fail "cherry_pick"
	fi
	git push origin HEAD:$branch || fail_push="1"
	if [[ "$fail_push" == "1" ]]; then
		printf "\nFailed to push branch '$branch'\n"
	fi
	git switch $initial_branch

}

set_step_warn () {
	echo ; echo "step_warn_$1=true" >> "$GITHUB_ENV"
}

set_step_fail () {
	echo ; echo "step_fail_$1=true" >> "$GITHUB_ENV"
}
