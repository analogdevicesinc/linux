sync_branches () {
	local fail_
	local fail=0

	echo "sync_branches_with_main on range $base_sha..$head_sha"

	if [[ "$fetch_depth_fallback" ]]; then
		echo "cannot sync from force push"
		set_step_fail "sync_branches"
		return
	fi
	for branch in $@ ; do
		echo "patching branch $branch"
		git switch -d
		git fetch origin $branch:$branch --depth=1 -f || continue # skip on missing ref
		git switch $branch

		range=$(($(git rev-list --count $base_sha..$head_sha) - 1 ))
		for (( iter=$range; iter >= 0; iter-- )); do
			fail_=0
			commit=$(git rev-parse $head_sha~$iter)
			# check if merge commit, and choose parent if so
			git rev-parse $commit^2  2>/dev/null &&
				merge="-m 1"|| merge=""
			git cherry-pick --allow-empty --empty=keep $merge $commit 1>/dev/null || fail_=1
			if [[ "$fail_" == "1" ]]; then
				git cherry-pick --abort
				break
			fi
		done
		if [[ "$fail_" == "1" ]]; then
			printf "failed to cherry-pick at $commit"
			fail=1
			break
		fi
		git push origin HEAD:$branch
	done

	if [[ "$fail" == "1" ]]; then
		set_step_fail "sync_branches"
	fi
}

set_step_warn () {
	echo ; echo "step_warn_$1=true" >> "$GITHUB_ENV"
}

set_step_fail () {
	echo ; echo "step_fail_$1=true" >> "$GITHUB_ENV"
}
