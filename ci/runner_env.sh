if [[ "$GITHUB_ACTIONS" == "true" ]]; then
	export _n='%0A'
else
	export _n=$'\n'
fi

_fmt() {
	msg=$(echo "$1" | tr -d '\t')
	if [[ ! "$_n" == $'\n' ]]; then
		msg=$(printf "$msg" | sed ':a;N;$!ba;s/\n/'"$_n"'/g')
	fi
	printf "%s\n" "$msg"
}

export_labels () {
	# DEPRECATED: use export_job_summary
	echo "warn=$(printenv | grep ^step_warn_ | grep -v =$ | tr '\n' ',' | sed 's/,$//')" >> "$GITHUB_OUTPUT"
	echo "fail=$(printenv | grep ^step_fail_ | grep -v =$ | tr '\n' ',' | sed 's/,$//')" >> "$GITHUB_OUTPUT"
}

export_job_summary () {
	json=$(printenv | jq -R -n '
		[inputs] | map(select(index("="))) | map(split("=") | {(.[0]): .[1]}) | add
		| to_entries
		| group_by(
				if .key | startswith("step_fail_") then
					"fail"
				elif .key | startswith("step_warn_") then
					"warn"
				else
					"other"
				end
			)
		| map(
				if .[0].key | startswith("step_fail_") then
					{"fail": map({ (.key|sub("^step_fail_";"")): .value }) | add }
				elif .[0].key | startswith("step_warn_") then
					{"warn": map({ (.key|sub("^step_warn_";"")): .value }) | add }
				else
					empty
				end
			)
		| add
	')
	if [[ "$json" == "null" ]]; then json="" ; fi

	echo "summary<<EOF" >> "$GITHUB_OUTPUT"
	echo "$json" >> $GITHUB_OUTPUT
	echo "EOF" >> "$GITHUB_OUTPUT"

	export_labels # DEPRECATED
}

assert_labels () {
	# DEPRECATED: use assert_job_summary with ${{ needs.*.outputs.summary }}
	local msg="Errors and warnings are annotated on the changed files, in each job step, as well as aggregated on the Summary page.%0ASome messages may not cause the step to fail, but still need to be checked."
	warn=$(printenv | grep ^job_warn_ | grep -v =$ | tr '\n' ';'  | \
	       sed -E -e 's/;/%0A/g' -e 's/\=true(,)?/%0A  /g' -e 's/=/:%0A  /g' -e 's/(job|step)_(fail|warn)_//g')
	fail=$(printenv | grep ^job_fail_ | grep -v =$ | tr '\n' ';'  | \
	       sed -E -e 's/;/%0A/g' -e 's/\=true(,)?/%0A  /g' -e 's/=/:%0A  /g' -e 's/(job|step)_(fail|warn)_//g')

	if [[ ! -z "$fail" ]]; then
	  if [[ ! -z "$warn" ]]; then
	    echo "::error::Some jobs didn't pass strict checks:%0A$fail%0AAnd non-strict checks:%0A$warn%0A$msg"
	  else
	    echo "::error::Some jobs didn't pass strict checks:%0A$fail%0A$msg"
	  fi
	  exit 1
	else
	  if [[ ! -z "$warn" ]]; then
	    warn=%0A$(echo $warn | tr '\n' ';' | sed 's/;/%0A/g')
	    echo "::warning::Some jobs didn't pass non-strict checks:%0A$warn"
	  fi
	fi
}

assert_job_summary () {
	local msg="
	Errors and warnings are annotated on the changed files, in each job step, as well as aggregated on the Summary page.
	Some messages may not cause the step to fail, but still need to be checked."
	local fail=""
	local warn=""

	for _job_var_name in $(compgen -A variable job_); do
		local job_name="${_job_var_name#job_}"
		local json="${!_job_var_name}"
		echo "$job_name"

		local fail_=$(
			echo "$json" | jq -r '
				.fail | to_entries |
				map(
					"    step \(.key)"
				) |
				.[]
			' 2>/dev/null
		)
		[[ -n "$fail_" ]] && fail+=$'\n'"  job $job_name:"$'\n'"$fail_"

		local warn_=$(
			echo "$json" | jq -r '
				.warn | to_entries |
				map(
					"    step \(.key)"
				) |
				.[]
			' 2>/dev/null
		)
		[[ -n "$warn_" ]] && warn+=$'\n'"  job $job_name:"$'\n'"$warn_"
	done

	if [[ -n "$fail" ]]; then
		if [[ -n "$warn" ]]; then
			msg="::error::Some jobs didn't pass strict checks:$fail"$'\n'$'\n'"And non-strict checks:$warn"$'\n'$'\n'"$msg"
		else
			msg="::error::Some jobs didn't pass strict checks:$fail"$'\n'$'\n'"$msg"
		fi
		_fmt "$msg"
		exit 1
	else
		if [[ -n "$warn" ]]; then
			msg="::warning::Some jobs didn't pass non-strict checks:$warn"$'\n'$'\n'"$msg"
		fi
		_fmt "$msg"
	fi
}
