export_labels () {
	echo "warn=$(printenv | grep ^step_warn_ | grep -v =$ | tr '\n' ',' | sed 's/,$//')" >> "$GITHUB_OUTPUT"
	echo "fail=$(printenv | grep ^step_fail_ | grep -v =$ | tr '\n' ',' | sed 's/,$//')" >> "$GITHUB_OUTPUT"
}

assert_labels () {
	warn=$(printenv | grep ^job_warn_ | grep -v =$ | tr '\n' ';'  | \
	       sed -E -e 's/;/%0A/g' -e 's/\=true(,)?/%0A  /g' -e 's/=/:%0A  /g' -e 's/(job|step)_(fail|warn)_//g')
	fail=$(printenv | grep ^job_fail_ | grep -v =$ | tr '\n' ';'  | \
	       sed -E -e 's/;/%0A/g' -e 's/\=true(,)?/%0A  /g' -e 's/=/:%0A  /g' -e 's/(job|step)_(fail|warn)_//g')

	if [[ ! -z "$fail" ]]; then
	  if [[ ! -z "$warn" ]]; then
	    echo "::error::Some jobs didn't pass strict checks:%0A$fail%0AAnd non-strict checks:%0A$warn"
	  else
	    echo "::error::Some jobs didn't pass strict checks:%0A$fail"
	  fi
	  exit 1
	else
	  if [[ ! -z "$warn" ]]; then
	    warn=%0A$(echo $warn | tr '\n' ';' | sed 's/;/%0A/g')
	    echo "::warning::Some jobs didn't pass non-strict checks:%0A$warn"
	  fi
	fi
}
