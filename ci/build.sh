if [[ -z "$run_id" ]]; then
	export run_id=$(uuidgen)
fi

if [[ "$GITHUB_ACTIONS" == "true" ]]; then
	export _n='%0A'
	export _c='%2C'
else
	export _n=$'\n'
	export _c=$','
fi

_fmt() {
	msg=$(echo "$1" | tr -d '\t')
	if [[ ! "$_n" == $'\n' ]]; then
		msg=$(printf "$msg" | sed ':a;N;$!ba;s/\n/'"$_n"'/g')
	fi
	printf "%s\n" "$msg"
}

_file () {
	$(echo "$1" | sed 's/,/'"$_c"'/g')
}

check_checkpatch() {
	export step_name="checkpatch"
	local mail=
	local fail=0
	local warn=0

	echo "$step_name on range $base_sha..$head_sha"

	python3.11 -m venv ~/venv
        source ~/venv/bin/activate
        pip3.11 install ply GitPython --upgrade

	# The output is not properly captured with --git
	for commit in $(git rev-list $base_sha..$head_sha --reverse); do
		git --no-pager show --format="%h %s" "$commit" --name-only
		# Skip empty commits, assume cover letter
		# and those only touching non-upstream directories .github ci and docs
		# A treeless (--filter=tree:0) fetch could be done to fetch
		# full commit message history before running checkpatch, but
		# that may mess-up the cache and is only useful for checking
		# SHA references in the commit message, with may not even apply
		# if the commit is from upstream. Instead, just delegate to the
		# user to double check the referenced SHA.
		local files=$(git diff --diff-filter=ACM --no-renames --name-only $commit~..$commit | grep -v ^ci | grep -v ^.github | grep -v ^docs || true)
		if [[ -z "$files" ]]; then
			echo "empty, skipped"
			continue
		fi
		mail=$(scripts/checkpatch.pl --git "$commit" \
			--strict \
			--ignore FILE_PATH_CHANGES \
			--ignore LONG_LINE \
			--ignore LONG_LINE_STRING \
			--ignore LONG_LINE_COMMENT \
			--ignore PARENTHESIS_ALIGNMENT \
			--ignore CAMELCASE \
			--ignore UNDOCUMENTED_DT_STRING)

		found=0
		msg=

		while read -r row
		do
			if [[ "$row" =~ ^total: ]]; then
				echo -e "\e[1m$row\e[0m"
				break
			fi

			# Additional parsing is needed
			if [[ "$found" == "1" ]]; then

				# The row is started with "#"
				if [[ "$row" =~ ^\# ]]; then
					# Split the string using ':' separator
					IFS=':' read -r -a list <<< "$row"

					# Get file-name after removing spaces.
					file=$(echo ${list[2]} | xargs)

					# Get line-number
					line=${list[3]}
					echo $row
				else
					msg=${msg}${row}$_n
					if [[ -z $row ]]; then
						if [[ -z $file ]]; then
							# If no file, add to file 0 of first file on list.
							file=$(git show --name-only --pretty=format: $commit | head -n 1)
							echo "::$type file=$(_file "$file"),line=0::$step_name: $msg"
						else
							echo "::$type file=$(_file "$file"),line=$line::$step_name: $msg"
						fi
						found=0
						file=
						line=
					fi
				fi
			fi

			if [[ "$row" =~ ^ERROR: ]]; then
				type="error"
				fail=1
			elif [[ "$row" =~ ^WARNING: ]]; then
				type="warning"
				warn=1
			elif [[ "$row" =~ ^CHECK: ]]; then
				type="warning"
			fi
			if [[ "$row" =~ ^(CHECK|WARNING|ERROR): ]]; then
				msg=$(echo "$row" | sed -E  's/^(CHECK|WARNING|ERROR): //')$_n
				# Suppress some cases:
				if [[ "$row" =~ ^"WARNING: Unknown commit id" ]]; then
					# Checkpatch may want to look back beyond fetched commits.
					echo $row
				else
					found=1
				fi
				file=
				line=
			else
				if [[ "$found" == "0" ]]; then
					echo $row
				fi
			fi

		done <<< "$mail"
	done

	_set_step_warn $warn
	return $fail
}

check_dt_binding_check() {
	export step_name="dt_binding_check"
	local files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha -- 'Documentation/devicetree/bindings/**/*.yaml')
	local fail=0

	echo "$step_name on range $base_sha..$head_sha"

	python3.11 -m venv ~/venv
        source ~/venv/bin/activate
        pip3.11 install dtschema yamllint --upgrade

	[[ -z "$files" ]] && return 0
	while read file; do
		local relative_yaml=${file#Documentation/devicetree/bindings/}
		local file_ex=$(realpath ${file%.yaml}.example.dtb)

		echo "Testing devicetree binding $file"

		# The dt_binding_check rule won't exit with an error
		# for schema errors, but will exit with an error for
		# dts example errors.
		#
		# All schema files must be validated before exiting,
		# so the script should not exit on error.
		if ! error_txt=$(make dt_binding_check CONFIG_DTC=y DT_CHECKER_FLAGS=-m DT_SCHEMA_FILES="$relative_yaml" 2>&1); then
			fail=1
		fi

		echo "$error_txt"

		# file name or realpath of example appears in output if it contains errors
		if echo "$error_txt" | grep -qF -e "$file" -e "$file_ex"; then
			fail=1
			echo "::error file=$(_file "$file"),line=0::$step_name contain errors"
		fi
	done <<< "$files"

	_set_step_warn $warn
	return $fail
}

check_coccicheck() {
	export step_name="coccicheck"
	local files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha -- '**/*.c')
	local mail=
	local warn=0

	echo "$step_name on range $base_sha..$head_sha"

	if [[ -z "$ARCH" ]]; then
		ARCH=x86
	fi

	coccis=$(ls scripts/coccinelle/**/*.cocci)
	[[ -z "$coccis" ]] && return 0
	[[ -z "$files" ]] && return 0
	while read file; do
		echo -e "\e[1m$file\e[0m"

		while read cocci; do

			mail=$(spatch -D report  --very-quiet --cocci-file $cocci --no-includes \
				    --include-headers --patch . \
				    -I arch/$ARCH/include -I arch/$ARCH/include/generated -I include \
				    -I arch/$ARCH/include/uapi -I arch/$ARCH/include/generated/uapi \
				    -I include/uapi -I include/generated/uapi \
				    --include include/linux/compiler-version.h \
				    --include include/linux/kconfig.h $file || true)

			msg=

			while read -r row
			do
				# There is no standard for cocci, so this is the best we can do
				# is match common beginnings and log the rest
				if [[ "$row" =~ ^$file: ]]; then
					type="warning"
					warn=1
				fi

				if [[ "$row" =~ ^(warning): ]]; then
					# warning: line 223: should nonseekable_open be a metavariable?
					# internal cocci warning, not user fault
					echo $row
				elif [[ "$row" =~ ^$file: ]]; then
					# drivers/iio/.../adi_adrv9001_fh.c:645:67-70: duplicated argument to & or |
					IFS=':' read -r -a list <<< "$row"
					line=${list[1]}
					msg=
					for ((i=2; i<${#list[@]}; i++)); do
						msg="$msg${list[$i]} "
					done
					echo "::$type file=$(_file "$file"),line=$line::$step_name: $msg"
				else
					if [[ "$row" ]]; then
						echo $row
					fi
				fi

			done <<< "$mail"

		done <<< "$coccis"

	done <<< "$files"

	_set_step_warn $warn
	return 0
}

_bad_licence_error() {
	local license_error="
	File is being added to Analog Devices Linux tree.
	Analog Devices code is being marked dual-licensed... Make sure this is really intended!
	If not intended, change MODULE_LICENSE() or the SPDX-License-Identifier accordingly.
	This is not as simple as one thinks and upstream might require a lawyer to sign the patches!"
	_fmt "::warning file=$1::$step_name: $license_error"
}

check_license() {
	export step_name="check_license"
	local fail=0

	echo "$step_name on range $base_sha..$head_sha"

	local added_files=$(git diff --diff-filter=A --name-only "$base_sha..$head_sha")

	# Get list of new files in the commit range
	for file in $added_files ; do
		if git diff $base_sha..$head_sha "$file" 2>/dev/null | grep "^+MODULE_LICENSE" | grep -q "Dual" ; then
			_bad_licence_error "$file"
			fail=1
		elif git diff $base_sha..$head_sha "$file" 2>/dev/null | grep "^+// SPDX-License-Identifier:" | grep -qi " OR " ; then
			# The below might catch bad licenses in header files and also helps to make sure dual licenses are
			# not in driver (e.g.: sometimes people have MODULE_LICENSE != SPDX-License-Identifier - which is also
			# wrong and maybe something to improve in this job).
			# For devicetree-related files, allow dual license if GPL-2.0 is one of them.
			if [[ "$file" == *.@(yaml|dts|dtsi|dtso) ]]; then
				if cat "$file" | grep "^// SPDX-License-Identifier:" | grep -q "GPL-2.0" ; then
					continue
				fi
			fi
			_bad_licence_error "$file"
			fail=1
		fi
	done

	_set_step_warn $warn
	return $fail
}

compile_devicetree() {
	export step_name="compile_devicetree"
	local exceptions_file="ci/travis/dtb_build_test_exceptions"
	local tmp_log_file=/dev/shm/$run_id.ci_compile_devicetree.log
	local err=0
	local fail=0
	local dtb_file=
	local dts_files=""
	local regex0='^([[:alnum:]/._-]+)(:([0-9]+)\.([0-9]+)-([0-9]+)\.([0-9]+))?: (Warning|Error) (.*)$'
	local regex1='^Error: ([[:alnum:]/._-]+)(:([0-9]+)\.([0-9]+)-([0-9]+))? (.+)$'

	if [[ -z "$ARCH" ]]; then
		echo "::error ::$step_name: ARCH is not set."
		return 1
	fi

	echo "compile devicetree on range $base_sha..$head_sha"

	local files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha)
	local dtsi_files=$(echo "$files" | grep ^arch/$ARCH/boot/dts/ | grep dtsi$ || true)
	if [[ ! -z "$dtsi_files" ]]; then
		echo "collecting dts files that include dtsi"
		while read file; do
			echo $file
			basename=$(basename $file)
			dirname=$(dirname $file)
			dts_files+=\ $(find "$dirname" -name "*.dts" | xargs -P "$(nproc)" -I{} \
				sh -c 'if grep -q "#include \"$(basename "$0")\"" "$1" ; then echo "$1" ; fi' "$file" {})
		done <<< "$dtsi_files"
	fi

	dts_files+=\ $(echo "$files" | grep ^arch/$ARCH/boot/dts/ | grep dts$ || true)
	dts_files=$(echo $dts_files | xargs)
	if [[ -z "$dts_files" ]]; then
		echo "no dts on range, skipped"
		return $err
	fi

	# Only check fpga arm & arm64 DTs, they are shipped via SD-card
	hdl_dts_files=$(echo "$dts_files" | tr ' ' '\n' | grep -E 'arch/arm(64)?' | grep -E '/(socfpga|xilinx|microblaze)/' || true)
	if [[ -f $exceptions_file ]]; then
		hdl_dts_files=$(comm -13 <(sort $exceptions_file) <(echo $hdl_dts_files | tr ' ' '\n' | sort))
	fi
	local hdl_dts_error="
	DTS does not contain 'hdl_project:' tag
	 Either:
	  1. Create a 'hdl_project' tag for it
	 OR
	  2. add it in file '$exceptions_file'"
	for file in $hdl_dts_files; do
		if ! grep -q "hdl_project:" $file ; then
			_fmt "::error file=$file::$step_name: $hdl_dts_error"
			fail=1
		fi
	done

	echo > $tmp_log_file
	dtb_files=$(for file in $dts_files; do
		echo $file | sed 's/dts\//=/g' | cut -d'=' -f2 | sed 's/\.dts\>/.dtb/g'
	done | sort -u)

	echo "compiling devicetrees"

	touch $tmp_log_file
	make -i $dtb_files -j$(nproc) 2>&1 | \
	(while IFS= read -r row; do
		echo $row
		if [[ "$row" =~ $regex0 ]]; then
			file=$(echo ${BASH_REMATCH[1]} | xargs)
			type=$(echo ${BASH_REMATCH[7]} | xargs  | tr '[:upper:]' '[:lower:]')
			msg_=${BASH_REMATCH[8]}

			if [[ -z "${BASH_REMATCH[2]}" ]]; then
				msg="::$type file=$file,line=0::$msg_"
			else
				line="${BASH_REMATCH[3]}"
				col="${BASH_REMATCH[4]}"
				end_line="${BASH_REMATCH[5]}"
				end_col="${BASH_REMATCH[6]}"
				echo "::$type file=$file,line=$line,col=$col,endLine=$end_line,endColumn=$end_col::$msg_" >> $tmp_log_file
			fi

			if [[ "$type" == "warning" ]]; then
				warn=1
			elif [[ "$type" == "error" ]]; then
				fail=1
			fi
		elif [[ "$row" =~ $regex1 ]]; then
			file=$(echo ${BASH_REMATCH[1]} | xargs)
			msg_="${BASH_REMATCH[6]}"

			if [[ -z "${BASH_REMATCH[2]}" ]]; then
				echo "::error file=$file,line=0::$msg_"
			else
				line="${BASH_REMATCH[3]}"
				col="${BASH_REMATCH[4]}"
				end_col="${BASH_REMATCH[5]}"
				echo "::error file=$file,line=$line,col=$col,endColumn=$end_col::$msg_" >> $tmp_log_file
			fi
		fi
	done) ; err=${PIPESTATUS[1]}

	if [[ $err -ne 0 ]]; then
		fail=1
	fi

	sort -u $tmp_log_file
	rm $tmp_log_file

	_set_step_warn $warn
	return $fail
}

compile_many_devicetrees() {
	export step_name="compile_many_devicetrees"
	local exceptions_file="ci/travis/dtb_build_test_exceptions"
	local err=0
	local dtb_file=
	local dts_files=""

	if [[ -z "$ARCHS" ]]; then
		echo "::error ::$step_name: ARCHS list is not set."
		return 1
	fi
	if [[ -z "$DTS_FILES" ]]; then
		echo "::error ::$step_name: DTS_FILES glob rules are not set."
		return 1
	fi

	dts_files=$DTS_FILES
	if [[ -f $exceptions_file ]]; then
		dts_files=$(comm -13 <(sort $exceptions_file) <(echo $dts_files | tr ' ' '\n' | sort))
	fi
	for ARCH in $ARCHS; do
		dts_files_=$(echo $dts_files | tr ' ' '\n' | grep ^arch/$ARCH/ | sed 's/dts\//=/g' | cut -d'=' -f2 | sed 's/\.dts\>/.dtb/')
		ARCH=$ARCH make allnoconfig
		ARCH=$ARCH make -k -j$(nproc) $dts_files_ || err=$?
	done

	return $err
}

compile_kernel() {
	export step_name="kernel"
	local tmp_log_file=/dev/shm/$run_id.ci_compile_kernel.log
	local err=0
	local regex='^[[:alnum:]/._-]+:[[:digit:]]+:[[:digit:]]+: .*$'
	local fail=0
	local warn=0
	local msg=

	# At this step, we only problem match if it exits with an error code
	# because we don't want duplicated errors/warnings on sparse,
	# neither warnings unrelated to the patch, but if the kernel
	# does not even compile, the checkers don't run and the reason
	# is at this step.

	echo "$step_name"

	echo > $tmp_log_file
	yes n 2>/dev/null | \
		make -j$(nproc) 2>&1 | \
		(while IFS= read -r row; do
		echo $row
		if [[ "$row" =~ $regex ]]; then
			if [[ "$found" == "1" ]]; then
				echo $msg >> $tmp_log_file
				msg=
			fi

			found=0
			IFS=':' read -r -a list <<< "$row"

			file=$(echo ${list[0]} | xargs)
			line=${list[1]}
			col=${list[2]}
			type=$(echo ${list[3]} | xargs)
			msg_=${list[4]}

			if [[ "$type" == "warning" ]]; then
				warn=1
			elif [[ "$type" == "error" ]]; then
				fail=1
			fi

			if [[ ! "$type" == "note" ]]; then
				found=1
				msg="::$type file=$file,line=$line,col=$col::$step_name: $msg_"
			fi

		else
			if [[ $found == "1" ]]; then
				msg=${msg}$_n${row}
			fi
		fi
	done) ; err=${PIPESTATUS[1]}

	if [[ $found == "1" ]]; then
		echo $msg >> $tmp_log_file
	fi

	if [[ $err -ne 0 ]]; then
		while read -r  line; do
			echo $line
		done <$tmp_log_file
		fail=1
	fi
	rm $tmp_log_file

	python3.11 scripts/clang-tools/gen_compile_commands.py

	_set_step_warn $warn
	return $fail
}

compile_kernel_sparse() {
	export step_name="kernel_sparse"
	local err=0
	local regex='^[[:alnum:]/._-]+:[[:digit:]]+:[[:digit:]]+: .*$'
	local fail=0
	local warn=0

	touch_files

	echo "$step_name (C=1)"

	yes n 2>/dev/null | \
		make -j$(nproc) C=1 $EXTRA_FLAGS 2>&1 | \
		(while IFS= read -r row; do
		if [[ "$row" =~ $regex ]]; then
			if [[ "$found" == "1" ]]; then
				echo $msg
				msg=
			fi

			found=0
			IFS=':' read -r -a list <<< "$row"

			file=$(echo ${list[0]} | xargs)
			line=${list[1]}
			col=${list[2]}
			type=$(echo ${list[3]} | xargs)
			msg_=${list[4]}

			if [[ "$type" == "warning" ]]; then
				warn=1
			elif [[ "$type" == "error" ]]; then
				fail=1
			fi

			if [[ "$type" == "note" ]]; then
				echo $row
			else
				found=1
				msg="::$type file=$file,line=$line,col=$col::$step_name: $msg_"
			fi

		else
			if [[ $found == "1" ]]; then
				msg=${msg}$_n${row}
			else
				echo $row
			fi
		fi
	done) ; err=${PIPESTATUS[1]}

	if [[ $found == "1" ]]; then
		echo $msg
	fi

	if [[ $err -ne 0 ]]; then
		fail=1
	fi

	_set_step_warn $warn
	return $fail
}

compile_kernel_smatch() {
	export step_name="kernel_smatch"
	local err=0
	local regex='^([[:alnum:]/._-]+):([[:digit:]]+) (.*) ([[:alpha:]]+): (.*)$'
	local fail=0
	local warn=0

	touch_files

	echo "$step_name (C=1)"

	if ! command -v smatch 2>&1 >/dev/null ; then
		if [[ ! -f /tmp/smatch/smatch ]]; then
			pushd /tmp
			rm -rf smatch
			git clone https://repo.or.cz/smatch.git smatch --depth=1 || (smatch_error=true ; true) && (
				cd smatch
				make -j$(nproc) || (smatch_error=true ; true)
			)
			popd
		fi

		alias smatch=/tmp/smatch/smatch
	fi

	if [[ "$smatch_error" == "true" ]]; then
		echo "::error ::$step_name: Fetching or compiling smatch failed, so the step was skipped."
		_set_step_warn '1'
		return 0;
	fi

	yes n 2>/dev/null | \
		make -j$(nproc) C=1 CHECK="smatch -p=kernel" $EXTRA_FLAGS | \
		(while IFS= read -r row; do
		if [[ "$row" =~ $regex ]]; then

			IFS=':' read -r -a list <<< "$row"

			file=${BASH_REMATCH[1]}
			line=${BASH_REMATCH[2]}
			type=${BASH_REMATCH[4]}
			msg_=${BASH_REMATCH[5]}

			if [[ "$type" == "warn" ]]; then
				type="warning"
				warn=1
			elif [[ "$type" == "error" ]]; then
				fail=1
			fi

			if [[ "$type" == "error" ]] || [[ "$type" == "warning" ]]; then
				echo "::$type file=$file,line=$line::$step_name: $msg_"
			else
				echo $row
			fi

		else
			if [[ $found == "1" ]]; then
				msg=${msg}$_n${row}
			else
				echo $row
			fi
		fi
	done) ; err=${PIPESTATUS[1]}

	if [[ $err -ne 0 ]]; then
		fail=1
	fi

	_set_step_warn $warn
	return $fail
}

compile_gcc_fanalyzer () {
	export step_name="gcc_fanalyzer"
	local exceptions_file="ci/travis/deadcode_exceptions"
	local files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha  -- '**/*.c')
	local regex='^[[:alnum:]/._-]+:[[:digit:]]+:[[:digit:]]+: .*$'
	local mail=
	local warn=0

	echo "recompile with gcc fanalyzer flag on range $base_sha..$head_sha"

	if [[ ! -f "compile_commands.json" ]]; then
		echo "::error ::$step_name: compile_commands.json does not exist."
		return 1
	fi

	if [[ -f $exceptions_file ]]; then
		files=$(comm -13 <(sort $exceptions_file) <(echo $files | tr ' ' '\n' | sort))
	fi
	[[ -z "$files" ]] && return 0
	while read file; do
		abs_file=$(realpath .)/$file
		compile_cmd=$(jq ".[] | select(.file == \"$abs_file\") |
			      .command" compile_commands.json |
			      sed 's/^"//;s/"$//g' |
			      sed 's/='\''\\"/=\\"/g;s/\\"'\''/\\"/g')
		if [[ -z "$compile_cmd" ]]; then
			echo "::error file=$file,line=0::$step_name: Failed to get compile command from compile_commands.json"
			warn=1
			continue
		fi

		echo -e "\e[1m$file\e[0m"
		compile_cmd=$(printf "$compile_cmd -fanalyzer")
		mail=$($compile_cmd 2>&1 || (
			echo "::error file=$file,line=0::$step_name: Exited with code '$?'" ; true)
		)
		found=0
		msg=

		while read -r row
		do
			if [[ "$row" =~ $regex ]]; then
				if [[ "$found" == "1" ]]; then
					echo $msg
					msg=
				fi

				found=0
				IFS=':' read -r -a list <<< "$row"

				file=$(echo ${list[0]} | xargs)
				line=${list[1]}
				col=${list[2]}
				type=$(echo ${list[3]} | xargs)
				msg_=${list[4]}

				if [[ "$type" == "note" ]]; then
					echo $row
				else
					warn=1
					found=1
					msg="::$type file=$file,line=$line,col=$col::gcc_fanalayzer: $msg_"
				fi

			else
				if [[ $found == "1" ]]; then
					msg=${msg}$_n${row}
				else
					echo $row
				fi
			fi

		done <<< "$mail"

		if [[ "$found" == "1" ]]; then
			echo $msg
		fi

	done <<< "$files"

	_set_step_warn $warn
	return 0
}

compile_clang_analyzer () {
	export step_name="clang_analyzer"
	local files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha -- '**/*.c')
	local regex='^[[:alnum:]/._-]+:[[:digit:]]+:[[:digit:]]+: .*$'
	local mail=
	local fail=0
	local warn=0

	echo "$step_name on range $base_sha..$head_sha"

	if [[ ! -f "compile_commands.json" ]]; then
		echo "::error ::$step_name: compile_commands.json does not exist."
		return 1
	fi
	[[ -z "$files" ]] && return 0
	while read file; do
		abs_file=$(realpath .)/$file
		compile_cmd=$(jq ".[] | select(.file == \"$abs_file\") |
			      .command" compile_commands.json |
			      sed 's/^"//;s/"$//g' |
			      sed 's/='\''\\"/=\\"/g;s/\\"'\''/\\"/g')
		if [[ -z "$compile_cmd" ]]; then
			echo "::error file=$file,line=0::$step_name: Failed to get compile command from compile_commands.json"
			fail=1
			continue
		fi

		echo -e "\e[1m$file\e[0m"
		compile_cmd=$(printf "$compile_cmd --analyze -Xanalyzer -analyzer-output=text")
		mail=$($compile_cmd 2>&1 || (
			echo "::error file=$file,line=0::$step_name: Exited with code '$?'" ; true)
		)
		found=0
		msg=

		while read -r row
		do
			if [[ "$row" =~ $regex ]]; then
				if [[ "$found" == "1" ]]; then
					echo $msg
					msg=
				fi

				found=0
				IFS=':' read -r -a list <<< "$row"

				file=$(echo ${list[0]} | xargs)
				line=${list[1]}
				col=${list[2]}
				type=$(echo ${list[3]} | xargs)
				msg_=${list[4]}

				if [[ "$type" == "note" ]]; then
					echo $row
				else
					if [[ "$type" == "error" ]]; then
						fail=1
					else
						warn=1
					fi
					found=1
					msg="::$type file=$file,line=$line,col=$col::clang_analyzer: $msg_"
				fi

			else
				if [[ $found == "1" ]]; then
					msg=${msg}$_n${row}
				else
					echo $row
				fi
			fi

		done <<< "$mail"

			if [[ "$found" == "1" ]]; then
				echo $msg
			fi

	done <<< "$files"

	_set_step_warn $warn
	return $fail
}

assert_compiled () {
	export step_name="assert_compiled"
	local exceptions_file="ci/travis/deadcode_exceptions"
	local files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha -- '**/*.c')
	local fail=0
	local error="
	 At least one file was not compiled during kernel compilation
	 Either:
	  1. ensure the Kconfig is able to enable it/them
	 OR
	  2. remove deadcode
	 OR
	  3. add it/them in file '$exceptions_file'"

	echo "$step_name were compiled on range $base_sha..$head_sha"

	if [[ ! -f "compile_commands.json" ]]; then
		echo "::error ::$step_name: compile_commands.json does not exist."
		return 1
	fi

	# Allows deadcode
	if [[ -f $exceptions_file ]]; then
		files=$(comm -13 <(sort $exceptions_file) <(echo $files | tr ' ' '\n' | sort))
	fi
	[[ -z "$files" ]] && return 0
	while read file; do
		echo -e "\e[1m$file\e[0m"
		abs_file=$(realpath .)/$file
		compile_cmd=$(jq ".[] | select(.file == \"$abs_file\") |
			      .command" compile_commands.json)
		if [[ -z "$compile_cmd" ]]; then
			echo "::error file=$file,line=0::$step_name: Was not compiled during kernel compilation."
			fail=1
		fi

	done <<< "$files"

	if [[ "$fail" == "true" ]]; then
		_fmt "::error ::$step_name: $error"
	fi

	return $fail
}

apply_prerun() {
	export step_name="apply_prerun"
	# Run cocci and bash scripts from ci/prerun
	# e.g. manipulate the source code depending on run conditons or target.
	local coccis=$(ls ci/prerun/*.cocci 2>/dev/null)
	local bashes=$(ls ci/prerun/*.sh 2>/dev/null)
	local files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha -- '**/*.c')

	echo "$step_name on range $base_sha..$head_sha"
	[[ -z "$files" ]] && return 0
	if [[ ! -z "$coccis" ]]; then
		while read cocci; do
			echo "apply $cocci"
			while read file; do
				echo -e "\e[1m$cocci $file\e[0m"
				spatch --sp-file $cocci --in-place $file
			done <<< "$files"
		done <<< "$coccis"
	fi

	if [[ ! -z "$bashes" ]]; then
		while read bashs; do
			echo "apply $bashs"
			while read file; do
				echo -e "\e[1m$bashs $file\e[0m"
				$bashs $file
			done <<< "$files"
		done <<< "$bashes"
	fi
}

touch_files () {
	local files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha)

	touch $files
}

auto_set_kconfig() {
	export step_name="auto_set_kconfig"
	local c_files=$(git diff --diff-filter=ACM --no-renames --name-only $base_sha..$head_sha -- '**/*.c')
	local k_blocks=$(git show --diff-filter=ACM --no-renames $base_sha..$head_sha -- '**/Kconfig' '**/Kconfig.*')
	declare -a o_files

	echo "$step_name on range $base_sha..$head_sha"

	while read file; do
		o_files+=("$(echo $file | sed 's/c$/o/')")
	done <<< "$c_files"

	local k_symbols=$(echo "$k_blocks" | awk -f ci/touched_kconfig.awk)
	symbols=$(python3.11 ci/symbols_depend.py ${k_symbols[@]} "${o_files[@]}")
	for sym in $symbols; do
		scripts/config -e $sym
	done
	make yes2modconfig

	# Some drivers have known issues as modules
	declare -a no_module_config=(
		"CONFIG_UIO"
		"CONFIG_FPGA"
		"CONFIG_FPGA_MGR_ZYNQ_FPGA"
	)
	for i in "${no_module_config[@]}"; do
		sed -i "s/^$i=m/$i=y/" .config
	done

	# We collect warnings and assert as the last ci step.
	scripts/config -d WERROR

	return 0
}

set_arch () {
	local version_gcc=14
	local version_llvm=19
	local arch_gcc=("gcc_arm" "gcc_aarch64" "gcc_x86")
	local arch_llvm=("llvm_x86")
	local arch=( "${arch_llvm[@]}" "${arch_gcc[@]}")
	local fail=false
	arch_="\<${1}\>"
	if [[ -z "$1" ]]; then
		printf "missing architecture"
		fail=true
	elif [[ "${arch[@]}" =~ $arch_ ]]; then
		unset CXX
		unset LLVM
		unset ARCH
		unset CROSS_COMPILE

		if [[ "${arch_gcc[@]}" =~ $arch_ ]]; then
			export CXX=gcc-$version_gcc
			case $1 in
				gcc_arm)
					export CROSS_COMPILE=arm-suse-linux-gnueabi-
					export ARCH=arm
					;;
				gcc_aarch64)
					export CROSS_COMPILE=aarch64-suse-linux-
					export ARCH=arm64
					;;
				gcc_x86)
					export ARCH=x86
					;;
			esac
			which ${CROSS_COMPILE}${CXX} 1>/dev/null
		elif [[ "${arch_llvm[@]}" =~ $arch_ ]]; then
			export LLVM=-$version_llvm
			case $1 in
				llvm_x86)
					export ARCH=x86
					;;
			esac
			which ${CROSS_COMPILE}clang${LLVM} 1>/dev/null
		fi
	else
		printf "unknown architecture '$1'"
		fail=true
	fi

	if [[ "$fail" == "true" ]]; then
		printf ", usage:\n"
		echo "  set_arch <arch>"
		echo "available architectures: "
		echo "  ${arch[@]}"
	else
		printenv | grep -i '^LLVM=\|^CXX=\|^ARCH=\|^CROSS_COMPILE='
	fi
}

set_step_warn () {
	echo ; echo "step_warn_$1=true" >> "$GITHUB_ENV"
}

set_step_fail () {
	echo ; echo "step_fail_$1=true" >> "$GITHUB_ENV"
}

_set_step_warn () {
	if [[ "$1" == "1" ]]; then
		set_step_warn "$step_name"
	fi
}

_set_step_fail () {
	if [[ ! -z "$step_name" ]]; then
		set_step_fail "$step_name"
		unset step_name
	fi
}

trap '_set_step_fail' ERR
