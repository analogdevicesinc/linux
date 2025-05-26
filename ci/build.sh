check_checkpatch() {
	local mail=
	local fail=0
	local warn=0

	echo "checkpatch on range $base_sha..$head_sha"

	python3.11 -m venv ~/venv
        source ~/venv/bin/activate
        pip3.11 install ply GitPython --upgrade

	# The output is not properly captured with --git
	for commit in $(git rev-list $base_sha..$head_sha --reverse); do
		git --no-pager show --format="%h %s" "$commit" --name-only
		# Skip empty commits, assume cover letter
		# and those only touching non-upstream directories .github ci and docs
		local files=$(git diff --diff-filter=ACM --name-only $commit~..$commit | grep -v ^ci | grep -v ^.github | grep -v ^docs || true)
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
					msg=${msg}${row}%0A
					if [[ -z $row ]]; then
						if [[ -z $file ]]; then
							# If no file, add to file 0 of first file on list.
							file=$(git show --name-only --pretty=format: $commit | head -n 1)
							echo "::$type file=$file,line=0::checkpatch: $msg"
						else
							echo "::$type file=$file,line=$line::checkpatch: $msg"
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
				msg=$(echo "$row" | sed -E  's/^(CHECK|WARNING|ERROR): //')%0A
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

	if [[ "$fail" == "1" ]]; then
		set_step_fail "checkpatch"
	fi
	if [[ "$warn" == "1" ]]; then
		set_step_warn "checkpatch"
	fi

	return 0
}

check_dt_binding_check() {
	local files=$(git diff --name-only $base_sha..$head_sha)
	local fail=0

	echo "dt_binding_check on range $base_sha..$head_sha"

	python3.11 -m venv ~/venv
        source ~/venv/bin/activate
        pip3.11 install dtschema yamllint --upgrade

	while read file; do
		case "$file" in
		*.yaml)
			local relative_yaml=${file#Documentation/devicetree/bindings/}

			if [[ "$relative_yaml" = "$file" ]]; then
				echo "$file not a devicetree binding, skip check..."
			else
				echo "Testing devicetree binding $file"

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
					fail=1
				fi
				set -e

				echo "$error_txt"

				# file name appears in output if it contains errors
				if echo "$error_txt" | grep -qF "$file"; then
					fail=1
					echo "::error file=$file,line=0::dt_binding_check contain errors"
				fi
			fi
			;;
		esac
	done <<< "$files"

	if [[ "$fail" == "1" ]]; then
		set_step_fail "dtbinding"
	fi

	return 0
}

check_coccicheck() {
	local files=$(git diff --name-only $base_sha..$head_sha)
	local mail=
	local warn=0

	echo "coccicheck on range $base_sha..$head_sha"

	if [[ -z "$ARCH" ]]; then
		ARCH=x86
	fi

	coccis=$(ls scripts/coccinelle/**/*.cocci)
	if [[ -z "$coccis" ]]; then
		return 0
	fi

	while read file; do
		case "$file" in
		*.c)
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
						file_=$(echo ${list[0]} | xargs)
						line=${list[1]}
						msg=
						for ((i=2; i<${#list[@]}; i++)); do
							msg="$msg${list[$i]} "
						done
						echo "::$type file=$file,line=$line::coccicheck: $msg"
					else
						if [[ "$row" ]]; then
							echo $row
						fi
					fi

				done <<< "$mail"

			done <<< "$coccis"
			;;
		esac

	done <<< "$files"

	if [[ "$warn" == "1" ]]; then
		set_step_warn "coccicheck"
	fi

	return 0
}

check_cppcheck () {
	local files=$(git diff --name-only $base_sha..$head_sha)
	local regex='^[[:alnum:]/._-]+:[[:digit:]]+:[[:digit:]]+: .*$'
	local mail=
	local fail=0
	local warn=0

	echo "cppcheck on range $base_sha..$head_sha"

	while read file; do
		case "$file" in
		*.c)
			# --force checks all configurations, overrides cppcheck default 12 limit.
			echo -e "\e[1m$file\e[0m"
			mail=$(cppcheck --check-level=exhaustive -Iinclude --force $file 2>&1 || (
				echo "::error file=$file,line=0::cppcheck: Exited with code '$?'" ; true)
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

					if [[ "$type" == "warning" ]]; then
						warn=1
					fi
					if [[ "$type" == "error" ]]; then
						fail=1
					fi
					if [[ "$type" == "error" ]] || [[ "$type" == "warning" ]]; then
						found=1
						msg="::$type file=$file,line=$line,col=$col::cppcheck: $msg_"
					else
						echo $row
					fi

				else
					if [[ $found == "1" ]]; then
						msg=${msg}%0A${row}
					else
						echo $row
					fi
				fi

			done <<< "$mail"

			if [[ "$found" == "1" ]]; then
				echo $msg
			fi
			;;
		esac

	done <<< "$files"

	if [[ "$fail" == "1" ]]; then
		set_step_fail "cppcheck"
	fi
	if [[ "$warn" == "1" ]]; then
		set_step_warn "cppcheck"
	fi

	return
}

compile_devicetree() {
	local tmp_log_file=.ci_compile_devicetree_log
	local err=0
	local dtb_file=
	local dts_files=""
	local regex0='^([[:alnum:]/._-]+)(:([0-9]+)\.([0-9]+)-([0-9]+)\.([0-9]+))?: (Warning|Error) (.*)$'
	local regex1='^Error: ([[:alnum:]/._-]+)(:([0-9]+)\.([0-9]+)-([0-9]+))? (.+)$'

	echo "compile devicetree on range $base_sha..$head_sha"

	local dtsi_files=$(git diff --diff-filter=ACM --name-only $base_sha..$head_sha | grep ^arch/$ARCH/boot/dts/ | grep dtsi$ || true)
	if [[ ! -z "$dtsi_files" ]]; then
		echo "collecting dts files that include dtsi"
		while read file; do
			echo $file
			basename=$(basename $file)
			dirname=$(dirname $file)
			for file_ in $dirname/*.dts; do
				if cat $file_ | grep -q "#include \"$(basename $file)\""; then
					dts_files="$file_"$'\n'"$dts_files"
				fi
			done
		done <<< "$dtsi_files"
	fi
	if [[ ! -z "$dts_files" ]]; then
		dts_files=${dts_files::-1}
	fi

	dts_files+=$(git diff --diff-filter=ACM --name-only $base_sha..$head_sha | grep ^arch/$ARCH/boot/dts/ | grep dts$ || true)
	if [[ -z "$dts_files" ]]; then
		echo "no dts on range, skipped"
		return $err
	fi

	echo "compiling devicetrees"
	echo > $tmp_log_file
	while read file; do
		dtb_file=$(echo $file | sed 's/dts\//=/g' | cut -d'=' -f2 | sed 's\dts\dtb\g')
		make -i $dtb_file 2>&1 | \
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
	done <<< "$dts_files"

	uniq $tmp_log_file
	rm $tmp_log_file

	if [[ "$fail" == "1" ]]; then
		set_step_fail "compile_devicetree"
	fi

	if [[ "$warn" == "1" ]]; then
		set_step_warn "compile_devicetree"
	fi

	return $err
}

compile_kernel() {
	local tmp_log_file=.ci_compile_kernel_log
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

	echo "compile kernel"

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
				msg="::$type file=$file,line=$line,col=$col::$msg_"
			fi

		else
			if [[ $found == "1" ]]; then
				msg=${msg}%0A${row}
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

	if [[ "$fail" == "1" ]]; then
		set_step_fail "kernel"
	fi

	if [[ "$warn" == "1" ]]; then
		set_step_warn "kernel"
	fi

	python3.11 scripts/clang-tools/gen_compile_commands.py

	return $err
}

compile_kernel_sparse() {
	local err=0
	local regex='^[[:alnum:]/._-]+:[[:digit:]]+:[[:digit:]]+: .*$'
	local fail=0
	local warn=0

	touch_files

	echo "compile kernel with sparce (C=1)"

	yes n 2>/dev/null | \
		make -j$(nproc) C=1 $EXTRA_FLAGS | \
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
				msg="::$type file=$file,line=$line,col=$col::$msg_"
			fi

		else
			if [[ $found == "1" ]]; then
				msg=${msg}%0A${row}
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

	if [[ "$fail" == "1" ]]; then
		set_step_fail "kernel_sparce"
	fi
	if [[ "$warn" == "1" ]]; then
		set_step_warn "kernel_sparce"
	fi

	return $err
}

compile_kernel_smatch() {
	local err=0
	local regex='^([[:alnum:]/._-]+):([[:digit:]]+) (.*) ([[:alpha:]]+): (.*)$'
	local fail=0
	local warn=0

	touch_files

	echo "compile kernel with smatch (C=1)"

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
		echo "::error ::smatch: Fetching or compiling smatch failed, so the step was skipped."
		set_step_warn "kernel_smatch_get_sources"
		return $err;
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
				echo "::$type file=$file,line=$line::smatch: $msg_"
			else
				echo $row
			fi

		else
			if [[ $found == "1" ]]; then
				msg=${msg}%0A${row}
			else
				echo $row
			fi
		fi
	done) ; err=${PIPESTATUS[1]}

	if [[ $err -ne 0 ]]; then
		fail=1
	fi

	if [[ "$fail" == "1" ]]; then
		set_step_fail "kernel_smatch"
	fi
	if [[ "$warn" == "1" ]]; then
		set_step_warn "kernel_smatch"
	fi

	return $err
}

compile_gcc_fanalyzer () {
	local files=$(git diff --diff-filter=ACM --name-only $base_sha..$head_sha)
	local regex='^[[:alnum:]/._-]+:[[:digit:]]+:[[:digit:]]+: .*$'
	local mail=
	local fail=0

	echo "recompile with gcc fanalyzer flag on range $base_sha..$head_sha"

	if [[ ! -f "compile_commands.json" ]]; then
		echo "::error ::gcc_fanalayzer: compile_commands.json does not exist! Was scripts/clang-tools/gen_compile_commands.py called?"
		set_step_fail "gcc_fanalyzer_no_compile_commands"
		return 0
	fi

	while read file; do
		case "$file" in
		*.c)
			abs_file=$(realpath .)/$file
			compile_cmd=$(jq ".[] | select(.file == \"$abs_file\") |
				      .command" compile_commands.json |
				      sed 's/^"//;s/"$//g' |
				      sed 's/='\''\\"/=\\"/g;s/\\"'\''/\\"/g')
			if [[ -z "$compile_cmd" ]]; then
				echo "::error file=$file,line=0::gcc_fanalayzer: Failed to get compile command from compile_commands.json"
				fail=1
				continue
			fi

			echo -e "\e[1m$file\e[0m"
			compile_cmd=$(printf "$compile_cmd -fanalyzer")
			mail=$($compile_cmd 2>&1 || (
				echo "::error file=$file,line=0::gcc_fanalayzer: Exited with code '$?'" ; true)
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
						fail=1
						found=1
						msg="::$type file=$file,line=$line,col=$col::gcc_fanalayzer: $msg_"
					fi

				else
					if [[ $found == "1" ]]; then
						msg=${msg}%0A${row}
					else
						echo $row
					fi
				fi

			done <<< "$mail"

			if [[ "$found" == "1" ]]; then
				echo $msg
			fi

			;;
		esac

	done <<< "$files"

	if [[ "$fail" == "1" ]]; then
		set_step_fail "gcc_fanalyzer"
	fi

	return
}

compile_clang_analyzer () {
	local files=$(git diff --diff-filter=ACM --name-only $base_sha..$head_sha)
	local regex='^[[:alnum:]/._-]+:[[:digit:]]+:[[:digit:]]+: .*$'
	local mail=
	local fail=0
	local warn=0

	echo "recompile with clang analyzer flag on range $base_sha..$head_sha"

	if [[ ! -f "compile_commands.json" ]]; then
		echo "::error ::clang_analyzer: compile_commands.json does not exist! Was scripts/clang-tools/gen_compile_commands.py called?"
		set_step_fail "clang_analyzer_no_compile_commands"
		return 0
	fi

	while read file; do
		case "$file" in
		*.c)
			abs_file=$(realpath .)/$file
			compile_cmd=$(jq ".[] | select(.file == \"$abs_file\") |
				      .command" compile_commands.json |
				      sed 's/^"//;s/"$//g' |
				      sed 's/='\''\\"/=\\"/g;s/\\"'\''/\\"/g')
			if [[ -z "$compile_cmd" ]]; then
				echo "::error file=$file,line=0::clang_analyzer: Failed to get compile command from compile_commands.json"
				fail=1
				continue
			fi

			echo -e "\e[1m$file\e[0m"
			compile_cmd=$(printf "$compile_cmd --analyze -Xanalyzer -analyzer-output=text")
			mail=$($compile_cmd 2>&1 || (
				echo "::error file=$file,line=0::clang_analyzer: Exited with code '$?'" ; true)
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
						msg=${msg}%0A${row}
					else
						echo $row
					fi
				fi

			done <<< "$mail"

			if [[ "$found" == "1" ]]; then
				echo $msg
			fi

			;;
		esac

	done <<< "$files"

	if [[ "$warn" == "1" ]]; then
		set_step_warn "clang_analyzer"
	fi
	if [[ "$fail" == "1" ]]; then
		set_step_fail "clang_analyzer"
	fi

	return
}

assert_compiled () {
	local files=$(git diff --diff-filter=ACM --name-only $base_sha..$head_sha)
	local fail=0

	echo "assert sources were compiled on range $base_sha..$head_sha"

	if [[ ! -f "compile_commands.json" ]]; then
		echo "::error ::assert_compiled: compile_commands.json does not exist! Was scripts/clang-tools/gen_compile_commands.py called?"
		set_step_fail "assert_compiled_no_compile_commands"
		return 0
	fi

	while read file; do
		case "$file" in
		*.c)
			echo -e "\e[1m$file\e[0m"
			abs_file=$(realpath .)/$file
			compile_cmd=$(jq ".[] | select(.file == \"$abs_file\") |
				      .command" compile_commands.json)
			if [[ -z "$compile_cmd" ]]; then
				echo "::error file=$file,line=0::assert_compiled: Was not compiled during kernel compilation, ensure defconfig enables it"
				fail=1
			fi
			;;
		esac

	done <<< "$files"

	if [[ "$fail" == "1" ]]; then
		set_step_fail "assert_compiled"
	fi

	return 0
}

apply_prerun() {
	# Run cocci and bash scripts from ci/prerun
	# e.g. manipulate the source code depending on run conditons or target.
	local coccis=$(ls ci/prerun/*.cocci 2>/dev/null)
	local bashes=$(ls ci/prerun/*.sh 2>/dev/null)
	local files=$(git diff --diff-filter=ACM --name-only $base_sha..$head_sha)

	echo "apply_prerun on range $base_sha..$head_sha"

	if [[ ! -z "$coccis" ]]; then
		while read cocci; do
			echo "apply $cocci"
			while read file; do
				case "$file" in
				*.c)
					echo -e "\e[1m$cocci $file\e[0m"
					spatch --sp-file $cocci --in-place $file
				esac
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
	local files=$(git diff --diff-filter=ACM --name-only $base_sha..$head_sha)

	touch $files
}

auto_set_kconfig() {
	local files=$(git diff --diff-filter=ACM --name-only $base_sha..$head_sha)
	declare -a o_files

	echo "get_kconfig on range $base_sha..$head_sha"

	while read file; do
		case "$file" in
		*.c)
			o_files+=("$(echo $file | sed 's/c$/o/')")
			;;
		esac
	done <<< "$files"

	symbols=$(python3.11 ci/symbols_depend.py "${o_files[@]}")
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

	return 0
}

set_step_warn () {
	echo ; echo "step_warn_$1=true" >> "$GITHUB_ENV"
}

set_step_fail () {
	echo ; echo "step_fail_$1=true" >> "$GITHUB_ENV"
}
