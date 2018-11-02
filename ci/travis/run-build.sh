#!/bin/bash
set -xe

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
		git fetch origin +refs/heads/${TRAVIS_BRANCH}:${TRAVIS_BRANCH}
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

BUILD_TYPE=${BUILD_TYPE:-default}

build_${BUILD_TYPE}
