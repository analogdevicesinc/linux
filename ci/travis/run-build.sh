#!/bin/bash
set -xe

build_default() {
	if [ -n "$TRAVIS_BRANCH" ]; then
		git fetch origin +refs/heads/${TRAVIS_BRANCH}:${TRAVIS_BRANCH}
	fi

	COMMIT_RANGE=$([ "$TRAVIS_PULL_REQUEST" == "false" ] &&  echo HEAD || echo ${TRAVIS_BRANCH}..)

	make ${DEFCONFIG_NAME}
	make -j`getconf _NPROCESSORS_ONLN` $IMAGE UIMAGE_LOADADDR=0x8000

	for file in $DTS_FILES; do
		make ${DTS_PREFIX}`basename $file | sed  -e 's\dts\dtb\g'` || exit 1
	done

	scripts/checkpatch.pl --git ${COMMIT_RANGE} \
		--ignore FILE_PATH_CHANGES \
		--ignore LONG_LINE \
		--ignore LONG_LINE_STRING \
		--ignore LONG_LINE_COMMENT
}

build_compile_test() {
	make ${DEFCONFIG_NAME}
	make -j`getconf _NPROCESSORS_ONLN`
}

BUILD_TYPE=default

if [ "$COMPILE_TEST" == "y" ] ; then
	BUILD_TYPE=compile_test
fi

build_${BUILD_TYPE}
