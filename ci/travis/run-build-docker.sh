#!/bin/bash
set -e

. ./ci/travis/lib.sh

ENV_VARS="BUILD_TYPE DEFCONFIG ARCH CROSS_COMPILE DTS_FILES IMAGE"

if [ "$DO_NOT_DOCKERIZE" = "1" ] ; then
	. ./ci/travis/run-build.sh
else
	cat /dev/null > "${FULL_BUILD_DIR}/env"
	BUILD_TYPE=${BUILD_TYPE:-default}
	for env in $ENV_VARS ; do
		val="$(eval echo "\$${env}")"
		if [ -n "$val" ] ; then
			echo "export ${env}=${val}" >> "${FULL_BUILD_DIR}/env"
		fi
	done
	prepare_docker_image "ubuntu:rolling"
	run_docker_script run-build.sh "ubuntu:rolling"
fi
