#!/bin/bash -e
# SPDX-License-Identifier: (GPL-1.0-only OR BSD-2-Clause)

bcm_types='bcm2709 bcm2711 bcmrpi'

artifacts_to_archive() {
	cd ${SOURCE_DIRECTORY}
	for bcm in $bcm_types; do
		tar -zcvf adi_${bcm}_defconfig.tar adi_${bcm}_defconfig
	done
}

artifacts_to_swdownloads() {
	chmod 600 ${KEY_FILE}
	for bcm in $bcm_type; do
		scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
		    -i ${KEY_FILE} -r ${SOURCE_DIRECTORY}/adi_${bcm}_defconfig.tar ${DEST_SERVER}
	done
}

artifacts_to_artifactory() {
	local path=($ARTIFACTORY_PATH1 $ARTIFACTORY_PATH2 $ARTIFACTORY_PATH3)
	local index=1
	for bcm in $bcm_type; do
		curl -u$USERNAME:$PASSWORD -T ${SOURCE_DIRECTORY}/adi_${bcm}_defconfig.tar ${path[index]}
		((index++))
	done
}

artifacts_to_${1}
