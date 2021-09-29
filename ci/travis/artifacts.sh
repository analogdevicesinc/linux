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
	cd ${SOURCE_DIRECTORY}
	chmod 600 ${KEY_FILE}
	for bcm in $bcm_types; do
		scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
		    -i ${KEY_FILE} -r ${SOURCE_DIRECTORY}/adi_${bcm}_defconfig.tar ${DEST_SERVER}
	done
}

artifacts_to_artifactory() {
	curl -u$USERNAME:$PASSWORD -T ${SOURCE_DIRECTORY}/adi_bcm2709_defconfig.tar $ARTIFACTORY_PATH1
	curl -u$USERNAME:$PASSWORD -T ${SOURCE_DIRECTORY}/adi_bcm2711_defconfig.tar $ARTIFACTORY_PATH2
	curl -u$USERNAME:$PASSWORD -T ${SOURCE_DIRECTORY}/adi_bcmrpi_defconfig.tar $ARTIFACTORY_PATH3
}

artifacts_to_${1}
