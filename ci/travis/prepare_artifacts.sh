#!/bin/bash -e
# SPDX-License-Identifier: (GPL-1.0-only OR BSD-2-Clause)

timestamp=$(date +%Y_%m_%d-%H_%M)
bcm_types='bcm2709 bcm2711 bcmrpi'

#prepare the structure of the folder containing artifacts
artifacts_structure() {
	cd ${SOURCE_DIRECTORY}
	mkdir ${timestamp}

	GIT_SHA=$(git rev-parse --short HEAD)
	GIT_SHA_DATE=$(git show -s --format="%ci" ${GIT_SHA} | sed -e "s/ \|\:/-/g")
	echo "git_sha=${GIT_SHA}" >> ${timestamp}/properties.txt
	echo "git_sha_date=${GIT_SHA_DATE}" >> ${timestamp}/properties.txt

	for bcm in $bcm_types; do
		cd adi_${bcm}_defconfig
		mkdir overlays
		mv ./*.dtbo ./overlays
		if [ "${bcm}" = "bcm2709" ]; then
			mv ./zImage ./kernel7.img
		elif [ "${bcm}" = "bcm2711" ]; then
			mv ./zImage ./kernel7l.img
		elif [ "${bcm}" = "bcmrpi" ]; then
			mv ./zImage ./kernel.img
		fi
		cd ../
		mv ./adi_${bcm}_defconfig ./${timestamp}/adi_${bcm}_defconfig
	done
}

#upload artifacts to Artifactory
artifacts_artifactory() {
	cd ${SOURCE_DIRECTORY}
	python /root/myagent/_work/1/s/ci/travis/upload_to_artifactory.py --base_path="${ARTIFACTORY_PATH}" --server_path="linux_rpi/${BUILD_SOURCEBRANCHNAME}" --local_path="./${timestamp}" --token="${ARTIFACTORY_TOKEN}"
}

#archive artifacts and upload to SWDownloads
artifacts_swdownloads() {
	cd ${SOURCE_DIRECTORY}/${timestamp}
	chmod 600 ${KEY_FILE}
	for bcm in $bcm_types; do
		tar -zcvf adi_${bcm}_defconfig.tar adi_${bcm}_defconfig
		scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
		    -i ${KEY_FILE} -r ${SOURCE_DIRECTORY}/${timestamp}/adi_${bcm}_defconfig.tar ${DEST_SERVER}
	done
}

artifacts_${1}
