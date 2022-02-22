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
	echo "git_sha=${GIT_SHA}" >> ${timestamp}/rpi_git_properties.txt
	echo "git_sha_date=${GIT_SHA_DATE}" >> ${timestamp}/rpi_git_properties.txt

	for bcm in $bcm_types; do
		cd adi_${bcm}_defconfig
		mkdir overlays
		mv ./*.dtbo ./overlays
		if [ "${bcm}" = "bcm2709" ]; then
			mv rpi_modules.tar.gz rpi_modules_kernel7.tar.gz
			mv ./zImage ./kernel7.img
		elif [ "${bcm}" = "bcm2711" ]; then
			mv rpi_modules.tar.gz rpi_modules_kernel7l.tar.gz
			mv ./zImage ./kernel7l.img
		elif [ "${bcm}" = "bcmrpi" ]; then
			mv rpi_modules.tar.gz rpi_modules_kernel.tar.gz
			mv ./zImage ./kernel.img
		fi
		cd ../
		cp -r ./adi_${bcm}_defconfig/* ./${timestamp}
	done
}

#upload artifacts to Artifactory
artifacts_artifactory() {
	cd ${SOURCE_DIRECTORY}
	python /root/myagent/_work/1/s/ci/travis/upload_to_artifactory.py --base_path="${ARTIFACTORY_PATH}" \
		--server_path="linux_rpi/${BUILD_SOURCEBRANCHNAME}" --local_path="./${timestamp}" --token="${ARTIFACTORY_TOKEN}"
}

#archive artifacts and upload to SWDownloads
artifacts_swdownloads() {
	cd ${SOURCE_DIRECTORY}/${timestamp}
	chmod 600 ${KEY_FILE}
	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
		-i ${KEY_FILE} -r *.tar.gz ${DEST_SERVER}
	md5_modules_kernel7=($(md5sum rpi_modules_kernel7.tar.gz| cut -d ' ' -f 1))
	md5_modules_kernel7l=($(md5sum rpi_modules_kernel7l.tar.gz| cut -d ' ' -f 1))
	md5_modules_kernel=($(md5sum rpi_modules_kernel.tar.gz| cut -d ' ' -f 1))

	rm rpi_modules_kernel7.tar.gz rpi_modules_kernel7l.tar.gz rpi_modules_kernel.tar.gz rpi_git_properties.txt
	tar -C ${PWD} -cvf rpi_latest_boot.tar.gz *
	md5_boot=($(md5sum rpi_latest_boot.tar.gz| cut -d ' ' -f 1))
	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
	    -i ${KEY_FILE} -r rpi_latest_boot.tar.gz ${DEST_SERVER}

	echo "boot_date=${timestamp}" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_modules_kernel7.tar.gz" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_modules_kernel7l.tar.gz" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_modules_kernel.tar.gz" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/latest_rpi_boot.tar.gz" >> rpi_archives_properties.txt
	echo "checksum_modules_kernel7=${md5_modules_kernel7}" >> rpi_archives_properties.txt
	echo "checksum_modules_kernel7l=${md5_modules_kernel7l}" >> rpi_archives_properties.txt
	echo "checksum_modules_kernel=${md5_modules_kernel}" >> rpi_archives_properties.txt
	echo "checksum_boot_files=${md5_boot}" >> rpi_archives_properties.txt

	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
                -i ${KEY_FILE} -r rpi_archives_properties.txt ${DEST_SERVER}
}

artifacts_${1}
