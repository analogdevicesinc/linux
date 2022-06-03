# SPDX-License-Identifier: (GPL-1.0-only OR BSD-2-Clause)
#!/bin/bash -e

timestamp=$(date +%Y_%m_%d-%H_%M)

#prepare the structure of the folder containing artifacts
artifacts_structure() {
	cd ${SOURCE_DIRECTORY}
	mkdir ${timestamp}

	GIT_SHA=$(git rev-parse --short HEAD)
	GIT_SHA_DATE=$(git show -s --format="%ci" ${GIT_SHA} | sed -e "s/ \|\:/-/g")
	echo "git_sha=${GIT_SHA}" >> ${timestamp}/rpi_git_properties.txt
	echo "git_sha_date=${GIT_SHA_DATE}" >> ${timestamp}/rpi_git_properties.txt

	typeBCM=( "bcm2709" "bcm2711" "bcmrpi" )
	typeKERNEL=( "kernel7" "kernel7l" "kernel" )
	for index in "${!typeBCM[@]}"; do
		cd adi_"${typeBCM[$index]}"_defconfig
		mkdir overlays modules
		mv ./*.dtbo ./overlays
		tar -xf rpi_modules.tar.gz -C modules
		rm rpi_modules.tar.gz
		mv ./zImage ./"${typeKERNEL[$index]}".img
		cd ../
		cp -r ./adi_"${typeBCM[$index]}"_defconfig/* ./${timestamp}
	done
	tar -C ${SOURCE_DIRECTORY}/${timestamp}/modules -czvf ${SOURCE_DIRECTORY}/${timestamp}/rpi_modules.tar.gz .
	rm -r ${SOURCE_DIRECTORY}/${timestamp}/modules
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
	md5_modules=($(md5sum rpi_modules.tar.gz| cut -d ' ' -f 1))

	rm rpi_modules.tar.gz rpi_git_properties.txt
	tar -C ${PWD} -czvf rpi_latest_boot.tar.gz *
	md5_boot=($(md5sum rpi_latest_boot.tar.gz| cut -d ' ' -f 1))
	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
	    -i ${KEY_FILE} -r rpi_latest_boot.tar.gz ${DEST_SERVER}

	echo "boot_date=${timestamp}" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_modules.tar.gz" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_latest_boot.tar.gz" >> rpi_archives_properties.txt
	echo "checksum_modules=${md5_modules}" >> rpi_archives_properties.txt
	echo "checksum_boot_files=${md5_boot}" >> rpi_archives_properties.txt

	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
                -i ${KEY_FILE} -r rpi_archives_properties.txt ${DEST_SERVER}
}

artifacts_${1}
