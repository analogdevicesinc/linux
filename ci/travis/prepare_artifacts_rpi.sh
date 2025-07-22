# SPDX-License-Identifier: GPL-2.0-only
#!/bin/bash -e

timestamp=$(date +%Y_%m_%d-%H_%M)
GIT_SHA=$(git rev-parse --short HEAD)
GIT_SHA_DATE=$(git show -s --format=%cd --date=format:'%Y-%m-%d %H:%M' ${GIT_SHA} | sed -e "s/ \|\:/-/g")

#create version file found in the boot partition
create_version_file() {
	mkdir -p ${timestamp}/${1}
	echo -e "RPI Boot Files: ${BUILD_SOURCEBRANCH} ${GIT_SHA_DATE}\n" > ${SOURCE_DIRECTORY}/${timestamp}/${1}/version_rpi.txt
	echo -e "  Linux repository: https://github.com/analogdevicesinc/linux" >> ${SOURCE_DIRECTORY}/${timestamp}/${1}/version_rpi.txt
	echo -e "  Linux branch: ${BUILD_SOURCEBRANCHNAME}" >> ${SOURCE_DIRECTORY}/${timestamp}/${1}/version_rpi.txt
	echo -e "  Linux git sha: ${GIT_SHA}\n" >> ${SOURCE_DIRECTORY}/${timestamp}/${1}/version_rpi.txt
	echo -e "Supported RaspberryPi platforms:\n" >> ${SOURCE_DIRECTORY}/${timestamp}/${1}/version_rpi.txt
	list=($2)
	for platform in "${list[@]}"; do
		echo "  ${platform}" >> ${SOURCE_DIRECTORY}/${timestamp}/${1}/version_rpi.txt
	done
}

#prepare the structure of the folder containing artifacts
artifacts_structure() {
	cd ${SOURCE_DIRECTORY}
	mkdir ${timestamp}

	typeBCM_32bit=( "bcm2709" "bcm2711" "bcmrpi" )
	typeKERNEL_32bit=( "kernel7" "kernel7l" "kernel" )
	typeBCM_64bit=( "bcm2711" "bcm2712" )
  typeKERNEL_64bit=( "kernel8" "kernel_2712" )

	mkdir ${timestamp}/32bit
	create_version_file 32bit "${typeBCM_32bit[*]}"
	for index in "${!typeBCM_32bit[@]}"; do
		cd adi_"${typeBCM_32bit[$index]}"_arm_defconfig
		mkdir modules
		tar -xf rpi_modules.tar.gz -C modules
		rm rpi_modules.tar.gz
		mv ./zImage ./"${typeKERNEL_32bit[$index]}".img
		cd ../
		cp -r ./adi_"${typeBCM_32bit[$index]}"_arm_defconfig/* ./${timestamp}/32bit
	done
	if [ -z "$(ls  ${SOURCE_DIRECTORY}/${timestamp}/32bit/*.dtb 2>/dev/null)" ] || [ -z "$(ls  ${SOURCE_DIRECTORY}/${timestamp}/32bit/overlays/*.dtbo 2>/dev/null)" ]; then
		echo "Missing one or more required files from the 32bit artifacts."
		exit 1
	fi
	tar -C ${SOURCE_DIRECTORY}/${timestamp}/32bit/modules -czvf ${SOURCE_DIRECTORY}/${timestamp}/32bit/rpi_modules_32bit.tar.gz .
	rm -r ${SOURCE_DIRECTORY}/${timestamp}/32bit/modules

	mkdir ${timestamp}/64bit
	create_version_file 64bit "${typeBCM_64bit[*]}"
	for index in "${!typeBCM_64bit[@]}"; do
		cd adi_"${typeBCM_64bit[$index]}"_arm64_defconfig
		mkdir modules
		tar -xf rpi_modules.tar.gz -C modules
		rm rpi_modules.tar.gz
		mv ./Image ./"${typeKERNEL_64bit[$index]}".img
		cd ../
		cp -r ./adi_"${typeBCM_64bit[$index]}"_arm64_defconfig/* ./${timestamp}/64bit
	done
	if [ -z "$(ls  ${SOURCE_DIRECTORY}/${timestamp}/64bit/*.dtb 2>/dev/null)" ] || [ -z "$(ls  ${SOURCE_DIRECTORY}/${timestamp}/64bit/overlays/*.dtbo 2>/dev/null)" ]; then
		echo "Missing one or more required files from the 64bit artifacts."
		exit 1
	fi
	tar -C ${SOURCE_DIRECTORY}/${timestamp}/64bit/modules -czvf ${SOURCE_DIRECTORY}/${timestamp}/64bit/rpi_modules_64bit.tar.gz .
	rm -r ${SOURCE_DIRECTORY}/${timestamp}/64bit/modules
}

#upload artifacts to Artifactory
artifacts_artifactory() {
	artifacts_structure
	cd ${SOURCE_DIRECTORY}
	python ../ci/travis/upload_to_artifactory.py --base_path="${ARTIFACTORY_PATH}" \
		--server_path="linux_rpi/${BUILD_SOURCEBRANCHNAME}" --local_path="${timestamp}" \
		--token="${ARTIFACTORY_TOKEN}" --log_file="upload_to_artifactory.log" \
		--properties="git_branch=$BUILD_SOURCEBRANCHNAME;git_sha=$GIT_SHA;git_sha_date=$GIT_SHA_DATE;" \
		--props_level=3
}

#archive artifacts and upload to SWDownloads
artifacts_swdownloads() {
	artifacts_structure
	chmod 600 ${KEY_FILE}

	cd ${SOURCE_DIRECTORY}/${timestamp}/32bit || exit 1
	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
		-i ${KEY_FILE} -r *.tar.gz ${DEST_SERVER}/${BUILD_SOURCEBRANCHNAME}
	md5_modules_32bit=($(md5sum rpi_modules_32bit.tar.gz| cut -d ' ' -f 1))
	rm rpi_modules_32bit.tar.gz
	tar -C ${PWD} -czvf rpi_latest_boot_32bit.tar.gz *
	md5_boot_32bit=($(md5sum rpi_latest_boot_32bit.tar.gz| cut -d ' ' -f 1))
	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
	    -i ${KEY_FILE} -r rpi_latest_boot_32bit.tar.gz ${DEST_SERVER}/${BUILD_SOURCEBRANCHNAME}

	cd ${SOURCE_DIRECTORY}/${timestamp}/64bit || exit 1
	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
		-i ${KEY_FILE} -r *.tar.gz ${DEST_SERVER}/${BUILD_SOURCEBRANCHNAME}
	md5_modules_64bit=($(md5sum rpi_modules_64bit.tar.gz| cut -d ' ' -f 1))
	rm rpi_modules_64bit.tar.gz
	tar -C ${PWD} -czvf rpi_latest_boot_64bit.tar.gz *
	md5_boot_64bit=($(md5sum rpi_latest_boot_64bit.tar.gz| cut -d ' ' -f 1))
	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
	    -i ${KEY_FILE} -r rpi_latest_boot_64bit.tar.gz ${DEST_SERVER}/${BUILD_SOURCEBRANCHNAME}

	echo "git_branch=${BUILD_SOURCEBRANCHNAME}" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_modules_32bit.tar.gz" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_latest_boot_32bit.tar.gz" >> rpi_archives_properties.txt
	echo "checksum_modules_32bit=${md5_modules_32bit}" >> rpi_archives_properties.txt
	echo "checksum_boot_files_32bit=${md5_boot_32bit}" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_modules_64bit.tar.gz" >> rpi_archives_properties.txt
	echo "https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_latest_boot_64bit.tar.gz" >> rpi_archives_properties.txt
	echo "checksum_modules_64bit=${md5_modules_64bit}" >> rpi_archives_properties.txt
	echo "checksum_boot_files_64bit=${md5_boot_64bit}" >> rpi_archives_properties.txt
	echo "git_sha=${GIT_SHA}" >> rpi_archives_properties.txt
	echo "git_sha_date=${GIT_SHA_DATE}" >> rpi_archives_properties.txt

	scp -2 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o HostKeyAlgorithms=+ssh-dss \
                -i ${KEY_FILE} -r rpi_archives_properties.txt ${DEST_SERVER}/${BUILD_SOURCEBRANCHNAME}
}

artifacts_${1}
