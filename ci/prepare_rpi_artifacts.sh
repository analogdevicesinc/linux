#!/bin/bash -e
# SPDX-License-Identifier: GPL-2.0-only
#
# RPI Artifact Processor
# Prepares Raspberry Pi boot files and kernel modules as tar.gz archives
#
# Environment variables:
#   SOURCE_DIRECTORY: where the downloaded artifacts are located
#   OUTPUT_DIRECTORY: where to create the tar.gz archives (defaults to current directory)
#   BUILD_SOURCEBRANCHNAME: Git branch name for versioning
#

SOURCE_DIRECTORY="${SOURCE_DIRECTORY:-$(pwd)}"
OUTPUT_DIRECTORY="${OUTPUT_DIRECTORY:-$(pwd)}"
BUILD_SOURCEBRANCHNAME="${BUILD_SOURCEBRANCHNAME:?ERROR: BUILD_SOURCEBRANCHNAME must be set}"

timestamp=$(date +%Y_%m_%d-%H_%M)

# Extract git info from context.txt (replaces git commands for CI artifacts)
for ctx in "${SOURCE_DIRECTORY}"/adi_bcm*-gcc-arm*/context.txt; do
	if [[ -f "$ctx" ]]; then
		GIT_SHA=$(grep "^git_sha=" "$ctx" | cut -d'=' -f2)
		git_sha_at=$(grep "^git_sha_at=" "$ctx" | cut -d'=' -f2)
		GIT_SHA_DATE=$(date -d "@$git_sha_at" +"%Y-%m-%d-%H-%M" 2>/dev/null || echo "unknown")
		break
	fi
done

# Temporary working directory
WORK_DIR=$(mktemp -d)
trap "rm -rf ${WORK_DIR}" EXIT

#create version file found in the boot partition
create_version_file() {
	mkdir -p ${WORK_DIR}/${1}
	echo -e "RPI Boot Files: ${BUILD_SOURCEBRANCHNAME} ${GIT_SHA_DATE}\n" > ${WORK_DIR}/${1}/version_rpi.txt
	echo -e "  Linux repository: https://github.com/analogdevicesinc/linux" >> ${WORK_DIR}/${1}/version_rpi.txt
	echo -e "  Linux branch: ${BUILD_SOURCEBRANCHNAME}" >> ${WORK_DIR}/${1}/version_rpi.txt
	echo -e "  Linux git sha: ${GIT_SHA}\n" >> ${WORK_DIR}/${1}/version_rpi.txt
	echo -e "Supported RaspberryPi platforms:\n" >> ${WORK_DIR}/${1}/version_rpi.txt
	list=($2)
	for platform in "${list[@]}"; do
		echo "  ${platform}" >> ${WORK_DIR}/${1}/version_rpi.txt
	done
}

# extract and rename kernel image from boot directory
# Usage: extract_kernel_image <boot_dir> <output_file>
extract_kernel_image() {
	local boot_dir="$1"
	local output_file="$2"
	local kernel_name=$(basename "$output_file")

	if [[ -f "${boot_dir}/Image.gz" ]]; then
		echo "  Extracting Image.gz -> ${kernel_name}"
		gunzip -c "${boot_dir}/Image.gz" > "${output_file}"
	elif [[ -f "${boot_dir}/Image" ]]; then
		echo "  Copying Image -> ${kernel_name}"
		cp "${boot_dir}/Image" "${output_file}"
	elif [[ -f "${boot_dir}/zImage" ]]; then
		echo "  Copying zImage -> ${kernel_name}"
		cp "${boot_dir}/zImage" "${output_file}"
	elif [[ -f "${boot_dir}/uImage" ]]; then
		echo "  Copying uImage -> ${kernel_name}"
		cp "${boot_dir}/uImage" "${output_file}"
	else
		echo "ERROR: No kernel image found in ${boot_dir}"
		return 1
	fi
}

#prepare the structure of the folder containing artifacts
artifacts_structure() {
	# BCM defconfig names map to RPi kernel image names
	typeBCM_32bit=( "bcm2709" "bcm2711" "bcmrpi" )
	typeKERNEL_32bit=( "kernel7" "kernel7l" "kernel" )
	typeBCM_64bit=( "bcm2711" "bcm2712" )
	typeKERNEL_64bit=( "kernel8" "kernel_2712" )

	echo "=== Processing 32-bit artifacts ==="
	mkdir -p ${WORK_DIR}/32bit
	mkdir -p ${WORK_DIR}/32bit/modules
	create_version_file 32bit "${typeBCM_32bit[*]}"
	for index in "${!typeBCM_32bit[@]}"; do
		local artifact_dir="adi_${typeBCM_32bit[$index]}_defconfig-gcc-arm"
		echo "Processing ${artifact_dir}..."

		# Extract and rename kernel image
		extract_kernel_image \
			"${SOURCE_DIRECTORY}/${artifact_dir}/boot" \
			"${WORK_DIR}/32bit/${typeKERNEL_32bit[$index]}.img"

		# Copy modules
		cp -r "${SOURCE_DIRECTORY}/${artifact_dir}/lib/modules"/* "${WORK_DIR}/32bit/modules/"
	done

	# Copy DTBs and overlays from dtb-gcc artifact
	cp ${SOURCE_DIRECTORY}/dtb-gcc/dtb/arch/arm/boot/dts/broadcom/*.dtb ${WORK_DIR}/32bit/
	mkdir -p ${WORK_DIR}/32bit/overlays
	# RPi firmware expects overlays without the -overlay suffix
	for overlay in ${SOURCE_DIRECTORY}/dtb-gcc/dtb/arch/arm/boot/dts/overlays/*-overlay.dtbo; do
		base=$(basename "$overlay" -overlay.dtbo)
		cp "$overlay" ${WORK_DIR}/32bit/overlays/${base}.dtbo
	done

	if [ -z "$(ls ${WORK_DIR}/32bit/*.dtb 2>/dev/null)" ] || [ -z "$(ls ${WORK_DIR}/32bit/overlays/*.dtbo 2>/dev/null)" ]; then
		echo "Missing one or more required files from the 32bit artifacts."
		exit 1
	fi

	echo "Creating rpi_modules_32bit.tar.gz..."
	tar -C ${WORK_DIR}/32bit/modules -czvf ${OUTPUT_DIRECTORY}/rpi_modules_32bit.tar.gz . >/dev/null
	rm -r ${WORK_DIR}/32bit/modules

	echo ""
	echo "=== Processing 64-bit artifacts ==="
	mkdir -p ${WORK_DIR}/64bit
	mkdir -p ${WORK_DIR}/64bit/modules
	create_version_file 64bit "${typeBCM_64bit[*]}"
	for index in "${!typeBCM_64bit[@]}"; do
		local artifact_dir="adi_${typeBCM_64bit[$index]}_defconfig-gcc-arm64"
		echo "Processing ${artifact_dir}..."

		# Extract and rename kernel image
		extract_kernel_image \
			"${SOURCE_DIRECTORY}/${artifact_dir}/boot" \
			"${WORK_DIR}/64bit/${typeKERNEL_64bit[$index]}.img"

		# Copy modules
		cp -r "${SOURCE_DIRECTORY}/${artifact_dir}/lib/modules"/* "${WORK_DIR}/64bit/modules/"
	done

	# Copy DTBs and overlays from dtb-gcc artifact
	cp ${SOURCE_DIRECTORY}/dtb-gcc/dtb/arch/arm64/boot/dts/broadcom/*.dtb ${WORK_DIR}/64bit/
	mkdir -p ${WORK_DIR}/64bit/overlays
	for overlay in ${SOURCE_DIRECTORY}/dtb-gcc/dtb/arch/arm/boot/dts/overlays/*-overlay.dtbo; do
		base=$(basename "$overlay" -overlay.dtbo)
		cp "$overlay" ${WORK_DIR}/64bit/overlays/${base}.dtbo
	done

	if [ -z "$(ls ${WORK_DIR}/64bit/*.dtb 2>/dev/null)" ] || [ -z "$(ls ${WORK_DIR}/64bit/overlays/*.dtbo 2>/dev/null)" ]; then
		echo "Missing one or more required files from the 64bit artifacts."
		exit 1
	fi

	echo "Creating rpi_modules_64bit.tar.gz..."
	tar -C ${WORK_DIR}/64bit/modules -czvf ${OUTPUT_DIRECTORY}/rpi_modules_64bit.tar.gz . >/dev/null
	rm -r ${WORK_DIR}/64bit/modules

	echo ""
	echo "=== Artifacts structure created ==="
}

# Prepare one architecture's *-devel artifacts into the headers layout
create_headers_structure() {
	local karch="$1"; shift
	local bcms=( "$@" )

	local stage="${OUTPUT_DIRECTORY}/headers-${karch}"
	local root="${stage}/lib/modules"
	local base_version="" shared_name=""

	for bcm in "${bcms[@]}"; do
		local src="${SOURCE_DIRECTORY}/adi_${bcm}_defconfig-gcc-${karch}-devel"
		if [[ ! -d "$src" ]]; then
			echo "  WARN: missing ${src}, skipping ${bcm}"
			continue
		fi

		local release
		release=$(cat "${src}/include/config/kernel.release")
		if [[ -z "$base_version" ]]; then
			base_version=$(echo "$release" | grep -oE '^[0-9]+\.[0-9]+\.[0-9]+')
			shared_name="rpi-${karch}-shared-${base_version}"
		fi

		local shared_dir="${root}/${shared_name}"
		local build_dir="${root}/${release}/build"

		# Populate the shared area once (first present flavor wins).
		if [[ ! -d "$shared_dir" ]]; then
			mkdir -p "${shared_dir}/include" "${shared_dir}/arch"
			cp -r "${src}/arch/${karch}" "${shared_dir}/arch/"
			for sub in "${src}/include"/*/; do
				local name=$(basename "$sub")
				[[ "$name" == "generated" || "$name" == "config" ]] && continue
				cp -r "$sub" "${shared_dir}/include/"
			done
			[[ -f "${src}/Makefile" ]] && cp "${src}/Makefile" "${shared_dir}/"
		fi

		echo "  ${bcm} -> lib/modules/${release}/build"
		mkdir -p "${build_dir}/include" "${build_dir}/arch"

		# Real per-flavor files.
		cp -r "${src}/include/generated" "${build_dir}/include/"
		cp -r "${src}/include/config"    "${build_dir}/include/"
		cp -r "${src}/scripts"           "${build_dir}/"
		[[ -f "${src}/Module.symvers" ]] && cp "${src}/Module.symvers" "${build_dir}/"
		[[ -f "${src}/context.txt" ]]    && cp "${src}/context.txt"    "${build_dir}/"

		# Relative symlinks into the shared dir (depths per layout).
		ln -s "../../../${shared_name}/arch/${karch}" "${build_dir}/arch/${karch}"
		ln -s "../../${shared_name}/Makefile"         "${build_dir}/Makefile"
		for sub in "${shared_dir}/include"/*/; do
			local name=$(basename "$sub")
			ln -s "../../../${shared_name}/include/${name}" "${build_dir}/include/${name}"
		done
	done

	if [[ -z "$base_version" ]]; then
		echo "  No ${karch} devel artifacts found; skipping ${karch} headers archive."
		return 0
	fi

	echo "Creating rpi_headers_${karch}.tar.gz..."
	tar -C "${stage}" -czf "${OUTPUT_DIRECTORY}/rpi_headers_${karch}.tar.gz" . >/dev/null
	rm -r "${stage}"
}

# Arrange both architectures' headers into the symlinked layout.
headers_structure() {
	echo ""
	echo "=== Arranging kernel headers layout ==="
	create_headers_structure "arm"   "bcmrpi" "bcm2709" "bcm2711"
	create_headers_structure "arm64" "bcm2711" "bcm2712"
	echo "=== Headers layout ready at ${OUTPUT_DIRECTORY}/headers ==="
}

#create final boot archives and properties file
create_rpi_boot_archives() {
	artifacts_structure
	headers_structure

	echo ""
	echo "=== Creating boot archives ==="

	cd ${WORK_DIR}/32bit || exit 1
	tar -czvf ${OUTPUT_DIRECTORY}/rpi_latest_boot_32bit.tar.gz . >/dev/null
	md5_modules_32bit=$(md5sum ${OUTPUT_DIRECTORY}/rpi_modules_32bit.tar.gz | cut -d ' ' -f 1)
	md5_boot_32bit=$(md5sum ${OUTPUT_DIRECTORY}/rpi_latest_boot_32bit.tar.gz | cut -d ' ' -f 1)

	cd ${WORK_DIR}/64bit || exit 1
	tar -czvf ${OUTPUT_DIRECTORY}/rpi_latest_boot_64bit.tar.gz . >/dev/null
	md5_modules_64bit=$(md5sum ${OUTPUT_DIRECTORY}/rpi_modules_64bit.tar.gz | cut -d ' ' -f 1)
	md5_boot_64bit=$(md5sum ${OUTPUT_DIRECTORY}/rpi_latest_boot_64bit.tar.gz | cut -d ' ' -f 1)

	cat > ${OUTPUT_DIRECTORY}/rpi_archives_properties.txt << EOF
git_branch=${BUILD_SOURCEBRANCHNAME}
https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_modules_32bit.tar.gz
https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_latest_boot_32bit.tar.gz
checksum_modules_32bit=${md5_modules_32bit}
checksum_boot_files_32bit=${md5_boot_32bit}
https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_modules_64bit.tar.gz
https://swdownloads.analog.com/cse/linux_rpi/${BUILD_SOURCEBRANCHNAME}/rpi_latest_boot_64bit.tar.gz
checksum_modules_64bit=${md5_modules_64bit}
checksum_boot_files_64bit=${md5_boot_64bit}
git_sha=${GIT_SHA}
git_sha_date=${GIT_SHA_DATE}
EOF

	echo ""
	echo "=== Done ==="
	echo "Output: ${OUTPUT_DIRECTORY}/"
	ls -lh ${OUTPUT_DIRECTORY}/rpi_*.tar.gz ${OUTPUT_DIRECTORY}/rpi_archives_properties.txt
}

# Run if executed directly (not sourced)
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    artifacts_${1:-local}
fi
