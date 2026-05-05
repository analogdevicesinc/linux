#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Artifacts Structure and Upload Script
# Reorganizes CI Linux build artifacts in a structured format. The full path
# will be used for versioning the artefact.
#
# Environment variables (should be exported before sourcing this script):
#   SOURCE_DIRECTORY        - Directory containing build artifacts (default: pwd)
#   TIMESTAMP               - Timestamp for output directory (default: auto-generated)
#   BUILD_SOURCEBRANCHNAME  - Git branch name (default: main)
#   GIT_SHA                 - Git commit SHA (default: unknown)
#   GIT_SHA_DATE            - Git commit date in YYYY-MM-DD-HH-MM format (default: unknown)

set -e
shopt -s nullglob  # Prevent glob patterns from returning themselves when no match

# Configuration
SOURCE_DIRECTORY="${SOURCE_DIRECTORY:-$(pwd)}"
TIMESTAMP="${TIMESTAMP:-$(date +%Y_%m_%d-%H_%M_%S)}"
BUILD_SOURCEBRANCHNAME="${BUILD_SOURCEBRANCHNAME:-main}"
GIT_SHA="${GIT_SHA:-unknown}"
GIT_SHA_DATE="${GIT_SHA_DATE:-unknown}"

# Paths
RAW_DIR="${SOURCE_DIRECTORY}/raw"
DTB_BASE="${RAW_DIR}/dtb-gcc/dtb"
OUTPUT_DIR="${SOURCE_DIRECTORY}/${TIMESTAMP}"

# Architecture to platform mapping
declare -A typeARCH
typeARCH=(
    ["arm"]="arria10 cyclone5 zynq"
    ["arm64"]="versal zynqmp"
    ["microblaze"]="kc705 kcu105 vc707 vcu118 vcu128"
)

# Kernel image locations per platform
declare -A image_to_copy
image_to_copy=(
    ["arria10"]="socfpga_adi_defconfig-gcc-arm/boot/zImage"
    ["cyclone5"]="socfpga_adi_defconfig-gcc-arm/boot/zImage"
    ["zynq"]="zynq_xcomm_adv7511_defconfig-gcc-arm/boot/uImage"
    ["versal"]="adi_versal_defconfig-gcc-arm64/boot/Image"
    ["zynqmp"]="adi_zynqmp_defconfig-gcc-arm64/boot/Image"
)

# DTB source paths per architecture
declare -A dtb_paths
dtb_paths=(
    ["arm_xilinx"]="arch/arm/boot/dts/xilinx"
    ["arm_intel"]="arch/arm/boot/dts/intel/socfpga"
    ["arm64"]="arch/arm64/boot/dts/xilinx"
    ["microblaze"]="arch/microblaze/boot/dts"
)

# DTB patterns per platform
declare -A dtb_patterns
dtb_patterns=(
    ["zynq"]="zynq-"
    ["arria10"]="socfpga_arria10"
    ["cyclone5"]="socfpga_cyclone5"
    ["versal"]="versal-"
    ["zynqmp"]="zynqmp-"
)


#######################################
# Log message with timestamp
#######################################
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

#######################################
# Create directory structure
#######################################
create_structure() {
    log "Creating directory structure: ${OUTPUT_DIR}"

    mkdir -p "${OUTPUT_DIR}"

    for arch in "${!typeARCH[@]}"; do
        mkdir -p "${OUTPUT_DIR}/${arch}"

        # Create platform subdirectories for non-microblaze architectures
        if [[ "${arch}" != "microblaze" ]]; then
            for platform in ${typeARCH[$arch]}; do
                mkdir -p "${OUTPUT_DIR}/${arch}/${platform}"
            done
        fi
    done
}

#######################################
# Generate git properties file
#######################################
generate_properties() {
    log "Generating git_properties.txt"

    cat > "${OUTPUT_DIR}/git_properties.txt" << EOF
            git_branch=${BUILD_SOURCEBRANCHNAME}
            git_sha=${GIT_SHA}
            git_sha_date=${GIT_SHA_DATE}
EOF
}

#######################################
# Create extlinux.conf for a platform
#######################################
create_extlinux() {
    local platform=$1
    local output_dir=$2

    if [[ "${platform}" == "arria10" ]]; then
        dtb_name="socfpga_arria10_socdk_sdmmc.dtb"
    else
        dtb_name="socfpga.dtb"
    fi

    cat > "${output_dir}/extlinux.conf" <<EOF
            LABEL Linux Default
            KERNEL ../zImage
            FDT ../${dtb_name}
            APPEND root=/dev/mmcblk0p2 rw rootwait earlyprintk console=ttyS0,115200n8
EOF
}

#######################################
# Copy kernel images to platform directories
#######################################
copy_kernel_images() {
    log "Copying kernel images"

    for arch in "${!typeARCH[@]}"; do
        # Skip microblaze - no kernel images
        [[ "${arch}" == "microblaze" ]] && continue

        for platform in ${typeARCH[$arch]}; do
            local image_src="${RAW_DIR}/${image_to_copy[$platform]}"
            local image_dst="${OUTPUT_DIR}/${arch}/${platform}/"

            if [[ -f "${image_src}" ]]; then
                cp "${image_src}" "${image_dst}"
                log "  Copied: ${image_to_copy[$platform]} -> ${arch}/${platform}/"

                # Create extlinux.conf for arria10 and cyclone5
                if [[ "${platform}" == "arria10" || "${platform}" == "cyclone5" ]]; then
                    create_extlinux "${platform}" "${image_dst}"
                    log "  Created: extlinux.conf -> ${arch}/${platform}/"
                fi
            else
                log "  WARNING: Image not found: ${image_src}"
            fi
        done
    done
}

#######################################
# Copy DTBs for ARM architecture
#######################################
copy_arm_dtbs() {
    log "Copying ARM DTBs"

    local count=0

    # Copy zynq DTBs from xilinx directory
    for dtb in "${DTB_BASE}/${dtb_paths[arm_xilinx]}"/zynq-*.dtb; do
        if [[ -f "$dtb" ]]; then
            cp "$dtb" "${OUTPUT_DIR}/arm/"
            count=$((count + 1))
        fi
    done

    # Copy arria10 and cyclone5 DTBs from intel/socfpga directory
    for dtb in "${DTB_BASE}/${dtb_paths[arm_intel]}"/socfpga_arria10*.dtb; do
        if [[ -f "$dtb" ]]; then
            cp "$dtb" "${OUTPUT_DIR}/arm/"
            count=$((count + 1))
        fi
    done

    for dtb in "${DTB_BASE}/${dtb_paths[arm_intel]}"/socfpga_cyclone5*.dtb; do
        if [[ -f "$dtb" ]]; then
            cp "$dtb" "${OUTPUT_DIR}/arm/"
            count=$((count + 1))
        fi
    done

    log "  Copied ${count} ARM DTBs"
}

#######################################
# Copy DTBs for ARM64 architecture
#######################################
copy_arm64_dtbs() {
    log "Copying ARM64 DTBs"

    local count=0

    # Copy versal DTBs
    for dtb in "${DTB_BASE}/${dtb_paths[arm64]}"/versal-*.dtb; do
        if [[ -f "$dtb" ]]; then
            cp "$dtb" "${OUTPUT_DIR}/arm64/"
            count=$((count + 1))
        fi
    done

    # Copy zynqmp DTBs
    for dtb in "${DTB_BASE}/${dtb_paths[arm64]}"/zynqmp-*.dtb; do
        if [[ -f "$dtb" ]]; then
            cp "$dtb" "${OUTPUT_DIR}/arm64/"
            count=$((count + 1))
        fi
    done

    log "  Copied ${count} ARM64 DTBs"
}

#######################################
# Copy DTBs for Microblaze architecture
#######################################
copy_microblaze_dtbs() {
    log "Copying Microblaze DTBs"

    local count=0

    for dtb in "${DTB_BASE}/${dtb_paths[microblaze]}"/*.dtb; do
        if [[ -f "$dtb" ]]; then
            cp "$dtb" "${OUTPUT_DIR}/microblaze/"
            count=$((count + 1))
        fi
    done

    log "  Copied ${count} Microblaze DTBs"
}

#######################################
# Print summary
#######################################
print_summary() {
    echo ""
    echo "=========================================="
    echo "Artifacts Structure Complete"
    echo "=========================================="
    echo ""
    echo "Output directory: ${OUTPUT_DIR}"
    echo ""
    echo "Structure:"
    echo "  ${TIMESTAMP}/"
    echo "  ├── git_properties.txt"
    echo "  ├── arm/"
    echo "  │   ├── arria10/    (zImage, extlinux.conf)"
    echo "  │   ├── cyclone5/   (zImage, extlinux.conf)"
    echo "  │   ├── zynq/       (uImage)"
    echo "  │   └── *.dtb       ($(ls "${OUTPUT_DIR}/arm/"*.dtb 2>/dev/null | wc -l) files)"
    echo "  ├── arm64/"
    echo "  │   ├── versal/     (Image)"
    echo "  │   ├── zynqmp/     (Image)"
    echo "  │   └── *.dtb       ($(ls "${OUTPUT_DIR}/arm64/"*.dtb 2>/dev/null | wc -l) files)"
    echo "  └── microblaze/"
    echo "      └── *.dtb       ($(ls "${OUTPUT_DIR}/microblaze/"*.dtb 2>/dev/null | wc -l) files)"
    echo ""
}

#######################################
# Main function
#######################################
main() {

    log "Starting artifacts structure"
    log "Source: ${SOURCE_DIRECTORY}"
    log "Output: ${OUTPUT_DIR}"

    # Validate source directory
    if [[ ! -d "${RAW_DIR}" ]]; then
        echo "ERROR: Raw directory not found: ${RAW_DIR}"
        exit 1
    fi

    # Execute steps
    create_structure
    generate_properties
    copy_kernel_images
    copy_arm_dtbs
    copy_arm64_dtbs
    copy_microblaze_dtbs
    print_summary
    
    log "Done!"
}

# Run main function
main "$@"
