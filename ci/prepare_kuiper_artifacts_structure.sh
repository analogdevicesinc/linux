#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
# Artifacts Structure Preparer
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
TIMESTAMP="${TIMESTAMP:-$(date +%Y_%m_%d-%H_%M)}"
BUILD_SOURCEBRANCHNAME="${BUILD_SOURCEBRANCHNAME:?ERROR: BUILD_SOURCEBRANCHNAME must be set}"
GIT_SHA="${GIT_SHA:?ERROR: GIT_SHA must be set}"
GIT_SHA_DATE="${GIT_SHA_DATE:-}"

# Paths
DIST_DIR="${SOURCE_DIRECTORY}/dist"
OUTPUT_DIR="${SOURCE_DIRECTORY}/${TIMESTAMP}"

# Architecture to platform mapping (used for directory structure and DTB filtering)
declare -A typeARCH
typeARCH=(
    ["arm"]="arria10 cyclone5 zynq"
    ["arm64"]="versal zynqmp"
    ["microblaze"]="kc705 kcu105 vc707 vcu118 vcu128"
)

# Kernel image locations per platform (paths within dist/)
declare -A image_to_copy
image_to_copy=(
    ["arria10"]="arm/boot/kernel/socfpga_adi_defconfig/zImage"
    ["cyclone5"]="arm/boot/kernel/socfpga_adi_defconfig/zImage"
    ["zynq"]="arm/boot/kernel/zynq_xcomm_adv7511_defconfig/uImage"
    ["versal"]="arm64/boot/kernel/adi_versal_defconfig/Image"
    ["zynqmp"]="arm64/boot/kernel/adi_zynqmp_defconfig/Image"
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
generate_git_properties() {
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
# Copy kernel images and DTBs for all architectures
#######################################
copy_artifacts() {
    log "Copying kernel images and DTBs"

    # Flatten all DTBs into a single directory for simple grep-based filtering
    local dtb_flat="${DIST_DIR}/DTBs"
    mkdir -p "${dtb_flat}"
    find "${DIST_DIR}" -name "*.dtb" -exec cp {} "${dtb_flat}/" \;
    # Microblaze DTBs are embedded into kernel images, not unpacked to dist/
    find "${SOURCE_DIRECTORY}" -path "*/arch/microblaze/boot/dts/*" -name "*.dtb" -exec cp {} "${dtb_flat}/" \; 2>/dev/null || true
    # Microblaze .strip files (kernel images with debug symbols stripped)
    find "${DIST_DIR}/microblaze/boot/kernel/adi_mb_defconfig" -name "*.strip" -exec cp {} "${dtb_flat}/" \; 2>/dev/null || true

    for arch in "${!typeARCH[@]}"; do
        local arch_dtb_count=0

        for platform in ${typeARCH[$arch]}; do
            # Copy kernel images (skip microblaze - no kernel images)
            if [[ "${arch}" != "microblaze" ]]; then
                local image_src="${DIST_DIR}/${image_to_copy[$platform]}"
                local image_dst="${OUTPUT_DIR}/${arch}/${platform}/"

                if [[ -f "${image_src}" ]]; then
                    cp "${image_src}" "${image_dst}"
                    log "  Copied: ${image_to_copy[$platform]} -> ${arch}/${platform}/"

                    # Create extlinux.conf for Intel platforms
                    if [[ "${platform}" == "arria10" || "${platform}" == "cyclone5" ]]; then
                        create_extlinux "${platform}" "${image_dst}"
                    fi
                else
                    log "  WARNING: Image not found: ${image_src}"
                fi
            fi

            # Match DTBs by platform prefix (e.g., zynq_*, socfpga_arria10_*, vcu118_*)
            local dtbs_to_copy
            if [[ "${arch}" == "microblaze" ]]; then
                dtbs_to_copy=$(ls "${dtb_flat}"/*.dtb "${dtb_flat}"/*.strip 2>/dev/null | xargs -n1 basename | grep -E "(kc705|kcu105|vc707|vcu118|vcu128)" || true)
            else
                dtbs_to_copy=$(ls "${dtb_flat}"/*.dtb 2>/dev/null | xargs -n1 basename | grep -E "^${platform}[-_]|^socfpga_${platform}" || true)
            fi

            for dtb in ${dtbs_to_copy}; do
                cp "${dtb_flat}/${dtb}" "${OUTPUT_DIR}/${arch}/"
                ((++arch_dtb_count))
            done
        done

        log "  Copied ${arch_dtb_count} ${arch} DTBs"
    done
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
    tree -I '*.dtb' --dirsfirst --noreport "${OUTPUT_DIR}"
    echo ""
    printf "DTBs: arm=%d, arm64=%d, microblaze=%d\n" \
        "$(find "${OUTPUT_DIR}/arm" -maxdepth 1 -name '*.dtb' | wc -l)" \
        "$(find "${OUTPUT_DIR}/arm64" -maxdepth 1 -name '*.dtb' | wc -l)" \
        "$(find "${OUTPUT_DIR}/microblaze" -maxdepth 1 -name '*.dtb' | wc -l)"
    echo ""
}

#######################################
# Prepare one architecture's *-devel artifacts into the kernel-headers layout.
# The arch/, include/ (minus generated+config) and Makefile are byte-identical
# across same-arch defconfigs, so they are stored once in a shared dir; each
# defconfig's build/ holds its real per-config files and symlinks the rest.
# build/ is keyed by defconfig (not kernel.release) because carrier defconfigs
# set no CONFIG_LOCALVERSION and thus share one release string.
# Usage: create_carrier_headers <arch> <defconfig>...
#######################################
create_carrier_headers() {
    local karch="$1"; shift
    local defconfigs=( "$@" )

    local stage="${OUTPUT_DIR}/headers-${karch}"
    local root="${stage}/lib/modules"
    local base_version="" shared_name=""

    for dc in "${defconfigs[@]}"; do
        local src="${SOURCE_DIRECTORY}/raw/${dc}-gcc-${karch}-devel"
        if [[ ! -d "$src" ]]; then
            log "  WARN: missing ${src}, skipping ${dc}"
            continue
        fi
        if [[ ! -f "${src}/include/config/kernel.release" ]]; then
            log "  WARN: ${src} has no include/config/kernel.release, skipping ${dc}"
            continue
        fi

        local release
        release=$(cat "${src}/include/config/kernel.release")
        if [[ -z "$base_version" ]]; then
            base_version=$(echo "$release" | grep -oE '^[0-9]+\.[0-9]+\.[0-9]+')
            shared_name="${karch}-shared-${base_version}"
        fi

        local shared_dir="${root}/${shared_name}"
        local build_dir="${root}/${dc}/build"

        # Populate the shared area once (first present defconfig wins).
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

        log "  ${dc} -> lib/modules/${dc}/build"
        mkdir -p "${build_dir}/include" "${build_dir}/arch"

        # Real per-defconfig files.
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
        log "  No ${karch} devel artifacts found; skipping ${karch} headers archive."
        return 0
    fi

    mkdir -p "${OUTPUT_DIR}/headers"
    log "Creating carrier_headers_${karch}.tar.gz..."
    tar -C "${stage}" -czf "${OUTPUT_DIR}/headers/carrier_headers_${karch}.tar.gz" . >/dev/null
    rm -r "${stage}"
}

#######################################
# Arrange arm and arm64 carrier headers into the symlinked layout.
#######################################
headers_structure() {
    echo ""
    log "Arranging carrier kernel headers layout"
    create_carrier_headers "arm"   zynq_xcomm_adv7511_defconfig socfpga_adi_defconfig zynq_pluto_defconfig zynq_m2k_defconfig
    create_carrier_headers "arm64" adi_zynqmp_defconfig adi_versal_defconfig
    log "Carrier headers ready at ${OUTPUT_DIR}/headers"
}

#######################################
# Main function
#######################################
main() {

    log "Starting artifacts structure"
    log "Source: ${SOURCE_DIRECTORY}"
    log "Output: ${OUTPUT_DIR}"

    # Validate source directory
    if [[ ! -d "${DIST_DIR}" ]]; then
        echo "ERROR: Dist directory not found: ${DIST_DIR}"
        exit 1
    fi

    # Execute steps
    create_structure
    generate_git_properties
    copy_artifacts
    headers_structure
    print_summary
    
    log "Done!"
}

# Run main function
main "$@"
