#!/bin/bash -e
# Note: Not using -u because SYSTEM_PULLREQUEST_* vars are unset (not empty) on branch builds.
#       Not using -o pipefail because grep returns exit 1 on no match (used in dtb copy loop).
# SPDX-License-Identifier: GPL-2.0-only
#
# Required environment variables:
#   SOURCE_DIRECTORY      - Directory containing build artifacts (set in azure-pipelines.yml)
#   CLOUDSMITH_API_KEY    - Cloudsmith API token (secret, passed via env: in pipeline)
#
# Azure DevOps auto-provided variables:
#   BUILD_SOURCEVERSION      - Commit SHA for the build
#   BUILD_SOURCEBRANCH       - Full branch ref (e.g., refs/heads/main)
#   BUILD_SOURCEBRANCHNAME   - Short branch name
#
# PR-only variables (set only for pull request builds):
#   SYSTEM_PULLREQUEST_SOURCECOMMITID   - PR source commit SHA
#   SYSTEM_PULLREQUEST_TARGETBRANCH     - PR target branch
#   SYSTEM_PULLREQUEST_PULLREQUESTNUMBER - PR number

TIMESTAMP=$(date +%Y_%m_%d-%H_%M_%S)
# For PRs, Azure makes a merge commit (HEAD) because of the shallow copy option
# (which we want). So, GIT_SHA in this case is the actual correct commit inside
# the PR, and MERGE_COMMIT is the commit made by Azure (which we extract the date
# from)
echo "$SYSTEM_PULLREQUEST_TARGETBRANCH"
if [[ "$SYSTEM_PULLREQUEST_SOURCECOMMITID" != "" ]]; then
    GIT_SHA=$SYSTEM_PULLREQUEST_SOURCECOMMITID
else
    GIT_SHA=$BUILD_SOURCEVERSION
fi
MERGE_COMMIT_SHA=$(git rev-parse --short HEAD)
GIT_SHA_DATE=$(git show -s --format=%cd --date=format:'%Y-%m-%d %H:%M' ${MERGE_COMMIT_SHA} | sed -e "s/ \|\:/-/g")
CLOUDSMITH_REPO="sdg-linux"
VERSION_PATH="linux/"
if [ "$SYSTEM_PULLREQUEST_TARGETBRANCH" != "" ]; then
    BRANCH_NAME="$SYSTEM_PULLREQUEST_TARGETBRANCH"
    VERSION_PATH+="PRs/"
else
    BRANCH_NAME="$(echo $BUILD_SOURCEBRANCH | awk -F'/' '{print $NF}')"
fi

set_cloudsmith_version_path() {
    if [ "$SYSTEM_PULLREQUEST_TARGETBRANCH" != "" ]; then
        VERSION_PATH+="$BRANCH_NAME/pr_$SYSTEM_PULLREQUEST_PULLREQUESTNUMBER"
    else
        if [ "$BRANCH_NAME" == "main" ]; then
            VERSION_PATH+="main"
        else
            VERSION_PATH+="releases/$BRANCH_NAME"
        fi
    fi
}

create_extlinux() {
    local platform=$1

    touch extlinux.conf
    if [ "${platform}" == "arria10" ]; then
        dtb_name="socfpga_arria10_socdk_sdmmc.dtb"
    else
        dtb_name="socfpga.dtb"
    fi

    echo "LABEL Linux Default"                > extlinux.conf
    echo "    KERNEL ../zImage"              >> extlinux.conf
    echo "    FDT ../${dtb_name}"            >> extlinux.conf
    echo "    APPEND root=/dev/mmcblk0p2 rw rootwait earlyprintk console=ttyS0,115200n8" >> extlinux.conf
}

# Prepare the structure of the folder containing artifacts
artifacts_structure() {
    cd ${SOURCE_DIRECTORY}

    # Create folder to be uploaded
    mkdir ${TIMESTAMP}

    # Generate properties file
    echo "git_branch=${BUILD_SOURCEBRANCHNAME}" >> ${TIMESTAMP}/git_properties.txt
    echo "git_sha=${GIT_SHA}" >> ${TIMESTAMP}/git_properties.txt
    echo "git_sha_date=${GIT_SHA_DATE}" >> ${TIMESTAMP}/git_properties.txt

    declare -A typeARCH
    typeARCH=( ["arm"]="arria10 cyclone5 zynq"
               ["arm64"]="versal zynqmp"
               ["microblaze"]="kc705 kcu105 vc707 vcu118 vcu128" )

    declare -A image_to_copy
    image_to_copy=( ["arria10"]="socfpga_adi_defconfig/zImage"
                    ["cyclone5"]="socfpga_adi_defconfig/zImage"
                    ["zynq"]="zynq_xcomm_adv7511_defconfig/uImage"
                    ["versal"]="adi_versal_defconfig/Image"
                    ["zynqmp"]="adi_zynqmp_defconfig/Image" )

    for arch in "${!typeARCH[@]}"; do
        mkdir ${TIMESTAMP}/${arch}
        for platform in ${typeARCH[$arch]}; do
            # First copy kernels and make extlinux files.
            if [ "${arch}" != "microblaze" ]; then
                image_location="${TIMESTAMP}/${arch}/${platform}"
                mkdir ${image_location}
                if [ "${platform}" == "arria10" ] || [ "${platform}" == "cyclone5" ]; then
                    create_extlinux ${platform}
                    cp ./extlinux.conf ${image_location}
                fi
                echo "IMAGE: ${image_to_copy[${platform}]}!"
                cp ${image_to_copy[${platform}]} ${image_location}
            fi

            if [ "${arch}" == "microblaze" ]; then
                dtbs_to_copy=$(ls -d -1 Microblaze/*)
            else
                dtbs_to_copy=$(ls -d -1 DTBs/* | grep "${platform}[-|_]")
            fi

            # Copy DTBs to the correct location
            for dtb in ${dtbs_to_copy}; do
                cp ${dtb} "${TIMESTAMP}/${arch}"
            done
        done
    done
}

#### Start
artifacts_structure
set_cloudsmith_version_path
# This script is called from azure-pipelines.yml after setting the CLOUDSMITH_API_KEY secret variable
# so we can use it directly here
# Upload to Cloudsmith done using upload_to_cloudsmith.py script from wiki-scripts repo

echo "Uploading artifacts to Cloudsmith repo adi/${CLOUDSMITH_REPO} at path ${VERSION_PATH}/ ..."

python3 ../wiki-scripts/utils/cloudsmith_utils/upload_to_cloudsmith.py \
        --repo="${CLOUDSMITH_REPO}" \
        --version="${VERSION_PATH}" \
        --local_path="${TIMESTAMP}" \
        --tags="git_sha-${GIT_SHA};timestamp_${TIMESTAMP}" \
        --token="${CLOUDSMITH_API_KEY}" \
        --log_file="upload_to_cloudsmith.log"

# Set these Azure env vars to be used later in the pipeline
echo "##vso[task.setvariable variable=TIMESTAMP;isOutput=true]${TIMESTAMP}"
echo "##vso[task.setvariable variable=BRANCH;isOutput=true]${BRANCH_NAME}"
if [ "$SYSTEM_PULLREQUEST_PULLREQUESTNUMBER" != "" ]; then
    echo "##vso[task.setvariable variable=PR_ID;isOutput=true]$SYSTEM_PULLREQUEST_PULLREQUESTNUMBER"
else
    echo "##vso[task.setvariable variable=PR_ID;isOutput=true]commit"
fi
