#!/bin/bash
set -e

GEN_GMSL_DIR=arch/arm64/boot/dts/gen_gmsl_dts
BASE_DTB=arch/arm64/boot/dts/xilinx/zynqmp-sm-k26-revA.dtb

# Arg is the number of cameras (default 1); selects the matching JSON config.
NUM_CAMERAS=${1:-1}
GMSL_JSON=${GEN_GMSL_DIR}/max96724_${NUM_CAMERAS}_max9295a_isx021_kria_adrd8012.json

if [ ! -f "${GMSL_JSON}" ]; then
	echo "error: no config for ${NUM_CAMERAS} camera(s): ${GMSL_JSON} not found" >&2
	echo "available:" >&2
	ls -1 ${GEN_GMSL_DIR}/max96724_*_max9295a_isx021_kria_adrd8012.json 2>/dev/null \
		| sed -E 's#.*max96724_([0-9]+)_.*#  \1 camera(s)#' >&2
	exit 1
fi

echo "Building ${NUM_CAMERAS}-camera config: ${GMSL_JSON}"

# Generate overlay from JSON config (FPGA + GMSL + RTP + mqnic nodes)
python3 ${GEN_GMSL_DIR}/gen_gmsl_dts.py --dtbo -o ${GEN_GMSL_DIR}/gmsl-adrd8012.dtso "${GMSL_JSON}"

# Compile overlay
dtc -@ -I dts -O dtb -o /tmp/gmsl-adrd8012.dtbo ${GEN_GMSL_DIR}/gmsl-adrd8012.dtso

# Apply overlay onto base DTB
fdtoverlay -o system.dtb -i "${BASE_DTB}" /tmp/gmsl-adrd8012.dtbo

echo "Built: system.dtb"
