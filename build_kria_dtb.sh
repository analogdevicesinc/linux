#!/bin/bash
set -e

GEN_GMSL_DIR=arch/arm64/boot/dts/gen_gmsl_dts
BASE_DTB=arch/arm64/boot/dts/xilinx/zynqmp-sm-k26-revA-sck-kv-g-revB.dtb
GMSL_JSON=${1:-${GEN_GMSL_DIR}/max96724_1_max9295a_isx021_kria_kv260.json}
# GMSL_JSON=${1:-${GEN_GMSL_DIR}/max96724_2_max9295a_isx021_kria_kv260.json}

# Generate overlay from JSON config (includes both FPGA and GMSL nodes)
python3 ${GEN_GMSL_DIR}/gen_gmsl_dts.py --dtbo -o ${GEN_GMSL_DIR}/gmsl-kria.dtso "${GMSL_JSON}"

# Compile overlay
dtc -@ -I dts -O dtb -o /tmp/gmsl-kria.dtbo ${GEN_GMSL_DIR}/gmsl-kria.dtso

# Apply overlay onto base DTB
fdtoverlay -o system.dtb -i "${BASE_DTB}" /tmp/gmsl-kria.dtbo

echo "Built: system.dtb"
