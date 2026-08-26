#!/bin/bash
#
# Process downloaded artifacts: unpack and expand images
#
# This script processes artifacts downloaded to raw/ and outputs to dist/
# - Unpacks DTBs and kernel images
# - Expands microblaze/nios2 images by embedding DTBs
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib.sh"

prepare_kernel_dist() {
#   1. Unpacks DTBs from raw/ to dist/
#   2. Unpacks kernel images and modules
#   3. Expands microblaze images (embeds DTBs into simpleImage)
#   4. Expands nios2 images (embeds DTBs into zImage)
	log_step "Process artifacts"

	mkdir -p dist

	# Unpack all raw artifacts
	for dir in raw/*/; do
		[[ ! -d "$dir" ]] && continue
		name=$(basename "$dir")
		if [[ "$name" == "dtb-gcc" ]]; then
			_unpack_dtb "$name"
		elif [[ "$name" != *-headers ]]; then
			_unpack_kernel "$name"
		fi
	done

	# Expand images
	[ "$SKIP_EXPAND_MICROBLAZE" == "true" ] && log_info "Expand microblaze skipped" || _expand_microblaze
	[ "$SKIP_EXPAND_NIOS2" == "true" ] && log_info "Expand nios2 skipped" || _expand_nios2

	log_info "Wrote to dist/"
	ls -la dist/
}

# Run if executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
	prepare_kernel_dist "$@"
fi
