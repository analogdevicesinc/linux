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

_read_be32() {
	local bytes=$(dd if="$1" bs=1 skip=$2 count=4 2>/dev/null | xxd -p)
	echo $((16#$bytes))
}

_read_le32() {
	local bytes=$(dd if="$1" bs=1 skip=$2 count=4 2>/dev/null | xxd -p)
	echo $((16#${bytes:6:2}${bytes:4:2}${bytes:2:2}${bytes:0:2}))
}

_write_le32() {
	local value=$1
	printf '\x%02x\x%02x\x%02x\x%02x' 2>/dev/null \
		$((value & 0xff)) \
		$(((value >> 8) & 0xff)) \
		$(((value >> 16) & 0xff)) \
		$(((value >> 24) & 0xff))
}

_find_empty_dtb() {
	# Finds dtb slot in a Image
	# Assumes you built the Image with an 'empty' dtb (/dts-v1/; / { };)
	# Returns: dtb_start dtb_size dtb_boundary dtb_available
	local image=$1

	python3 -c "
import struct
import sys

# 'Empty' DTB struct block: FDT_BEGIN_NODE, '', FDT_END_NODE, FDT_END
EMPTY_STRUCT = bytes.fromhex('00000001000000000000000200000009')
DTB_MAGIC = 0xd00dfeed
OFF_DT_STRUCT = 56

with open('$image', 'rb') as f:
    data = f.read()

pos = 0
while True:
    pos = data.find(EMPTY_STRUCT, pos)
    if pos == -1:
        break

    dtb_start = pos - OFF_DT_STRUCT
    if dtb_start < 0:
        pos += 1
        continue

    magic = struct.unpack('>I', data[dtb_start:dtb_start+4])[0]
    if magic != DTB_MAGIC:
        pos += 1
        continue

    off_dt_struct = struct.unpack('>I', data[dtb_start+8:dtb_start+12])[0]
    if off_dt_struct != OFF_DT_STRUCT:
        pos += 1
        continue

    dtb_size = struct.unpack('>I', data[dtb_start+4:dtb_start+8])[0]
    dtb_end = dtb_start + dtb_size

    # Find boundary (first non-zero byte after DTB)
    boundary = dtb_end
    while boundary < len(data) and data[boundary] == 0:
        boundary += 1

    available = boundary - dtb_start
    print(f'{dtb_start} {dtb_size} {boundary} {available}')
    sys.exit(0)

sys.exit(1)
"
}

_unpack_dtb () {
	local name="$1"
	local arch=

	echo "  Unpack dtb"

	for path in "raw/$name/dtb/arch"/*; do
		arch=$(echo "$path" | cut -d'/' -f5)
		[[ "$arch" == "microblaze" ]] && continue
		[[ "$arch" == "nios2" ]] && continue

		echo "    $arch"

		mkdir -p "dist/$arch/boot"
		cp -a "$path/boot/dts" "dist/$arch/boot"
	done
}

_unpack_kernel () {
	local name="$1"

	ctx="raw/$name/context.txt"
	[[ ! -f "$ctx" ]] && return
	source "$ctx"
	out="dist/$compiler_arch"

	echo "  Unpack $kernel_defconfig"

	mkdir -p "$out/boot/kernel/$kernel_defconfig"
	cp -a "raw/$name/boot/$kernel" "$out/boot/kernel/$kernel_defconfig"

	mkdir -p "$out/lib/modules_set/$kernel_defconfig"
	cp -a "raw/$name/lib/modules/$kernel_release" "$out/lib/modules_set/$kernel_defconfig"
}

_expand_microblaze () {
	local image="dist/microblaze/boot/kernel/adi_mb_defconfig/simpleImage.generic.strip"
	local path="raw/dtb-gcc/dtb/arch/microblaze/boot/dts"
	local count=0

	log_info "Expand microblaze"

	[[ ! -d "$path" ]] && return
	[[ ! -f "$image" ]] && return

	local dtb_info
	dtb_info=$(_find_empty_dtb "$image")
	[ -z "$dtb_info" ] && { log_error "Empty DTB not found (expected /dts-v1/; / { };)" ; return 1 ; }

	local dtb_start dtb_size dtb_boundary dtb_available
	read dtb_start dtb_size dtb_boundary dtb_available <<< "$dtb_info"

	echo "  Found empty DTB at 0x$(printf '%x' $dtb_start) (size: $dtb_size, available: $dtb_available bytes)"

	local dtb_length=$((dtb_boundary - dtb_start))

	for file in "$path"/*; do
		filename=$(basename $file)
		filename="${filename%.*}"
		image_out="$(dirname $image)/simpleImage.$filename.strip"

		echo -n "  $filename"
		[ -f $image_out ] && { echo " (cached)" ; continue ;}
		echo ""

		local new_dtb_size=$(stat -c%s "$file")
		[ $new_dtb_size -gt $dtb_available ] && { log_error "DTB too large: $new_dtb_size > $dtb_available bytes" ; return 1 ; }

		local src_magic=$(_read_be32 "$file" 0)
		[ $src_magic -ne $((0xd00dfeed)) ] && { log_error "Source DTB has invalid magic: 0x$(printf '%x' $src_magic)" ; return 1 ;}

		cp "$image" "$image_out"
		dd if=/dev/zero status=none \
			of="$image_out" \
			bs=1 seek=$dtb_start conv=notrunc count=$dtb_length
		dd if="$file" status=none  \
			of="$image_out" \
			bs=1 seek=$dtb_start conv=notrunc
		((count+=1))
	done

	command rm $image

	echo "  Expanded $count simpleImages"
}

_expand_nios2 () {
	local image="dist/nios2/boot/kernel/adi_nios2_defconfig/zImage"
	local path="raw/dtb-gcc/dtb/arch/nios2/boot/dts"
	local vmlinux_gz_offset=$((16#4064))
	local input_len_offset=$((16#4060))
	local count=0

	log_info "Expand nios2"

	[[ ! -d "$path" ]] && return
	[[ ! -f "$image" ]] && return

	local tmpdir=$(mktemp -d)
	local size=$(_read_le32 "$image" $input_len_offset)

	dd if="$image" status=none \
		of="$tmpdir/vmlinux.gz" \
		bs=1 skip=$vmlinux_gz_offset count=$size
	gunzip -c "$tmpdir/vmlinux.gz" > "$tmpdir/vmlinux.bin"

	local dtb_info
	dtb_info=$(_find_empty_dtb "$tmpdir/vmlinux.bin")
	[ -z "$dtb_info" ] && { log_error "Empty DTB not found (expected /dts-v1/; / { };)" ; command rm -r "$tmpdir" ; return 1 ; }

	local dtb_start dtb_size dtb_boundary dtb_available
	read dtb_start dtb_size dtb_boundary dtb_available <<< "$dtb_info"

	echo "  Found empty DTB at 0x$(printf '%x' $dtb_start) (size: $dtb_size, available: $dtb_available bytes)"

	for file in "$path"/*; do
		filename=$(basename "${file%.*}")
		image_out="$(dirname "$image")/zImage.$filename"

		echo -n "  $filename"
		[ -f "$image_out" ] && { echo " (cached)" ; continue ;}

		echo ""
		local new_dtb_size=$(stat -c%s "$file")
		[ $new_dtb_size -gt $dtb_available ] && { log_error "DTB too large: $new_dtb_size > $dtb_available bytes" ; return 1 ; }
		local src_magic=$(_read_be32 "$file" 0)
		[ $src_magic -ne $((0xd00dfeed)) ] && { log_error "Source DTB has invalid magic: 0x$(printf '%x' $src_magic)" ; return 1 ;}

		cp "$tmpdir/vmlinux.bin" "$tmpdir/vmlinux.0.bin"
		dd if="$file" status=none \
			of="$tmpdir/vmlinux.0.bin" \
			bs=1 seek=$dtb_start conv=notrunc status=none
		gzip -n -9 < "$tmpdir/vmlinux.0.bin" > "$tmpdir/vmlinux.0.gz"

		local new_size=$(stat -c %s "$tmpdir/vmlinux.0.gz")
		cp "$image" "$image_out"
		_write_le32 $new_size | dd status=none \
			of="$image_out" \
			bs=1 seek=$input_len_offset conv=notrunc
		dd if="$tmpdir/vmlinux.0.gz" status=none \
			of="$image_out" \
			bs=1 seek=$vmlinux_gz_offset conv=notrunc
		((count+=1))
	done

	command rm -r "$tmpdir"
	command rm "$image"

	echo "  Expanded $count zImages"
}

process_artifacts() {
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
}

# Run if executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
	process_artifacts "$@"
fi
