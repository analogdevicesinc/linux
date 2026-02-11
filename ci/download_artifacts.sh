#!/bin/bash

source "ci/lib.sh"

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
	# Suppress 'missing hex digit for \x'
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
export -f _unpack_dtb

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
export -f _unpack_kernel

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

_get_artifact () {
	local cloudsmith_token="$1"
	local git_sha="$2"
	local tuple="$3"
	local url=
	local tags=

	IFS='|' read -r url tags <<< "$tuple"

	[[ "$url" == "null" ]] && return
	[[ "$url" =~ $git_sha/adi_ci_defconfig- ]] && return
	[[ "$url" = *-headers ]] && return

	name=$(basename "$url")
	local zip="$name.zip"
	echo -n "  Artifact $name"

	local ret="200"
	unzip -t "raw/$zip" &>/dev/null && \
		echo -n " (cached zip)" || \
		ret=$(curl -sL \
		-H "X-Api-Key: $cloudsmith_token" \
		-w "%{http_code}\n" \
		-o "raw/$zip" \
		$url)

	echo "tags=$tags" > "raw/$name.metadata.txt"

	[[ "$ret" != "200" ]] && return 1

	[[ ! -d "raw/$name" ]] && \
		unzip -q "raw/$zip" -d "raw/$name" || \
		echo -n " (cached dir)"
	echo ""

	[[ "$name" == "dtb-gcc" ]] && \
		_unpack_dtb "$name" || \
		_unpack_kernel "$name"
}
export -f _get_artifact

download_artifacts() {
	local git_sha="${1-e8367b3a3994}"
	local org_repository="${2-adi/linux}"
	local no_cache="${3-false}"
	local cloudsmith_token=${4}

	[ "$no_cache" == "true" ] && command rm -rf dist/ raw/ .get_artifacts

	[ -f '.get_artifacts' ] && { log_step "Get artifacts (checkpoint)" ; return ;} || log_step "Get artifacts"

	git_sha=${git_sha:0:12}

	[[ -z "$cloudsmith_token" ]] && { log_error "At this time, a CLOUDSMITH_API_KEY is required." ; return 1 ; }
	[[ -z "$git_sha" ]] && { log_error "No git sha provided." ; return 1 ;}
	[[ "${#git_sha}" != "12" ]] && { log_error "Git sha is not 12 characters long." ; return 1 ; }

	command -v unzip 1>/dev/null || { log_error "Command unzip not installed." ; return 1 ; }

	local tmpdir=$(mktemp -d)
	mkdir -p dist
	mkdir -p raw

	query="version:^$git_sha\$"
	log_info "Fetching $git_sha"
	ret=$(curl -sL \
		-w "%{http_code}\n" \
		-o "$tmpdir/.query" \
		-H "X-Api-Key: $cloudsmith_token" \
		"https://api.cloudsmith.io/v1/packages/$org_repository/?query=$query&sort=-date&page_size=100")
	[[ "$ret" != "200" ]] && return 1

	# Extract cdn_url and tags
	tuple=$(cat "$tmpdir/.query" | (jq -r '.[] | "\(.cdn_url)|\(.tags.info | join(","))"'))
	[[ -z "$tuple" ]] && { echo "No artifacts found." ; return 1 ; }

	log_info "Got $(echo "$tuple" | wc -l) artifacts"

	if command -v parallel &>/dev/null ; then
		echo "$tuple" |
			parallel --jobs 8 \
				_get_artifact "$cloudsmith_token" "$git_sha" "{}"
	else
		log_info "Tip: install gnu_parallel to fetch in parallel."
		for url in $tuple; do
			_get_artifact "$cloudsmith_token" "$git_sha" "$url"
		done
	fi

	[ "$SKIP_EXPAND_MICROBLAZE" == "true" ] && log_info "Expand microblaze skipped" || _expand_microblaze
	[ "$SKIP_EXPAND_NIOS2" == "true" ] && log_info "Expand nios2 skipped" || _expand_nios2

	command rm -r $tmpdir
	touch .get_artifacts

	log_info "Wrote to dist/"
}

