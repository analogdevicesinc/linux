#!/bin/bash

set -e

version="glibc--stable-2024.05-1" # 13.3.0
[[ -n "$opt_path" ]] || opt_path="/opt/gcc"
[[ -n "$bin_path" ]] || bin_path="/usr/local/bin/"

install_gcc () {
	mkdir -p $opt_path $bin_path
	arch="$1"
	version="$2"
	version_sha="$3"
	arch_alt="${4-$arch}"
	curl -o gcc.tar.xz -L https://toolchains.bootlin.com/downloads/releases/toolchains/$arch/tarballs/$arch--${version}.tar.xz && \
	    echo -n "${version_sha} gcc.tar.xz" | sha256sum -c && \
	    tar xf gcc.tar.xz
	mv "$arch--$version" "$opt_path/$arch"
	(cd "$opt_path/$arch/bin" && for f in *-13.3.0; do ln -s $opt_path/$arch/bin/$f "${f/13.3.0/13}"; done)
	(cd "$opt_path/$arch/lib" && rm -rf python3.* libpython3.* )
	ln -s $opt_path/$arch/bin/$arch_alt-* $bin_path
	rm gcc.tar.xz
}

gcc_install_microblazeel () {
	install_gcc \
		"microblazeel" \
		"$version" \
		"2468c298089873e4d86026b84ca68e46b4802413639a55f2db1c6aba181d66d6"
}

gcc_install_nios2 () {
	install_gcc \
		"nios2" \
		"$version" \
		"db7a12e0b22037f3aad01cd126b5f1bfe7dc514e37f8c484eb587b861f20fbcd"
}

gcc_install_arm () {
	install_gcc \
		"armv7-eabihf" \
		"$version" \
		"608263bc9dc3eadf0962ddb1165f1c2291001190f9927dee47d464e26374462c" \
		"arm"
}

gcc_install_aarch64 () {
	install_gcc \
		"aarch64" \
		"$version" \
		"b0fad860eb94b503a56d66ca8b9ba06d2d4826943e37ebd1d7217423f6ea5bb2"
}

gcc_install () {
	gcc_install_microblazeel
	gcc_install_nios2
	gcc_install_arm
	gcc_install_aarch64
}

suse_install () {
	zypper install -y --no-recommends \
		clang19 llvm19 lld19 \
		gcc13 gcc13-c++
}
