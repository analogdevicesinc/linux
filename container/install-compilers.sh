#!/bin/bash

set -e

version="glibc--stable-2024.05-1" # 13.3.0

zypper install -y --no-recommends \
	clang19 llvm19 lld19 \
	gcc13 gcc13-c++

install_gcc (){
	arch="$1"
	version="$2"
	version_sha="$3"
	arch_alt="${4-$arch}"
	curl -o gcc.tar.xz -L https://toolchains.bootlin.com/downloads/releases/toolchains/$arch/tarballs/$arch--${version}.tar.xz && \
	    echo -n "${version_sha} gcc.tar.xz" | sha256sum -c && \
	    tar xf gcc.tar.xz
	mv $arch--$version /opt/gcc/$arch
	(cd /opt/gcc/$arch/bin && for f in *-13.3.0; do ln -s "/opt/gcc/$arch/bin/$f" "${f/13.3.0/13}"; done)
	ln -s /opt/gcc/$arch/bin/$arch_alt-* /usr/local/bin/
	rm gcc.tar.xz
}

mkdir /opt/gcc

install_gcc \
	"armv7-eabihf" \
	"$version" \
	"608263bc9dc3eadf0962ddb1165f1c2291001190f9927dee47d464e26374462c" \
	"arm"

install_gcc \
	"aarch64" \
	"$version" \
	"b0fad860eb94b503a56d66ca8b9ba06d2d4826943e37ebd1d7217423f6ea5bb2"
