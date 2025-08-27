#!/bin/bash

set -e

zypper addrepo -c http://download.opensuse.org/distribution/leap/16.0/repo/oss leap-16.0-repo-oss
zypper refresh

#core, linux, linux gcc, linux clang, arm, aarch64, linux checks
zypper install -y --no-recommends \
    git \
    libncurses6 swig bc u-boot-tools flex bison tar kmod xz gawk diffutils \
    gcc14 libelf1 libelf-devel \
    clang19 llvm19 lld19 \
    cross-arm-gcc14 \
    cross-aarch64-gcc14-bootstrap \
    sparse coccinelle ocaml ocaml-findlib cppcheck python311 python311-pip python311-devel

update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50
update-alternatives --install /usr/bin/cpp cpp /usr/bin/cpp-14 50
