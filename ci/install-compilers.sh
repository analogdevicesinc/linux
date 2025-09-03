#!/bin/bash

set -e

#core, linux, linux gcc, linux clang, arm, aarch64, linux checks
zypper install -y --no-recommends \
    git \
    libncurses6 swig bc u-boot-tools flex bison tar kmod xz gawk diffutils \
    gcc15 libelf1 libelf-devel \
    clang19 llvm19 lld19 \
    cross-arm-gcc15 \
    cross-aarch64-gcc15 \
    sparse coccinelle ocaml ocaml-findlib cppcheck python313 python313-pip python313-devel

update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-15 50
update-alternatives --install /usr/bin/cpp cpp /usr/bin/cpp-15 50
