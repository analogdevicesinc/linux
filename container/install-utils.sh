#!/bin/bash

set -e

#core, linux, linux gcc, linux clang, arm, aarch64, linux checks
zypper install -y --no-recommends \
    git \
    libncurses6 swig bc u-boot-tools flex bison tar bzip2 kmod xz gawk diffutils \
    libelf1 libelf-devel \
    clang19 llvm19 lld19 \
    sparse coccinelle ocaml ocaml-findlib cppcheck python313 python313-pip python313-devel

