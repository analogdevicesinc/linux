#!/bin/bash

set -e

zypper addrepo -c http://download.opensuse.org/distribution/leap/16.0/repo/oss leap-16.0-repo-oss
zypper refresh

#core, linux (gcc), linux (clang), x86, arm, aarch64, linux checks
zypper install -y --no-recommends \
    git \
    libncurses6 swig bc u-boot-tools flex bison tar kmod xz gawk diffutils gcc \
    gcc15 libelf1 libelf-devel \
    clang19 llvm19 lld19 \
    cross-arm-gcc15 \
    cross-aarch64-gcc15 \
    sparse coccinelle ocaml ocaml-findlib cppcheck python311 python311-pip python311-devel

