#!/bin/bash
# for: build-doc-*

set -e

# Install packages required by Buildroot:
# https://buildroot.org/downloads/manual/manual.html#requirement-mandatory
zypper install -y --no-recommends \
    gcc gcc-c++ make \
    glibc-devel libstdc++-devel binutils ccache \
    git wget rsync bc unzip cpio file \
    which sed diffutils bash patch gzip bzip2 perl tar \
    findutils gawk \
