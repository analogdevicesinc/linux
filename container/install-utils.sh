#!/bin/bash

set -e

#core, gcc extensions
zypper install -y --no-recommends \
    git \
    libncurses6 swig bc u-boot-tools flex bison tar bzip2 kmod xz gawk diffutils \
    libelf1 libelf-devel \
    python313 python313-pip python313-devel \
    gmp-devel mpc-devel mpfr-devel

