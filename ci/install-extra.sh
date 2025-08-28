#!/bin/bash
# for: build-doc-*

set -e

# linux checks
zypper install -y --no-recommends \
    sqlite3-devel

git clone https://repo.or.cz/smatch.git smatch --depth=1
cd smatch
make -j$(nproc)

ln -s $(realpath smatch) /usr/local/bin/smatch
