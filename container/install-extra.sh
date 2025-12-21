#!/bin/bash
# for: build-doc-*

set -e

# linux patch headers_install
zypper install -y --no-recommends \
	xxd rsync

# linux checks
zypper install -y --no-recommends \
	sparse coccinelle ocaml ocaml-findlib cppcheck \
	sqlite3-devel

git clone https://repo.or.cz/smatch.git smatch --depth=1
cd smatch
make -j$(nproc)

ln -s $(realpath smatch) /usr/local/bin/smatch

# user
zypper install -y --no-recommends \
    bash-completion openssh openssh-server
