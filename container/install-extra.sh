#!/bin/bash
# for: build-doc-*

set -e

# linux patch headers_install
zypper install -y --no-recommends \
	xxd rsync

# linux checks
zypper install -y --no-recommends \
	coccinelle ocaml ocaml-findlib cppcheck \
	sqlite3-devel

zypper install -y --no-recommends \
	llvm19-devel
git clone https://git.kernel.org/pub/scm/devel/sparse/sparse.git sparse --depth=50
pushd sparse
git switch -d fbdde3127b83e6d09e0ba808d7925dd84407f3c6
make -j$(nproc)

make install PREFIX=/usr/local install
popd
zypper remove -y --clean-deps llvm19-devel

git clone https://repo.or.cz/smatch.git smatch --depth=1
pushd smatch
make -j$(nproc)

ln -s $(realpath smatch) /usr/local/bin/smatch

popd

# user
zypper install -y --no-recommends \
    bash-completion openssh openssh-server
