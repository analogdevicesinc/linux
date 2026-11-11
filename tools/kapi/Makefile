# SPDX-License-Identifier: GPL-2.0
# Makefile wrapper for the kapi tool (Rust userspace binary).
#
# See Documentation/dev-tools/kernel-api-spec.rst for details.

PREFIX ?= /usr/local

.PHONY: all build release debug clean install test fmt clippy

all: release

release:
	cargo build --release

build: release

debug:
	cargo build

test:
	cargo test

fmt:
	cargo fmt --all -- --check

clippy:
	cargo clippy --all-targets --all-features -- -D warnings

clean:
	cargo clean

install: release
	install -D -m 0755 target/release/kapi $(DESTDIR)$(PREFIX)/bin/kapi
