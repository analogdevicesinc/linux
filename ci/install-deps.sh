#!/bin/bash

set -e

#core, gh-actions
zypper install -y --no-recommends \
    which wget make openssl openssl-devel ca-certificates ca-certificates-mozilla \
    nodejs npm-default

update-ca-certificates
