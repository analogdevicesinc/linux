#!/bin/bash

set -e

#core
zypper install -y --no-recommends \
    update-alternatives \
    which wget make openssl openssl-devel ca-certificates ca-certificates-mozilla

update-ca-certificates
