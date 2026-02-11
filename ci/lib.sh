#!/bin/bash
#
# Common library functions
#

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
GREY='\033[0;90m'
NC='\033[0m'


log_step() {
	echo -e "[${BLUE}step${NC}]: $1"
}

log_info() {
	echo -e "[${GREEN}info${NC}] $1"
}

log_warn() {
	echo -e "[${YELLOW}warn${NC}] $1"
}

log_error() {
	echo -e "[${RED}error${NC}] $1"
}

log_debug() {
	[ "$verbose" == "true" ] && echo -e "[${GREY}debug${NC}] $1" || true
}
