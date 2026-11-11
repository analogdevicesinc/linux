#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# author: Andrea Mayer <andrea.mayer@uniroma2.it>
# author: Paolo Lungaroni <paolo.lungaroni@uniroma2.it>
#
# This script is designed to test the support for "flavors" in the SRv6 End
# behavior.
#
# Flavors defined in RFC8986 [1] represent additional operations that can modify
# or extend the existing SRv6 End, End.X and End.T behaviors. For the sake of
# convenience, we report the list of flavors described in [1] hereafter:
#   - Penultimate Segment Pop (PSP);
#   - Ultimate Segment Pop (USP);
#   - Ultimate Segment Decapsulation (USD).
#
# The End, End.X, and End.T behaviors can support these flavors either
# individually or in combinations.
# Currently in this selftest we consider the PSP flavor for the SRv6 End
# behavior, both alone and combined with the NEXT-C-SID flavor defined in
# RFC9800 [2]. However, it is possible to extend the script as soon as other
# flavors will be supported in the kernel.
#
# The purpose of the PSP flavor consists in instructing the penultimate node
# listed in the SRv6 policy to remove (i.e. pop) the SRH from the IPv6 header.
# A PSP enabled SRv6 End behavior instance processes the SRH by:
#  - decrementing the Segment Left (SL) value from 1 to 0;
#  - copying the last SID from the SID List into the IPv6 Destination Address
#    (DA);
#  - removing the SRH from the extension headers following the IPv6 header.
#
# Once the SRH is removed, the IPv6 packet is forwarded to the destination using
# the IPv6 DA updated during the PSP operation (i.e. the IPv6 DA corresponding
# to the last SID carried by the removed SRH).
#
# Although the PSP flavor can be set for any SRv6 End behavior instance on any
# SR node, it will be active only on such behaviors bound to a penultimate SID
# for a given SRv6 policy.
#                                                SL=2 SL=1 SL=0
#                                                  |    |    |
# For example, given the SRv6 policy (SID List := <X,   Y,   Z>):
#  - a PSP enabled SRv6 End behavior bound to SID Y will apply the PSP operation
#    as Segment Left (SL) is 1, corresponding to the Penultimate Segment of the
#    SID List;
#  - a PSP enabled SRv6 End behavior bound to SID X will *NOT* apply the PSP
#    operation as the Segment Left is 2. This behavior instance will apply the
#    "standard" End packet processing, ignoring the configured PSP flavor at
#    all.
#
# The NEXT-C-SID flavor (referred to as NEXT-CSID in RFC9800 [2]) and the
# C-SID containers are described in srv6_end_next_csid_l3vpn_test.sh, which
# also tests this flavor for the SRv6 End behavior.
#
# When the NEXT-C-SID flavor is combined with the PSP, USP or USD flavors, these
# flavors are left unchanged [2]. The NEXT-C-SID processing comes first: with a
# non-zero Argument, the IPv6 DA is updated with the next C-SID and the packet
# is forwarded without changing the SRH, if any. The packet goes through the End
# behavior with the other flavors only when the Argument is zero, i.e. when the
# IPv6 DA carries no further C-SID. For example, the PSP operation takes place
# only for a C-SID container with a zero Argument and SL=1.
#
# [1] RFC8986: https://datatracker.ietf.org/doc/html/rfc8986
# [2] RFC9800: https://datatracker.ietf.org/doc/html/rfc9800
#
# Network topology
# ================
#
# The network topology used in this selftest is depicted hereafter, composed by
# two hosts (hs-1, hs-2) and four routers (rt-1, rt-2, rt-3, rt-4).
# Hosts hs-1 and hs-2 are connected to routers rt-1 and rt-2, respectively,
# allowing them to communicate with each other.
# Traffic exchanged between hs-1 and hs-2 can follow different network paths.
# The network operator, through specific SRv6 Policies can steer traffic to one
# path rather than another. In this selftest this is implemented as follows:
#
#   i) The SRv6 H.Insert behavior applies SRv6 Policies on traffic received by
#      connected hosts. It pushes the Segment Routing Header (SRH) after the
#      IPv6 header. The SRH contains the SID List (i.e. SRv6 Policy) needed for
#      steering traffic across the segments/waypoints specified in that list;
#
#  ii) The SRv6 End behavior advances the active SID in the SID List carried by
#      the SRH;
#
# iii) The PSP enabled SRv6 End behavior is used to remove the SRH when such
#      behavior is configured on a node bound to the Penultimate Segment carried
#      by the SID List.
#
#                cafe::1                      cafe::2
#              +--------+                   +--------+
#              |        |                   |        |
#              |  hs-1  |                   |  hs-2  |
#              |        |                   |        |
#              +---+----+                   +--- +---+
#     cafe::/64    |                             |      cafe::/64
#                  |                             |
#              +---+----+                   +----+---+
#              |        |  fcf0:0:1:2::/64  |        |
#              |  rt-1  +-------------------+  rt-2  |
#              |        |                   |        |
#              +---+----+                   +----+---+
#                  |      .               .      |
#                  |  fcf0:0:1:3::/64   .        |
#                  |          .       .          |
#                  |            .   .            |
#  fcf0:0:1:4::/64 |              .              | fcf0:0:2:3::/64
#                  |            .   .            |
#                  |          .       .          |
#                  |  fcf0:0:2:4::/64   .        |
#                  |      .               .      |
#              +---+----+                   +----+---+
#              |        |                   |        |
#              |  rt-4  +-------------------+  rt-3  |
#              |        |  fcf0:0:3:4::/64  |        |
#              +---+----+                   +----+---+
#
# Every fcf0:0:x:y::/64 network interconnects the SRv6 routers rt-x with rt-y in
# the IPv6 operator network.
#
#
# Local SID table
# ===============
#
# Each SRv6 router is configured with a Local SID table in which SIDs are
# stored. Considering the given SRv6 router rt-x, at least two SIDs are
# configured in the Local SID table:
#
#   Local SID table for SRv6 router rt-x
#   +---------------------------------------------------------------------+
#   |fcff:x::e is associated with the SRv6 End behavior                   |
#   |fcff:x::ef1 is associated with the SRv6 End behavior with PSP flavor |
#   |fcbb:0:xpyy::/48 is associated with the SRv6 behavior and flavors    |
#   |   given by the Function yy, and with parameter p of the behavior    |
#   +---------------------------------------------------------------------+
#
# The fcff::/16 prefix is reserved by the operator for the SIDs. Reachability of
# SIDs is ensured by proper configuration of the IPv6 operator's network and
# SRv6 routers.
#
# Similarly, the fcbb:0::/32 prefix is reserved by the operator for the C-SID
# containers and it is used as their Locator-Block.
# A C-SID (16 bits) is written as xpyy, where:
#  - x is the Locator-Node (4 bits), i.e. the ID of the router;
#  - p is a parameter of the behavior (4 bits), 0 for End, which has none;
#  - yy is the Function (8 bits). Its first digit selects the SRv6 behavior
#    (e for End, as in fcff:x::e). Its second digit is the set of flavors, as
#    a bit mask: 1 for PSP, 2 for USP, 4 for USD and 8 for NEXT-C-SID (as the
#    f1 of fcff:x::ef1 for PSP).
# In the terms of RFC9800 [2], p and yy together are the Function of the C-SID
# (12 bits), and p is not part of the Argument:
#
#   +----------+----------+--------------------+
#   | Function | behavior | flavors            |
#   +----------+----------+--------------------+
#   | e9       | End      | NEXT-C-SID and PSP |
#   +----------+----------+--------------------+
#
# For example, fcbb:0:40e9:20e9:: is a C-SID container carrying the C-SID of
# rt-4 followed by the C-SID of rt-2, both with the NEXT-C-SID and PSP flavors.
#
#
# SRv6 Policies
# =============
#
# An SRv6 ingress router applies different SRv6 Policies to the traffic received
# from connected hosts on the basis of the destination addresses.
# In case of SRv6 H.Insert behavior, the SRv6 Policy enforcement consists of
# pushing the SRH (carrying a given SID List) after the existing IPv6 header.
# Note that in the inserting mode, there is no encapsulation at all.
#
#   Before applying an SRv6 Policy using the SRv6 H.Insert behavior
#   +------+---------+
#   | IPv6 | Payload |
#   +------+---------+
#
#   After applying an SRv6 Policy using the SRv6 H.Insert behavior
#   +------+-----+---------+
#   | IPv6 | SRH | Payload |
#   +------+-----+---------+
#
# Traffic from hs-1 to hs-2
# -------------------------
#
# Packets generated from hs-1 and directed towards hs-2 are
# handled by rt-1 which applies the following SRv6 Policy:
#
#   i.a) IPv6 traffic, SID List=fcff:3::e,fcff:4::ef1,fcff:2::ef1,cafe::2
#
# Router rt-1 is configured to enforce the Policy (i.a) through the SRv6
# H.Insert behavior which pushes the SRH after the existing IPv6 header. This
# Policy steers the traffic from hs-1 across rt-3, rt-4, rt-2 and finally to the
# destination hs-2.
#
# As the packet reaches the router rt-3, the SRv6 End behavior bound to SID
# fcff:3::e is triggered. The behavior updates the Segment Left (from SL=3 to
# SL=2) in the SRH, the IPv6 DA with fcff:4::ef1 and forwards the packet to the
# next router on the path, i.e. rt-4.
#
# When router rt-4 receives the packet, the PSP enabled SRv6 End behavior bound
# to SID fcff:4::ef1 is executed. Since the SL=2, the PSP operation is *NOT*
# kicked in and the behavior applies the default End processing: the Segment
# Left is decreased (from SL=2 to SL=1), the IPv6 DA is updated with the SID
# fcff:2::ef1 and the packet is forwarded to router rt-2.
#
# The PSP enabled SRv6 End behavior on rt-2 is associated with SID fcff:2::ef1
# and is executed as the packet is received. Because SL=1, the behavior applies
# the PSP processing on the packet as follows: i) SL is decreased, i.e. from
# SL=1 to SL=0; ii) last SID (cafe::2) is copied into the IPv6 DA; iii) the
# outermost SRH is removed from the extension headers following the IPv6 header.
# Once the PSP processing is completed, the packet is forwarded to the host hs-2
# (destination).
#
# Traffic from hs-2 to hs-1
# -------------------------
#
# Packets generated from hs-2 and directed to hs-1 are handled by rt-2 which
# applies the following SRv6 Policy:
#
#   i.b) IPv6 traffic, SID List=fcff:1::ef1,cafe::1
#
# Router rt-2 is configured to enforce the Policy (i.b) through the SRv6
# H.Insert behavior which pushes the SRH after the existing IPv6 header. This
# Policy steers the traffic from hs-2 across rt-1 and finally to the
# destination hs-1
#
#
# When the router rt-1 receives the packet, the PSP enabled SRv6 End behavior
# associated with the SID fcff:1::ef1 is triggered. Since the SL=1,
# the PSP operation takes place: i) the SL is decremented; ii) the IPv6 DA is
# set with the last SID; iii) the SRH is removed from the extension headers
# after the IPv6 header. At this point, the packet with IPv6 DA=cafe::1 is sent
# to the destination, i.e. hs-1.
#
# Traffic to the loopback addresses of the hosts
# ----------------------------------------------
#
# Each host has an address on its loopback, cafe::x:e9. The traffic from the
# other host towards this address is steered with C-SID containers: after the
# ingress router, every router crossed applies the SRv6 End behavior with the
# NEXT-C-SID and PSP flavors. The following SRv6 Policies are applied by rt-1
# to the traffic towards cafe::2:e9 (i.c) and by rt-2 to the traffic towards
# cafe::1:e9 (i.d):
#
#   i.c) IPv6 traffic, SID List=fcbb:0:30e9::,fcbb:0:40e9:20e9::,cafe::2:e9
#   i.d) IPv6 traffic, SID List=fcbb:0:40e9:30e9:10e9::,cafe::1:e9
#
# In the Policy (i.c), rt-3 receives a zero Argument with SL=2 and applies the
# "standard" End processing. rt-4 receives a non-zero Argument with SL=1: it
# advances to the next C-SID and does *NOT* apply the PSP operation. rt-2
# receives a zero Argument with SL=1 and applies the PSP operation.
#
# In the Policy (i.d), a single C-SID container carries three C-SIDs. rt-4 and
# rt-3 receive a non-zero Argument with SL=1: they advance to the next C-SID and
# do *NOT* apply the PSP operation. rt-1 receives a zero Argument with SL=1
# and applies the PSP operation.
#
# The hosts use cafe::x as the source address, not the loopback address. The
# replies are sent to cafe::x and follow the Policies (i.a) and (i.b), which
# are tested first. This way, each ping checks a single Policy with the
# NEXT-C-SID flavor, and a failure points directly to that Policy.
#
# The hosts drop the packets that carry an SRH (seg6_enabled is set to 0 on
# the hosts). The loopback address of a host is reached only through the
# router connected to that host: the ping succeeds only if the SRH is removed
# by that router.
#
# Traffic with a consumed SRH
# ---------------------------
#
# A packet can carry more than one SRH. The SRv6 End behavior skips an SRH with
# SL=0 and processes the next header [1].
#
# The traffic of this section reuses the Policies (i.a), (i.b), (i.c) and (i.d).
# A router on the path inserts an extra SRH in front of the Policy's SRH, and
# another router then consumes the extra SRH.
#
# The Policy (i.a) has SID List=fcff:3::e,fcff:4::ef1,fcff:2::ef1,cafe::2. A
# packet from hs-1 to hs-2 goes as follows:
#
#  - rt-1 applies the Policy and sends the packet to rt-3:
#      IPv6 DA=fcff:3::e | Policy's SRH, SL=3 | Payload
#
#  - rt-3 (End) processes the Policy's SRH and sends the packet to rt-4:
#      IPv6 DA=fcff:4::ef1 | Policy's SRH, SL=2 | Payload
#
#  - rt-4 (End with PSP and SL>1, acting as End) processes the Policy's SRH.
#    The new DA is fcff:2::ef1:
#      IPv6 DA=fcff:2::ef1 | Policy's SRH, SL=1 | Payload
#    On rt-4, the route added by the test for fcff:2::ef1 then inserts the
#    extra SRH, with SID List=fcff:3::e,fcff:2::ef1, and the packet goes back to
#    rt-3:
#      IPv6 DA=fcff:3::e | extra SRH, SL=1 | Policy's SRH, SL=1 | Payload
#
#  - rt-3 (End) consumes the extra SRH and sets the DA back to fcff:2::ef1:
#      IPv6 DA=fcff:2::ef1 | extra SRH, SL=0 | Policy's SRH, SL=1 | Payload
#
#  - rt-2 (End with PSP) skips the extra SRH, processes the Policy's SRH and
#    removes it. The Next Header of the extra SRH now indicates the payload
#    instead of the Policy's SRH:
#      IPv6 DA=cafe::2 | extra SRH, SL=0 | Payload
#
# The extra SRH reaches hs-2, which skips it as its SL is 0. The other
# Policies work in the same way:
#
#   (i.b) hs-2 -> hs-1: on the packets with DA=fcff:1::ef1, rt-2 inserts the
#         extra SRH with SID List=fcff:3::e,fcff:1::ef1, and rt-3 consumes it.
#   (i.c) hs-1 -> hs-2: on the packets with DA=fcbb:0:20e9::, rt-4 inserts the
#         extra SRH with SID List=fcff:3::e,fcbb:0:20e9::, and rt-3 consumes it.
#   (i.d) hs-2 -> hs-1: on the packets with DA=fcbb:0:10e9::, rt-3 inserts the
#         extra SRH with SID List=fcff:4::e,fcbb:0:10e9::, and rt-4 consumes it.
#
# The Policy (i.b) has no router between rt-2 and rt-1. For this reason, rt-2
# applies the Policy, inserts the extra SRH and sends the packet to rt-3:
#      IPv6 DA=fcff:3::e | extra SRH, SL=1 | Policy's SRH, SL=1 | Payload
# rt-3 consumes the extra SRH, and rt-1 removes the Policy's SRH.
#
# The hosts need seg6_enabled set to 1 to accept the extra SRH, and it is set
# back to 0 at the end. With seg6_enabled set to 1, a host also accepts the
# packet when the Policy's SRH is not removed, or when the extra SRH is removed
# in its place, as it skips every SRH with SL=0. For this reason, the receiving
# host accepts the echo request only when its first SRH is the extra SRH, with
# SL=0 and followed by the ICMPv6 header. Any other echo request is dropped,
# including one without any SRH. The extra SRH is recognized by its last segment
# (Segment List[0]), which is the SID that triggered its insertion (e.g.
# fcff:2::ef1 for the Policy (i.a)).
# These tests need ip6tables with the srh match, and they are skipped without
# it.

# shellcheck source=lib.sh
source lib.sh

readonly RDMSUFF="$(mktemp -u XXXXXXXX)"
readonly DUMMY_DEVNAME="dum0"
readonly RT2HS_DEVNAME="veth1"
readonly LOCALSID_TABLE_ID=90
readonly IPv6_RT_NETWORK=fcf0:0
readonly IPv6_HS_NETWORK=cafe
readonly IPv6_TESTS_ADDR=2001:db8::1
readonly LOCATOR_SERVICE=fcff
readonly END_FUNC=000e
readonly END_PSP_FUNC=0ef1
readonly END_CSID_PSP_FUNC=e9

readonly LCBLOCK_ADDR=fcbb:0
readonly LCBLOCK_BLEN=32
# do not add ':' as separator. The Locator-Node is followed by the parameter
# of the behavior, 0 for End.
readonly LCNODE_FMT="%x0"
readonly LCNODE_BLEN=4
readonly LCNODEFUNC_BLEN=16

readonly LCBLOCK_NODE_BLEN=$((LCBLOCK_BLEN + LCNODE_BLEN))
readonly LCBLOCK_NODEFUNC_BLEN=$((LCBLOCK_BLEN + LCNODEFUNC_BLEN))
# lengths of Locator-Block and C-SID of the NEXT-C-SID flavor
readonly CSID_CFG="lblen ${LCBLOCK_BLEN} nflen ${LCNODEFUNC_BLEN}"

PING_TIMEOUT_SEC=4
PAUSE_ON_FAIL=${PAUSE_ON_FAIL:=no}

# IDs of routers and hosts are initialized during the setup of the testing
# network
ROUTERS=''
HOSTS=''

SETUP_ERR=1

# set by check_srh_match_support
HAS_SRH_MATCH=false

ret=${ksft_skip}
nsuccess=0
nfail=0
nskip=0

log_test()
{
	log_test_expected "$1" "$2" "$3"
}

# Log a test that is not run (same format as log_test_expected of lib.sh).
log_test_skipped()
{
	local msg="$1"

	nskip=$((nskip+1))
	printf "    TEST: %-60s  [SKIP]\n" "${msg}"
}

print_log_test_results()
{
	printf "\nTests passed: %3d\n" "${nsuccess}"
	printf "Tests failed: %3d\n"   "${nfail}"
	if [ "${nskip}" -gt 0 ]; then
		printf "Tests skipped: %2d\n" "${nskip}"
	fi

	# when a test fails, the value of 'ret' is set to 1 (error code).
	# Conversely, when all tests are passed successfully, the 'ret' value
	# is set to 0 (success code).
	if [ "${ret}" -ne 1 ]; then
		ret=0
	fi
}

log_section()
{
	echo
	echo "################################################################################"
	echo "TEST SECTION: $*"
	echo "################################################################################"
}

test_command_or_ksft_skip()
{
	local cmd="$1"

	if [ ! -x "$(command -v "${cmd}")" ]; then
		echo "SKIP: Could not run test without \"${cmd}\" tool";
		exit "${ksft_skip}"
	fi
}

get_nodename()
{
	local name="$1"

	echo "${name}-${RDMSUFF}"
}

get_rtname()
{
	local rtid="$1"

	get_nodename "rt-${rtid}"
}

get_hsname()
{
	local hsid="$1"

	get_nodename "hs-${hsid}"
}

__create_namespace()
{
	local name="$1"

	ip netns add "${name}"
}

create_router()
{
	local rtid="$1"
	local nsname

	nsname="$(get_rtname "${rtid}")"

	__create_namespace "${nsname}"
}

create_host()
{
	local hsid="$1"
	local nsname

	nsname="$(get_hsname "${hsid}")"

	__create_namespace "${nsname}"
}

cleanup()
{
	local nsname
	local i

	# destroy routers
	for i in ${ROUTERS}; do
		nsname="$(get_rtname "${i}")"

		ip netns del "${nsname}" &>/dev/null || true
	done

	# destroy hosts
	for i in ${HOSTS}; do
		nsname="$(get_hsname "${i}")"

		ip netns del "${nsname}" &>/dev/null || true
	done

	# check whether the setup phase was completed successfully or not. In
	# case of an error during the setup phase of the testing environment,
	# the selftest is considered as "skipped".
	if [ "${SETUP_ERR}" -ne 0 ]; then
		echo "SKIP: Setting up the testing environment failed"
		exit "${ksft_skip}"
	fi

	exit "${ret}"
}

add_link_rt_pairs()
{
	local rt="$1"
	local rt_neighs="$2"
	local neigh
	local nsname
	local neigh_nsname

	nsname="$(get_rtname "${rt}")"

	for neigh in ${rt_neighs}; do
		neigh_nsname="$(get_rtname "${neigh}")"

		ip link add "veth-rt-${rt}-${neigh}" netns "${nsname}" \
			type veth peer name "veth-rt-${neigh}-${rt}" \
			netns "${neigh_nsname}"
	done
}

get_network_prefix()
{
	local rt="$1"
	local neigh="$2"
	local p="${rt}"
	local q="${neigh}"

	if [ "${p}" -gt "${q}" ]; then
		p="${q}"; q="${rt}"
	fi

	echo "${IPv6_RT_NETWORK}:${p}:${q}"
}

# Given the description of a router <id:op> as an input, the function returns
# the <id> token which represents the ID of the router.
# i.e. input: "12:psp"
#      output: "12"
__get_srv6_rtcfg_id()
{
	local element="$1"

	echo "${element}" | cut -d':' -f1
}

# Given the description of a router <id:op> as an input, the function returns
# the <op> token which represents the operation (e.g. End behavior with or
# without flavors) configured for the node.

# Note that when the operation represents an End behavior with a list of
# flavors, the output is the ordered version of that list.
# i.e. input: "5:usp,psp,usd"
#      output: "psp,usd,usp"
__get_srv6_rtcfg_op()
{
	local element="$1"

	# return the lexicographically ordered flavors
	echo "${element}" | cut -d':' -f2 | sed 's/,/\n/g' | sort | \
		xargs | sed 's/ /,/g'
}

# Return the C-SID of a router: the Locator-Node (4 bits), i.e. the ID of the
# router, followed by the parameter of the behavior (4 bits, 0) and the
# Function (8 bits).
# args:
#  $1 - router
#  $2 - Function (e.g. "e9")
# e.g. input: "4" "e9"
#      output: "40e9"
build_csid()
{
	local nodeid="$1"
	local func="$2"

	printf "${LCNODE_FMT}%s" "${nodeid}" "${func}"
}

# Return the C-SID container with the C-SIDs of the routers listed in a
# "next-csid,<flavors>" element, in the same order.
# args:
#  $1 - <nodeid> of the element: a comma-separated list of routers
#  $2 - Function (e.g. "e9")
# e.g. input: "4,2" "e9"
#      output: "fcbb:0:40e9:20e9::"
build_csid_container()
{
	local nodeids="$1"
	local func="$2"
	local csids=''
	local nodes
	local node

	nodes="$(echo "${nodeids}" | sed 's/,/ /g')"
	for node in ${nodes}; do
		csids="${csids}:$(build_csid "${node}" "${func}")"
	done

	echo "${LCBLOCK_ADDR}${csids}::"
}

# Setup the basic networking for the routers
setup_rt_networking()
{
	local rt="$1"
	local rt_neighs="$2"
	local nsname
	local net_prefix
	local devname
	local neigh

	nsname="$(get_rtname "${rt}")"

	for neigh in ${rt_neighs}; do
		devname="veth-rt-${rt}-${neigh}"

		net_prefix="$(get_network_prefix "${rt}" "${neigh}")"

		ip -netns "${nsname}" addr \
			add "${net_prefix}::${rt}/64" dev "${devname}" nodad

		ip -netns "${nsname}" link set "${devname}" up
	done

	ip -netns "${nsname}" link set lo up

	ip -netns "${nsname}" link add ${DUMMY_DEVNAME} type dummy
	ip -netns "${nsname}" link set ${DUMMY_DEVNAME} up

	ip netns exec "${nsname}" sysctl -wq net.ipv6.conf.all.accept_dad=0
	ip netns exec "${nsname}" sysctl -wq net.ipv6.conf.default.accept_dad=0
	ip netns exec "${nsname}" sysctl -wq net.ipv6.conf.all.forwarding=1
}

# Setup local SIDs for an SRv6 router
setup_rt_local_sids()
{
	local rt="$1"
	local rt_neighs="$2"
	local net_prefix
	local devname
	local nsname
	local neigh
	local csid

	nsname="$(get_rtname "${rt}")"

	for neigh in ${rt_neighs}; do
		devname="veth-rt-${rt}-${neigh}"

		net_prefix="$(get_network_prefix "${rt}" "${neigh}")"

		# set underlay network routes for SIDs reachability
		ip -netns "${nsname}" -6 route \
			add "${LOCATOR_SERVICE}:${neigh}::/32" \
			table "${LOCALSID_TABLE_ID}" \
			via "${net_prefix}::${neigh}" dev "${devname}"

		# set underlay network routes to reach the C-SID containers
		# that start with a C-SID of the neighbor, for any parameter and
		# Function.
		csid="$(build_csid "${neigh}" 00)"
		ip -netns "${nsname}" -6 route \
			add "${LCBLOCK_ADDR}:${csid}::/${LCBLOCK_NODE_BLEN}" \
			table "${LOCALSID_TABLE_ID}" \
			via "${net_prefix}::${neigh}" dev "${devname}"
	done

	# Local End behavior (note that "dev" is a dummy interface chosen for
	# the sake of simplicity).
	ip -netns "${nsname}" -6 route \
		add "${LOCATOR_SERVICE}:${rt}::${END_FUNC}" \
		table "${LOCALSID_TABLE_ID}" \
		encap seg6local action End dev "${DUMMY_DEVNAME}"

	# all SIDs start with a common locator. Routes and SRv6 Endpoint
	# behavior instances are grouped together in the 'localsid' table.
	ip -netns "${nsname}" -6 rule \
		add to "${LOCATOR_SERVICE}::/16" \
		lookup "${LOCALSID_TABLE_ID}" prio 999

	# the C-SID containers use their own Locator-Block. Their routes and
	# SRv6 Endpoint behavior instances are in the 'localsid' table too.
	ip -netns "${nsname}" -6 rule \
		add to "${LCBLOCK_ADDR}::/${LCBLOCK_BLEN}" \
		lookup "${LOCALSID_TABLE_ID}" prio 999

	# set default routes to unreachable
	ip -netns "${nsname}" -6 route \
		add unreachable default metric 4278198272 \
		dev "${DUMMY_DEVNAME}"
}

# Return success if the operation <op> of an element contains the NEXT-C-SID
# flavor, i.e. the SIDs of the element are C-SIDs.
# e.g. input: "next-csid,psp" -> success; "psp" -> failure
op_has_next_csid()
{
	local op_type="$1"

	case ",${op_type}," in
	*,next-csid,*)
		return 0
		;;
	esac

	return 1
}

# Return the SID to which the SRv6 Endpoint behavior of a router is bound.
# With NEXT-C-SID, it is the C-SID of the router as a prefix: on rt-4,
# fcbb:0:40e9::/48 also matches the DA fcbb:0:40e9:20e9::.
# args:
#  $1 - router
#  $2 - operation <op> of the element (e.g. "next-csid,psp")
#  $3 - Function (e.g. "0ef1", or "e9" with NEXT-C-SID)
# e.g. input: "4" "psp" "0ef1"
#      output: "fcff:4::0ef1"
#      input: "4" "next-csid,psp" "e9"
#      output: "fcbb:0:40e9::/48"
get_rt_sid()
{
	local rt="$1"
	local op_type="$2"
	local func="$3"
	local csid

	if op_has_next_csid "${op_type}"; then
		csid="$(build_csid "${rt}" "${func}")"
		echo "${LCBLOCK_ADDR}:${csid}::/${LCBLOCK_NODEFUNC_BLEN}"
		return
	fi

	echo "${LOCATOR_SERVICE}:${rt}::${func}"
}

# Add an SRv6 Endpoint behavior to a router, bound to a SID. Nothing is added
# if the router already has a route for that SID.
# args:
#  $1 - router
#  $2 - SID, or C-SID of the router as a prefix
#  $3 - SRv6 Endpoint behavior and flavors (e.g. "End flavors psp")
setup_rt_endpoint()
{
	local rt="$1"
	local sid="$2"
	local behavior_cfg="$3"
	local nsname

	nsname="$(get_rtname "${rt}")"

	# the prefix length is part of the SID: two SIDs can have the same
	# address and different lengths.
	if [ -n "$(ip -netns "${nsname}" -6 route show \
		table "${LOCALSID_TABLE_ID}" exact "${sid}")" ]; then
		return
	fi

	ip -netns "${nsname}" -6 route \
		add "${sid}" table "${LOCALSID_TABLE_ID}" \
		encap seg6local action ${behavior_cfg} \
		dev "${DUMMY_DEVNAME}"
}

# Add an SRv6 Endpoint behavior to each router of a Policy element <nodeid:op>.
# args:
#  $1 - <nodeid> of the element: one router, or a comma-separated list
#  $2 - operation <op> of the element (e.g. "next-csid,psp")
#  $3 - Function (e.g. "0ef1", or "e9" with NEXT-C-SID)
#  $4 - SRv6 Endpoint behavior and flavors (e.g. "End flavors psp")
# e.g. input: "4,2" "next-csid,psp" "e9"
#             "End flavors next-csid,psp lblen 32 nflen 16"
#      result: the behavior is added on rt-4, bound to fcbb:0:40e9::/48, and
#              on rt-2, bound to fcbb:0:20e9::/48
setup_rt_endpoints()
{
	local node="$1"
	local op_type="$2"
	local func="$3"
	local behavior_cfg="$4"
	local sid
	local rt

	for rt in $(echo "${node}" | sed 's/,/ /g'); do
		sid="$(get_rt_sid "${rt}" "${op_type}" "${func}")"
		setup_rt_endpoint "${rt}" "${sid}" "${behavior_cfg}"
	done
}

# This helper function builds and installs the SID List (i.e. SRv6 Policy)
# to be applied on incoming packets at the ingress node. Moreover, it
# configures the SRv6 nodes specified in the SID List to process the traffic
# according to the operations required by the Policy itself.
# args:
#  $1 - destination host x (i.e. the cafe::x host), or x:yy for the address
#       cafe::x:yy on the loopback of the host (e.g. 2:e9)
#  $2 - SRv6 router configured for enforcing the SRv6 Policy
#  $3 - compact way to represent a list of SRv6 routers with their operations
#       (i.e. behaviors) that each of them needs to perform. Every <nodeid:op>
#       element constructs a SID that is associated with the behavior <op> on
#       the <nodeid> node. The list of such elements forms an SRv6 Policy.
#       For "next-csid,psp", <nodeid> is a comma-separated list of node IDs.
#       The element constructs a C-SID container with the C-SIDs of these
#       nodes, e.g. "4,2:next-csid,psp" gives fcbb:0:40e9:20e9::.
__setup_rt_policy()
{
	local dst="$1"
	local encap_rt="$2"
	local policy_rts="$3"
	local behavior_cfg
	local in_nsname
	local policy=''
	local container
	local function
	local op_type
	local node
	local n

	in_nsname="$(get_rtname "${encap_rt}")"

	for n in ${policy_rts}; do
		node="$(__get_srv6_rtcfg_id "${n}")"
		op_type="$(__get_srv6_rtcfg_op "${n}")"

		case "${op_type}" in
		"noflv")
			policy="${policy}${LOCATOR_SERVICE}:${node}::${END_FUNC},"
			function="${END_FUNC}"
			behavior_cfg="End"
			;;

		"psp")
			policy="${policy}${LOCATOR_SERVICE}:${node}::${END_PSP_FUNC},"
			function="${END_PSP_FUNC}"
			behavior_cfg="End flavors psp"
			;;

		"next-csid,psp")
			container="$(build_csid_container "${node}" \
				"${END_CSID_PSP_FUNC}")"
			policy="${policy}${container},"
			function="${END_CSID_PSP_FUNC}"
			behavior_cfg="End flavors next-csid,psp ${CSID_CFG}"
			;;

		*)
			break
			;;
		esac

		# add SRv6 Endpoint behavior to the selected routers
		setup_rt_endpoints "${node}" \
			"${op_type}" "${function}" "${behavior_cfg}"
	done

	# we need to remove the trailing comma to avoid inserting an empty
	# address (::0) in the SID List.
	policy="${policy%,}"

	# add SRv6 policy to incoming traffic sent by connected hosts
	ip -netns "${in_nsname}" -6 route \
		add "${IPv6_HS_NETWORK}::${dst}" \
		encap seg6 mode inline segs "${policy}" \
		dev "${DUMMY_DEVNAME}"

	ip -netns "${in_nsname}" -6 neigh \
		add proxy "${IPv6_HS_NETWORK}::${dst}" \
		dev "${RT2HS_DEVNAME}"
}

# see __setup_rt_policy
setup_rt_policy_ipv6()
{
	__setup_rt_policy "$1" "$2" "$3"
}

setup_hs()
{
	local hs="$1"
	local rt="$2"
	local hsname
	local rtname

	hsname="$(get_hsname "${hs}")"
	rtname="$(get_rtname "${rt}")"

	ip netns exec "${hsname}" sysctl -wq net.ipv6.conf.all.accept_dad=0
	ip netns exec "${hsname}" sysctl -wq net.ipv6.conf.default.accept_dad=0
	ip netns exec "${hsname}" sysctl -wq net.ipv6.conf.all.seg6_enabled=0

	ip -netns "${hsname}" link add veth0 type veth \
		peer name "${RT2HS_DEVNAME}" netns "${rtname}"

	ip -netns "${hsname}" addr \
		add "${IPv6_HS_NETWORK}::${hs}/64" dev veth0 nodad

	ip -netns "${hsname}" link set veth0 up
	ip -netns "${hsname}" link set lo up

	ip -netns "${rtname}" addr \
		add "${IPv6_HS_NETWORK}::254/64" dev "${RT2HS_DEVNAME}" nodad

	ip -netns "${rtname}" link set "${RT2HS_DEVNAME}" up

	ip netns exec "${rtname}" \
		sysctl -wq net.ipv6.conf."${RT2HS_DEVNAME}".proxy_ndp=1
}

# Set up an address on the loopback of a host, and the route towards it on the
# router connected to the host.
# args:
#  $1 - host
#  $2 - router connected to the host
#  $3 - suffix of the address (i.e. cafe::<host>:<suffix>)
setup_hs_lo_addr()
{
	local hs="$1"
	local rt="$2"
	local suffix="$3"
	local hsname
	local rtname
	local addr

	hsname="$(get_hsname "${hs}")"
	rtname="$(get_rtname "${rt}")"
	addr="${IPv6_HS_NETWORK}::${hs}:${suffix}"

	ip -netns "${hsname}" addr add "${addr}/128" dev lo nodad

	ip -netns "${rtname}" -6 route \
		add "${addr}" \
		via "${IPv6_HS_NETWORK}::${hs}" dev "${RT2HS_DEVNAME}"
}

setup()
{
	local i

	# create routers
	ROUTERS="1 2 3 4"; readonly ROUTERS
	for i in ${ROUTERS}; do
		create_router "${i}"
	done

	# create hosts
	HOSTS="1 2"; readonly HOSTS
	for i in ${HOSTS}; do
		create_host "${i}"
	done

	# set up the links for connecting routers
	add_link_rt_pairs 1 "2 3 4"
	add_link_rt_pairs 2 "3 4"
	add_link_rt_pairs 3 "4"

	# set up the basic connectivity of routers and routes required for
	# reachability of SIDs.
	setup_rt_networking 1 "2 3 4"
	setup_rt_networking 2 "1 3 4"
	setup_rt_networking 3 "1 2 4"
	setup_rt_networking 4 "1 2 3"

	# set up the hosts connected to routers
	setup_hs 1 1
	setup_hs 2 2

	# set up the loopback addresses of the hosts used by the Policies (i.c)
	# and (i.d).
	setup_hs_lo_addr 1 1 "${END_CSID_PSP_FUNC}"
	setup_hs_lo_addr 2 2 "${END_CSID_PSP_FUNC}"

	# set up default SRv6 Endpoints (i.e. SRv6 End behavior)
	setup_rt_local_sids 1 "2 3 4"
	setup_rt_local_sids 2 "1 3 4"
	setup_rt_local_sids 3 "1 2 4"
	setup_rt_local_sids 4 "1 2 3"

	# set up SRv6 policies
	# create a connection between hosts hs-1 and hs-2.
	# The path between hs-1 and hs-2 traverses SRv6 aware routers.
	# For each direction two path are chosen:
	#
	# Direction hs-1 -> hs-2 (PSP flavor)
	#  - rt-1 (SRv6 H.Insert policy)
	#  - rt-3 (SRv6 End behavior)
	#  - rt-4 (SRv6 End flavor PSP with SL>1, acting as End behavior)
	#  - rt-2 (SRv6 End flavor PSP with SL=1)
	#
	# Direction hs-2 -> hs-1 (PSP flavor)
	#  - rt-2 (SRv6 H.Insert policy)
	#  - rt-1 (SRv6 End flavor PSP with SL=1)
	setup_rt_policy_ipv6 2 1 "3:noflv 4:psp 2:psp"
	setup_rt_policy_ipv6 1 2 "1:psp"

	# Direction hs-1 -> hs-2 (cafe::2:e9, NEXT-C-SID and PSP flavors)
	#  - rt-1 (SRv6 H.Insert policy)
	#  - rt-3 (Argument=0 and SL>1, acting as an End behavior)
	#  - rt-4 (Argument!=0 and SL=1, advancing the C-SID)
	#  - rt-2 (Argument=0 and SL=1, PSP)
	#
	# Direction hs-2 -> hs-1 (cafe::1:e9, NEXT-C-SID and PSP flavors)
	#  - rt-2 (SRv6 H.Insert policy)
	#  - rt-4 (Argument!=0 and SL=1, advancing the C-SID)
	#  - rt-3 (Argument!=0 and SL=1, advancing the C-SID)
	#  - rt-1 (Argument=0 and SL=1, PSP)
	setup_rt_policy_ipv6 "2:${END_CSID_PSP_FUNC}" 1 \
		"3:next-csid,psp 4,2:next-csid,psp"
	setup_rt_policy_ipv6 "1:${END_CSID_PSP_FUNC}" 2 "4,3,1:next-csid,psp"

	# testing environment was set up successfully
	SETUP_ERR=0
}

check_rt_connectivity()
{
	local rtsrc="$1"
	local rtdst="$2"
	local prefix
	local rtsrc_nsname

	rtsrc_nsname="$(get_rtname "${rtsrc}")"

	prefix="$(get_network_prefix "${rtsrc}" "${rtdst}")"

	ip netns exec "${rtsrc_nsname}" ping -c 1 -W "${PING_TIMEOUT_SEC}" \
		"${prefix}::${rtdst}" >/dev/null 2>&1
}

check_and_log_rt_connectivity()
{
	local rtsrc="$1"
	local rtdst="$2"

	check_rt_connectivity "${rtsrc}" "${rtdst}"
	log_test $? 0 "Routers connectivity: rt-${rtsrc} -> rt-${rtdst}"
}

check_hs_ipv6_connectivity()
{
	local hssrc="$1"
	local hsdst="$2"
	local hssrc_nsname

	hssrc_nsname="$(get_hsname "${hssrc}")"

	ip netns exec "${hssrc_nsname}" ping -c 1 -W "${PING_TIMEOUT_SEC}" \
		"${IPv6_HS_NETWORK}::${hsdst}" >/dev/null 2>&1
}

check_and_log_hs2gw_connectivity()
{
	local hssrc="$1"

	check_hs_ipv6_connectivity "${hssrc}" 254
	log_test $? 0 "IPv6 Hosts connectivity: hs-${hssrc} -> gw"
}

check_and_log_hs_ipv6_connectivity()
{
	local hssrc="$1"
	local hsdst="$2"

	check_hs_ipv6_connectivity "${hssrc}" "${hsdst}"
	log_test $? 0 "IPv6 Hosts connectivity: hs-${hssrc} -> hs-${hsdst}"
}

check_and_log_hs_connectivity()
{
	local hssrc="$1"
	local hsdst="$2"

	check_and_log_hs_ipv6_connectivity "${hssrc}" "${hsdst}"
}

check_and_log_hs_lo_addr_connectivity()
{
	local hssrc="$1"
	local hsdst="$2"
	local suffix="$3"
	local dst

	dst="${IPv6_HS_NETWORK}::${hsdst}:${suffix}"

	check_hs_ipv6_connectivity "${hssrc}" "${hsdst}:${suffix}"
	log_test $? 0 \
		"IPv6 Hosts connectivity: hs-${hssrc} -> hs-${hsdst} (${dst})"
}

router_tests()
{
	local i
	local j

	log_section "IPv6 routers connectivity test"

	for i in ${ROUTERS}; do
		for j in ${ROUTERS}; do
			if [ "${i}" -eq "${j}" ]; then
				continue
			fi

			check_and_log_rt_connectivity "${i}" "${j}"
		done
	done
}

host2gateway_tests()
{
	local hs

	log_section "IPv6 connectivity test among hosts and gateways"

	for hs in ${HOSTS}; do
		check_and_log_hs2gw_connectivity "${hs}"
	done
}

host_srv6_end_flv_psp_tests()
{
	log_section "SRv6 connectivity test hosts (h1 <-> h2, PSP flavor)"

	check_and_log_hs_connectivity 1 2
	check_and_log_hs_connectivity 2 1
}

host_srv6_end_flv_next_csid_psp_tests()
{
	log_section "SRv6 connectivity test hosts (h1 <-> h2, NEXT-C-SID,PSP)"

	check_and_log_hs_lo_addr_connectivity 1 2 "${END_CSID_PSP_FUNC}"
	check_and_log_hs_lo_addr_connectivity 2 1 "${END_CSID_PSP_FUNC}"
}

# On a router, add a route for a SID with seg6 encap in inline mode. The packets
# that the router forwards to that SID get an extra SRH with the End SID of the
# router that consumes it.
# args:
#  $1 - router on which the route is added
#  $2 - SID of the route
#  $3 - router with the SRv6 End behavior that consumes the extra SRH
setup_rt_consumed_srh()
{
	local rt="$1"
	local sid="$2"
	local end_rt="$3"
	local nsname

	nsname="$(get_rtname "${rt}")"

	ip -netns "${nsname}" -6 route \
		add "${sid}" table "${LOCALSID_TABLE_ID}" \
		encap seg6 mode inline \
		segs "${LOCATOR_SERVICE}:${end_rt}::${END_FUNC}" \
		dev "${DUMMY_DEVNAME}"
}

cleanup_rt_consumed_srh()
{
	local rt="$1"
	local sid="$2"
	local nsname

	nsname="$(get_rtname "${rt}")"

	ip -netns "${nsname}" -6 route \
		del "${sid}" table "${LOCALSID_TABLE_ID}"
}

set_hs_seg6_enabled()
{
	local val="$1"
	local hsname
	local hs

	for hs in ${HOSTS}; do
		hsname="$(get_hsname "${hs}")"

		ip netns exec "${hsname}" \
			sysctl -wq net.ipv6.conf.all.seg6_enabled="${val}"
		ip netns exec "${hsname}" \
			sysctl -wq net.ipv6.conf.veth0.seg6_enabled="${val}"
	done
}

# In a network namespace, accept the echo requests whose first SRH has the given
# SID as last segment (Segment List[0]), SL=0 and the ICMPv6 header after it,
# and drop any other echo request, including one without any SRH.
# args:
#  $1 - network namespace in which the rules are added
#  $2 - expected last segment of the first SRH
add_srh_check_rules()
{
	local nsname="$1"
	local sid="$2"

	ip netns exec "${nsname}" ip6tables -A INPUT \
		-p ipv6-icmp --icmpv6-type echo-request \
		-m srh --srh-lsid "${sid}" --srh-segs-left-eq 0 \
		--srh-next-hdr 58 -j ACCEPT &&
	ip netns exec "${nsname}" ip6tables -A INPUT \
		-p ipv6-icmp --icmpv6-type echo-request -j DROP
}

# On a host, check that the echo requests carry the extra SRH alone.
# args:
#  $1 - host on which the rules are added
#  $2 - expected last segment of the first SRH
setup_hs_srh_check()
{
	local hs="$1"
	local sid="$2"
	local hsname

	hsname="$(get_hsname "${hs}")"

	add_srh_check_rules "${hsname}" "${sid}"
}

cleanup_hs_srh_check()
{
	local hs="$1"
	local hsname

	hsname="$(get_hsname "${hs}")"

	ip netns exec "${hsname}" ip6tables -F INPUT
}

# Test a Policy with a consumed SRH: a router inserts the extra SRH on the
# packets towards a SID, and another router consumes it. The destination host
# accepts only the echo requests that carry the extra SRH alone.
# args:
#  $1 - router that inserts the extra SRH
#  $2 - SID that triggers the insertion
#  $3 - router with the SRv6 End behavior that consumes the extra SRH
#  $4 - source host
#  $5 - destination host
#  $6 - suffix of the address on the loopback of the destination host
#       (i.e. cafe::<host>:<suffix>), if any
check_and_log_consumed_srh()
{
	local rt="$1"
	local sid="$2"
	local end_rt="$3"
	local hssrc="$4"
	local hsdst="$5"
	local suffix="$6"
	local msg

	if ! ${HAS_SRH_MATCH}; then
		msg="IPv6 Hosts connectivity: hs-${hssrc} -> hs-${hsdst}"
		if [ -n "${suffix}" ]; then
			msg="${msg} (${IPv6_HS_NETWORK}::${hsdst}:${suffix})"
		fi
		log_test_skipped "${msg}"
		return
	fi

	if ! setup_rt_consumed_srh "${rt}" "${sid}" "${end_rt}"; then
		log_test 1 0 "Extra SRH on rt-${rt} for ${sid}"
		return
	fi

	if ! setup_hs_srh_check "${hsdst}" "${sid}"; then
		log_test 1 0 "SRH check on hs-${hsdst} for ${sid}"
	elif [ -n "${suffix}" ]; then
		check_and_log_hs_lo_addr_connectivity "${hssrc}" "${hsdst}" \
			"${suffix}"
	else
		check_and_log_hs_connectivity "${hssrc}" "${hsdst}"
	fi

	cleanup_hs_srh_check "${hsdst}"
	cleanup_rt_consumed_srh "${rt}" "${sid}"
}

host_srv6_end_flv_consumed_srh_tests()
{
	local sid

	log_section \
		"SRv6 connectivity test hosts (h1 <-> h2, PSP with a consumed SRH)"

	if ! ${HAS_SRH_MATCH}; then
		echo "SKIP: ip6tables srh match not available"
	fi

	set_hs_seg6_enabled 1

	# Policy (i.a): rt-4 inserts the extra SRH consumed by rt-3
	sid="${LOCATOR_SERVICE}:2::${END_PSP_FUNC}"
	check_and_log_consumed_srh 4 "${sid}" 3 1 2

	# Policy (i.b): rt-2 inserts the extra SRH consumed by rt-3
	sid="${LOCATOR_SERVICE}:1::${END_PSP_FUNC}"
	check_and_log_consumed_srh 2 "${sid}" 3 2 1

	# Policy (i.c): rt-4 inserts the extra SRH consumed by rt-3
	sid="$(build_csid_container 2 "${END_CSID_PSP_FUNC}")"
	check_and_log_consumed_srh 4 "${sid}" 3 1 2 "${END_CSID_PSP_FUNC}"

	# Policy (i.d): rt-3 inserts the extra SRH consumed by rt-4
	sid="$(build_csid_container 1 "${END_CSID_PSP_FUNC}")"
	check_and_log_consumed_srh 3 "${sid}" 4 2 1 "${END_CSID_PSP_FUNC}"

	set_hs_seg6_enabled 0
}

test_iproute2_supp_or_ksft_skip()
{
	local flavor="$1"

	if ! ip route help 2>&1 | grep -qo "${flavor}"; then
		echo "SKIP: Missing SRv6 ${flavor} flavor support in iproute2"
		exit "${ksft_skip}"
	fi
}

test_kernel_supp_or_ksft_skip()
{
	local flavor="$1"
	local test_netns

	test_netns="kflv-$(mktemp -u XXXXXXXX)"

	if ! ip netns add "${test_netns}"; then
		echo "SKIP: Cannot set up netns to test kernel support for flavors"
		exit "${ksft_skip}"
	fi

	if ! ip -netns "${test_netns}" link \
		add "${DUMMY_DEVNAME}" type dummy; then
		echo "SKIP: Cannot set up dummy dev to test kernel support for flavors"

		ip netns del "${test_netns}"
		exit "${ksft_skip}"
	fi

	if ! ip -netns "${test_netns}" link \
		set "${DUMMY_DEVNAME}" up; then
		echo "SKIP: Cannot activate dummy dev to test kernel support for flavors"

		ip netns del "${test_netns}"
		exit "${ksft_skip}"
	fi

	if ! ip -netns "${test_netns}" -6 route \
		add "${IPv6_TESTS_ADDR}" encap seg6local \
		action End flavors "${flavor}" dev "${DUMMY_DEVNAME}"; then
		echo "SKIP: ${flavor} flavor not supported in kernel"

		ip netns del "${test_netns}"
		exit "${ksft_skip}"
	fi

	ip netns del "${test_netns}"
}

# Before enabling the tests with a consumed SRH, make sure that the rules they
# use can be added.
check_srh_match_support()
{
	local srhm_ns

	if [ ! -x "$(command -v ip6tables)" ]; then
		return
	fi

	if ! setup_ns srhm_ns; then
		return
	fi

	if add_srh_check_rules "${srhm_ns}" "${IPv6_TESTS_ADDR}" \
		&>/dev/null; then
		HAS_SRH_MATCH=true
	fi

	cleanup_ns "${srhm_ns}"
}

test_dummy_dev_or_ksft_skip()
{
	local test_netns

	test_netns="dummy-$(mktemp -u XXXXXXXX)"

	if ! ip netns add "${test_netns}"; then
		echo "SKIP: Cannot set up netns for testing dummy dev support"
		exit "${ksft_skip}"
	fi

	modprobe dummy &>/dev/null || true
	if ! ip -netns "${test_netns}" link \
		add "${DUMMY_DEVNAME}" type dummy; then
		echo "SKIP: dummy dev not supported"

		ip netns del "${test_netns}"
		exit "${ksft_skip}"
	fi

	ip netns del "${test_netns}"
}

if [ "$(id -u)" -ne 0 ]; then
	echo "SKIP: Need root privileges"
	exit "${ksft_skip}"
fi

# required programs to carry out this selftest
test_command_or_ksft_skip ip
test_command_or_ksft_skip ping
test_command_or_ksft_skip sysctl
test_command_or_ksft_skip grep
test_command_or_ksft_skip cut
test_command_or_ksft_skip sed
test_command_or_ksft_skip sort
test_command_or_ksft_skip xargs

test_dummy_dev_or_ksft_skip
# NEXT-C-SID is not checked: every kernel and iproute2 version that
# supports PSP also supports NEXT-C-SID.
test_iproute2_supp_or_ksft_skip psp
test_kernel_supp_or_ksft_skip psp
check_srh_match_support

set -e
trap cleanup EXIT

setup
set +e

router_tests
host2gateway_tests
host_srv6_end_flv_psp_tests
host_srv6_end_flv_next_csid_psp_tests
host_srv6_end_flv_consumed_srh_tests

print_log_test_results
