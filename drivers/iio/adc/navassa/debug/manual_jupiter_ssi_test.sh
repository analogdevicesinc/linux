#!/usr/bin/env bash
set -euo pipefail

# Manual per-channel SSI delay set + full 2RX2TX PN15 test
#
# Usage:
#   ./manual_jupiter_ssi_test.sh <rx|tx> <chan:0|1> <clk> <strobe> <i_data> <q_data>
#
# Example:
#   ./manual_jupiter_ssi_test.sh rx 1 3 4 4 4

DEV_PHY="${DEV_PHY:-/sys/kernel/debug/iio/iio:device1}"
DEV_AXI="${DEV_AXI:-/sys/kernel/debug/iio/iio:device2}"
AXI_BACKEND="${AXI_BACKEND:-auto}"                 # auto | iio_reg | debugfs
AXI_IIO_DEV="${AXI_IIO_DEV:-axi-adrv9002-rx-lpc}"  # RX AXI core
IIO_REG_BIN="${IIO_REG_BIN:-iio_reg}"
FIXUP="${FIXUP:-1}"                               # mimic existing TX test reset path
SLEEP_SECS="${SLEEP_SECS:-0.05}"

SIDE="${1:-}"
CHAN="${2:-}"
CLK_DELAY="${3:-}"
STROBE_DELAY="${4:-}"
I_DATA_DELAY="${5:-}"
Q_DATA_DELAY="${6:-}"

die() { echo "error: $*" >&2; exit 1; }
req() { [[ -e "$1" ]] || die "missing: $1"; }
w() { printf '%s' "$2" > "$1"; }
r() { cat "$1"; }

usage() {
  cat <<'EOF'
Usage:
  ./manual_jupiter_ssi_test.sh <rx|tx> <chan:0|1> <clk> <strobe> <i_data> <q_data>

Applies delays only to the selected channel, then runs PN15 checks for:
  Rx 0, Rx 1, Tx 0, Tx 1
EOF
}

[[ $# -eq 6 ]] || { usage; die "expected 6 arguments"; }
[[ "$SIDE" == "rx" || "$SIDE" == "tx" ]] || die "side must be rx or tx"
[[ "$CHAN" == "0" || "$CHAN" == "1" ]] || die "channel must be 0 or 1"
for v in "$CLK_DELAY" "$STROBE_DELAY" "$I_DATA_DELAY" "$Q_DATA_DELAY"; do
  [[ "$v" =~ ^[0-7]$ ]] || die "delay values must be in range 0..7"
done

req "$DEV_PHY/ssi_delays"

DRA_MAGIC=$((0x80000000))
CHAN_STATUS=$((0x0404))
CHAN_CNTRL_3=$((0x0418))
CHAN_STRIDE=$((0x40))
PN_ERR=$((1 << 2))
PN_OOS=$((1 << 1))
OVR=$((1 << 0))
PN_MASK=$((PN_ERR | PN_OOS | OVR))
PN15_SEL=5

pcore_reg() { echo $((DRA_MAGIC | $1)); }

use_iio_reg_backend() {
  case "$AXI_BACKEND" in
    iio_reg) return 0 ;;
    debugfs) return 1 ;;
    auto)
      command -v "$IIO_REG_BIN" >/dev/null 2>&1 || return 1
      "$IIO_REG_BIN" "$AXI_IIO_DEV" 0x0 >/dev/null 2>&1
      ;;
    *)
      die "AXI_BACKEND must be auto, iio_reg, or debugfs"
      ;;
  esac
}

AXI_ACCESS_MODE=""
AXI_ACCESS_READY=0

init_axi_access() {
  if (( AXI_ACCESS_READY )); then
    return
  fi

  if [[ "$AXI_BACKEND" == "iio_reg" ]]; then
    command -v "$IIO_REG_BIN" >/dev/null 2>&1 || die "missing AXI backend tool: $IIO_REG_BIN"
    "$IIO_REG_BIN" "$AXI_IIO_DEV" 0x0 >/dev/null 2>&1 || \
      die "cannot access AXI IIO device via $IIO_REG_BIN: $AXI_IIO_DEV"
    AXI_ACCESS_MODE="iio_reg"
  elif use_iio_reg_backend; then
    AXI_ACCESS_MODE="iio_reg"
  else
    AXI_ACCESS_MODE="debugfs"
    req "$DEV_AXI/direct_reg_access"
  fi

  AXI_ACCESS_READY=1
}

reg_write() {
  local reg="$1" val="$2"
  init_axi_access
  if [[ "$AXI_ACCESS_MODE" == "iio_reg" ]]; then
    "$IIO_REG_BIN" "$AXI_IIO_DEV" \
      "$(printf '0x%X' "$reg")" \
      "$(printf '0x%X' "$val")" >/dev/null
  else
    printf "0x%X 0x%X" "$reg" "$val" > "$DEV_AXI/direct_reg_access"
  fi
}

reg_read() {
  local reg="$1"
  init_axi_access
  if [[ "$AXI_ACCESS_MODE" == "iio_reg" ]]; then
    "$IIO_REG_BIN" "$AXI_IIO_DEV" "$(printf '0x%X' "$reg")" | tr -d '[:space:]'
  else
    printf "0x%X" "$reg" > "$DEV_AXI/direct_reg_access"
    tr -d '[:space:]' < "$DEV_AXI/direct_reg_access"
  fi
}

reg_write_pcore() { reg_write "$(pcore_reg "$1")" "$2"; }
reg_read_pcore()  { reg_read  "$(pcore_reg "$1")"; }

restore_patterns() {
  local ch
  for ch in rx0 rx1 tx0 tx1; do
    w "$DEV_PHY/${ch}_ssi_test_mode_data" "TESTMODE_DATA_NORMAL"
    w "$DEV_PHY/${ch}_ssi_test_mode_configure" 1
  done
}

apply_selected_delays() {
  local prefix="${SIDE}${CHAN}"
  w "$DEV_PHY/${prefix}_ssi_clk_delay" "$CLK_DELAY"
  w "$DEV_PHY/${prefix}_ssi_strobe_delay" "$STROBE_DELAY"
  w "$DEV_PHY/${prefix}_ssi_i_data_delay" "$I_DATA_DELAY"
  w "$DEV_PHY/${prefix}_ssi_q_data_delay" "$Q_DATA_DELAY"
  w "$DEV_PHY/ssi_delays" 1
}

tx_test_prepare_channel() {
  local ch="$1"
  if [[ "$FIXUP" == "1" ]]; then
    w "$DEV_PHY/tx${ch}_ssi_test_mode_data" "TESTMODE_DATA_FIXED_PATTERN"
    w "$DEV_PHY/tx${ch}_ssi_test_mode_configure" 1
    r "$DEV_PHY/tx${ch}_ssi_test_mode_status" >/dev/null
  fi
  w "$DEV_PHY/tx${ch}_ssi_test_mode_data" "TESTMODE_DATA_PRBS15"
  w "$DEV_PHY/tx${ch}_ssi_test_mode_configure" 1
}

run_rx_tests() {
  local ch c reg v

  for ch in 0 1; do
    w "$DEV_PHY/rx${ch}_ssi_test_mode_data" "TESTMODE_DATA_PRBS15"
    w "$DEV_PHY/rx${ch}_ssi_test_mode_configure" 1
  done

  for c in 0 1 2 3; do
    reg=$((CHAN_CNTRL_3 + CHAN_STRIDE * c))
    reg_write_pcore "$reg" $((PN15_SEL << 16))
  done

  for c in 0 1 2 3; do
    reg=$((CHAN_STATUS + CHAN_STRIDE * c))
    reg_write_pcore "$reg" "$PN_MASK"
  done

  sleep "$SLEEP_SECS"

  rx0_pass=1
  rx1_pass=1
  for c in 0 1 2 3; do
    reg=$((CHAN_STATUS + CHAN_STRIDE * c))
    v="$(reg_read_pcore "$reg")"
    if (( v != 0 )); then
      if (( c < 2 )); then
        rx0_pass=0
      else
        rx1_pass=0
      fi
    fi
  done
}

tx_status_ok() {
  local ch="$1"
  local status de
  status="$(r "$DEV_PHY/tx${ch}_ssi_test_mode_status")"
  de="$(awk '/dataError/ {print $2}' <<<"$status")"
  [[ "${de:-1}" == "0" ]]
}

run_tx_tests() {
  tx_test_prepare_channel 0
  tx_test_prepare_channel 1
  sleep "$SLEEP_SECS"

  if tx_status_ok 0; then
    tx0_pass=1
  else
    tx0_pass=0
  fi

  if tx_status_ok 1; then
    tx1_pass=1
  else
    tx1_pass=0
  fi
}

print_result() {
  local label="$1" ok="$2"
  if (( ok )); then
    echo "$label pass"
  else
    echo "$label fail"
  fi
}

req "$DEV_PHY/rx0_ssi_test_mode_data"
req "$DEV_PHY/rx0_ssi_test_mode_configure"
req "$DEV_PHY/rx1_ssi_test_mode_data"
req "$DEV_PHY/rx1_ssi_test_mode_configure"
req "$DEV_PHY/tx0_ssi_test_mode_data"
req "$DEV_PHY/tx0_ssi_test_mode_configure"
req "$DEV_PHY/tx1_ssi_test_mode_data"
req "$DEV_PHY/tx1_ssi_test_mode_configure"
req "$DEV_PHY/tx0_ssi_test_mode_status"
req "$DEV_PHY/tx1_ssi_test_mode_status"

trap restore_patterns EXIT
apply_selected_delays
run_rx_tests
run_tx_tests

echo "Applied ${SIDE}${CHAN} delays: clk=$CLK_DELAY strobe=$STROBE_DELAY i=$I_DATA_DELAY q=$Q_DATA_DELAY"
print_result "Rx 0" "$rx0_pass"
print_result "Rx 1" "$rx1_pass"
print_result "Tx 0" "$tx0_pass"
print_result "Tx 1" "$tx1_pass"
