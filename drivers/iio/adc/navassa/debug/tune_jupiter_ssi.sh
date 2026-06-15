
# How to run on your Jupiter SDR system
#
#   1. RX + TX tuning (default):
#
#      ./tune_jupiter_ssi.sh
#
#   2. RX only:
#
#      ./tune_jupiter_ssi.sh rx
#
#   3. TX only:
#
#      ./tune_jupiter_ssi.sh tx
#
#   4. Dry run (report only):
#
#      APPLY=0 ./tune_jupiter_ssi.sh
#
#   5. 2RX2TX independent channel debug:
#
#      MIRROR_RX2TX2=0 ./tune_jupiter_ssi.sh rx 1


#!/usr/bin/env bash
set -euo pipefail

# --- configuration (updated for your setup) ---
DEV_PHY="${DEV_PHY:-/sys/kernel/debug/iio/iio:device1}"   # ADRV9002 debugfs
DEV_AXI="${DEV_AXI:-/sys/kernel/debug/iio/iio:device2}"   # AXI ADC core debugfs
AXI_BACKEND="${AXI_BACKEND:-auto}"                        # auto | iio_reg | debugfs
AXI_IIO_DEV="${AXI_IIO_DEV:-axi-adrv9002-rx-lpc}"         # used by iio_reg backend
IIO_REG_BIN="${IIO_REG_BIN:-iio_reg}"
MODE="${1:-both}"                                        # rx | tx | both
CHAN="${2:-0}"                                           # 0 or 1
RX2TX2="${RX2TX2:-1}"                                    # you are using rx2tx2
MIRROR_RX2TX2="${MIRROR_RX2TX2:-1}"                      # 1=copy CHAN0 delays to CHAN1, 0=tune CHAN independently
APPLY="${APPLY:-1}"                                      # 1=apply final delays, 0=just report
VERBOSE="${VERBOSE:-1}"
FIXUP="${FIXUP:-1}"                                      # TX fixup (matches driver behavior)
REPORT_ONLY="${REPORT_ONLY:-0}"                          # internal: report table without keeping scanned delays
PRINT_SECONDARY_TABLES="${PRINT_SECONDARY_TABLES:-1}"    # internal: print CHAN1 tables in independent 2RX2TX mode

DEBUG="${DEBUG:-2}"                                      # TX fixup (matches driver behavior)

if [[ $DEBUG == 1 ]]; then
  echo DEV_PHY  $DEV_PHY
  echo DEV_AXI  $DEV_AXI
  echo AXI_BACKEND $AXI_BACKEND
  echo AXI_IIO_DEV $AXI_IIO_DEV
  echo MODE     $MODE
  echo CHAN     $CHAN
  echo RX2TX2   $RX2TX2
  echo MIRROR_RX2TX2 $MIRROR_RX2TX2
  echo APPLY    $APPLY
  echo VERBOSE  $VERBOSE
  echo FIXUP    $FIXUP
fi

die() { echo "error: $*" >&2; exit 1; }
req() { [[ -e "$1" ]] || die "missing: $1"; }
w() { printf '%s' "$2" > "$1"; }
r() { cat "$1"; }
mirror_rx2tx2() { (( RX2TX2 )) && (( MIRROR_RX2TX2 )); }
secondary_report_enabled() {
  (( PRINT_SECONDARY_TABLES )) && (( RX2TX2 )) && ! mirror_rx2tx2 && [[ "$CHAN" == "0" ]] && [[ "$MODE" == "both" ]]
}

report_second_channel() {
  local mode="$1"
  local saved_chan="$CHAN"
  local saved_apply="$APPLY"
  local saved_report_only="$REPORT_ONLY"
  local saved_print_secondary="$PRINT_SECONDARY_TABLES"

  CHAN=1
  APPLY=0
  REPORT_ONLY=1
  PRINT_SECONDARY_TABLES=0

  echo
  echo "Reporting ${mode^^} table for independent CHAN=1"
  if [[ "$mode" == "rx" ]]; then
    rx_tune
  else
    tx_tune
  fi

  CHAN="$saved_chan"
  APPLY="$saved_apply"
  REPORT_ONLY="$saved_report_only"
  PRINT_SECONDARY_TABLES="$saved_print_secondary"
}

setup_rx_axi_window() {
  if (( RX2TX2 )); then
    base=0
    if mirror_rx2tx2; then
      start_chan=0
      end_chan=4
    else
      start_chan=$((CHAN * 2))
      end_chan=$((start_chan + 2))
    fi
  else
    base=$(( CHAN ? RX2_BASE : 0 ))
    start_chan=0
    end_chan=2
  fi
}

[[ "$CHAN" == "0" || "$CHAN" == "1" ]] || die "CHAN must be 0 or 1"
if (( RX2TX2 )) && (( MIRROR_RX2TX2 )) && [[ "$CHAN" != "0" ]]; then
  die "CHAN=1 in RX2TX2 mode requires MIRROR_RX2TX2=0"
fi

req "$DEV_PHY/ssi_delays"

# --- AXI ADC register helpers ---
DRA_MAGIC=$((0x80000000))
RX2_BASE=$((0x1000))
CHAN_STATUS=$((0x0404))
CHAN_CNTRL_3=$((0x0418))
CHAN_STRIDE=$((0x40))
PN_ERR=$((1<<2))  # 0x4
PN_OOS=$((1<<1))  # 0x2
OVR=$((1))  # 0x1
PN_MASK=$((PN_ERR | PN_OOS | OVR))

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

# --- tuning utilities ---
pick_pattern() {
  local avail_file="$1"
  local avail
  avail="$(r "$avail_file")"
  if grep -q "TESTMODE_DATA_PRBS15" <<<"$avail"; then echo "TESTMODE_DATA_PRBS15"; return; fi
  if grep -q "TESTMODE_DATA_RAMP_NIBBLE" <<<"$avail"; then echo "TESTMODE_DATA_RAMP_NIBBLE"; return; fi
  if grep -q "TESTMODE_DATA_PRBS7" <<<"$avail"; then echo "TESTMODE_DATA_PRBS7"; return; fi
  if grep -q "TESTMODE_DATA_RAMP_16_BIT" <<<"$avail"; then echo "TESTMODE_DATA_RAMP_16_BIT"; return; fi
  die "no usable test pattern found in $avail_file"
}

pn_sel_from_pattern() {
  case "$1" in
    TESTMODE_DATA_RAMP_NIBBLE) echo 10 ;; # ADC_PN_RAMP_NIBBLE
    TESTMODE_DATA_PRBS15)      echo 5  ;; # ADC_PN15
    TESTMODE_DATA_PRBS7)       echo 4  ;; # ADC_PN7
    TESTMODE_DATA_RAMP_16_BIT) echo 11 ;; # ADC_PN_RAMP_16
    *) die "pattern not supported for AXI PN monitor: $1" ;;
  esac
}

apply_delays() {
  local mode="$1" clk="$2" data="$3"
  if [[ "$mode" == "tx" ]]; then
    w "$DEV_PHY/tx${CHAN}_ssi_clk_delay"     "$clk"
    w "$DEV_PHY/tx${CHAN}_ssi_strobe_delay"  "$data"
    w "$DEV_PHY/tx${CHAN}_ssi_i_data_delay"  "$data"
    w "$DEV_PHY/tx${CHAN}_ssi_q_data_delay"  "$data"
    if mirror_rx2tx2; then
      w "$DEV_PHY/tx1_ssi_clk_delay"     "$clk"
      w "$DEV_PHY/tx1_ssi_strobe_delay"  "$data"
      w "$DEV_PHY/tx1_ssi_i_data_delay"  "$data"
      w "$DEV_PHY/tx1_ssi_q_data_delay"  "$data"
    fi
  else
    w "$DEV_PHY/rx${CHAN}_ssi_clk_delay"     "$clk"
    w "$DEV_PHY/rx${CHAN}_ssi_strobe_delay"  "$data"
    w "$DEV_PHY/rx${CHAN}_ssi_i_data_delay"  "$data"
    w "$DEV_PHY/rx${CHAN}_ssi_q_data_delay"  "$data"

    if mirror_rx2tx2; then
      w "$DEV_PHY/rx1_ssi_clk_delay"     "$clk"
      w "$DEV_PHY/rx1_ssi_strobe_delay"  "$data"
      w "$DEV_PHY/rx1_ssi_i_data_delay"  "$data"
      w "$DEV_PHY/rx1_ssi_q_data_delay"  "$data"
    fi
  fi
  w "$DEV_PHY/ssi_delays" 1
}

declare -A ssi_delays_snapshot

snapshot_ssi_delays() {
  ssi_delays_snapshot=()
  while IFS=': ' read -r key val; do
    [[ -n "${key:-}" ]] && ssi_delays_snapshot["$key"]="$val"
  done < "$DEV_PHY/ssi_delays"
}

save_tx_delays() {
  snapshot_ssi_delays
  saved_tx_clk="${ssi_delays_snapshot[tx${CHAN}_ClkDelay]}"
  saved_tx_refclk="${ssi_delays_snapshot[tx${CHAN}_RefClkDelay]}"
  saved_tx_strobe="${ssi_delays_snapshot[tx${CHAN}_StrobeDelay]}"
  saved_tx_i="${ssi_delays_snapshot[tx${CHAN}_rxIDataDelay]}"
  saved_tx_q="${ssi_delays_snapshot[tx${CHAN}_rxQDataDelay]}"
  if mirror_rx2tx2; then
    saved_tx1_clk="${ssi_delays_snapshot[tx1_ClkDelay]}"
    saved_tx1_refclk="${ssi_delays_snapshot[tx1_RefClkDelay]}"
    saved_tx1_strobe="${ssi_delays_snapshot[tx1_StrobeDelay]}"
    saved_tx1_i="${ssi_delays_snapshot[tx1_rxIDataDelay]}"
    saved_tx1_q="${ssi_delays_snapshot[tx1_rxQDataDelay]}"
  fi
}

restore_tx_delays() {
  w "$DEV_PHY/tx${CHAN}_ssi_clk_delay" "$saved_tx_clk"
  if [[ -n "${saved_tx_refclk:-}" && -e "$DEV_PHY/tx${CHAN}_ssi_refclk_delay" ]]; then
    w "$DEV_PHY/tx${CHAN}_ssi_refclk_delay" "$saved_tx_refclk"
  fi
  w "$DEV_PHY/tx${CHAN}_ssi_strobe_delay" "$saved_tx_strobe"
  w "$DEV_PHY/tx${CHAN}_ssi_i_data_delay" "$saved_tx_i"
  w "$DEV_PHY/tx${CHAN}_ssi_q_data_delay" "$saved_tx_q"
  if mirror_rx2tx2; then
    w "$DEV_PHY/tx1_ssi_clk_delay" "$saved_tx1_clk"
    if [[ -n "${saved_tx1_refclk:-}" && -e "$DEV_PHY/tx1_ssi_refclk_delay" ]]; then
      w "$DEV_PHY/tx1_ssi_refclk_delay" "$saved_tx1_refclk"
    fi
    w "$DEV_PHY/tx1_ssi_strobe_delay" "$saved_tx1_strobe"
    w "$DEV_PHY/tx1_ssi_i_data_delay" "$saved_tx1_i"
    w "$DEV_PHY/tx1_ssi_q_data_delay" "$saved_tx1_q"
  fi
  w "$DEV_PHY/ssi_delays" 1

  echo "restore tx - $saved_tx_clk"
  echo "restore tx - $saved_tx_strobe"
  echo "restore tx - $saved_tx_i"
  echo "restore tx - $saved_tx_q"
}

save_rx_delays() {
  snapshot_ssi_delays
  saved_rx_clk="${ssi_delays_snapshot[rx${CHAN}_ClkDelay]}"
  saved_rx_strobe="${ssi_delays_snapshot[rx${CHAN}_StrobeDelay]}"
  saved_rx_i="${ssi_delays_snapshot[rx${CHAN}_rxIDataDelay]}"
  saved_rx_q="${ssi_delays_snapshot[rx${CHAN}_rxQDataDelay]}"
  if mirror_rx2tx2; then
    saved_rx1_clk="${ssi_delays_snapshot[rx1_ClkDelay]}"
    saved_rx1_strobe="${ssi_delays_snapshot[rx1_StrobeDelay]}"
    saved_rx1_i="${ssi_delays_snapshot[rx1_rxIDataDelay]}"
    saved_rx1_q="${ssi_delays_snapshot[rx1_rxQDataDelay]}"
  fi
}

restore_rx_delays() {

  echo "restore rx - $saved_rx_clk"
  echo "restore rx - $saved_rx_strobe"
  echo "restore rx - $saved_rx_i"
  echo "restore rx - $saved_rx_q"
  w "$DEV_PHY/rx${CHAN}_ssi_clk_delay"    "$saved_rx_clk"
  w "$DEV_PHY/rx${CHAN}_ssi_strobe_delay" "$saved_rx_strobe"
  w "$DEV_PHY/rx${CHAN}_ssi_i_data_delay" "$saved_rx_i"
  w "$DEV_PHY/rx${CHAN}_ssi_q_data_delay" "$saved_rx_q"
  if mirror_rx2tx2; then
    w "$DEV_PHY/rx1_ssi_clk_delay"     "$saved_rx1_clk"
    w "$DEV_PHY/rx1_ssi_strobe_delay"  "$saved_rx1_strobe"
    w "$DEV_PHY/rx1_ssi_i_data_delay"  "$saved_rx1_i"
    w "$DEV_PHY/rx1_ssi_q_data_delay"  "$saved_rx1_q"
  fi
  w "$DEV_PHY/ssi_delays" 1
}

print_grid() {
  local label="$1" array_name="${2:-field}"
  echo "$label"
  echo "  0:1:2:3:4:5:6:7"
  for clk in {0..7}; do
    printf "%d:" "$clk"
    for data in {0..7}; do
      local idx=$((clk*8+data))
      local v
      eval "v=\${${array_name}[$idx]-1}"
      [[ "$v" == "0" ]] && printf "o " || printf "# "
    done
    printf "\n"
  done
  echo
}

best_eye() {
  local best_clk=0 best_start=0 best_len=0
  for clk in {0..7}; do
    local start=-1 cnt=0 max_cnt=0 max_start=0
    for data in {0..7}; do
      local v="${field[$((clk*8+data))]}"
      if [[ "$v" == "0" ]]; then
        [[ $start -lt 0 ]] && start="$data"
        ((cnt++))
      else
        if (( cnt > max_cnt )); then max_cnt="$cnt"; max_start="$start"; fi
        start=-1; cnt=0
      fi
    done
    if (( cnt > max_cnt )); then max_cnt="$cnt"; max_start="$start"; fi
    if (( max_cnt > best_len )); then
      best_len="$max_cnt"; best_clk="$clk"; best_start="$max_start"
    fi
  done

  if (( best_len == 0 )); then
    echo "none"
  else
    local best_data=$((best_start + best_len / 2))
    echo "$best_clk $best_data $best_len"
  fi
}

# --- TX tuning ---
tx_tune() {
  req "$DEV_PHY/tx${CHAN}_ssi_test_mode_data"
  req "$DEV_PHY/tx${CHAN}_ssi_test_mode_configure"
  req "$DEV_PHY/tx${CHAN}_ssi_test_mode_status"

  save_tx_delays

  local pattern
  pattern="$(pick_pattern "$DEV_PHY/tx_ssi_test_mode_data_available")"
  local saved_pattern
  saved_pattern="$(r "$DEV_PHY/tx${CHAN}_ssi_test_mode_data")"
  if mirror_rx2tx2; then
    saved_pattern_tx1="$(r "$DEV_PHY/tx1_ssi_test_mode_data")"
  fi
  restore_tx_pattern() {
    w "$DEV_PHY/tx${CHAN}_ssi_test_mode_data" "TESTMODE_DATA_NORMAL"
    w "$DEV_PHY/tx${CHAN}_ssi_test_mode_configure" 1
    w "$DEV_PHY/tx${CHAN}_ssi_test_mode_data" "$saved_pattern"
    w "$DEV_PHY/tx${CHAN}_ssi_test_mode_configure" 1
    if mirror_rx2tx2; then
      w "$DEV_PHY/tx1_ssi_test_mode_data" "TESTMODE_DATA_NORMAL"
      w "$DEV_PHY/tx1_ssi_test_mode_configure" 1
      w "$DEV_PHY/tx1_ssi_test_mode_data" "$saved_pattern_tx1"
      w "$DEV_PHY/tx1_ssi_test_mode_configure" 1
    fi
  }

  w "$DEV_PHY/tx${CHAN}_ssi_test_mode_data" "$pattern"
  w "$DEV_PHY/tx${CHAN}_ssi_test_mode_configure" 1
  if mirror_rx2tx2; then
    w "$DEV_PHY/tx1_ssi_test_mode_data" "$pattern"
    w "$DEV_PHY/tx1_ssi_test_mode_configure" 1
  fi

  declare -a field=()
  for clk in {0..7}; do
    for data in {0..7}; do
      apply_delays tx "$clk" "$data"

      if [[ "$FIXUP" == "1" ]]; then
        w "$DEV_PHY/tx${CHAN}_ssi_test_mode_data" "TESTMODE_DATA_FIXED_PATTERN"
        w "$DEV_PHY/tx${CHAN}_ssi_test_mode_configure" 1
        r "$DEV_PHY/tx${CHAN}_ssi_test_mode_status" >/dev/null
        w "$DEV_PHY/tx${CHAN}_ssi_test_mode_data" "$pattern"
      fi

      w "$DEV_PHY/tx${CHAN}_ssi_test_mode_configure" 1
      local status de
      status="$(r "$DEV_PHY/tx${CHAN}_ssi_test_mode_status")"
      de="$(awk '/dataError/ {print $2}' <<<"$status")"
      [[ "${de:-1}" == "0" ]] && field[$((clk*8+data))]=0 || field[$((clk*8+data))]=1
    done
  done

  (( VERBOSE )) && print_grid "TX$((CHAN+1))"

  local best
  best="$(best_eye)"
  if [[ "$best" == "none" ]]; then
    restore_tx_delays
    restore_tx_pattern
    die "TX tuning failed (no valid window)"
  fi

  read -r best_clk best_data best_len <<<"$best"
  (( VERBOSE )) && echo "TX best: clk=$best_clk data=$best_data (run_len=$best_len)"

  if (( APPLY )); then
    apply_delays tx "$best_clk" "$best_data"
  elif (( REPORT_ONLY )); then
    restore_tx_delays
  fi

  restore_tx_pattern

  if secondary_report_enabled; then
    report_second_channel tx
  fi
}

# --- RX tuning ---
rx_tune() {
  req "$DEV_PHY/rx${CHAN}_ssi_test_mode_data"
  req "$DEV_PHY/rx${CHAN}_ssi_test_mode_configure"

  save_rx_delays

  local pattern
  pattern="$(pick_pattern "$DEV_PHY/rx_ssi_test_mode_data_available")"
  local saved_pattern
  saved_pattern="$(r "$DEV_PHY/rx${CHAN}_ssi_test_mode_data")"
  if mirror_rx2tx2; then
    saved_pattern_rx1="$(r "$DEV_PHY/rx1_ssi_test_mode_data")"
  fi
  restore_rx_pattern() {
    w "$DEV_PHY/rx${CHAN}_ssi_test_mode_data" "TESTMODE_DATA_NORMAL"
    w "$DEV_PHY/rx${CHAN}_ssi_test_mode_configure" 1
    w "$DEV_PHY/rx${CHAN}_ssi_test_mode_data" "$saved_pattern"
    w "$DEV_PHY/rx${CHAN}_ssi_test_mode_configure" 1
    if mirror_rx2tx2; then
      w "$DEV_PHY/rx1_ssi_test_mode_data" "TESTMODE_DATA_NORMAL"
      w "$DEV_PHY/rx1_ssi_test_mode_configure" 1
      w "$DEV_PHY/rx1_ssi_test_mode_data" "$saved_pattern_rx1"
      w "$DEV_PHY/rx1_ssi_test_mode_configure" 1
    fi
  }

  # Configure ADRV9002 RX test mode once
  w "$DEV_PHY/rx${CHAN}_ssi_test_mode_data" "$pattern"
  w "$DEV_PHY/rx${CHAN}_ssi_test_mode_configure" 1
  if mirror_rx2tx2; then
    w "$DEV_PHY/rx1_ssi_test_mode_data" "$pattern"
    w "$DEV_PHY/rx1_ssi_test_mode_configure" 1
  fi

  local sel
  sel="$(pn_sel_from_pattern "$pattern")"

  init_axi_access
  (( VERBOSE )) && echo "RX PN pattern: $pattern (sel=$sel, axi=$AXI_ACCESS_MODE:$AXI_IIO_DEV)"
  (( VERBOSE )) && (( RX2TX2 )) && echo "RX2TX2 delay mode: $([[ "$MIRROR_RX2TX2" == "1" ]] && echo mirrored || echo independent)"

  local base start_chan end_chan
  setup_rx_axi_window
  (( VERBOSE > 1 )) && echo "RX AXI window: base=$(printf '0x%X' "$base") chans=${start_chan}..$((end_chan - 1))"

  # Program AXI ADC PN selection for all channels
  for ((c=start_chan; c<end_chan; c++)); do
    local reg=$((base + CHAN_CNTRL_3 + CHAN_STRIDE*c))
    reg_write_pcore "$reg" $((sel << 16))
  done

  declare -a field=()
  declare -a field_rx0=()
  declare -a field_rx1=()
  for clk in {0..7}; do
    for data in {0..7}; do
      apply_delays rx "$clk" "$data"

      # clear status
      for ((c=start_chan; c<end_chan; c++)); do
        local reg=$((base + CHAN_STATUS + CHAN_STRIDE*c))
        reg_write_pcore "$reg" "$PN_MASK"
	#printf "clear %x %x\n" "$reg" "$PN_MASK"
      done
      sleep 0.05

      local ok=1 ok_rx0=1 ok_rx1=1
      for ((c=start_chan; c<end_chan; c++)); do
        local reg=$((base + CHAN_STATUS + CHAN_STRIDE*c))
        local v
        v="$(reg_read_pcore "$reg")"
	#printf "status %x %x\n" "$reg" "$v"
        if (( v != 0 )); then
          ok=0
          if mirror_rx2tx2; then
            if (( c < 2 )); then
              ok_rx0=0
            else
              ok_rx1=0
            fi
          fi
        fi
      done

      [[ "$ok" == "1" ]] && field[$((clk*8+data))]=0 || field[$((clk*8+data))]=1
      if mirror_rx2tx2; then
        [[ "$ok_rx0" == "1" ]] && field_rx0[$((clk*8+data))]=0 || field_rx0[$((clk*8+data))]=1
        [[ "$ok_rx1" == "1" ]] && field_rx1[$((clk*8+data))]=0 || field_rx1[$((clk*8+data))]=1
      fi
    done
  done

  if (( VERBOSE )); then
    print_grid "RX$((CHAN+1))"
    if mirror_rx2tx2; then
      print_grid "RX1 Mirrored View" field_rx0
      print_grid "RX2 Mirrored View" field_rx1
    fi
  fi

  local best
  best="$(best_eye)"
  if [[ "$best" == "none" ]]; then
    restore_rx_delays
    restore_rx_pattern
    die "RX tuning failed (no valid window)"
  fi

  read -r best_clk best_data best_len <<<"$best"
  (( VERBOSE )) && echo "RX best: clk=$best_clk data=$best_data (run_len=$best_len)"

  if (( APPLY )); then
    apply_delays rx "$best_clk" "$best_data"
  elif (( REPORT_ONLY )); then
    restore_rx_delays
  fi

  restore_rx_pattern

  if secondary_report_enabled; then
    report_second_channel rx
  fi
}

case "$MODE" in
  rx)   rx_tune ;;
  tx)   tx_tune ;;
  both) rx_tune; tx_tune ;;
  *)    die "MODE must be rx, tx, or both" ;;
esac
