.. SPDX-License-Identifier: GPL-2.0

===================================================
AD9084/AD9088 Multi-Chip Synchronization (MCS) Architecture
===================================================

This document describes the Multi-Chip Synchronization (MCS) architecture
for AD9084/AD9088 Apollo devices, including driver interactions, signal
flows, and the complete synchronization process.

Overview
========

Multi-Chip Synchronization (MCS) enables multiple AD9084/AD9088 devices to
achieve ±10ps alignment accuracy. This is critical for phased array radar,
electronic warfare, and other applications requiring precise timing across
multiple data converters.

The MCS system consists of three main components:

1. **AD9084/AD9088 (Apollo)**: The mixed-signal front-end with integrated
   ADCs and DACs
2. **ADF4030**: Precision SYSREF/BSYNC generator with TDC (Time-to-Digital
   Converter) for path delay measurement
3. **ADF4382**: Low-noise PLL providing the sampling clock with automatic
   phase alignment capability

System Architecture
===================

Hardware Topology
-----------------

::

    +------------------+
    |    Reference     |
    |    Clock Source  |
    +--------+---------+
             |
             v
    +------------------+       +------------------+
    |     ADF4030      |       |     ADF4382      |
    |  SYSREF/BSYNC    |       |   Clock PLL      |
    |    Generator     |       |  (Auto-Align)    |
    +--------+---------+       +--------+---------+
             |                          |
             | BSYNC/SYSREF             | Sampling Clock
             |                          |
    +--------v--------------------------v---------+
    |                                             |
    |              AD9084/AD9088                  |
    |               (Apollo)                      |
    |                                             |
    |  +----------+  +----------+  +----------+  |
    |  |   ADC    |  |   DAC    |  |  JESD204 |  |
    |  |  Cores   |  |  Cores   |  | Interface|  |
    |  +----------+  +----------+  +----------+  |
    |                                             |
    |  GPIO ──────────────────────────────────────┼──► DELADJ/DELSTR
    |  (Phase adjustment strobes to ADF4382)      |    (to ADF4382)
    +---------------------------------------------+
             |
             v
    +------------------+
    |      FPGA        |
    |  (JESD204 + DMA) |
    +------------------+


Multi-Device Configuration
--------------------------

In a multi-device system, each Apollo device has its own dedicated ADF4382
and receives BSYNC from the ADF4030 through separate output channels.
Each Apollo controls its ADF4382 via GPIO-driven DELADJ (delay adjust)
and DELSTR (delay strobe) signals for phase correction:

::

    +------------------+
    |     ADF4030      |
    |                  |
    | CH0  CH1  CH2    |  (BSYNC outputs)
    +--+---+----+------+
       |   |    |
       |   |    +───────────────────────────────────────────────┐
       |   |                                                    |
       |   +─────────────────────────────────┐                  |
       |                                     |                  |
       v                                     v                  v
    +─────────────────────+  +─────────────────────+  +─────────────────────+
    |   Apollo Device 0   |  |   Apollo Device 1   |  |   Apollo Device 2   |
    |                     |  |                     |  |                     |
    |  GPIO─►┌─────────┐  |  |  GPIO─►┌─────────┐  |  |  GPIO─►┌─────────┐  |
    |  DELADJ│ ADF4382 │  |  |  DELADJ│ ADF4382 │  |  |  DELADJ│ ADF4382 │  |
    |  DELSTR│  (PLL)  │  |  |  DELSTR│  (PLL)  │  |  |  DELSTR│  (PLL)  │  |
    |        └────┬────┘  |  |        └────┬────┘  |  |        └────┬────┘  |
    |             │ CLK   |  |             │ CLK   |  |             │ CLK   |
    +─────────────┼───────+  +─────────────┼───────+  +─────────────┼───────+
                  │                        │                        │
                  v                        v                        v
              Sampling               Sampling                Sampling
               Clock                  Clock                   Clock


ADF4382 Phase Control via Apollo GPIO
-------------------------------------

Each Apollo device controls its dedicated ADF4382's phase adjustment through
two GPIO-driven strobe signals:

- **DELADJ (Delay Adjust)**: Sets the direction and magnitude of phase
  correction. Apollo firmware writes the bleed current values (polarity,
  coarse, fine) and pulses this strobe.

- **DELSTR (Delay Strobe)**: Triggers the ADF4382 to apply the phase
  adjustment. Each strobe pulse causes the PLL to adjust its output phase.

::

    Apollo Tracking Calibration Loop:

    ┌──────────────────────────────────────────────────────────────────┐
    │  Apollo Firmware                                                 │
    │  ┌─────────────────────────────────────────────────────────┐     │
    │  │ 1. Measure phase error via internal TDC                 │     │
    │  │ 2. Calculate correction (bleed_pol, coarse, fine)       │     │
    │  │ 3. Write correction values to internal registers        │     │
    │  │ 4. Pulse DELADJ GPIO ─────────────────────────────────────────┼──► ADF4382
    │  │ 5. Pulse DELSTR GPIO ─────────────────────────────────────────┼──► ADF4382
    │  │ 6. Repeat in background                                 │     │
    │  └─────────────────────────────────────────────────────────┘     │
    └──────────────────────────────────────────────────────────────────┘

This architecture allows each Apollo to independently maintain phase
alignment with its clock source, compensating for temperature drift and
other environmental factors without software intervention.


Linux Driver Architecture
=========================

Driver Stack
------------

::

    +----------------------------------------------------------+
    |                    User Space                             |
    |  (iio-oscilloscope, libiio applications)                  |
    +----------------------------------------------------------+
                              |
    ==========================|=================================
                              | sysfs / debugfs
                              v
    +----------------------------------------------------------+
    |                  IIO Subsystem                            |
    |  - Channel attributes (frequency, phase, enable)          |
    |  - Buffer management (DMA ring buffers)                   |
    |  - Event handling                                         |
    +----------------------------------------------------------+
           |              |              |              |
           v              v              v              v
    +-----------+  +-----------+  +-----------+  +-----------+
    |  ad9088   |  |  adf4030  |  |  adf4382  |  | axi_aion  |
    |  driver   |  |  driver   |  |  driver   |  |  driver   |
    +-----------+  +-----------+  +-----------+  +-----------+
           |              |              |              |
           v              v              v              v
    +----------------------------------------------------------+
    |              JESD204 FSM Framework                        |
    |  - Link state machine                                     |
    |  - Multi-device coordination                              |
    |  - Topology management                                    |
    +----------------------------------------------------------+
           |
           v
    +----------------------------------------------------------+
    |                    Hardware                               |
    +----------------------------------------------------------+


Driver File Organization
------------------------

**AD9088 (Apollo) Driver:**

::

    drivers/iio/adc/apollo/
    ├── ad9088.c              # Main driver, IIO interface
    ├── ad9088.h              # Driver header, structures
    ├── ad9088_dt.c           # Device tree parsing
    ├── ad9088_jesd204_fsm.c  # JESD204 state machine callbacks
    ├── ad9088_mcs.c          # MCS calibration functions
    ├── ad9088_debugfs.c      # Debugfs interface
    ├── ad9088_cal.c          # Calibration save/restore
    └── public/               # Apollo API (vendor library)

**Related Drivers:**

::

    drivers/iio/frequency/
    ├── adf4030.c             # ADF4030 SYSREF generator
    └── adf4382.c             # ADF4382 clock PLL

    drivers/iio/logic/
    └── axi_aion_trig.c       # FPGA trigger/sync controller


IIO Channel Relationships
-------------------------

The AD9088 driver connects to ADF4030 and ADF4382 via IIO consumer channels
defined in the device tree:

::

    Device Tree:
    ┌─────────────────────────────────────────────────────────┐
    │  ad9088@0 {                                             │
    │      io-channels = <&adf4030 5>, <&adf4382 0>;          │
    │      io-channel-names = "bsync", "clk";                 │
    │  };                                                     │
    └─────────────────────────────────────────────────────────┘

    Runtime Channel Access:
    ┌─────────────┐     iio_channel_get()      ┌─────────────┐
    │   ad9088    │ ◄──────────────────────────│   adf4030   │
    │             │    phy->iio_adf4030        │  (provider) │
    │ (consumer)  │                            └─────────────┘
    │             │     iio_channel_get()      ┌─────────────┐
    │             │ ◄──────────────────────────│   adf4382   │
    └─────────────┘    phy->iio_adf4382        │  (provider) │
                                               └─────────────┘

    API Usage in ad9088_jesd204_fsm.c:
    - iio_read_channel_attribute()   # Read phase, frequency
    - iio_write_channel_attribute()  # Write phase offset
    - iio_read_channel_ext_info()    # Read extended attributes
    - iio_write_channel_ext_info()   # Write extended attributes


JESD204 State Machine Integration
=================================

The MCS calibration is integrated into the JESD204 FSM framework, which
coordinates initialization across all devices in the JESD204 link.

FSM Stage Execution Order
-------------------------

::

    JESD204 State Machine Stages (simplified):

    ┌──────────────────────────────────────────────────────────────┐
    │                    DEVICE_INIT                                │
    └──────────────────────────────────────────────────────────────┘
                                │
                                v
    ┌──────────────────────────────────────────────────────────────┐
    │                    LINK_INIT                                  │
    └──────────────────────────────────────────────────────────────┘
                                │
                                v
    ┌──────────────────────────────────────────────────────────────┐
    │                   CLK_SYNC_STAGE1-3                           │
    │  - ADF4030: Initial BSYNC output alignment                    │
    │    (adf4030_jesd204_clks_sync3)                               │
    └──────────────────────────────────────────────────────────────┘
                                │
                                v
    ┌──────────────────────────────────────────────────────────────┐
    │                   LINK_ENABLE                                 │
    └──────────────────────────────────────────────────────────────┘
                                │
                                v
    ┌──────────────────────────────────────────────────────────────┐
    │                   LINK_RUNNING                                │
    └──────────────────────────────────────────────────────────────┘
                                │
                                v
    ┌──────────────────────────────────────────────────────────────┐
    │               OPT_POST_SETUP_STAGE1                           │
    │  - AD9088: BSYNC ToF measurement & MCS calibration            │
    │    (ad9088_jesd204_post_setup_stage1)                         │
    └──────────────────────────────────────────────────────────────┘
                                │
                                v
    ┌──────────────────────────────────────────────────────────────┐
    │               OPT_POST_SETUP_STAGE2                           │
    │  - AD9088: Trigger sync setup                                 │
    │  - AION: FPGA BSYNC ToF measurement                           │
    │    (axi_aion_jesd204_opt_setup_stage1)                        │
    └──────────────────────────────────────────────────────────────┘
                                │
                                v
    ┌──────────────────────────────────────────────────────────────┐
    │               OPT_POST_RUNNING_STAGE                          │
    │  - AD9088: Trigger phase validation                           │
    │    (ad9088_jesd204_post_setup_stage4)                         │
    └──────────────────────────────────────────────────────────────┘


AD9088 JESD204 Callbacks
------------------------

.. code-block:: c

    static const struct jesd204_dev_data ad9088_jesd204_data = {
        .state_ops = {
            [JESD204_OP_OPT_POST_SETUP_STAGE1] = {
                .per_device = ad9088_jesd204_post_setup_stage1,  // MCS ToF + Init Cal
            },
            [JESD204_OP_OPT_POST_SETUP_STAGE2] = {
                .per_device = ad9088_jesd204_post_setup_stage2,  // Trigger setup
            },
            [JESD204_OP_OPT_POST_SETUP_STAGE3] = {
                .per_device = ad9088_jesd204_post_setup_stage3,  // Trigger pulse
            },
            [JESD204_OP_OPT_POST_RUNNING_STAGE] = {
                .per_device = ad9088_jesd204_post_setup_stage4,  // Phase validation
            },
        },
    };


BSYNC Time-of-Flight Measurement
================================

The ToF measurement determines the propagation delay between ADF4030 and
Apollo, allowing precise phase compensation.

Measurement Principle
---------------------

::

    ┌─────────────────────────────────────────────────────────────────┐
    │                    Delta-T0 Measurement                         │
    │                  (ADF4030 drives BSYNC)                         │
    ├─────────────────────────────────────────────────────────────────┤
    │                                                                 │
    │   ADF4030                          Apollo                       │
    │  ┌───────┐                        ┌───────┐                     │
    │  │ TDC   │ ──── BSYNC_OUT ──────► │ TDC   │                     │
    │  │       │      (t0_out)          │       │                     │
    │  └───────┘                        └───────┘                     │
    │                                                                 │
    │  adf4030_delta_t0 = TDC reading   apollo_delta_t0 = TDC reading │
    │                                                                 │
    └─────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────────┐
    │                    Delta-T1 Measurement                         │
    │                   (Apollo drives BSYNC)                         │
    ├─────────────────────────────────────────────────────────────────┤
    │                                                                 │
    │   ADF4030                          Apollo                       │
    │  ┌───────┐                        ┌───────┐                     │
    │  │ TDC   │ ◄──── BSYNC_IN ─────── │ BSYNC │                     │
    │  │       │      (t1_in)           │  OUT  │                     │
    │  └───────┘                        └───────┘                     │
    │                                                                 │
    │  adf4030_delta_t1 = TDC reading   apollo_delta_t1 = TDC reading │
    │                                                                 │
    └─────────────────────────────────────────────────────────────────┘


Path Delay Calculation
----------------------

The path delay is calculated from the four TDC measurements:

.. code-block:: c

    // From ad9088_jesd204_fsm.c
    calc_delay = (adf4030_delta_t0 - adf4030_delta_t1)
               - (apollo_delta_t1 - apollo_delta_t0);

    // Handle wraparound with modulo arithmetic
    round_trip_delay = (calc_delay + bsync_period) % bsync_period;

    // One-way path delay is half the round trip
    path_delay = round_trip_delay / 2;

    // Apply negative offset to compensate
    iio_write_channel_attribute(adf4030, -path_delay, IIO_CHAN_INFO_PHASE);

**Mathematical Derivation:**

::

    Let:
      T_fwd = Forward path delay (ADF4030 → Apollo)
      T_rev = Reverse path delay (Apollo → ADF4030)

    Delta-T0 measurement (ADF4030 transmits):
      adf4030_delta_t0 = reference time at ADF4030
      apollo_delta_t0  = adf4030_delta_t0 + T_fwd

    Delta-T1 measurement (Apollo transmits):
      apollo_delta_t1  = reference time at Apollo
      adf4030_delta_t1 = apollo_delta_t1 + T_rev

    Solving for round-trip delay:
      (adf4030_delta_t0 - adf4030_delta_t1) = forward contribution
      (apollo_delta_t1 - apollo_delta_t0)   = reverse contribution

      calc_delay = T_fwd + T_rev = round_trip_delay
      path_delay = round_trip_delay / 2  (assuming symmetric paths)


Complete MCS Flow Sequence
==========================

The complete MCS flow executed in ``ad9088_jesd204_post_setup_stage1()``:

::

    ┌─────────────────────────────────────────────────────────────────┐
    │                     MCS Calibration Flow                        │
    └─────────────────────────────────────────────────────────────────┘
                                  │
                                  v
    ┌─────────────────────────────────────────────────────────────────┐
    │ 1. Initialize MCS Configuration                                 │
    │    - ad9088_mcs_init_cal_config()                               │
    │    - Set offset parameters (C for single, A/B for dual clock)   │
    └─────────────────────────────────────────────────────────────────┘
                                  │
                                  v
    ┌─────────────────────────────────────────────────────────────────┐
    │ 2. Delta-T0 Measurement                                         │
    │    - Enable ADF4030 BSYNC output                                │
    │    - ad9088_delta_t_measurement_set(phy, 0)                     │
    │    - Read ADF4030 phase (adf4030_delta_t0)                      │
    │    - Read Apollo delta_t (apollo_delta_t0)                      │
    └─────────────────────────────────────────────────────────────────┘
                                  │
                                  v
    ┌─────────────────────────────────────────────────────────────────┐
    │ 3. Delta-T1 Measurement                                         │
    │    - Disable ADF4030 BSYNC output (Apollo drives)               │
    │    - ad9088_delta_t_measurement_set(phy, 1)                     │
    │    - Read ADF4030 phase (adf4030_delta_t1)                      │
    │    - Read Apollo delta_t (apollo_delta_t1)                      │
    └─────────────────────────────────────────────────────────────────┘
                                  │
                                  v
    ┌─────────────────────────────────────────────────────────────────┐
    │ 4. Calculate & Apply Path Delay                                 │
    │    - calc_delay = (adf4030_t0 - adf4030_t1) - (apollo_t1 - t0)  │
    │    - path_delay = ((calc_delay + period) % period) / 2          │
    │    - Write -path_delay to ADF4030 phase                         │
    └─────────────────────────────────────────────────────────────────┘
                                  │
                                  v
    ┌─────────────────────────────────────────────────────────────────┐
    │ 5. Run MCS Init Calibration                                     │
    │    - adi_apollo_mcs_cal_init_run()                              │
    │    - Aligns internal SYSREF to external SYSREF                  │
    └─────────────────────────────────────────────────────────────────┘
                                  │
                                  v
    ┌─────────────────────────────────────────────────────────────────┐
    │ 6. Validate Init Calibration                                    │
    │    - ad9088_mcs_init_cal_validate()                             │
    │    - Check timing difference < 0.4 clock cycles                 │
    │    - Check SYSREF lock status                                   │
    │    - Return error if validation fails                           │
    └─────────────────────────────────────────────────────────────────┘
                                  │
                          ┌───────┴───────┐
                          │ Dual Clock?   │
                          └───────┬───────┘
                           No     │     Yes
                    ┌─────────────┴─────────────┐
                    v                           v
    ┌───────────────────────────┐   ┌───────────────────────────┐
    │ 7a. Setup Tracking Cal    │   │ 7b. Skip Tracking Cal     │
    │   - ad9088_mcs_tracking_  │   │   (HW limitation)         │
    │     cal_setup()           │   │   Log informational msg   │
    │   - Uses DT decimation    │   └───────────────────────────┘
    │     or default 1023       │               │
    └───────────────────────────┘               │
                    │                           │
                    v                           │
    ┌───────────────────────────┐               │
    │ 8. Enable ADF4382         │               │
    │    Auto-Align             │               │
    │   - Write en_auto_align=1 │               │
    └───────────────────────────┘               │
                    │                           │
                    v                           │
    ┌───────────────────────────┐               │
    │ 9. Run FG Tracking Cal    │               │
    │   - adi_apollo_mcs_cal_   │               │
    │     fg_tracking_run()     │               │
    └───────────────────────────┘               │
                    │                           │
                    v                           │
    ┌───────────────────────────┐               │
    │ 10. Run BG Tracking Cal   │               │
    │   - adi_apollo_mcs_cal_   │               │
    │     bg_tracking_run()     │               │
    │   - Continuous operation  │               │
    └───────────────────────────┘               │
                    │                           │
                    └───────────┬───────────────┘
                                v
    ┌─────────────────────────────────────────────────────────────────┐
    │                    MCS Calibration Complete                     │
    └─────────────────────────────────────────────────────────────────┘


Tracking Calibration
====================

Tracking calibration maintains synchronization over time as temperature
and other environmental factors cause phase drift.

ADF4382 Bleed Current Adjustment
--------------------------------

The ADF4382 PLL uses a bleed current DAC to make fine phase adjustments.
Each Apollo controls its dedicated ADF4382 via GPIO-driven DELADJ and DELSTR
strobe signals:

::

    ┌─────────────────────────────────────────────────────────────────┐
    │                  ADF4382 Phase Adjustment                       │
    ├─────────────────────────────────────────────────────────────────┤
    │                                                                 │
    │   Apollo Firmware                     ADF4382 Hardware          │
    │   (tracks phase error)                (adjusts PLL phase)       │
    │                                                                 │
    │   ┌──────────────────┐                ┌──────────────────┐      │
    │   │ Bleed Polarity   │   DELADJ       │ Register 0x64-65 │      │
    │   │ Coarse Current   │ ──GPIO──────►  │ DEL_CNT field    │      │
    │   │ Fine Current     │   DELSTR       │ (latched on      │      │
    │   └──────────────────┘ ──GPIO──────►  │  strobe edge)    │      │
    │                                       └──────────────────┘      │
    │                                                                 │
    │   DELADJ: Sets direction/magnitude of phase adjustment          │
    │   DELSTR: Triggers ADF4382 to apply the phase correction        │
    │                                                                 │
    │   Values should match when tracking is synchronized             │
    │                                                                 │
    └─────────────────────────────────────────────────────────────────┘


Tracking Cal Validation
-----------------------

The ``mcs_track_cal_validate`` debugfs attribute compares Apollo firmware
values with ADF4382 hardware values:

.. code-block:: c

    // From ad9088_mcs.c
    int ad9088_mcs_tracking_cal_validate(struct ad9088_phy *phy, ...)
    {
        // Freeze tracking to get stable reading
        adi_apollo_mcs_cal_bg_tracking_freeze(device);

        // Read Apollo firmware values
        adi_apollo_mcs_cal_tracking_status_get(device, &tracking_cal_status);
        fw_bleed_pol = tracking_cal_status.data.adf4382_status->Bleed_Pol;
        fw_coarse = tracking_cal_status.data.adf4382_status->Coarse;
        fw_fine = tracking_cal_status.data.adf4382_status->Fine;

        // Read ADF4382 hardware values via IIO
        iio_read_channel_ext_info(adf4382, "bleed_pol", &hw_bleed_pol);
        iio_read_channel_ext_info(adf4382, "coarse_current", &hw_coarse);
        iio_read_channel_ext_info(adf4382, "fine_current", &hw_fine);

        // Compare and report
        if (fw_bleed_pol == hw_bleed_pol &&
            fw_coarse == hw_coarse &&
            fw_fine == hw_fine) {
            status = "SYNCHRONIZED";
        } else {
            status = "MISMATCH - check hardware";
        }

        adi_apollo_mcs_cal_bg_tracking_unfreeze(device);
    }


Single vs Dual Clock Mode
=========================

The AD9084/AD9088 supports two clocking configurations:

Single Clock Mode
-----------------

::

    ┌─────────────────────────────────────────────────────────────────┐
    │                    Single Clock Mode                            │
    ├─────────────────────────────────────────────────────────────────┤
    │                                                                 │
    │                    ┌───────────────┐                            │
    │                    │   ADF4382     │                            │
    │                    │  (Center PLL) │                            │
    │                    └───────┬───────┘                            │
    │                            │                                    │
    │                            v                                    │
    │              ┌─────────────────────────────┐                    │
    │              │         Apollo              │                    │
    │              │  ┌───────────────────────┐  │                    │
    │              │  │     Center Clock      │  │                    │
    │              │  │   (drives both sides) │  │                    │
    │              │  └───────────────────────┘  │                    │
    │              │    │               │        │                    │
    │              │    v               v        │                    │
    │              │ Side A          Side B      │                    │
    │              └─────────────────────────────┘                    │
    │                                                                 │
    │  - Uses offset_C_femtoseconds for calibration                   │
    │  - Checks is_C_Locked for lock status                           │
    │  - Tracking calibration supported                               │
    │                                                                 │
    └─────────────────────────────────────────────────────────────────┘


Dual Clock Mode
---------------

::

    ┌─────────────────────────────────────────────────────────────────┐
    │                     Dual Clock Mode                             │
    ├─────────────────────────────────────────────────────────────────┤
    │                                                                 │
    │      ┌───────────────┐         ┌───────────────┐                │
    │      │   ADF4382-A   │         │   ADF4382-B   │                │
    │      │  (Side A PLL) │         │  (Side B PLL) │                │
    │      └───────┬───────┘         └───────┬───────┘                │
    │              │                         │                        │
    │              v                         v                        │
    │  ┌───────────────────────────────────────────────────┐          │
    │  │                    Apollo                         │          │
    │  │   ┌─────────────┐         ┌─────────────┐         │          │
    │  │   │   Side A    │         │   Side B    │         │          │
    │  │   │   Clock     │         │   Clock     │         │          │
    │  │   └─────────────┘         └─────────────┘         │          │
    │  └───────────────────────────────────────────────────┘          │
    │                                                                 │
    │  - Uses offset_A_femtoseconds and offset_B_femtoseconds         │
    │  - Checks is_A_Locked and is_B_Locked for lock status           │
    │  - Tracking calibration NOT supported (HW limitation)          │
    │                                                                 │
    └─────────────────────────────────────────────────────────────────┘


Device Tree Configuration
=========================

Complete Example
----------------

.. code-block:: dts

    /* ADF4030 - SYSREF/BSYNC Generator */
    adf4030: adf4030@0 {
        compatible = "adi,adf4030";
        reg = <0>;
        spi-max-frequency = <1000000>;

        /* Channel configuration */
        #io-channel-cells = <1>;

        /* BSYNC output channels */
        adi,channel@0 {
            reg = <0>;
            adi,output-enable;
            adi,auto-align-on-sync-en;
        };
        adi,channel@5 {
            reg = <5>;
            adi,output-enable;
            adi,auto-align-on-sync-en;
            adi,reference-channel = <0>;  /* Align to channel 0 */
        };
    };

    /* ADF4382 - Clock PLL */
    adf4382: adf4382@1 {
        compatible = "adi,adf4382";
        reg = <1>;
        spi-max-frequency = <1000000>;

        #io-channel-cells = <1>;

        clocks = <&ref_clk>;
        clock-names = "ref_clk";

        adi,ref-frequency-hz = <125000000>;
        adi,output-frequency-hz = <12000000000>;
    };

    /* AD9088 - Apollo Device */
    trx0_ad9084: ad9088@0 {
        compatible = "adi,ad9088";
        reg = <0>;
        spi-max-frequency = <1000000>;

        /* MCS IIO channel connections */
        io-channels = <&adf4030 5>, <&adf4382 0>;
        io-channel-names = "bsync", "clk";

        /* MCS Configuration */
        adi,mcs-track-decimation = /bits/ 16 <1023>;
        adi,trigger-sync-en;

        /* Profile and calibration */
        adi,device-profile-fw-name = "ad9088_profile.bin";
        adi,device-calibration-data-name = "ad9088_cal.bin";

        /* ... other properties ... */
    };


Key Device Tree Properties for MCS
-----------------------------------

+-------------------------------+--------+----------------------------------+
| Property                      | Type   | Description                      |
+===============================+========+==================================+
| ``io-channels``               | phandle| References to ADF4030 (bsync)    |
|                               | array  | and ADF4382 (clk) channels       |
+-------------------------------+--------+----------------------------------+
| ``io-channel-names``          | string | Must be "bsync", "clk"           |
|                               | array  |                                  |
+-------------------------------+--------+----------------------------------+
| ``adi,mcs-track-decimation``  | u16    | TDC decimation rate (1-32767)    |
|                               |        | Default: 1023                    |
+-------------------------------+--------+----------------------------------+
| ``adi,trigger-sync-en``       | bool   | Enable trigger synchronization   |
+-------------------------------+--------+----------------------------------+


Debugfs Interface
=================

MCS Attributes
--------------

::

    /sys/kernel/debug/iio/iio:deviceX/
    ├── mcs_init                  # Initialize MCS config (write 1)
    ├── mcs_dt0_measurement       # Delta-T0 measurement (write 1, read value)
    ├── mcs_dt1_measurement       # Delta-T1 measurement (write 1, read value)
    ├── mcs_dt_restore            # Restore delta-T state (write 1)
    ├── mcs_cal_run               # Run init calibration (write 1, read status)
    ├── mcs_init_cal_status       # Read init cal detailed status
    ├── mcs_track_cal_setup       # Setup tracking cal (write 1)
    ├── mcs_fg_track_cal_run      # Run foreground tracking (write 1)
    ├── mcs_bg_track_cal_run      # Control background tracking (write 1/0)
    ├── mcs_bg_track_cal_freeze   # Freeze/unfreeze tracking (write 1/0)
    ├── mcs_track_status          # Read tracking cal status
    └── mcs_track_cal_validate    # Validate tracking synchronicity (read)


Example Usage
-------------

.. code-block:: shell

    # Check MCS init calibration status
    cat /sys/kernel/debug/iio/iio:device0/mcs_init_cal_status

    # Validate tracking calibration is synchronized
    cat /sys/kernel/debug/iio/iio:device0/mcs_track_cal_validate

    # Temporarily freeze background tracking
    echo 1 > /sys/kernel/debug/iio/iio:device0/mcs_bg_track_cal_freeze
    # ... perform measurements ...
    echo 0 > /sys/kernel/debug/iio/iio:device0/mcs_bg_track_cal_freeze


Error Handling and Diagnostics
==============================

Driver Validation Checks
------------------------

The driver performs several validation checks during MCS calibration:

1. **MCS Init Cal Failure**

   - Condition: SYSREF not locked or timing difference > 0.4 cycles
   - Action: Returns error, stops JESD204 FSM
   - Log: ``MCS Initcal Status: Failed``

2. **Trigger Phase Margin**

   - Condition: Phase outside 25%-75% of SYSREF period
   - Action: Warning logged, continues operation
   - Log: ``Trigger phase X outside safe margin [Y, Z]``

3. **Path Delay Sanity**

   - Condition: calc_delay > 2x period or path_delay at extremes
   - Action: Warning logged, continues operation
   - Log: ``Path delay X fs is very small``

4. **Tracking Cal Mismatch**

   - Condition: Apollo FW values != ADF4382 HW values
   - Action: Reported via mcs_track_cal_validate
   - Status: ``MISMATCH - check hardware``


Kernel Log Messages
-------------------

Normal operation::

    ad9088 spi0.0: Total BSYNC path delay 12345678 fs
    ad9088 spi0.0: MCS Initcal Status: Passed
    ad9088 spi0.0: Trigger Phase 64 (ideal 64) period 500000 fs

Warning conditions::

    ad9088 spi0.0: calc_delay -500000000 fs exceeds 2x BSYNC period
    ad9088 spi0.0: Trigger phase 12 outside safe margin [16, 48]
    ad9088 spi0.0: Path delay 1000 fs is very small (< 5% of period)

Dual clock mode::

    ad9088 spi0.0: Dual clock mode: skipping MCS tracking calibration


References
==========

- :adi:`AD9084/AD9088 User Guide (UG-2300) <media/en/technical-documentation/user-guides/ad9084-ad9088-device-ug-2300.pdf>`
- :adi:`ADF4030 Data Sheet <media/en/technical-documentation/data-sheets/adf4030.pdf>`
- :adi:`ADF4382 Data Sheet <media/en/technical-documentation/data-sheets/adf4382.pdf>`
- :dokuwiki:`JESD204 FSM Framework <resources/tools-software/linux-drivers/jesd204/jesd204-fsm-framework>`
