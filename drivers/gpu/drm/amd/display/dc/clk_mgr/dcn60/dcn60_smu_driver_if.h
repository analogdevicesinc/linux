// SPDX-License-Identifier: MIT
//
// Copyright 2026 Advanced Micro Devices, Inc.
//
// This is a stripped-down version of the smu13_driver_if.h file for the relevant DAL interfaces.

#define DCN6_DRIVER_IF_VERSION  0x1

//Only Clks that have DPM descriptors are listed here
// This should be the same order as DPM_e
typedef enum {
  PPCLK_GFXCLK = 0,
  PPCLK_BTNCLK = 1,
  PPCLK_LCLK = 2,
  PPCLK_UCLK = 3,
  PPCLK_FCLK = 4,
  PPCLK_G7FCLK = 5,
  PPCLK_SOCCLK = 6,
  PPCLK_ACLK = 7,
  PPCLK_DCFCLK = 8,
  PPCLK_VPECLK = 9,
  PPCLK_DISPCLK = 10,
  PPCLK_DPPCLK = 11,
  PPCLK_DPREFCLK = 12,
  PPCLK_CFPUCLK = 13,
  PPCLK_LOGANCLK = 14,
  PPCLK_MSPCLK = 15,
  PPCLK_MOVADCLK = 16,
  PPCLK_DTBCLK = 17,
  PPCLK_DCLK_0 = 18,
  PPCLK_VCLK_0 = 19,
  PPCLK_COUNT = 20,
} PPCLK_e;

typedef struct {
  uint8_t  WmSetting;
  uint8_t  Flags;
  uint8_t  Padding[2];

} WatermarkRowGeneric_t;

#define NUM_WM_RANGES 4

typedef enum {
  WATERMARKS_CLOCK_RANGE = 0,
  WATERMARKS_DUMMY_PSTATE,
  WATERMARKS_MALL,
  WATERMARKS_COUNT,
} WATERMARKS_FLAGS_e;

typedef struct {
  // Watermarks
  WatermarkRowGeneric_t WatermarkRow[NUM_WM_RANGES];
} Watermarks_t;

typedef struct {
	Watermarks_t Watermarks;
  uint32_t  Spare[16];

  uint32_t     MmHubPadding[8]; // SMU internal use
} WatermarksExternal_t;

// Table types
#define TABLE_PMFW_PPTABLE            0
#define TABLE_COMBO_PPTABLE           1
#define TABLE_WATERMARKS              2
#define TABLE_AVFS_PSM_DEBUG          3
#define TABLE_PMSTATUSLOG             4
#define TABLE_SMU_METRICS             5
#define TABLE_DRIVER_SMU_CONFIG       6
#define TABLE_ACTIVITY_MONITOR_COEFF  7
#define TABLE_OVERDRIVE               8
#define TABLE_I2C_COMMANDS            9
#define TABLE_DRIVER_INFO             10
#define TABLE_ECCINFO                 11
#define TABLE_COUNT                   12
