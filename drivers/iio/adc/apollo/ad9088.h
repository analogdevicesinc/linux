/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for AD9088 and similar mixed signal front end (MxFE®)
 *
 * Copyright 2022 Analog Devices Inc.
 */
//#define DEBUG

#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/crc32.h>
#include <linux/vmalloc.h>

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#include <linux/gpio/driver.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/consumer.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>

#define JESD204_OF_PREFIX       "adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>
#include "adi_apollo_bf_serdes_txdig_phy_core1p2.h"
#include "adi_apollo_bf_serdes_rxdig_phy_core1p3.h"
#include "public/inc/adi_apollo.h"
#include "adi_apollo_bf_custom.h"
#include "adi_apollo_adc.h"
#include "adi_apollo_hal.h"
#include "adi_apollo_mailbox.h"
#include "adi_apollo_loopback.h"
#include "adi_apollo_utils.h"

#include "adi_apollo_bf_mcs_sync.h"
#include "adi_apollo_bf_txrx_prefsrc_reconf.h"
#include "adi_apollo_bf_master_bias_ctrl.h"
#include "adi_apollo_sniffer.h"
#include "public/src/adi_apollo_nco_local.h"
#include "adi_apollo_bf_txrx_coarse_nco.h"
#include "adi_utils.h"

#include "adi_utils/inc/adi_utils.h"
#include "public/src/adi_apollo_dformat_local.h"
#include "public/inc/adi_apollo_gpio_hop.h"

#include "../cf_axi_adc.h"

#include <dt-bindings/iio/adc/adi,ad9088.h>
#include "../../../misc/adi-axi-hsci.h"

#define CHIPID_AD9084 0x9084
#define CHIPID_AD9088 0x9088

#define CHIPID_MASK 0xFFFF
#define FW_TRANSFER_CHUNK_SIZE  (16 * 1024)

#define MAX_NUM_MAIN_DATAPATHS ADI_APOLLO_CNCO_NUM
#define MAX_NUM_CHANNELIZER ADI_APOLLO_FNCO_NUM
#define MAX_NUM_RX_NCO_CHAN_REGS 16
#define MAX_NUM_TX_NCO_CHAN_REGS 31

#define NUM_RXTX 2

enum {
	ADI_APOLLO_LOOPBACK_NONE,
	ADI_APOLLO_LOOPBACK_0,
	ADI_APOLLO_LOOPBACK_1,
	ADI_APOLLO_LOOPBACK_2,
	ADI_APOLLO_LOOPBACK_3,
};

enum {
	CDDC_NCO_FREQ,
	FDDC_NCO_FREQ,
	CDDC_NCO_FREQ_AVAIL,
	FDDC_NCO_FREQ_AVAIL,
	CDDC_NCO_PHASE,
	FDDC_NCO_PHASE,
	CDDC_HB1_6DB_GAIN,
	CDDC_TB1_6DB_GAIN,
	FDDC_6DB_GAIN,
	CDDC_TEST_TONE_EN,
	FDDC_TEST_TONE_EN,
	CDDC_TEST_TONE_OFFSET,
	FDDC_TEST_TONE_OFFSET,
	TRX_CONVERTER_RATE,
	TRX_ENABLE,
	ADC_CDDC_FFH_TRIG_HOP_EN,
	ADC_FFH_GPIO_MODE_SET,
	DAC_FFH_GPIO_MODE_SET,
	DAC_FFH_FREQ_SET,
	DAC_INVSINC_EN,
	CFIR_PROFILE_SEL,
	CFIR_ENABLE,
	FFH_FNCO_INDEX,
	FFH_FNCO_FREQUENCY,
	FFH_FNCO_SELECT,
	FFH_FNCO_MODE,
	FFH_CNCO_INDEX,
	FFH_CNCO_FREQUENCY,
	FFH_CNCO_SELECT,
	FFH_CNCO_MODE,
	BMEM_CDDC_DELAY,
	BMEM_FDDC_DELAY,
};

enum ad9088_iio_dev_attr {
	AD9088_JESD204_FSM_ERROR,
	AD9088_JESD204_FSM_PAUSED,
	AD9088_JESD204_FSM_STATE,
	AD9088_JESD204_FSM_RESUME,
	AD9088_JESD204_FSM_CTRL,
	AD9088_LOOPBACK_MODE_SIDE_A,
	AD9088_LOOPBACK_MODE_SIDE_B,
	AD9088_LOOPBACK1_BLEND_SIDE_A,
	AD9088_LOOPBACK1_BLEND_SIDE_B,
};

struct ad9088_jesd204_priv {
	struct ad9088_phy *phy;
	bool serdes_jrx_cal_run;
};

enum ad9088_clocks {
	RX_SAMPL_CLK,
	TX_SAMPL_CLK,
	RX_SAMPL_CLK_LINK2, /* Dual Link */
	NUM_AD9088_CLKS,
};

struct ad9088_clock {
	struct clk_hw hw;
	struct spi_device *spi;
	struct ad9088_phy *phy;
	unsigned long rate;
	enum ad9088_clocks source;
};

#define to_clk_priv(_hw) container_of(_hw, struct ad9088_clock, hw)

struct _ad9088_ffh {
	struct {
		u8 index[ADI_APOLLO_NUM_SIDES*ADI_APOLLO_FNCO_NUM];
		u64 frequency[ADI_APOLLO_FNCO_PROFILE_NUM];
		u8 select[ADI_APOLLO_NUM_SIDES*ADI_APOLLO_FNCO_NUM];
		bool en[ADI_APOLLO_NUM_SIDES*ADI_APOLLO_FNCO_NUM];
        u8 mode[ADI_APOLLO_FNCO_PROFILE_NUM];
	} fnco;
	struct {
		u8 index[ADI_APOLLO_NUM_SIDES*ADI_APOLLO_CNCO_NUM];
		u64 frequency[ADI_APOLLO_CNCO_PROFILE_NUM];
		u8 select[ADI_APOLLO_NUM_SIDES*ADI_APOLLO_CNCO_NUM];
        u8 mode[ADI_APOLLO_CNCO_PROFILE_NUM];
	} cnco;
};

union ad9088_ffh {
	struct {
		struct _ad9088_ffh rx;
		struct _ad9088_ffh tx;
	};
	struct _ad9088_ffh dir[2];
};

struct ad9088_debugfs_entry {
	struct iio_dev *indio_dev;
	const char *propname;
	void *out_value;
	u32 val;
	s64 delta_t;
	u8 size;
	u8 cmd;
};

/**
 * struct ad9088_chan_map - IIO channel to hardware block mapping
 * @fddc_num: FDDC number (0-7 per side)
 * @fddc_mask: FDDC bitmask (ADI_APOLLO_FDDC_Ax or ADI_APOLLO_FDDC_Bx)
 * @cddc_num: CDDC number (0-3 per side for 8T8R, 0-1 for 4T4R)
 * @cddc_mask: CDDC bitmask (ADI_APOLLO_CNCO_Ax or ADI_APOLLO_CNCO_Bx)
 * @adcdac_num: ADC/DAC number (0-3 per side)
 * @adcdac_mask: ADC/DAC bitmask (ADI_APOLLO_ADC_Ax or ADI_APOLLO_DAC_Ax)
 * @side: Chip side (0=A, 1=B)
 *
 * Pre-computed mapping from IIO channel to hardware blocks.
 * Derived from profile mux configuration at init time.
 */
struct ad9088_chan_map {
	u8 fddc_num;
	u32 fddc_mask;
	u8 cddc_num;
	u32 cddc_mask;
	u8 adcdac_num;
	u32 adcdac_mask;
	u8 side;
};

struct ad9088_phy {
	struct spi_device *spi;
	struct jesd204_dev *jdev;
	const struct firmware *fw;
	adi_apollo_device_t ad9088;
	adi_apollo_top_t profile;
	adi_cms_chip_id_t chip_id;
	struct axiadc_chip_info chip_info;
	struct clk *dev_clk;
	struct bin_attribute pfilt;
	struct bin_attribute cfir;
	struct bin_attribute cal_data;
	struct gpio_chip gpiochip;

	struct gpio_desc *rx1_en_gpio;
	struct gpio_desc *rx2_en_gpio;
	struct gpio_desc *tx1_en_gpio;
	struct gpio_desc *tx2_en_gpio;
	struct gpio_desc *triq_req_gpio;
	struct gpio_desc *transceiver_reset_gpio;
	struct regulator *supply_reg;
	struct axi_hsci_state *hsci;

	struct clk *clks[NUM_AD9088_CLKS];
	struct clock_scale clkscale[NUM_AD9088_CLKS];
	struct ad9088_clock clk_priv[NUM_AD9088_CLKS];
	struct clk_onecell_data clk_data;
	struct delayed_work dwork;

	/*
	 * Synchronize access to members of driver state, and ensure atomicity
	 * of consecutive regmap operations.
	 */
	struct mutex            lock;

	bool is_initialized;
	bool standalone;
	bool device_profile_firmware_load;
	bool side_b_use_own_tpl_en;
	bool complex_tx;
	bool complex_rx;
	bool spi_3wire_en;
	bool log_silent;
	bool trig_sync_en;
	bool mcs_cal_bg_tracking_run;
	bool mcs_cal_bg_tracking_freeze;
	bool cddc_sample_delay_en;
	bool fddc_sample_delay_en;
	u32 multidevice_instance_count;

	struct ad9088_debugfs_entry debugfs_entry[32];
	u32 ad9088_debugfs_entry_index;

	const char **rx_labels;
	const char **tx_labels;

	char rx_chan_labels[MAX_NUM_CHANNELIZER][32];
	char tx_chan_labels[MAX_NUM_CHANNELIZER][32];

	long long cnco_phase[NUM_RXTX][ADI_APOLLO_NUM_SIDES][MAX_NUM_MAIN_DATAPATHS];
	u16 cnco_test_tone_offset[NUM_RXTX][ADI_APOLLO_NUM_SIDES][MAX_NUM_MAIN_DATAPATHS];
	bool cnco_test_tone_en[NUM_RXTX][ADI_APOLLO_NUM_SIDES][MAX_NUM_MAIN_DATAPATHS];

	long long fnco_phase[NUM_RXTX][ADI_APOLLO_NUM_SIDES][MAX_NUM_CHANNELIZER];
	u16 fnco_test_tone_offset[NUM_RXTX][ADI_APOLLO_NUM_SIDES][MAX_NUM_CHANNELIZER];
	bool fnco_test_tone_en[NUM_RXTX][ADI_APOLLO_NUM_SIDES][MAX_NUM_CHANNELIZER];

	u8 cfir_profile[NUM_RXTX][ADI_APOLLO_CFIR_ALL][ADI_APOLLO_CFIR_DP_ALL];
	u8 cfir_enable[NUM_RXTX][ADI_APOLLO_CFIR_ALL][ADI_APOLLO_CFIR_DP_ALL];

	u32 cddc_sample_delay[NUM_RXTX][MAX_NUM_MAIN_DATAPATHS];
	u32 fddc_sample_delay[NUM_RXTX][MAX_NUM_CHANNELIZER];

	u32 rx_nyquist_zone;
	u8 jrx_lanes[24];
	u8 jtx_lanes[24];
	u8 jrx_lanes_used;
	u8 jtx_lanes_used;

	u8 rx_en_mask;
	u8 tx_en_mask;

	bool hsci_use_auto_linkup_mode;
	bool hsci_disable_after_initial_configuration;
	bool aion_background_serial_alignment_en;
	bool fnco_dual_modulus_mode_en;
	bool cnco_dual_modulus_mode_en;
	bool sniffer_en;
	bool cal_data_loaded_from_fw;

	struct iio_channel      *iio_adf4030;
	struct iio_channel      *iio_adf4382;

	adi_apollo_fw_provider_t fw_provider;
	union ad9088_ffh ffh;

	adi_apollo_gpio_hop_profile_t gpio_hop_profile;
	adi_apollo_gpio_hop_block_t gpio_hop_block;
	adi_apollo_gpio_hop_side_t gpio_hop_side;
	adi_apollo_gpio_hop_slice_t gpio_hop_slice;
	adi_apollo_gpio_hop_terminal_t gpio_hop_terminal;

	u8 hsci_buf[ADI_APOLLO_HAL_REGIO_HSCI_STREAM_DEFAULT_SIZE];
	u8 gpios_exported[ADI_APOLLO_NUM_GPIO];
	char dbuf[60000];

	u8 loopback_mode[ADI_APOLLO_NUM_SIDES];
	u8 lb1_blend[ADI_APOLLO_NUM_SIDES];

	u8 *nvm_adc_cal;
	u8 *adc_cal;
	size_t adc_cal_len;

	/* Calibration restore buffer for multi-write accumulation */
	u8 *cal_restore_buf;
	size_t cal_restore_size;
	size_t cal_restore_received;

	/* Pre-computed IIO channel to hardware block mapping (indexed by chan->address) */
	struct ad9088_chan_map rx_chan_map[MAX_NUM_CHANNELIZER];
	struct ad9088_chan_map tx_chan_map[MAX_NUM_CHANNELIZER];

	/*
	 * RX IIO channel scan_index remapping for lane swap compensation.
	 * When FPGA lane routing causes DMA buffer positions to not match
	 * the physical channel order, use this array to remap scan_index.
	 * Value at index i specifies which DMA buffer position IIO channel i
	 * should read from. A value of -1 means no remapping (identity).
	 *
	 * Array size: MAX_NUM_CHANNELIZER * 2 (I/Q) * max multidevice_instance_count (4)
	 */
#define MAX_NUM_REMAP_CHANNELS	(MAX_NUM_CHANNELIZER * 2 * 4)
	s8 rx_iio_to_phy_remap[MAX_NUM_REMAP_CHANNELS];

	/* Device label from DT for sub-device naming */
	const char *device_label;
};

extern int ad9088_iio_write_channel_ext_info(struct ad9088_phy *phy, struct iio_channel *chan,
										     const char *ext_name, long long val);
extern int ad9088_iio_read_channel_ext_info(struct ad9088_phy *phy, struct iio_channel *chan,
					    const char *ext_name, long long *val);
extern int ad9088_parse_dt(struct ad9088_phy *phy);
extern int ad9088_fft_sniffer_probe(struct ad9088_phy *phy, adi_apollo_side_select_e side_sel);
extern int ad9088_ffh_probe(struct ad9088_phy *phy);
extern int32_t adi_ad9088_calc_nco_ftw(struct ad9088_phy *phy,
					u64 freq, s64 nco_shift, u32 div, u32 bits,
					u64 *ftw, u64 *frac_a, u64 *frac_b);

extern void ad9088_iiochan_to_fddc_cddc(struct ad9088_phy *phy, const struct iio_chan_spec *chan,
										u8 *fddc_num, u32 *fddc_mask, u8 *cddc_num, u32 *cddc_mask,
										u8 *side);
extern void ad9088_iiochan_to_fddc_cddc_from_profile(struct ad9088_phy *phy,
						     const struct iio_chan_spec *chan,
						     u8 *fddc_num, u32 *fddc_mask,
						     u8 *cddc_num, u32 *cddc_mask,
						     u8 *adcdac_num, u32 *adcdac_mask,
						     u8 *side);

extern const struct ad9088_chan_map *ad9088_get_chan_map(struct ad9088_phy *phy,
						  const struct iio_chan_spec *chan);

extern ssize_t ad9088_ext_info_read_ffh(struct iio_dev *indio_dev,
										uintptr_t private,
										const struct iio_chan_spec *chan, char *buf);
extern ssize_t ad9088_ext_info_write_ffh(struct iio_dev *indio_dev,
										 uintptr_t private,
										 const struct iio_chan_spec *chan,
										 const char *buf, size_t len);

extern int ad9088_check_apollo_error(struct device *dev, int ret,
				     const char *api_name);

int ad9088_bmem_probe(struct ad9088_phy *phy);

/* Calibration data format */
#define AD9088_CAL_MAGIC	0x41443930  /* "AD90" */
#define AD9088_CAL_VERSION	2

struct ad9088_cal_header {
	u32 magic;			/* Magic number for validation */
	u32 version;			/* File format version */
	u32 chip_id;			/* Chip ID (0x9084 or 0x9088) */
	u8  is_8t8r;			/* 1 = 8T8R, 0 = 4T4R */
	u8  num_adcs;			/* Number of ADCs */
	u8  num_serdes_rx;		/* Number of SERDES RX 12-packs */
	u8  num_clk_cond;		/* Number of clock conditioning sides */
	u8  reserved[4];		/* Reserved for future use */

	/* Offsets to each section (from start of file) */
	u32 adc_cal_offset;		/* Offset to ADC calibration data */
	u32 serdes_rx_cal_offset;	/* Offset to SERDES RX calibration data */
	u32 clk_cond_cal_offset;	/* Offset to clock conditioning cal data */

	/* Sizes of each section */
	u32 adc_cal_size;		/* Total size of all ADC cal data */
	u32 serdes_rx_cal_size;		/* Total size of all SERDES RX cal data */
	u32 clk_cond_cal_size;		/* Total size of all clock conditioning cal data */

	u32 total_size;			/* Total file size including CRC */
	u32 reserved2[4];		/* Reserved for future use */
} __packed;

int ad9088_cal_save(struct ad9088_phy *phy, u8 **buf, size_t *len);
int ad9088_cal_restore(struct ad9088_phy *phy, const u8 *buf, size_t len);
int ad9088_cal_load_from_firmware(struct ad9088_phy *phy);
ssize_t ad9088_cal_data_read(struct file *filp, struct kobject *kobj,
			     struct bin_attribute *bin_attr,
			     char *buf, loff_t off, size_t count);
ssize_t ad9088_cal_data_write(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *bin_attr,
			      char *buf, loff_t off, size_t count);

/* Debugfs interface (ad9088_debugfs.c) */
int ad9088_debugfs_register(struct iio_dev *indio_dev);
int ad9088_status_show(struct seq_file *file, void *offset);
void ad9088_print_sysref_phase(struct ad9088_phy *phy);

/* JESD204 FSM interface (ad9088_jesd204_fsm.c) */
extern const struct jesd204_dev_data jesd204_ad9088_init;

/* Helper functions used by JESD204 FSM (ad9088.c) */
extern const char *const ad9088_fsm_links_to_str[];
u8 ad9088_to_link(u8 linkid);
int ad9088_check_apollo_error(struct device *dev, int ret, const char *func_name);
int ad9088_inspect_jrx_link_all(struct ad9088_phy *phy);
int ad9088_inspect_jtx_link_all(struct ad9088_phy *phy);
void ad9088_print_link_phase(struct ad9088_phy *phy, struct jesd204_link *lnk);
int ad9088_jesd_tx_link_status_print(struct ad9088_phy *phy, struct jesd204_link *lnk, int retry);
int ad9088_jesd_rx_link_status_print(struct ad9088_phy *phy, struct jesd204_link *lnk, int retry);
int ad9088_iio_write_channel_ext_info(struct ad9088_phy *phy, struct iio_channel *chan,
				      const char *ext_name, long long val);
int ad9088_mcs_init_cal_setup(struct ad9088_phy *phy);
int ad9088_delta_t_measurement_set(struct ad9088_phy *phy, u32 mode);
int ad9088_delta_t_measurement_get(struct ad9088_phy *phy, u32 mode, s64 *apollo_delta_t);
int ad9088_mcs_init_cal_validate(struct ad9088_phy *phy,
				 adi_apollo_mcs_cal_init_status_t *init_cal_status);
int ad9088_mcs_tracking_cal_setup(struct ad9088_phy *phy, u16 mcs_track_decimation,
				  u16 initialize_track_cal);
int ad9088_mcs_init_cal_status_print(struct ad9088_phy *phy, char *buf,
				     adi_apollo_mcs_cal_init_status_t *status);
int ad9088_mcs_track_cal_status_print(struct ad9088_phy *phy, char *buf,
				      adi_apollo_mcs_cal_status_t *cal_status,
				      u8 print_full_state);
int ad9088_mcs_tracking_cal_validate(struct ad9088_phy *phy, char *buf, size_t buf_size);
