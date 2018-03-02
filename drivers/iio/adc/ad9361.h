/*
 * AD9361
 *
 * Copyright 2013-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_FREQUENCY_AD9361_H_
#define IIO_FREQUENCY_AD9361_H_

#include "ad9361_regs.h"

/*
 *	Driver
 */

enum rx_gain_table_type {
	RXGAIN_FULL_TBL,
	RXGAIN_SPLIT_TBL,
};

enum rx_gain_table_name {
	TBL_200_1300_MHZ,
	TBL_1300_4000_MHZ,
	TBL_4000_6000_MHZ,
	RXGAIN_TBLS_END,
};

enum fir_dest {
	FIR_TX1 = 0x01,
	FIR_TX2 = 0x02,
	FIR_TX1_TX2 = 0x03,
	FIR_RX1 = 0x81,
	FIR_RX2 = 0x82,
	FIR_RX1_RX2 = 0x83,
	FIR_IS_RX = 0x80,
};

struct rf_gain_ctrl {
	u32 ant;
	u8 mode;
};

enum rf_gain_ctrl_mode {
	RF_GAIN_MGC,
	RF_GAIN_FASTATTACK_AGC,
	RF_GAIN_SLOWATTACK_AGC,
	RF_GAIN_HYBRID_AGC
};

enum f_agc_target_gain_index_type {
	MAX_GAIN,
	SET_GAIN,
	OPTIMIZED_GAIN,
	NO_GAIN_CHANGE,
};

struct gain_control {
	enum rf_gain_ctrl_mode rx1_mode;
	enum rf_gain_ctrl_mode rx2_mode;

	/* Common */
	u8 adc_ovr_sample_size; /* 1..8 Sum x samples, AGC_CONFIG_3 */
	u8 adc_small_overload_thresh; /* 0..255, 0x105 */
	u8 adc_large_overload_thresh; /* 0..255, 0x104 */

	u16 lmt_overload_high_thresh; /* 16..800 mV, 0x107 */
	u16 lmt_overload_low_thresh; /* 16..800 mV, 0x108 */
	u16 dec_pow_measuremnt_duration; /* Samples, 0x15C */
	u8 low_power_thresh; /* -64..0 dBFS, 0x114 */
	bool use_rx_fir_out_for_dec_pwr_meas; /* clears 0x15C:6 USE_HB1_OUT_FOR_DEC_PWR_MEAS */

	bool dig_gain_en; /* should be turned off, since ADI GT doesn't use dig gain */
	u8 max_dig_gain; /* 0..31 */

	/* MGC */
	bool mgc_rx1_ctrl_inp_en; /* Enables Pin control on RX1 default SPI ctrl */
	bool mgc_rx2_ctrl_inp_en; /* Enables Pin control on RX2 default SPI ctrl */

	u8 mgc_inc_gain_step; /* 1..8 */
	u8 mgc_dec_gain_step; /* 1..8 */
	u8 mgc_split_table_ctrl_inp_gain_mode; /* 0=AGC determine this, 1=only in LPF, 2=only in LMT */

	/* AGC */
	u8 agc_attack_delay_extra_margin_us; /* 0..31 us */

	u8 agc_outer_thresh_high;
	u8 agc_outer_thresh_high_dec_steps;
	u8 agc_inner_thresh_high;
	u8 agc_inner_thresh_high_dec_steps;
	u8 agc_inner_thresh_low;
	u8 agc_inner_thresh_low_inc_steps;
	u8 agc_outer_thresh_low;
	u8 agc_outer_thresh_low_inc_steps;

	u8 adc_small_overload_exceed_counter; /* 0..15, 0x122 */
	u8 adc_large_overload_exceed_counter; /* 0..15, 0x122 */
	u8 adc_large_overload_inc_steps; /* 0..15, 0x106 */

	bool adc_lmt_small_overload_prevent_gain_inc; /* 0x120 */

	u8 lmt_overload_large_exceed_counter; /* 0..15, 0x121 */
	u8 lmt_overload_small_exceed_counter; /* 0..15, 0x121 */
	u8 lmt_overload_large_inc_steps; /* 0..7, 0x121 */

	u8 dig_saturation_exceed_counter; /* 0..15, 0x128 */
	u8 dig_gain_step_size; /* 1..8, 0x100 */
	bool sync_for_gain_counter_en; /* 0x128:4 !Hybrid */

	u32 gain_update_interval_us; /* in us */
	bool immed_gain_change_if_large_adc_overload; /* 0x123:3 */
	bool immed_gain_change_if_large_lmt_overload; /* 0x123:7 */

	/*
	 * Fast AGC
	 */
	u32 f_agc_dec_pow_measuremnt_duration;  /* Samples, 0x15C */
	u32 f_agc_state_wait_time_ns; /* 0x117 0..31 RX samples -> time_ns */
	/* Fast AGC - Low Power */
	bool f_agc_allow_agc_gain_increase; /* 0x110:1 */
	u8 f_agc_lp_thresh_increment_time; /* 0x11B RX samples */
	u8 f_agc_lp_thresh_increment_steps; /* 0x117 1..8 */

	/* Fast AGC - Lock Level */
	u8 f_agc_lock_level; /* NOT USED: 0x101 0..-127 dBFS same as agc_inner_thresh_high */
	bool f_agc_lock_level_lmt_gain_increase_en; /* 0x111:6 */
	u8 f_agc_lock_level_gain_increase_upper_limit; /* 0x118 0..63 */
	/* Fast AGC - Peak Detectors and Final Settling */
	u8 f_agc_lpf_final_settling_steps; /* 0x112:6 0..3 (Post Lock Level Step)*/
	u8 f_agc_lmt_final_settling_steps; /* 0x113:6 0..3 (Post Lock Level Step)*/
	u8 f_agc_final_overrange_count; /* 0x116:5 0..7 */
	/* Fast AGC - Final Power Test */
	bool f_agc_gain_increase_after_gain_lock_en; /* 0x110:7  */
	/* Fast AGC - Unlocking the Gain */
	/* 0 = MAX Gain, 1 = Set Gain, 2 = Optimized Gain */
	enum f_agc_target_gain_index_type f_agc_gain_index_type_after_exit_rx_mode; /* 0x110:[4,2]  */
	bool f_agc_use_last_lock_level_for_set_gain_en; /* 0x111:7 */
	u8 f_agc_optimized_gain_offset;	/*0x116 0..15 steps */
	bool f_agc_rst_gla_stronger_sig_thresh_exceeded_en; /* 0x110:~6 */
	u8 f_agc_rst_gla_stronger_sig_thresh_above_ll;	/*0x113 0..63 dbFS */
	bool f_agc_rst_gla_engergy_lost_sig_thresh_exceeded_en; /* 0x110:6 */
	bool f_agc_rst_gla_engergy_lost_goto_optim_gain_en; /* 0x110:6 */
	u8 f_agc_rst_gla_engergy_lost_sig_thresh_below_ll; /* 0x112:6 */
	u8 f_agc_energy_lost_stronger_sig_gain_lock_exit_cnt; /* 0x119 0..63 RX samples */
	bool f_agc_rst_gla_large_adc_overload_en; /*0x110:~1 and 0x114:~7 */
	bool f_agc_rst_gla_large_lmt_overload_en; /*0x110:~1 */
	bool f_agc_rst_gla_en_agc_pulled_high_en;
	/* 0 = Max Gain, 1 = Set Gain, 2 = Optimized Gain, 3 = No Gain Change */

	enum f_agc_target_gain_index_type f_agc_rst_gla_if_en_agc_pulled_high_mode; /* 0x0FB, 0x111 */
	u8 f_agc_power_measurement_duration_in_state5; /* 0x109, 0x10a RX samples 0..524288*/
	u8 f_agc_large_overload_inc_steps; /* 0x106 [D6:D4] 0..7 */

};

struct auxdac_control {
	u16 dac1_default_value;
	u16 dac2_default_value;

	bool auxdac_manual_mode_en;

	bool dac1_in_rx_en;
	bool dac1_in_tx_en;
	bool dac1_in_alert_en;

	bool dac2_in_rx_en;
	bool dac2_in_tx_en;
	bool dac2_in_alert_en;

	u8 dac1_rx_delay_us;
	u8 dac1_tx_delay_us;
	u8 dac2_rx_delay_us;
	u8 dac2_tx_delay_us;
};

enum rssi_restart_mode {
	AGC_IN_FAST_ATTACK_MODE_LOCKS_THE_GAIN,
	EN_AGC_PIN_IS_PULLED_HIGH,
	ENTERS_RX_MODE,
	GAIN_CHANGE_OCCURS,
	SPI_WRITE_TO_REGISTER,
	GAIN_CHANGE_OCCURS_OR_EN_AGC_PIN_PULLED_HIGH,
};

struct rssi_control {
	enum rssi_restart_mode restart_mode;
	bool rssi_unit_is_rx_samples;	/* default unit is time */
	u32 rssi_delay;
	u32 rssi_wait;
	u32 rssi_duration;
};

struct rx_gain_info {
	enum rx_gain_table_type tbl_type;
	int starting_gain_db;
	int max_gain_db;
	int gain_step_db;
	int max_idx;
	int idx_step_offset;
};

struct port_control {
	u8			pp_conf[3];
	u8			rx_clk_data_delay;
	u8			tx_clk_data_delay;
	u8			digital_io_ctrl;
	u8			lvds_bias_ctrl;
	u8			lvds_invert[2];
};

struct ctrl_outs_control {
	u8			index;
	u8			en_mask;
};

struct elna_control {
	u16			gain_mdB;
	u16			bypass_loss_mdB;
	u32			settling_delay_ns;
	bool			elna_1_control_en; /* GPO0 */
	bool			elna_2_control_en; /* GPO1 */
	bool			elna_in_gaintable_all_index_en;
};

struct auxadc_control {
	s8			offset;
	u32			temp_time_inteval_ms;
	u32			temp_sensor_decimation;
	bool			periodic_temp_measuremnt;
	u32			auxadc_clock_rate;
	u32			auxadc_decimation;
};

struct gpo_control {
	u32 gpo_manual_mode_enable_mask;
	bool gpo_manual_mode_en;
	bool gpo0_inactive_state_high_en;
	bool gpo1_inactive_state_high_en;
	bool gpo2_inactive_state_high_en;
	bool gpo3_inactive_state_high_en;
	bool gpo0_slave_rx_en;
	bool gpo0_slave_tx_en;
	bool gpo1_slave_rx_en;
	bool gpo1_slave_tx_en;
	bool gpo2_slave_rx_en;
	bool gpo2_slave_tx_en;
	bool gpo3_slave_rx_en;
	bool gpo3_slave_tx_en;
	u8 gpo0_rx_delay_us;
	u8 gpo0_tx_delay_us;
	u8 gpo1_rx_delay_us;
	u8 gpo1_tx_delay_us;
	u8 gpo2_rx_delay_us;
	u8 gpo2_tx_delay_us;
	u8 gpo3_rx_delay_us;
	u8 gpo3_tx_delay_us;
};

struct tx_monitor_control {
	bool tx_mon_track_en;
	bool one_shot_mode_en;
	u32 low_high_gain_threshold_mdB;
	u8 low_gain_dB;
	u8 high_gain_dB;
	u16 tx_mon_delay;
	u16 tx_mon_duration;
	u8 tx1_mon_front_end_gain;
	u8 tx2_mon_front_end_gain;
	u8 tx1_mon_lo_cm;
	u8 tx2_mon_lo_cm;
};

enum ad9361_pdata_rx_freq {
	BBPLL_FREQ,
	ADC_FREQ,
	R2_FREQ,
	R1_FREQ,
	CLKRF_FREQ,
	RX_SAMPL_FREQ,
	NUM_RX_CLOCKS,
};

enum ad9361_pdata_tx_freq {
	IGNORE,
	DAC_FREQ,
	T2_FREQ,
	T1_FREQ,
	CLKTF_FREQ,
	TX_SAMPL_FREQ,
	NUM_TX_CLOCKS,
};

enum ad9361_clkout {
	CLKOUT_DISABLE,
	BUFFERED_XTALN_DCXO,
	ADC_CLK_DIV_2,
	ADC_CLK_DIV_3,
	ADC_CLK_DIV_4,
	ADC_CLK_DIV_8,
	ADC_CLK_DIV_16,
};

enum synth_pd_ctrl {
	LO_DONTCARE,
	LO_OFF,
	LO_ON,
};

struct ad9361_phy_platform_data {
	bool			rx2tx2;
	bool			fdd;
	bool			fdd_independent_mode;
	bool			split_gt;
	bool			use_extclk;
	bool			ensm_pin_pulse_mode;
	bool			ensm_pin_ctrl;
	bool			debug_mode;
	bool			tdd_use_dual_synth;
	bool			tdd_skip_vco_cal;
	bool			use_ext_rx_lo;
	bool			use_ext_tx_lo;
	bool			rx1rx2_phase_inversion_en;
	bool			qec_tracking_slow_mode_en;
	bool			dig_interface_tune_fir_disable;
	u8			dc_offset_update_events;
	u8			dc_offset_attenuation_high;
	u8			dc_offset_attenuation_low;
	u8			rf_dc_offset_count_high;
	u8			rf_dc_offset_count_low;
	u8			dig_interface_tune_skipmode;
	u32			dcxo_coarse;
	u32			dcxo_fine;
	u32			rf_rx_input_sel;
	u32			rf_tx_output_sel;
	bool			rf_rx_input_sel_lock;
	bool			rf_tx_output_sel_lock;
	u32			rx1tx1_mode_use_rx_num;
	u32			rx1tx1_mode_use_tx_num;
	unsigned long		rx_path_clks[NUM_RX_CLOCKS];
	unsigned long		tx_path_clks[NUM_TX_CLOCKS];
	u32			trx_synth_max_fref;
	u64			rx_synth_freq;
	u64			tx_synth_freq;
	u32			rf_rx_bandwidth_Hz;
	u32			rf_tx_bandwidth_Hz;
	int			tx_atten;
	bool			update_tx_gain_via_alert;
	u32			rx_fastlock_delay_ns;
	u32			tx_fastlock_delay_ns;
	bool			trx_fastlock_pinctrl_en[2];

	enum ad9361_clkout	ad9361_clkout_mode;

	struct gain_control	gain_ctrl;
	struct rssi_control	rssi_ctrl;
	u32		rssi_lna_err_tbl[4];
	u32		rssi_mixer_err_tbl[16];
	u32		rssi_gain_step_calib_reg_val[5];
	bool	rssi_skip_calib;
	struct port_control	port_ctrl;
	struct ctrl_outs_control	ctrl_outs_ctrl;
	struct elna_control	elna_ctrl;
	struct auxadc_control	auxadc_ctrl;
	struct auxdac_control	auxdac_ctrl;
	struct gpo_control	gpo_ctrl;
	struct tx_monitor_control txmon_ctrl;

	struct gpio_desc			*reset_gpio;
	/*  MCS SYNC */
	struct gpio_desc			*sync_gpio;
	struct gpio_desc			*cal_sw1_gpio;
	struct gpio_desc			*cal_sw2_gpio;

};

struct rf_rx_gain {
	u32 ant;		/* Antenna number to read gain */
	s32 gain_db;		/* gain value in dB */
	u32 fgt_lmt_index;	/* Full Gain Table / LNA-MIXER-TIA gain index */
	u32 lmt_gain;		/* LNA-MIXER-TIA gain in dB (Split GT mode only)*/
	u32 lpf_gain;		/* Low pass filter gain in dB / index (Split GT mode only)*/
	u32 digital_gain;	/* Digital gain in dB / index */
	/* Debug only */
	u32 lna_index;		/* LNA Index (Split GT mode only) */
	u32 tia_index;		/* TIA Index (Split GT mode only) */
	u32 mixer_index;		/* MIXER Index (Split GT mode only) */

};
struct rf_rssi {
	u32 ant;		/* Antenna number for which RSSI is reported */
	u32 symbol;		/* Runtime RSSI */
	u32 preamble;		/* Initial RSSI */
	s32 multiplier;	/* Multiplier to convert reported RSSI */
	u8 duration;		/* Duration to be considered for measuring */
};

struct SynthLUT {
	u16 VCO_MHz;
	u8 VCO_Output_Level;
	u8 VCO_Varactor;
	u8 VCO_Bias_Ref;
	u8 VCO_Bias_Tcf;
	u8 VCO_Cal_Offset;
	u8 VCO_Varactor_Reference;
	u8 Charge_Pump_Current;
	u8 LF_C2;
	u8 LF_C1;
	u8 LF_R1;
	u8 LF_C3;
	u8 LF_R3;
};

#define SYNTH_LUT_SIZE	53

enum {
	LUT_FTDD_40,
	LUT_FTDD_60,
	LUT_FTDD_80,
	LUT_FTDD_ENT,
};

enum ad9361_clocks {
	BB_REFCLK,
	RX_REFCLK,
	TX_REFCLK,
	BBPLL_CLK,
	ADC_CLK,
	R2_CLK,
	R1_CLK,
	CLKRF_CLK,
	RX_SAMPL_CLK,
	DAC_CLK,
	T2_CLK,
	T1_CLK,
	CLKTF_CLK,
	TX_SAMPL_CLK,
	RX_RFPLL_INT,
	TX_RFPLL_INT,
	RX_RFPLL_DUMMY,
	TX_RFPLL_DUMMY,
	RX_RFPLL,
	TX_RFPLL,
	NUM_AD9361_CLKS,
};

enum debugfs_cmd {
	DBGFS_NONE,
	DBGFS_INIT,
	DBGFS_LOOPBACK,
	DBGFS_BIST_PRBS,
	DBGFS_BIST_TONE,
	DBGFS_BIST_DT_ANALYSIS,
	DBGFS_RXGAIN_1,
	DBGFS_RXGAIN_2,
	DBGFS_MCS,
	DBGFS_CAL_SW_CTRL,
	DBGFS_DIGITAL_TUNE,
};

enum dig_tune_flags {
	BE_VERBOSE = 1,
	BE_MOREVERBOSE = 2,
	DO_IDELAY = 4,
	DO_ODELAY = 8,
	SKIP_STORE_RESULT = 16,
	RESTORE_DEFAULT = 32,
};

enum ad9361_bist_mode {
	BIST_DISABLE,
	BIST_INJ_TX,
	BIST_INJ_RX,
};

enum {
	ID_AD9361,
	ID_AD9364,
	ID_AD9361_2,
	ID_AD9363A,
};

enum digital_tune_skip_mode {
	TUNE_RX_TX,
	SKIP_TX,
	SKIP_ALL,
};

struct ad9361_rf_phy;
struct ad9361_debugfs_entry {
	struct ad9361_rf_phy *phy;
	const char *propname;
	void *out_value;
	u32 val;
	u8 size;
	u8 cmd;
};

struct ad9361_fastlock_entry {
#define FASTLOOK_INIT	1
	u8 flags;
	u8 alc_orig;
	u8 alc_written;
};

struct ad9361_fastlock {
	u8 save_profile;
	u8 current_profile[2];
	struct ad9361_fastlock_entry entry[2][8];
};

struct ad9361_dig_tune_data {
	u32 bist_loopback_mode;
	u32 bist_config;
	u32 ensm_state;
	u8 skip_mode;
};

struct refclk_scale {
	struct clk_hw		hw;
	struct spi_device	*spi;
	struct ad9361_rf_phy	*phy;
	unsigned long		rate;
	u32			mult;
	u32			div;
	enum ad9361_clocks 	source;
};

struct ad9361_rf_phy {
	struct spi_device 	*spi;
	struct clk 		*clk_refin;
	struct clk 		*clk_ext_lo_rx;
	struct clk 		*clk_ext_lo_tx;
	struct clk 		*clks[NUM_AD9361_CLKS];
	struct notifier_block   clk_nb_tx;
	struct notifier_block   clk_nb_rx;
	struct refclk_scale	clk_priv[NUM_AD9361_CLKS];
	struct clk_onecell_data	clk_data;
	struct ad9361_phy_platform_data *pdata;
	struct ad9361_debugfs_entry debugfs_entry[180];
	struct bin_attribute 	bin;
	struct bin_attribute 	bin_gt;
	struct iio_dev 		*indio_dev;
	struct work_struct 	work;
	struct completion       complete;
	struct gain_table_info  *gt_info;
	char			*bin_attr_buf;
	u32 			ad9361_debugfs_entry_index;
	u8 			prev_ensm_state;
	u8			curr_ensm_state;
	u8			cached_rx_rfpll_div;
	u8			cached_tx_rfpll_div;
	u8			cached_synth_pd[2];
	int			tx_quad_lpf_tia_match;
	int			current_table;
	int			rx_sampl_freq_avail[3];
	int			tx_sampl_freq_avail[3];
	int 			rx_gain_avail[3];

	bool 			ensm_pin_ctl_en;

	bool			auto_cal_en;
	bool 			manual_tx_quad_cal_en;
	u64			last_tx_quad_cal_freq;
	u32			last_tx_quad_cal_phase;
	u64			current_tx_lo_freq;
	u64			current_rx_lo_freq;
	bool			current_tx_use_tdd_table;
	bool			current_rx_use_tdd_table;
	unsigned long		current_rx_path_clks[NUM_RX_CLOCKS];
	unsigned long		current_tx_path_clks[NUM_TX_CLOCKS];
	unsigned long		flags;
	unsigned long		cal_threshold_freq;
	u32			current_rx_bw_Hz;
	u32			current_tx_bw_Hz;
	u32			rxbbf_div;
	u32			rate_governor;
	bool			bypass_rx_fir;
	bool			bypass_tx_fir;
	bool			rx_eq_2tx;
	bool			filt_valid;
	unsigned long		filt_rx_path_clks[NUM_RX_CLOCKS];
	unsigned long		filt_tx_path_clks[NUM_TX_CLOCKS];
	u32			filt_rx_bw_Hz;
	u32			filt_tx_bw_Hz;
	u8			tx_fir_int;
	u8			tx_fir_ntaps;
	u8			rx_fir_dec;
	u8			rx_fir_ntaps;
	u8			agc_mode[2];
	bool			rfdc_track_en;
	bool			bbdc_track_en;
	bool			quad_track_en;
	bool			txmon_tdd_en;
	u16 			auxdac1_value;
	u16 			auxdac2_value;
	u32			tx1_atten_cached;
	u32			tx2_atten_cached;
	u8			bist_loopback_mode;
	u8			bist_config;

	struct ad9361_fastlock	fastlock;
};


ssize_t ad9361_dig_interface_timing_analysis(struct ad9361_rf_phy *phy,
						   char *buf, unsigned buflen);
int ad9361_hdl_loopback(struct ad9361_rf_phy *phy, bool enable);
int ad9361_register_axi_converter(struct ad9361_rf_phy *phy);
struct ad9361_rf_phy* ad9361_spi_to_phy(struct spi_device *spi);
int ad9361_spi_read(struct spi_device *spi, u32 reg);
int ad9361_spi_write(struct spi_device *spi, u32 reg, u32 val);
int ad9361_bist_loopback(struct ad9361_rf_phy *phy, unsigned mode);
int ad9361_bist_prbs(struct ad9361_rf_phy *phy, enum ad9361_bist_mode mode);
int ad9361_find_opt(u8 *field, u32 size, u32 *ret_start);
int ad9361_ensm_mode_disable_pinctrl(struct ad9361_rf_phy *phy);
int ad9361_ensm_mode_restore_pinctrl(struct ad9361_rf_phy *phy);
void ad9361_ensm_force_state(struct ad9361_rf_phy *phy, u8 ensm_state);
void ad9361_ensm_restore_state(struct ad9361_rf_phy *phy, u8 ensm_state);
void ad9361_ensm_restore_prev_state(struct ad9361_rf_phy *phy);
int ad9361_set_trx_clock_chain_freq(struct ad9361_rf_phy *phy,
					  unsigned long freq);
int ad9361_set_trx_clock_chain_default(struct ad9361_rf_phy *phy);
int ad9361_dig_tune(struct ad9361_rf_phy *phy, unsigned long max_freq,
			   enum dig_tune_flags flags);
int ad9361_tx_mute(struct ad9361_rf_phy *phy, u32 state);
int ad9361_write_bist_reg(struct ad9361_rf_phy *phy, u32 val);
bool ad9361_uses_rx2tx2(struct ad9361_rf_phy *phy);
int ad9361_get_dig_tune_data(struct ad9361_rf_phy *phy,
			     struct ad9361_dig_tune_data *data);
int ad9361_read_clock_data_delays(struct ad9361_rf_phy *phy);
int ad9361_write_clock_data_delays(struct ad9361_rf_phy *phy);
bool ad9361_uses_lvds_mode(struct ad9361_rf_phy *phy);

#endif

