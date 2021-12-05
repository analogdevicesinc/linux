

::: linux:Documentation/devicetree/bindings/iio/adc/adi,ad7606.yaml


=== "sysfs"

    ```bash
    root:/> cd /sys/bus/iio/devices/
    root:/sys/bus/iio/devices> ls
    iio:device4  iio:trigger0

    root:/sys/bus/iio/devices> cd iio:device4

    root:/sys/bus/iio/devices/iio:device4> ls -l
    drwxr-xr-x    2 root     root             0 Jan  1 00:00 buffer
    -r--r--r--    1 root     root          4096 Jan  1 00:00 dev
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_calibbias
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_calibphase
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_calibscale
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_filter_high_pass_3db_frequency
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage0_test_mode
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_calibbias
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_calibphase
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_calibscale
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_filter_high_pass_3db_frequency
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage1_test_mode
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage_sampling_frequency
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 in_voltage_scale
    -r--r--r--    1 root     root          4096 Jan  1 00:00 in_voltage_scale_available
    -r--r--r--    1 root     root          4096 Jan  1 00:00 in_voltage_test_mode_available
    -r--r--r--    1 root     root          4096 Jan  1 00:00 name
    drwxr-xr-x    2 root     root             0 Jan  1 00:00 scan_elements
    lrwxrwxrwx    1 root     root             0 Jan  1 00:00 subsystem -> ../../../../bus/iio
    -rw-r--r--    1 root     root          4096 Jan  1 00:00 uevent
    root:/sys/bus/iio/devices/iio:device4>
    ```

=== "libiio API "

	<div class="iio-context">
	
	<p><img src='https://analogdevicesinc.github.io/libiio/img/iio_logo.png' alt='libiio' style='height:40px;vertical-align: middle;'> <span style='vertical-align: middle;'><b>ad9361-phy</b></span></p>
	<p>This is a device</p>
	<p class="devattr-header"><i>DEVICE ATTRIBUTES</i>: <b>ad9361-phy</b></p>
	<!-- <hr class="devattr-table-hr"> -->
	<table class="devattr-table">
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">calib_mode</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">Set calibration mode</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">calib_mode_available</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">dcxo_tune_coarse</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">dcxo_tune_coarse_available</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">dcxo_tune_fine</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">dcxo_tune_fine_available</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">ensm_mode</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">ensm_mode_available</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">filter_fir_config</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">gain_table_config</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">multichip_sync</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">rssi_gain_step_error</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">rx_path_rates</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">trx_rate_governor</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">trx_rate_governor_available</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">tx_path_rates</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">xo_correction</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">xo_correction_available</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>

	</table>

	<br>
	<!-- <hr> -->
	<br>
	<p class="devattr-header"><i>CHANNEL ATTRIBUTES</i>: <b>ad9361-phy</b></p>
	<table class="devattr-table">
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">external</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage1, altvoltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage1, altvoltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">fastlock_load</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage1, altvoltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage1, altvoltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">fastlock_recall</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage1, altvoltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage1, altvoltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">fastlock_save</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage1, altvoltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage1, altvoltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">fastlock_store</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage1, altvoltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage1, altvoltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">frequency</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage1, altvoltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage1, altvoltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">frequency_available</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage1, altvoltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage1, altvoltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">powerdown</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage1, altvoltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage1, altvoltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">bb_dc_offset_tracking_en</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">filter_fir_en</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">gain_control_mode</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">gain_control_mode_available</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">hardwaregain</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">hardwaregain_available</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">quadrature_tracking_en</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">rf_bandwidth</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">rf_bandwidth_available</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">rf_dc_offset_tracking_en</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">rf_port_select</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">rf_port_select_available</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">rssi</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">sampling_frequency</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">sampling_frequency_available</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage3, voltage2, voltage0, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">raw</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage3, voltage2, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage3, voltage2, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">scale</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage3, voltage2, voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage3, voltage2, voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">input</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>temp0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>temp0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">offset</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">voltage_filter_fir_en</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>out</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>out</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>

	</table>    


	<p><b>xadc</b></p>
	<p>This is a device</p>
	<p class="devattr-header"><i>DEVICE ATTRIBUTES</i>: <b>xadc</b></p>
	<!-- <hr class="devattr-table-hr"> -->
	<table class="devattr-table">
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">sampling_frequency</td>
	<td class="devattr-type-col">str</td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>

	</table>

	<br>
	<!-- <hr> -->
	<br>
	<p class="devattr-header"><i>CHANNEL ATTRIBUTES</i>: <b>xadc</b></p>
	<table class="devattr-table">
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">raw</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage5, voltage0, voltage4, temp0, voltage7, voltage1, voltage2, voltage3, voltage8, voltage6</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage5, voltage0, voltage4, temp0, voltage7, voltage1, voltage2, voltage3, voltage8, voltage6</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">scale</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage5, voltage0, voltage4, temp0, voltage7, voltage1, voltage2, voltage3, voltage8, voltage6</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage5, voltage0, voltage4, temp0, voltage7, voltage1, voltage2, voltage3, voltage8, voltage6</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">offset</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>temp0</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>temp0</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>

	</table>    


	<br>
	<!-- <hr> -->
	<br>
	<p class="devattr-header"><i>CHANNEL ATTRIBUTES</i>: <b>cf-ad9361-dds-core-lpc</b></p>
	<table class="devattr-table">
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">calibphase</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">calibscale</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">sampling_frequency</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1, altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1, altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">sampling_frequency_available</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">frequency</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">phase</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">raw</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">scale</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>altvoltage3, altvoltage1, altvoltage0, altvoltage2</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>

	</table>    


	<br>
	<!-- <hr> -->
	<br>
	<p class="devattr-header"><i>CHANNEL ATTRIBUTES</i>: <b>cf-ad9361-lpc</b></p>
	<table class="devattr-table">
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">calibbias</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">calibphase</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">calibscale</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">samples_pps</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">sampling_frequency</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>
	<tr>
	<td class="devattr-topleft-col"></td>
	<td class="devattr-name-col">sampling_frequency_available</td>
	<td class="devattr-type-col">str</td>
	<!-- <span>voltage0, voltage1</span></td> -->
	</tr>
	<tr>
	<td></td>
	<td></td>
	<td class="devattr-type-col"><span>voltage0, voltage1</span></td>
	</tr>
	<tr>
	<td class="devattr-bottomleft-col"></td>
	<td class="devattr-spacer-col"></td>
	<td class="devattr-about">NA</td>
	</tr>

	</table>    



	</div>
