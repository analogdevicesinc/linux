# AD9088 Calibration Data Save/Restore

## Overview

The AD9088 driver provides a comprehensive calibration data management system that allows you to save and restore calibration data across power cycles. This is useful for:

- Reducing boot time by skipping calibration
- Maintaining consistent performance across reboots
- Factory calibration persistence
- Backup and restore of device calibration

## File Format

The calibration data is stored in a binary format with the following structure:

### Header (64 bytes)
- **Magic** (4 bytes): 0x41443930 ("AD90") - File identification
- **Version** (4 bytes): Format version (currently 1)
- **Chip ID** (4 bytes): Device chip ID (0x9084 or 0x9088)
- **Device Config** (4 bytes): 8T8R vs 4T4R configuration
- **Section Offsets** (16 bytes): Offsets to each calibration section
- **Section Sizes** (16 bytes): Size of each calibration section
- **Reserved** (16 bytes): For future use

### Calibration Sections

1. **ADC Calibration Data**
   - Sequential mode data for all ADCs
   - Random mode data for all ADCs
   - Each ADC has its own calibration data with embedded CRC

2. **DAC Calibration Data**
   - Calibration data for all DACs
   - Each DAC has its own calibration data with embedded CRC

3. **SERDES RX Calibration Data**
   - Calibration data for all SERDES RX 12-packs
   - Each SERDES has its own calibration data with embedded CRC

4. **SERDES TX Calibration Data**
   - Calibration data for all SERDES TX 12-packs
   - Each SERDES has its own calibration data with embedded CRC

### Footer
- **CRC32** (4 bytes): Checksum of entire file (excluding CRC itself)

## Usage

### Sysfs Interface

The calibration data is accessible via a sysfs bin attribute:

```
/sys/bus/spi/devices/spi0.0/iio:device0/calibration_data
```

### Saving Calibration Data

After the device has been initialized and calibrated, save the calibration data:

```bash
# Read calibration data from device and save to file
cat /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data > /lib/firmware/ad9088_cal.bin

# Verify the file was created
ls -lh /lib/firmware/ad9088_cal.bin
```

The save operation will:
1. Read all ADC calibration data (sequential and random modes)
2. Read all DAC calibration data
3. Read all SERDES RX calibration data
4. Read all SERDES TX calibration data
5. Build a binary file with header and CRC32
6. Return the complete calibration image

Typical calibration data size:
- **4T4R**: ~50-100 KB
- **8T8R**: ~100-200 KB

### Restoring Calibration Data

To restore calibration data from a previously saved file:

```bash
# Write calibration data back to device
cat /lib/firmware/ad9088_cal.bin > /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data
```

The restore operation will:
1. Validate magic number and file format version
2. Verify chip ID matches current device
3. Verify device configuration (4T4R/8T8R) matches
4. Validate CRC32 of entire file
5. Restore ADC calibration data
6. Restore DAC calibration data
7. Restore SERDES RX calibration data
8. Restore SERDES TX calibration data

### Complete Workflow

Here's the recommended workflow for using automatic calibration restore:

#### First Boot (Capture Calibration)

1. **Boot without calibration restore** (don't add device tree property yet):
   ```bash
   # Device boots and performs full calibration (takes longer)
   ```

2. **Save calibration data**:
   ```bash
   cat /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data > /lib/firmware/ad9088_cal.bin
   ```

3. **Verify the saved data**:
   ```bash
   ls -lh /lib/firmware/ad9088_cal.bin
   # Should show file size (typically 50-200KB depending on configuration)
   ```

4. **Test restore** (optional but recommended):
   ```bash
   # Write the calibration data back to verify it works
   cat /lib/firmware/ad9088_cal.bin > /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data
   # Check dmesg for "Calibration data restored successfully"
   ```

5. **Update device tree** to enable automatic restore:
   ```dts
   adi,device-calibration-data-name = "ad9088_cal.bin";
   ```

6. **Rebuild and deploy** device tree

#### Subsequent Boots

On subsequent boots, the driver will:
- Automatically load `/lib/firmware/ad9088_cal.bin`
- Restore calibration data to hardware
- Skip time-consuming calibration procedures
- Boot faster while maintaining performance

### Automatic Restore on Boot (Alternative Methods)

If you don't want to use device tree integration, you can use a systemd service or init script:

#### Systemd Service

Create `/etc/systemd/system/ad9088-cal-restore.service`:

```ini
[Unit]
Description=Restore AD9088 Calibration Data
After=multi-user.target
Requires=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'if [ -f /lib/firmware/ad9088_cal.bin ]; then cat /lib/firmware/ad9088_cal.bin > /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data; fi'
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
```

Enable the service:

```bash
systemctl enable ad9088-cal-restore.service
systemctl start ad9088-cal-restore.service
```

#### Init Script

Add to your init script (e.g., `/etc/rc.local`):

```bash
# Restore AD9088 calibration data if available
if [ -f /lib/firmware/ad9088_cal.bin ]; then
    cat /lib/firmware/ad9088_cal.bin > /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data
fi
```

## Error Handling

### Save Errors

If saving fails, check:
- Device is properly initialized
- Initial calibration has completed
- Sufficient memory is available

### Restore Errors

Common errors and solutions:

| Error | Cause | Solution |
|-------|-------|----------|
| Invalid magic | Wrong file format | Use file saved by this driver |
| Version mismatch | Incompatible format version | Re-save calibration data with current driver |
| Chip ID mismatch | File from different device | Use calibration data from same chip type |
| Config mismatch | 4T4R vs 8T8R mismatch | Use calibration data from same device config |
| CRC error | File corruption | Re-save calibration data |
| Size mismatch | Truncated file | Check file was completely written |

## Best Practices

1. **Save After Initial Calibration**: Save calibration data immediately after successful initial calibration

2. **Verify Saved Data**: Always verify the saved file can be restored before relying on it:
   ```bash
   # Save
   cat /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data > /tmp/test_cal.bin

   # Restore and verify no errors
   cat /tmp/test_cal.bin > /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data
   ```

3. **Keep Backups**: Maintain multiple calibration backups:
   ```bash
   # Timestamped backup
   cat /sys/bus/spi/devices/spi0.0/iio:device0/calibration_data > \
       /lib/firmware/ad9088_cal_$(date +%Y%m%d_%H%M%S).bin
   ```

4. **Temperature Considerations**: Calibration data is temperature-dependent. Consider:
   - Saving calibration at operating temperature
   - Re-calibrating if temperature changes significantly
   - Maintaining separate calibration files for different temperature ranges

5. **Version Control**: Track calibration data with device information:
   ```bash
   # Create metadata file
   echo "Chip ID: $(dmesg | grep AD9088 | grep 'Chip ID')" > /lib/firmware/ad9088_cal.txt
   echo "Date: $(date)" >> /lib/firmware/ad9088_cal.txt
   echo "Temperature: $(cat /sys/class/hwmon/hwmon0/temp1_input)" >> /lib/firmware/ad9088_cal.txt
   ```

## Integration with Device Tree

The calibration restore can be integrated with device initialization to automatically load calibration data during driver probe. This eliminates the need for manual systemd services or init scripts.

### Automatic Calibration Load at Boot

Add the calibration firmware property to your device tree:

```dts
&spi0 {
    ad9088@0 {
        compatible = "adi,ad9088";
        reg = <0>;
        spi-max-frequency = <10000000>;

        /* Automatically load calibration data from firmware at boot */
        adi,device-calibration-data-name = "ad9088_cal.bin";
    };
};
```

When this property is present, the driver will:
1. Request the firmware file from `/lib/firmware/` during probe
2. Validate the calibration data (magic, version, chip ID, CRC)
3. Restore the calibration data to hardware
4. Continue with normal device initialization

If the property is not present, the driver will skip calibration restore and perform normal initialization with full calibration.

**Important Notes:**
- The calibration file must be placed in `/lib/firmware/` before boot
- If the file is missing or invalid, driver probe will fail
- This happens automatically after firmware load but before any other hardware configuration
- The calibration data must match the device (same chip ID and 4T4R/8T8R configuration)

## Debugging

Enable debug output:

```bash
# Enable driver debug messages
echo 8 > /proc/sys/kernel/printk
echo 'file ad9088_cal.c +p' > /sys/kernel/debug/dynamic_debug/control

# Check kernel messages
dmesg | grep -i "calibration\|ad9088"
```

Example successful save output:
```
ad9088 spi0.0: Saving calibration data...
ad9088 spi0.0: Reading ADC calibration data...
ad9088 spi0.0: Reading DAC calibration data...
ad9088 spi0.0: Reading SERDES RX calibration data...
ad9088 spi0.0: Reading SERDES TX calibration data...
ad9088 spi0.0: Calibration data saved: 102400 bytes (ADC: 51200, DAC: 25600, SERDES RX: 12800, SERDES TX: 12800)
```

Example successful restore output:
```
ad9088 spi0.0: Restoring calibration data...
ad9088 spi0.0: Restoring ADC calibration data...
ad9088 spi0.0: Restoring DAC calibration data...
ad9088 spi0.0: Restoring SERDES RX calibration data...
ad9088 spi0.0: Restoring SERDES TX calibration data...
ad9088 spi0.0: Calibration data restored successfully
```

## File Format Changes

If the file format needs to be updated in the future, the version field in the header will be incremented. The driver will reject files with unsupported versions to prevent corruption.

## Security Considerations

- The calibration_data sysfs attribute has 0600 permissions (root read/write only)
- Calibration data is device-specific and cannot be shared between different chip instances
- CRC32 validation ensures data integrity
- No sensitive information is stored in calibration data
