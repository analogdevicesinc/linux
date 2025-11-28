# AD9088 Tools

This directory contains standalone utility tools for the AD9088 driver.

## Tools

### ad9088_cal_dump

A command-line utility for inspecting and validating AD9088 calibration data files.

**Features:**
- Validates calibration file format and CRC
- Displays header information (chip ID, configuration, version)
- Shows section offsets and sizes
- Previews calibration data
- Detects corrupted or uninitialized data

**Building:**

```bash
make
```

**Usage:**

```bash
./ad9088_cal_dump <calibration_file>
```

**Example:**

```bash
./ad9088_cal_dump /lib/firmware/ad9088_cal.bin
```

**Installation (optional):**

```bash
sudo make install
```

This installs to `/usr/local/bin/ad9088_cal_dump`.


