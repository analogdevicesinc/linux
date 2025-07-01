# gen_gmsl_dts tool

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Usage](#usage)
- [JSON Configuration Parameters](#json-configuration-parameters)
- [Example Configuration](#example-configuration)
- [Generating the DTS Overlay](#generating-the-dts-overlay)
- [Applying the Overlay](#applying-the-overlay)
- [Troubleshooting](#troubleshooting)
- [References](#references)

## Overview

The `gen_gmsl_dts` tool automates the generation of Device Tree Source (DTS) overlays tailored for Gigabit Multimedia Serial Link (GMSL) camera configurations on Raspberry Pi platforms. It simplifies the integration of GMSL serializers, deserializers, and camera modules by translating JSON configuration files into corresponding DTS overlays.

## Prerequisites

- Raspberry Pi running a compatible Linux kernel (e.g., `gmsl/rpi-6.13.y` branch from Analog Devices).
- Python 3.x installed on the system.
- Access to the `gen_gmsl_dts` directory within the Analog Devices Linux kernel repository.

## Usage

1. **Navigate to the Tool Directory:**

   ```bash
   cd linux/arch/arm/boot/dts/overlays/gen_gmsl_dts
   ```

2. **Prepare Your JSON Configuration:**

   In `gen_gmsl_dts` folder there are examples of pre defined configuration JSON files for a variety of serializers, deserializers and camera sensors. If none of these suit your usecase one can create a new JSON file (e.g., `gmsl_config.json`). Plsease refer to the [JSON Configuration Parameters](#json-configuration-parameters) section for details.

3. **Generate the DTS Overlay:**

   ```bash
   python3 gen_gmsl_dts.py gmsl_config.json --dtbo --o ../gmsl-overlay.dts
   ```

   This command will generate a DTS overlay file named `gmsl-overlay.dts` based on your configuration.

## JSON Configuration Parameters

The JSON configuration file defines the GMSL setup. Below are the primary parameters:

- **`name`**: Specifies the deserializer model. Can be one of:
  - `"max9296a"`, `"max96712"`, `"max96714"`, `"max96716"`, `"max96724"`, `"max96792a"`

- **`i2c_bus`**: Configure the RPI I2C bus the deserializer is connected to. Can be:
  - `"i2c_csi_dsi0"`: CAM0 on RPI5
  - `"i2c_csi_dsi1"`: CAM1 on RPI5
  - `"i2c_csi_dsi"`: CAM port on RPI4

- **`platform_cfg`**: Platform (RPI) specific configurations:
  - `name`: RPI model (e.g., `"rpi-5-b"`, `"rpi-4-b"`)
  - `csi_idx`: CSI port (e.g., `1` for RPI4 | `0` or `1` for RPI5)
  - `phy_idx`: Deserializer MIPI PHY the RPI is connected to

- **`phys`**: List of MIPI PHY specificn configurations:
  - `phy_idx`: Deserializer PHY configured in this block (e.g., `0`, `2`)
  - `num_lanes`: Number of MIPI lanes the PHY will use
  - `link_frequencies`: The MIPI PHY lane rate. Because of double data rate on MIPI
  the rate must be half of the lane rate. (e.g. `[750000000]` for 1.5Gbps lane rate)
  - `clock_lanes`: `[0]` for normal or `[5]` alternate clocking mode
  - `data_lanes`: `[1, 2]` for 2 lanes | `[1, 2, 3, 4]` for 4 lanes
- **`links`**: List of GMSL links. One entry for every connected serializer
  - `name`: Specifies the deserializer model. Can be one of:
    - `"max96717"`, `"max9295a"`, `"max96793"`
  - `cameras`: Camera connected to the serializer:
    - `name`: camera model can be `"imx219"` or `"ov5640"`
  - `"pool_addrs"`: is the range of addresses that the ATC and assign to the camera device

## Example Configuration

```json
[
    {
        "name": "max96724",
        "i2c_bus": "i2c_csi_dsi0",
        "platform_cfg": {
            "name": "rpi-5-b",
            "csi_idx": 0,
            "phy_idx": 2
        },
        "phys": [
            {
                "phy_idx": 2,
                "num_lanes": 4,
                "link_frequencies": [750000000],
                "clock_lanes": [0],
                "data_lanes": [1, 2, 3, 4]
            }
        ],
        "links": [
            {
                "name": "max96717",
                "cameras": [
                    {
                        "name": "imx219"
                    }
                ],
                "pool_addrs": ["0x50", "0x51"]
            },
            {
                "name": "max96717",
                "cameras": [
                    {
                        "name": "imx219"
                    }
                ],
                "pool_addrs": ["0x52", "0x53"]
            },
            {
                "name": "max96717",
                "cameras": [
                    {
                        "name": "imx219"
                    }
                ],
                "pool_addrs": ["0x54", "0x55"]
            },
            {
                "name": "max96717",
                "cameras": [
                    {
                        "name": "imx219"
                    }
                ],
                "pool_addrs": ["0x56", "0x57"]
            }
        ]
    }
]
```

## Applying the Overlay

1. **Compile the DTS Overlay:**

   ```bash
   make dtbs
   ``` 

2. **Copy the Overlay to the Boot Directory:**

   ```bash
   sudo cp gmsl.dtbo /boot/overlays/
   ```

3. **Edit the Boot Configuration:**

   Add the overlay to your `/boot/config.txt` file:

   ```ini
   dtoverlay=gmsl
   ```

4. **Reboot the System:**

   ```bash
   sudo reboot
   ```

## Troubleshooting

- **I2C Device Detection:**

  Ensure that the I2C devices are detected:

  ```bash
  sudo i2cdetect -y 10
  ```

  You should see entries corresponding to your deserializer and serializers.

- **Video Device Verification:**

  Check if the video devices are recognized:

  ```bash
  v4l2-ctl --list-devices
  ```

  - **Media Device Verification:**

  Check if the media devices are recognized:

  ```bash
  media-ctl -p
  ```

  Your GMSL parts should appear in the media devices tree. Take care there may be multiple devices created in /dev/media*. Select the correct one with -d flag.

## References

- Analog Devices GMSL Linux Kernel Repository: [https://github.com/analogdevicesinc/linux](https://github.com/analogdevicesinc/linux)
- GMSL support landing page: [Gigabit Multimedia Serial Link™ (GMSL) technology from Analog Devices Inc.](https://github.com/analogdevicesinc/gmsl)
- Raspberry Pi User Guide for GMSL Cameras: [analogdevicesinc.github.io/documentation](https://analogdevicesinc.github.io/documentation/solutions/reference-designs/ad-gmslcamrpi-adp/raspberry-pi-user-guide/index.html)
