# gen_gmsl_dts tool

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [JSON Configuration Parameters](#json-configuration-parameters)
- [Example Configuration](#example-configuration)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [References](#references)

## Overview

The `gen_gmsl_dts` tool automates the generation of Device Tree Source (DTS) overlays tailored for Gigabit Multimedia Serial Link (GMSL) camera configurations on NVIDIA Jetson Orin (Tegra234) platforms. It simplifies the integration of GMSL serializers, deserializers, and camera modules by translating JSON configuration files into corresponding DTS overlays.

## Prerequisites

- NVIDIA Jetson Orin running a compatible Linux kernel (e.g., `gmsl/tegra-6.12.y` branch from Analog Devices).
- Python 3.x installed on the system.
- Access to the `gen_gmsl_dts` directory within the Analog Devices Linux kernel repository.

## JSON Configuration Parameters

The JSON configuration file defines the GMSL setup. Below are the primary parameters:

- **`name`**: Specifies the deserializer model. Can be one of:
  - `"max96716a"`, `"max96724"`

- **`i2c_bus`**: Configure the I2C bus the deserializer is connected to. For Tegra234:
  - `"cam_mux"`: Camera I2C mux on Jetson Orin

- **`platform_cfg`**: Platform specific configurations:
  - `name`: Platform name (e.g., `"tegra-234"`)
  - `csi_idx`: CSI port index (e.g., `0` for CAM0 | `1` for CAM1)
  - `phy_idx`: Deserializer MIPI PHY the platform is connected to
  - `port_index`: CSI port index on the Tegra SoC (e.g., `1` for CAM0 | `2` for CAM1)
  - `sensor_i2c_base`: Base I2C bus number for the camera sensors (e.g., `11`, `12`)

- **`phys`**: List of MIPI PHY specific configurations:
  - `phy_idx`: Deserializer PHY configured in this block (e.g., `0`, `2`)
  - `num_lanes`: Number of MIPI lanes the PHY will use
  - `link_frequencies`: The MIPI PHY lane rate. Because of double data rate on MIPI
  the rate must be half of the lane rate. (e.g. `[750000000]` for 1.5Gbps lane rate)
  - `clock_lanes`: `[0]` for normal or `[5]` alternate clocking mode
  - `data_lanes`: `[1, 2]` for 2 lanes | `[1, 2, 3, 4]` for 4 lanes
- **`links`**: List of GMSL links. One entry for every connected serializer
  - `name`: Specifies the serializer model. Can be:
    - `"max96717"`
  - `cameras`: Camera connected to the serializer:
    - `name`: Specifies the camera model. Can be:
        - `"imx219-tegra"`
  - `"pool_addrs"`: is the range of addresses that the ATC can assign to the camera device

## Example Configuration

```json
[
    {
        "name": "max96716a",
        "i2c_bus": "cam_mux",
        "platform_cfg": {
            "name": "tegra-234",
            "csi_idx": 0,
            "phy_idx": 0,
            "port_index": 1,
            "sensor_i2c_base": 11
        },
        "phys": [
            {
                "phy_idx": 0,
                "num_lanes": 2,
                "link_frequencies": [750000000],
                "clock_lanes": [0],
                "data_lanes": [1, 2]
            }
        ],
        "links": [
            {
                "name": "max96717",
                "cameras": [
                    {
                        "name": "imx219-tegra"
                    }
                ],
                "pool_addrs": ["0x50", "0x51"]
            }
        ]
    }
]
```

## Usage

1. **Navigate to the Tool Directory:**

   ```bash
   cd linux/arch/arm64/boot/dts/gen_gmsl_dts
   ```

2. **Prepare Your JSON Configuration:**

   In `gen_gmsl_dts` folder there are examples of pre defined configuration JSON files for a variety of serializers, deserializers and camera sensors. If none of these suit your usecase one can create a new JSON file (e.g., `gmsl_config.json`). Please refer to the [JSON Configuration Parameters](#json-configuration-parameters) section for details.

3. **Generate the DTS Overlay:**

   ```bash
   python3 gen_gmsl_dts.py gmsl_config.json --dtbo -o ../gmsl-overlay.dts
   ```

   This command will generate a DTS overlay file named `gmsl-overlay.dts` based on your configuration.

4. **Compile the DTS Overlay:**

   From the `linux/arch/arm64/boot/dts/gen_gmsl_dts` folder, compile the generated overlay using `dtc`:

   ```bash
   dtc -@ -I dts -O dtb -o gmsl.dtbo ../gmsl-overlay.dts
   ```

   **Note:** Warnings from `dtc` are expected and can be safely ignored.

5. **(Optional) Copy the Overlay from the Host to the Target (Jetson Orin):**

   If the overlay was compiled on a host machine, copy the `.dtbo` file to the Jetson Orin:

   ```bash
   scp gmsl.dtbo <target:path/to/gmsl_dtbo_dir>
   ```

   Example of `<target:path/to/gmsl_dtbo_dir>`: `analog@jetson-orin:/home/analog/`

### **Note:** The following commands can only be run on the Jetson Orin

6. **Apply the Overlay Over the Base DTB:**

   Use `fdtoverlay` to merge the GMSL overlay into the base Tegra device tree:

   ```bash
   fdtoverlay -i /boot/tegra234-p3768-0000+p3767-0005-nv-super.dtb -o tegra234-p3768-0000+p3767-0005-nv-super-gmsl-cam.dtb gmsl.dtbo
   ```

   **Note:** The base DTB name may vary depending on your Jetson Orin module variant. Check `/boot/` for the correct DTB file name.

7. **Copy the New Device Tree to the Boot Directory:**

   ```bash
   sudo cp tegra234-p3768-0000+p3767-0005-nv-super-gmsl-cam.dtb /boot/
   ```

8. **Update the Boot Configuration:**

   Edit `/boot/extlinux/extlinux.conf` (for example, by using `nano` or `vim`) and update the `FDT` line to point to the new DTB:

   ```
   FDT /boot/tegra234-p3768-0000+p3767-0005-nv-super-gmsl-cam.dtb
   ```

9. **Reboot the System:**

   ```bash
   sudo reboot
   ```

## Troubleshooting

- **Driver Probe:**

  Check that the deserializer and serializer drivers probed successfully:

  ```bash
  sudo dmesg | grep max
  ```

  The output shows the I2C bus and address (e.g., `max96724 1-0027` means bus 1, address 0x27). Use these bus numbers to scan for devices:

  ```bash
  sudo i2cdetect -y -r 1
  ```

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

  Your GMSL parts should appear in the media devices tree. There may be multiple devices created in `/dev/media*`. Select the correct one with the `-d` flag.

## References

- Analog Devices GMSL Linux Kernel Repository: [https://github.com/analogdevicesinc/linux](https://github.com/analogdevicesinc/linux)
- GMSL support landing page: [Gigabit Multimedia Serial Link™ (GMSL) technology from Analog Devices Inc.](https://github.com/analogdevicesinc/gmsl)
