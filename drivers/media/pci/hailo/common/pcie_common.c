// SPDX-License-Identifier: GPL-2.0
/**
 * Copyright (c) 2019-2022 Hailo Technologies Ltd. All rights reserved.
 **/

#include "pcie_common.h"
#include "fw_operation.h"

#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/kernel.h>


#define BSC_IMASK_HOST (0x0188)
#define BCS_ISTATUS_HOST (0x018C)
#define BCS_SOURCE_INTERRUPT_PER_CHANNEL (0x400)
#define BCS_DESTINATION_INTERRUPT_PER_CHANNEL (0x500)

#define PO2_ROUND_UP(size, alignment) ((size + alignment-1) & ~(alignment-1))

#define ATR0_PARAM (0x17)
#define ATR0_SRC_ADDR (0x0)
#define ATR0_TRSL_ADDR2 (0x0)
#define ATR0_TRSL_PARAM (6)

#define ATR0_PCIE_BRIDGE_OFFSET (0x700)
#define ATR0_TABLE_SIZE (0x1000u)
#define ATR0_TABLE_SIZE_MASK (0x1000u - 1)

#define MAXIMUM_APP_FIRMWARE_CODE_SIZE (0x40000)
#define MAXIMUM_CORE_FIRMWARE_CODE_SIZE (0x20000)

#define FIRMWARE_LOAD_WAIT_MAX_RETRIES (100)
#define FIRMWARE_LOAD_SLEEP_MS         (50)

#define PCIE_APP_CPU_DEBUG_OFFSET (8*1024)
#define PCIE_CORE_CPU_DEBUG_OFFSET (PCIE_APP_CPU_DEBUG_OFFSET + DEBUG_BUFFER_TOTAL_SIZE)

#define PCIE_D2H_NOTIFICATION_SRAM_OFFSET (0x640 + 0x640)
#define PCIE_REQUEST_SIZE_OFFSET (0x640)

#define PCIE_CONFIG_VENDOR_OFFSET (0x0098)

#define HAILO_PCIE_HOST_DMA_DATA_ID (0)
#define HAILO_PCIE_DMA_DEVICE_INTERRUPTS_BITMASK    (1 << 4)
#define HAILO_PCIE_DMA_HOST_INTERRUPTS_BITMASK      (1 << 5)

typedef u32 hailo_ptr_t;

struct hailo_fw_addresses {
    u32 boot_fw_header;
    u32 app_fw_code_ram_base;
    u32 boot_key_cert;
    u32 boot_cont_cert;
    u32 boot_fw_trigger;
    u32 core_code_ram_base;
    u32 core_fw_header;
    u32 atr0_trsl_addr1;
    u32 raise_ready_offset;
};

struct hailo_atr_config {
    u32 atr_param;
    u32 atr_src;
    u32 atr_trsl_addr_1;
    u32 atr_trsl_addr_2;
    u32 atr_trsl_param;
};

struct hailo_board_compatibility {
    struct hailo_fw_addresses fw_addresses;
    const char *fw_filename;
    const struct hailo_config_constants board_cfg;
    const struct hailo_config_constants fw_cfg;
};

static const struct hailo_board_compatibility compat[HAILO_BOARD_TYPE_COUNT] = {
    [HAILO_BOARD_TYPE_HAILO8] = {
        .fw_addresses = {
            .boot_fw_header = 0xE0030,
            .boot_fw_trigger = 0xE0980,
            .boot_key_cert = 0xE0048,
            .boot_cont_cert = 0xE0390,
            .app_fw_code_ram_base = 0x60000,
            .core_code_ram_base = 0xC0000,
            .core_fw_header = 0xA0000,
            .atr0_trsl_addr1 = 0x60000000,
            .raise_ready_offset = 0x1684,
        },
        .fw_filename = "hailo/hailo8_fw.bin",
        .board_cfg = {
            .filename = "hailo/hailo8_board_cfg.bin",
            .address = 0x60001000,
            .max_size = PCIE_HAILO8_BOARD_CFG_MAX_SIZE,
        },
        .fw_cfg = {
            .filename = "hailo/hailo8_fw_cfg.bin",
            .address = 0x60001500,
            .max_size = PCIE_HAILO8_FW_CFG_MAX_SIZE,
        },
    },
    [HAILO_BOARD_TYPE_HAILO15] = {
        .fw_addresses = {
            .boot_fw_header = 0x88000,
            .boot_fw_trigger = 0x88c98,
            .boot_key_cert = 0x88018,
            .boot_cont_cert = 0x886a8,
            .app_fw_code_ram_base = 0x20000,
            .core_code_ram_base = 0x60000,
            .core_fw_header = 0xC0000,
            .atr0_trsl_addr1 = 0x000BE000,
            .raise_ready_offset = 0x1754,
        },
        .fw_filename = "hailo/hailo15_fw.bin",
        .board_cfg = {
            .filename = NULL,
            .address = 0,
            .max_size = 0,
        },
        .fw_cfg = {
            .filename = NULL,
            .address = 0,
            .max_size = 0,
        },
    },
    // HRT-11344 : none of these matter except raise_ready_offset seeing as we load fw seperately - not through driver
    // After implementing bootloader put correct values here
    [HAILO_BOARD_TYPE_PLUTO] = {
        .fw_addresses = {
            .boot_fw_header = 0x88000,
            .boot_fw_trigger = 0x88c98,
            .boot_key_cert = 0x88018,
            .boot_cont_cert = 0x886a8,
            .app_fw_code_ram_base = 0x20000,
            .core_code_ram_base = 0x60000,
            .core_fw_header = 0xC0000,
            .atr0_trsl_addr1 = 0x000BE000,
            // NOTE: After they update hw consts - check register fw_access_interrupt_w1s of pcie_config
            .raise_ready_offset = 0x174c,
        },
        .fw_filename = "hailo/pluto_fw.bin",
        .board_cfg = {
            .filename = NULL,
            .address = 0,
            .max_size = 0,
        },
        .fw_cfg = {
            .filename = NULL,
            .address = 0,
            .max_size = 0,
        },
    }
};


bool hailo_pcie_read_interrupt(struct hailo_pcie_resources *resources, struct hailo_pcie_interrupt_source *source)
{
    u32 channel_data_source = 0;
    u32 channel_data_dest = 0;
    memset(source, 0, sizeof(*source));

    source->interrupt_bitmask = hailo_resource_read32(&resources->config, BCS_ISTATUS_HOST);
    if (0 == source->interrupt_bitmask) {
        return false;
    }

    // clear signal
    hailo_resource_write32(&resources->config, BCS_ISTATUS_HOST, source->interrupt_bitmask);

    if (source->interrupt_bitmask & BCS_ISTATUS_HOST_VDMA_SRC_IRQ_MASK) {
        channel_data_source = hailo_resource_read32(&resources->config, BCS_SOURCE_INTERRUPT_PER_CHANNEL);
        hailo_resource_write32(&resources->config, BCS_SOURCE_INTERRUPT_PER_CHANNEL, channel_data_source);
    }
    if (source->interrupt_bitmask & BCS_ISTATUS_HOST_VDMA_DEST_IRQ_MASK) {
        channel_data_dest = hailo_resource_read32(&resources->config, BCS_DESTINATION_INTERRUPT_PER_CHANNEL);
        hailo_resource_write32(&resources->config, BCS_DESTINATION_INTERRUPT_PER_CHANNEL, channel_data_dest);
    }
    source->vdma_channels_bitmap = channel_data_source | channel_data_dest;

    return true;
}

int hailo_pcie_write_firmware_control(struct hailo_pcie_resources *resources, const struct hailo_fw_control *command)
{
    int err = 0;
    u32 request_size = 0;
    u8 fw_access_value = FW_ACCESS_APP_CPU_CONTROL_MASK;
    const struct hailo_fw_addresses *fw_addresses = &(compat[resources->board_type].fw_addresses);

    if (!hailo_pcie_is_firmware_loaded(resources)) {
        return -ENODEV;
    }

    // Copy md5 + buffer_len + buffer
    request_size = sizeof(command->expected_md5) + sizeof(command->buffer_len) + command->buffer_len;
    err = hailo_resource_write_buffer(&resources->fw_access, 0, PO2_ROUND_UP(request_size, FW_CODE_SECTION_ALIGNMENT),
        command);
    if (err < 0) {
        return err;
    }

    // Raise the bit for the CPU that will handle the control
    fw_access_value = (command->cpu_id == HAILO_CPU_ID_CPU1) ? FW_ACCESS_CORE_CPU_CONTROL_MASK :
        FW_ACCESS_APP_CPU_CONTROL_MASK;
    
    // Raise ready flag to FW
    hailo_resource_write32(&resources->fw_access, fw_addresses->raise_ready_offset, (u32)fw_access_value);
    return 0;
}

int hailo_pcie_read_firmware_control(struct hailo_pcie_resources *resources, struct hailo_fw_control *command)
{
    u32 response_header_size = 0;

    // Copy response md5 + buffer_len
    response_header_size = sizeof(command->expected_md5) + sizeof(command->buffer_len);

    hailo_resource_read_buffer(&resources->fw_access, PCIE_REQUEST_SIZE_OFFSET, response_header_size, command);

    if (sizeof(command->buffer) < command->buffer_len) {
        return -EINVAL;
    }

    // Copy response buffer
    hailo_resource_read_buffer(&resources->fw_access, PCIE_REQUEST_SIZE_OFFSET + (size_t)response_header_size,
        command->buffer_len, &command->buffer);
    
    return 0;
}

void hailo_pcie_write_firmware_driver_shutdown(struct hailo_pcie_resources *resources)
{
    const struct hailo_fw_addresses *fw_addresses = &(compat[resources->board_type].fw_addresses);
    const u32 fw_access_value = FW_ACCESS_DRIVER_SHUTDOWN_MASK;

    // Write shutdown flag to FW
    hailo_resource_write32(&resources->fw_access, fw_addresses->raise_ready_offset, fw_access_value);
}

int hailo_pcie_read_firmware_notification(struct hailo_pcie_resources *resources,
    struct hailo_d2h_notification *notification)
{
    struct hailo_resource notification_resource;

    if (PCIE_D2H_NOTIFICATION_SRAM_OFFSET > resources->fw_access.size) {
        return -EINVAL;
    }

    notification_resource.address = resources->fw_access.address + PCIE_D2H_NOTIFICATION_SRAM_OFFSET,
    notification_resource.size = sizeof(struct hailo_d2h_notification);

    return hailo_read_firmware_notification(&notification_resource, notification);
}

static void write_atr_table(struct hailo_pcie_resources *resources,
    struct hailo_atr_config *atr)
{
    hailo_resource_write_buffer(&resources->config, ATR0_PCIE_BRIDGE_OFFSET,
        sizeof(*atr), (void*)atr);
}

static void read_atr_table(struct hailo_pcie_resources *resources,
    struct hailo_atr_config *atr)
{
    hailo_resource_read_buffer(&resources->config, ATR0_PCIE_BRIDGE_OFFSET,
        sizeof(*atr), (void*)atr);
}

static void configure_atr_table(struct hailo_pcie_resources *resources,
    hailo_ptr_t base_address)
{
    struct hailo_atr_config atr = {
        .atr_param = ATR0_PARAM,
        .atr_src = ATR0_SRC_ADDR,
        .atr_trsl_addr_1 = (u32)base_address,
        .atr_trsl_addr_2 = ATR0_TRSL_ADDR2,
        .atr_trsl_param = ATR0_TRSL_PARAM
    };
    write_atr_table(resources, &atr);
}

static void write_memory_chunk(struct hailo_pcie_resources *resources,
    hailo_ptr_t dest, u32 dest_offset, const void *src, u32 len)
{
    BUG_ON(dest_offset + len > (u32)resources->fw_access.size);

    configure_atr_table(resources, dest);
    (void)hailo_resource_write_buffer(&resources->fw_access, dest_offset, len, src);
}

static void read_memory_chunk(
    struct hailo_pcie_resources *resources, hailo_ptr_t src, u32 src_offset, void *dest, u32 len)
{
    BUG_ON(src_offset + len > (u32)resources->fw_access.size);

    configure_atr_table(resources, src);
    (void)hailo_resource_read_buffer(&resources->fw_access, src_offset, len, dest);
}

// Note: this function modify the device ATR table (that is also used by the firmware for control and vdma).
// Use with caution, and restore the original atr if needed.
static void write_memory(struct hailo_pcie_resources *resources, hailo_ptr_t dest, const void *src, u32 len)
{
    hailo_ptr_t base_address = dest & ~ATR0_TABLE_SIZE_MASK;
    u32 chunk_len = 0;
    u32 offset = 0;

    if (base_address != dest) {
        // Data is not aligned, write the first chunk
        chunk_len = min(base_address + ATR0_TABLE_SIZE - dest, len);
        write_memory_chunk(resources, base_address, dest - base_address, src, chunk_len);
        offset += chunk_len;
    }

    while (offset < len) {
        chunk_len = min(len - offset, ATR0_TABLE_SIZE);
        write_memory_chunk(resources, dest + offset, 0, (const u8*)src + offset, chunk_len);
        offset += chunk_len;
    }
}

// Note: this function modify the device ATR table (that is also used by the firmware for control and vdma).
// Use with caution, and restore the original atr if needed.
static void read_memory(struct hailo_pcie_resources *resources, hailo_ptr_t src, void *dest, u32 len)
{
    hailo_ptr_t base_address = src & ~ATR0_TABLE_SIZE_MASK;
    u32 chunk_len = 0;
    u32 offset = 0;

    if (base_address != src) {
        // Data is not aligned, write the first chunk
        chunk_len = min(base_address + ATR0_TABLE_SIZE - src, len);
        read_memory_chunk(resources, base_address, src - base_address, dest, chunk_len);
        offset += chunk_len;
    }

    while (offset < len) {
        chunk_len = min(len - offset, ATR0_TABLE_SIZE);
        read_memory_chunk(resources, src + offset, 0, (u8*)dest + offset, chunk_len);
        offset += chunk_len;
    }
}

static void hailo_write_app_firmware(struct hailo_pcie_resources *resources, firmware_header_t *fw_header,
    secure_boot_certificate_t *fw_cert)
{
    const struct hailo_fw_addresses *fw_addresses = &(compat[resources->board_type].fw_addresses);
    void *fw_code = (void*)((u8*)fw_header + sizeof(firmware_header_t));
    void *key_data = &fw_cert->certificates_data[0];
    void *content_data = &fw_cert->certificates_data[fw_cert->key_size];

    write_memory(resources, fw_addresses->boot_fw_header, fw_header, sizeof(firmware_header_t));

    write_memory(resources, fw_addresses->app_fw_code_ram_base, fw_code, fw_header->code_size);

    write_memory(resources, fw_addresses->boot_key_cert, key_data, fw_cert->key_size);
    write_memory(resources, fw_addresses->boot_cont_cert, content_data, fw_cert->content_size);
}

static void hailo_write_core_firmware(struct hailo_pcie_resources *resources, firmware_header_t *fw_header)
{
    const struct hailo_fw_addresses *fw_addresses = &(compat[resources->board_type].fw_addresses);
    void *fw_code = (void*)((u8*)fw_header + sizeof(firmware_header_t));

    write_memory(resources, fw_addresses->core_code_ram_base, fw_code, fw_header->code_size);
    write_memory(resources, fw_addresses->core_fw_header, fw_header, sizeof(firmware_header_t));
}

static void hailo_trigger_firmware_boot(struct hailo_pcie_resources *resources)
{
    const struct hailo_fw_addresses *fw_addresses = &(compat[resources->board_type].fw_addresses);
    u32 pcie_finished = 1;

    write_memory(resources, fw_addresses->boot_fw_trigger,
        (void*)&pcie_finished, sizeof(pcie_finished));
}

/**
* Validates the FW headers.
* @param[in] address                    Address of the firmware.
* @param[in] firmware_size              Size of the firmware.
* @param[out] out_app_firmware_header   (optional) App firmware header
* @param[out] out_core_firmware_header  (optional) Core firmware header
* @param[out] out_firmware_cert         (optional) Firmware certificate header
*/
static int FW_VALIDATION__validate_fw_headers(uintptr_t firmware_base_address, size_t firmware_size,
    firmware_header_t **out_app_firmware_header, firmware_header_t **out_core_firmware_header,
    secure_boot_certificate_t **out_firmware_cert, enum hailo_board_type board_type)
{
    firmware_header_t *app_firmware_header = NULL;
    firmware_header_t *core_firmware_header = NULL;
    secure_boot_certificate_t *firmware_cert = NULL;
    int err = -EINVAL;
    u32 consumed_firmware_offset = 0;

    err = FW_VALIDATION__validate_fw_header(firmware_base_address, firmware_size, MAXIMUM_APP_FIRMWARE_CODE_SIZE,
        &consumed_firmware_offset, &app_firmware_header, board_type);
    if (0 != err) {
        err = -EINVAL;
        goto exit;
    }

    err = FW_VALIDATION__validate_cert_header(firmware_base_address, firmware_size,
        &consumed_firmware_offset, &firmware_cert);
    if (0 != err) {
        err = -EINVAL;
        goto exit;
    }

    err = FW_VALIDATION__validate_fw_header(firmware_base_address, firmware_size, MAXIMUM_CORE_FIRMWARE_CODE_SIZE,
        &consumed_firmware_offset, &core_firmware_header, board_type);
    if (0 != err) {
        err = -EINVAL;
        goto exit;
    }

    if (consumed_firmware_offset != firmware_size) {
        /* it is an error if there is leftover data after the last firmware header */
        err = -EINVAL;
        goto exit;
    }

    /* the out params are all optional */
    if (NULL != out_app_firmware_header) {
        *out_app_firmware_header = app_firmware_header;
    }
    if (NULL != out_firmware_cert) {
        *out_firmware_cert = firmware_cert;
    }
    if (NULL != out_core_firmware_header) {
        *out_core_firmware_header = core_firmware_header;
    }
    err = 0;

exit:
    return err;
}

int hailo_pcie_write_firmware(struct hailo_pcie_resources *resources, const void *fw_data, size_t fw_size)
{
    firmware_header_t *app_firmware_header = NULL;
    secure_boot_certificate_t *firmware_cert = NULL;
    firmware_header_t *core_firmware_header = NULL;

    int err = FW_VALIDATION__validate_fw_headers((uintptr_t)fw_data, fw_size,
        &app_firmware_header, &core_firmware_header, &firmware_cert, resources->board_type);
    if (err < 0) {
        return err;
    }

    hailo_write_app_firmware(resources, app_firmware_header, firmware_cert);
    hailo_write_core_firmware(resources, core_firmware_header);

    hailo_trigger_firmware_boot(resources);

    return 0;
}

bool hailo_pcie_is_firmware_loaded(struct hailo_pcie_resources *resources)
{
    u32 offset = ATR0_PCIE_BRIDGE_OFFSET + offsetof(struct hailo_atr_config, atr_trsl_addr_1);
    u32 atr_value = hailo_resource_read32(&resources->config, offset);
    return atr_value == compat[resources->board_type].fw_addresses.atr0_trsl_addr1;
}

bool hailo_pcie_wait_for_firmware(struct hailo_pcie_resources *resources)
{
    size_t retries;
    for (retries = 0; retries < FIRMWARE_LOAD_WAIT_MAX_RETRIES; retries++) {
        if (hailo_pcie_is_firmware_loaded(resources)) {
            return true;
        }

        msleep(FIRMWARE_LOAD_SLEEP_MS);
    }

    return false;
}

int hailo_pcie_write_config_common(struct hailo_pcie_resources *resources, const void* config_data,
    const size_t config_size, const struct hailo_config_constants *config_consts)
{
    if (config_size > config_consts->max_size) {
        return -EINVAL;
    }

    write_memory(resources, config_consts->address, config_data, (u32)config_size);
    return 0;
}

const struct hailo_config_constants* hailo_pcie_get_board_config_constants(const enum hailo_board_type board_type) {
    BUG_ON(board_type >= HAILO_BOARD_TYPE_COUNT || board_type < 0);
    return &compat[board_type].board_cfg;
}

const struct hailo_config_constants* hailo_pcie_get_user_config_constants(const enum hailo_board_type board_type) {
    BUG_ON(board_type >= HAILO_BOARD_TYPE_COUNT || board_type < 0);
    return &compat[board_type].fw_cfg;
}

const char* hailo_pcie_get_fw_filename(const enum hailo_board_type board_type) {
    BUG_ON(board_type >= HAILO_BOARD_TYPE_COUNT || board_type < 0);
    return compat[board_type].fw_filename;
}

void hailo_pcie_update_channel_interrupts_mask(struct hailo_pcie_resources* resources, u32 channels_bitmap)
{
    size_t i = 0;
    u32 mask = hailo_resource_read32(&resources->config, BSC_IMASK_HOST);

    // Clear old channel interrupts
    mask &= ~BCS_ISTATUS_HOST_VDMA_SRC_IRQ_MASK;
    mask &= ~BCS_ISTATUS_HOST_VDMA_DEST_IRQ_MASK;
    // Set interrupt by the bitmap
    for (i = 0; i < MAX_VDMA_CHANNELS_PER_ENGINE; ++i) {
        if (hailo_test_bit(i, &channels_bitmap)) {
            // based on 18.5.2 "vDMA Interrupt Registers" in PLDA documentation
            u32 offset = (i < VDMA_DEST_CHANNELS_START) ? 0 : 8;
            hailo_set_bit((((int)i*8) / MAX_VDMA_CHANNELS_PER_ENGINE) + offset, &mask);
        }
    }
    hailo_resource_write32(&resources->config, BSC_IMASK_HOST, mask);
}

void hailo_pcie_enable_interrupts(struct hailo_pcie_resources *resources)
{
    u32 mask = hailo_resource_read32(&resources->config, BSC_IMASK_HOST);

    hailo_resource_write32(&resources->config, BCS_ISTATUS_HOST, 0xFFFFFFFF);
    hailo_resource_write32(&resources->config, BCS_DESTINATION_INTERRUPT_PER_CHANNEL, 0xFFFFFFFF);
    hailo_resource_write32(&resources->config, BCS_SOURCE_INTERRUPT_PER_CHANNEL, 0xFFFFFFFF);

    mask |= BCS_ISTATUS_HOST_FW_IRQ_CONTROL_MASK | BCS_ISTATUS_HOST_FW_IRQ_NOTIFICATION | BCS_ISTATUS_HOST_DRIVER_DOWN;
    hailo_resource_write32(&resources->config, BSC_IMASK_HOST, mask);
}

void hailo_pcie_disable_interrupts(struct hailo_pcie_resources* resources)
{
    hailo_resource_write32(&resources->config, BSC_IMASK_HOST, 0);
}

long hailo_pcie_read_firmware_log(struct hailo_pcie_resources *resources, struct hailo_read_log_params *params)
{
    long err = 0;
    struct hailo_resource log_resource = {resources->fw_access.address, DEBUG_BUFFER_TOTAL_SIZE};

    if (HAILO_CPU_ID_CPU0 == params->cpu_id) {
        log_resource.address += PCIE_APP_CPU_DEBUG_OFFSET;
    } else if (HAILO_CPU_ID_CPU1 == params->cpu_id) {
        log_resource.address += PCIE_CORE_CPU_DEBUG_OFFSET;
    } else {
        return -EINVAL;
    }

    if (0 == params->buffer_size) {
        params->read_bytes = 0;
        return 0;
    }

    err = hailo_read_firmware_log(&log_resource, params);
    if (0 != err) {
        return err;
    }

    return 0;
}

static int direct_memory_transfer(struct hailo_pcie_resources *resources,
    struct hailo_memory_transfer_params *params)
{
    int err = -EINVAL;
    struct hailo_atr_config previous_atr = {0};

    if (params->address > U32_MAX) {
        return -EFAULT;
    }

    // Store previous ATR (Read/write modify the ATR).
    read_atr_table(resources, &previous_atr);

    switch (params->transfer_direction) {
    case TRANSFER_READ:
        read_memory(resources, (u32)params->address, params->buffer, (u32)params->count);
        break;
    case TRANSFER_WRITE:
        write_memory(resources, (u32)params->address, params->buffer, (u32)params->count);
        break;
    default:
        err = -EINVAL;
        goto restore_atr;
    }

    err = 0;
restore_atr:
    write_atr_table(resources, &previous_atr);
    return err;
}

int hailo_pcie_memory_transfer(struct hailo_pcie_resources *resources, struct hailo_memory_transfer_params *params)
{
    if (params->count > ARRAY_SIZE(params->buffer)) {
        return -EINVAL;
    }

    switch (params->memory_type) {
    case HAILO_TRANSFER_DEVICE_DIRECT_MEMORY:
        return direct_memory_transfer(resources, params);
    case HAILO_TRANSFER_MEMORY_PCIE_BAR0:
        return hailo_resource_transfer(&resources->config, params);
    case HAILO_TRANSFER_MEMORY_PCIE_BAR2:
    case HAILO_TRANSFER_MEMORY_VDMA0:
        return hailo_resource_transfer(&resources->vdma_registers, params);
    case HAILO_TRANSFER_MEMORY_PCIE_BAR4:
        return hailo_resource_transfer(&resources->fw_access, params);
    default:
        return -EINVAL;
    }
}

bool hailo_pcie_is_device_connected(struct hailo_pcie_resources *resources)
{
    return PCI_VENDOR_ID_HAILO == hailo_resource_read16(&resources->config, PCIE_CONFIG_VENDOR_OFFSET);
}

// On PCIe, just return the address
static u64 encode_dma_address(dma_addr_t dma_address, u8 channel_id)
{
    (void)channel_id;
    return (u64)dma_address;
}

struct hailo_vdma_hw hailo_pcie_vdma_hw = {
    .hw_ops = {
        .encode_desc_dma_address = encode_dma_address
    },
    .ddr_data_id = HAILO_PCIE_HOST_DMA_DATA_ID,
    .device_interrupts_bitmask = HAILO_PCIE_DMA_DEVICE_INTERRUPTS_BITMASK,
    .host_interrupts_bitmask = HAILO_PCIE_DMA_HOST_INTERRUPTS_BITMASK,

};