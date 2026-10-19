/*!
 * \brief     HAL SPI function implementations
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 *
 * MISRA Deviation Summary: Rule 17.2 (Functions shall not call themselves recursively)
 *
 * This file contains indirect recursion between register access functions. The recursion
 * is bounded and safe due to runtime parameter protection:
 *
 * Write Path: spi_regio_write → direct_or_paged_write → mask_reg_set → spi_regio_write
 *   Protection: spi_regio_write always passes NOMASK, causing direct_or_paged_write to
 *               skip mask_reg_set on recursive calls (see line 568 condition check).
 *               Maximum depth: 2 levels.
 *
 * Read Path: Similar protection exists for indirect_read → indirect_readx → spi_regio_read.
 *
 * Static analysis tools flag these as violations because they cannot detect the runtime
 * parameter-based protection. All 34 Rule 17.2 violations in this file are false positives.
 */

#include "adi_apollo_config.h"
#include "adi_apollo_bf_core.h"
#include "adi_apollo_hal_regio_local.h"
#include "adi_apollo_hal_regio_spi_types.h"

#define FIFO_MODE_BYTE_CNT_EN_THRESH 1U    /* Number of bytes where spi fifo mode enabled */

static int32_t spi_regio_write(void *desc, uint32_t reg, uint8_t data);
static int32_t spi_regio_write32(void *desc, uint32_t reg, uint32_t data);
static int32_t spi_regio_rmw_write(void *desc, uint32_t reg, uint8_t data, uint8_t mask, uint8_t is_paged);
static int32_t spi_regio_paged_base_addr_set(void *desc, uint32_t reg);
static int32_t spi_regio_paged_write(void *desc, uint32_t reg, uint8_t data);
static int32_t spi_regio_paged_write32(void *desc, uint32_t reg, uint32_t data);
static int32_t spi_regio_paged_data_write(void *desc, uint32_t address, uint8_t *data, uint32_t data_len);
static int32_t spi_regio_read(void *desc, uint32_t reg, uint8_t *data);
static int32_t spi_regio_read32(void *desc, uint32_t reg, uint32_t *data);
static int32_t spi_regio_poll_read(void *desc, uint32_t reg, uint8_t *data, uint32_t timeout_us, uint8_t reg_expect, uint8_t reg_mask);
static int32_t spi_regio_stream_write(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint8_t is_cont);
static int32_t spi_regio_stream_write32(void *desc, uint32_t reg, uint32_t *data, uint32_t data_len_words, uint8_t is_cont);
static int32_t spi_regio_stream_read(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint8_t is_cont);
static int32_t spi_regio_stream_read32(void *desc, uint32_t reg, uint32_t *data, uint32_t data_len_words, uint8_t is_cont);
static int32_t indirect_poll_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t timeout_us, uint8_t reg_expect, uint8_t reg_mask);
static int32_t alt_spi_regio_poll_read(void *desc, uint32_t reg, uint8_t *data, uint32_t timeout_us, uint8_t reg_expect, uint8_t reg_mask);
static int32_t spi_regio_read_bf(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint64_t mask);
static int32_t spi_regio_read_bf_direct(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint64_t mask);
static int32_t spi_regio_read_bf_indirect(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint64_t mask);
static int32_t indirect_readx(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size);
static int32_t indirect_writex(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t data, adi_apollo_hal_regio_spi_bus_size_e bus_size);
static int32_t stream_readx(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t stream_bytes, uint8_t is_cont);
static int32_t stream_writex(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t stream_bytes, uint8_t is_cont);

/* SPI implementations of regio operations */
static adi_apollo_hal_regio_ops_t spi_regio_ops = {
    .wr_reg =               &spi_regio_write,
    .wr_reg32 =             &spi_regio_write32,
    .wr_reg_rmw =           &spi_regio_rmw_write,
    .wr_reg_paged =         &spi_regio_paged_write,
    .wr_reg32_paged =       &spi_regio_paged_write32,
    .wr_data_paged =        &spi_regio_paged_data_write,
    .rd_reg =               &spi_regio_read,
    .rd_reg32 =             &spi_regio_read32,
    .rd_reg_poll =          &spi_regio_poll_read,
    .set_page_base_addr =   &spi_regio_paged_base_addr_set,
    .stream_wr_reg =        &spi_regio_stream_write,
    .stream_wr_reg32 =      &spi_regio_stream_write32,
    .stream_rd_reg =        &spi_regio_stream_read,
    .stream_rd_reg32 =      &spi_regio_stream_read32,
    .rd_bf =                &spi_regio_read_bf,
};

/* Predefined default transaction configs. */
static adi_apollo_hal_txn_config_t default_txn_cfgs[] = {
    {.addr_len = 2, .data_len = 1, .stream_len = 1, .mask = 0x000000ffULL, .is_bf_txn = 0},        /* Single 8-bit transaction */
    {.addr_len = 2, .data_len = 2, .stream_len = 1, .mask = 0x0000ffffULL, .is_bf_txn = 0},        /* Single 16-bit transaction */
    {.addr_len = 2, .data_len = 4, .stream_len = 1, .mask = 0xffffffffULL, .is_bf_txn = 0},        /* Single 32-bit transaction */
};

static int32_t direct_or_paged_write(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t mask, uint8_t is_paged);
static int32_t indirect_write(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t data, adi_apollo_hal_regio_spi_bus_size_e bus_size);
static int32_t direct_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size);
static int32_t indirect_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size);
static int32_t mask_reg_set(adi_apollo_hal_regio_spi_desc_t *desc, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t mask);
static int32_t indirect_addr_set(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t dma_ctrl);
static int32_t direct_poll_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t timeout_us, uint8_t reg_expect, uint8_t reg_mask);
static int32_t stream_write(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t stream_bytes, uint8_t is_cont);
static int32_t stream_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t stream_bytes, uint8_t is_cont);
static int32_t set_spi_fifo_mode(adi_apollo_hal_regio_spi_desc_t *desc, uint8_t is_fifo_enabled, adi_apollo_hal_regio_spi_bus_size_e bus_size);
static int32_t align_page_base_address(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, adi_apollo_hal_regio_spi_bus_size_e size, uint32_t *reg_offset_p);
static int32_t align_page_base_address_range(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t range, uint32_t *reg_offset_p);
static int32_t txn_config_set(adi_apollo_hal_regio_spi_desc_t *desc, uint8_t addr_siz_bytes, uint8_t data_siz_bytes, uint8_t stream_len, uint64_t mask, uint8_t is_bf_txn);
static int32_t txn_config_default_set(adi_apollo_hal_regio_spi_desc_t *desc, adi_apollo_hal_regio_spi_bus_size_e spi_bus_size);
static int32_t spi_write_stream_fifo_mode(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint64_t mask, uint8_t is_bf);

/* Default poll read if not defined */
static int32_t alt_spi_regio_poll_read(void *desc, uint32_t reg, uint8_t *data, uint32_t timeout_us, uint8_t reg_expect, uint8_t reg_mask)
{
    (void)timeout_us;  // Mark as intentionally unused
    (void)reg_expect;  // Mark as intentionally unused
    (void)reg_mask;  // Mark as intentionally unused


    return spi_regio_read(desc, reg, data);
}

int32_t adi_apollo_l_hal_regio_spi_init(adi_apollo_hal_regio_spi_desc_t *desc, uint8_t is_spi1)
{
    if (desc == NULL) {
        ADI_CMS_RETURN(API_CMS_ERROR_NULL_PARAM);
    }

    desc->base_regio.ops = &spi_regio_ops;     // interface
    desc->base_regio.child_desc = desc;        // instance (child)
    desc->base_regio.protocol_id = (is_spi1 != 0U) ? ADI_APOLLO_HAL_PROTOCOL_SPI1 : ADI_APOLLO_HAL_PROTOCOL_SPI0;
    desc->is_spi1 = is_spi1;
    desc->base_addr_cache.enabled = 0;
    desc->base_addr_cache.valid = 0;
    desc->indir_cache.enabled = 0;
    desc->indir_cache.valid = 0;

    /* If the user didn't specify an optional poll read hal for SPI0, use the built in alternate */
    if ((is_spi1 == 0U) && (desc->poll_read == NULL)) {
        spi_regio_ops.rd_reg_poll = &alt_spi_regio_poll_read;
        desc->base_regio.poll_read_returns_val = 1;
    }
    desc->base_regio.poll_read_en = (uint8_t)(desc->poll_read != NULL);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/* MISRA Deviation: Rule 17.2 - Indirect recursion via direct_or_paged_write/mask_reg_set.
 * Justification: Recursion is bounded and safe. When mask_reg_set writes to mask registers,
 *                it calls spi_regio_write which ALWAYS passes ADI_APOLLO_HAL_REGIO_NOMASK,
 *                preventing further recursion. Maximum recursion depth is 2 levels.
 *                Static analysis cannot detect this runtime protection.
 * Call chain: spi_regio_write → direct_or_paged_write → mask_reg_set → spi_regio_write (NOMASK)
 *             The NOMASK parameter breaks the cycle at direct_or_paged_write line 558. */
/* 8-bit register write */
static int32_t spi_regio_write(void *desc, uint32_t reg, uint8_t data)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;

    if ((reg >> 24U) == 0x47U) {
        /* spi direct */
        err = direct_or_paged_write(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, ADI_APOLLO_HAL_REGIO_NOMASK, ADI_APOLLO_HAL_REGIO_NOT_PAGED);
    } else {
        /* spi indirect */
        err = indirect_write(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8);
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 32-bit register write */
static int32_t spi_regio_write32(void *desc, uint32_t reg, uint32_t data)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;

    if ((reg >> 24U) == 0x47U) {
        /* spi direct */
        err = direct_or_paged_write(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32, ADI_APOLLO_HAL_REGIO_NOMASK, ADI_APOLLO_HAL_REGIO_NOT_PAGED);
    } else {
        /* spi indirect */
        err = indirect_write(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32);
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 8-bit read-modify-write */
static int32_t spi_regio_rmw_write(void *desc, uint32_t reg, uint8_t data, uint8_t mask, uint8_t is_paged)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err = (int32_t)API_CMS_ERROR_OK;
    uint32_t reg_offset = reg;

    if (((reg >> 24U) == 0x47U) && (is_paged == 0U)) {
        reg_offset = reg;
    } else if (is_paged != 0U) {
        err = align_page_base_address(spi_desc, reg, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, &reg_offset);
    } else {
        /* spi indirect doesn't support masked writes */
        ADI_CMS_RETURN(API_CMS_ERROR_SPI_REGIO_XFER);
    }

    if (err == (int32_t)API_CMS_ERROR_OK) {
        err = direct_or_paged_write(spi_desc, reg_offset, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, mask, is_paged);
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

static int32_t spi_regio_paged_base_addr_set(void *desc, uint32_t reg)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    uint32_t reg_word_aligned = reg & ~0x03U;    /* Page address must be on 32-bit boundary */

    // Program the spi paged base address
    uint32_t spi_page_base_reg = (spi_desc->is_spi1 != 0U) ?  REG_SPI1_PAGE_ADDRESS_31TO24_ADDR : REG_SPI0_BASE_PAGE_ADDRESS_31TO24_ADDR;

    if ((spi_desc->base_addr_cache.enabled == 0U) || (spi_desc->base_addr_cache.valid == 0U))
    {
        // Byte-level caching is either disabled or invalid; update the entire register in h/w
        err = spi_regio_write(desc, spi_page_base_reg + 0U, (uint8_t)(reg_word_aligned >> 24U));
        ADI_APOLLO_ERROR_RETURN(err);

        err = spi_regio_write(desc, spi_page_base_reg + 1U, (uint8_t)(reg_word_aligned >> 16U));
        ADI_APOLLO_ERROR_RETURN(err);

        err = spi_regio_write(desc, spi_page_base_reg + 2U, (uint8_t)(reg_word_aligned >> 8U));
        ADI_APOLLO_ERROR_RETURN(err);

        err = spi_regio_write(desc, spi_page_base_reg + 3U, (uint8_t)(reg_word_aligned >> 0U));
        ADI_APOLLO_ERROR_RETURN(err);

        // Update ADI's own word-level cache variable
        spi_desc->page_base_addr = reg_word_aligned;

        // Now we can mark our cache as valid
        spi_desc->base_addr_cache.valid = 1;
        ADI_CMS_RETURN(API_CMS_ERROR_OK);
    } else {
        // Byte-level caching is enabled; contents are valid.
        // Only update the bytes that have changed.

        if ((spi_desc->page_base_addr & 0xFF000000U) != (reg_word_aligned & 0xFF000000U)) {
            err = spi_regio_write(desc, spi_page_base_reg + 0U, (uint8_t)(reg_word_aligned >> 24U));
            ADI_APOLLO_ERROR_RETURN(err);
        }

        if ((spi_desc->page_base_addr & 0x00FF0000U) != (reg_word_aligned & 0x00FF0000U)) {
            err = spi_regio_write(desc, spi_page_base_reg + 1U, (uint8_t)(reg_word_aligned >> 16U));
            ADI_APOLLO_ERROR_RETURN(err);
        }

        if ((spi_desc->page_base_addr & 0x0000FF00U) != (reg_word_aligned & 0x0000FF00U)) {
            err = spi_regio_write(desc, spi_page_base_reg + 2U, (uint8_t)(reg_word_aligned >> 8U));
            ADI_APOLLO_ERROR_RETURN(err);
        }

        if ((spi_desc->page_base_addr & 0x000000FFU) != (reg_word_aligned & 0x000000FFU)) {
            err = spi_regio_write(desc, spi_page_base_reg + 3U, (uint8_t)(reg_word_aligned >> 0U));
            ADI_APOLLO_ERROR_RETURN(err);
        }

        // Also update word-level cache variable
        spi_desc->page_base_addr = reg_word_aligned;
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/* 8-bit paged write */
static int32_t spi_regio_paged_write(void *desc, uint32_t reg, uint8_t data)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    uint32_t reg_offset;

    /* spi paged write - base addr is on 32-bit boundary. Offsets can be 0 to 0x3fff */
    err = align_page_base_address(spi_desc, reg, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, &reg_offset);
    ADI_APOLLO_ERROR_RETURN(err);

    err = direct_or_paged_write(spi_desc, reg_offset, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, ADI_APOLLO_HAL_REGIO_NOMASK, ADI_APOLLO_HAL_REGIO_PAGED);

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 32-bit paged write */
static int32_t spi_regio_paged_write32(void *desc, uint32_t reg, uint32_t data)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    uint32_t reg_offset;

    /* spi paged write - base addr is on 32-bit boundary. Offsets can be 0 to 0x3fff */
    err = align_page_base_address(spi_desc, reg, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32, &reg_offset);
    ADI_APOLLO_ERROR_RETURN(err);

    /* spi paged write 32 */
    err = direct_or_paged_write(spi_desc, reg_offset, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32, ADI_APOLLO_HAL_REGIO_NOMASK, ADI_APOLLO_HAL_REGIO_PAGED);

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* Paged write of a contiguous byte run using SPI streaming (auto-incrementing address). */
static int32_t spi_regio_paged_data_write(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    uint32_t reg_offset;
    uint32_t i;
    uint8_t in_data[2 + 255] = {0};

    if ((data_len == 0U) || (data_len > 255U)) {
        return API_CMS_ERROR_INVALID_PARAM;
    }

    /* Ensure the whole run [reg, reg+data_len-1] fits in the active page. */
    err = align_page_base_address_range(spi_desc, reg, data_len - 1U, &reg_offset);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Paged address, then N contiguous data bytes streamed to auto-incrementing addresses. */
    in_data[0] = (uint8_t)(((reg_offset >> 8U) & 0x3FU) | 0x40U);
    in_data[1] = (uint8_t)((reg_offset >> 0U) & 0xFFU);
    for (i = 0U; i < data_len; i++) {
        in_data[2 + i] = data[i];
    }

    err = txn_config_set(spi_desc, 2, (uint8_t)data_len, 1, 0xFFFFFFFFULL, 0);
    ADI_APOLLO_ERROR_RETURN(err);

    err = spi_desc->write(spi_desc->dev_obj, in_data, 2 + data_len, &spi_desc->txn_config);

    return (err == API_CMS_ERROR_OK) ? API_CMS_ERROR_OK : API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 8-bit register read */
static int32_t spi_regio_read(void *desc, uint32_t reg, uint8_t *data)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    uint32_t rd_val;

    if ((reg >> 24U) == 0x47U) {
        /* spi direct */
        err = direct_read(spi_desc, reg, &rd_val, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8);
    } else {
        /* spi indirect */
        err = indirect_read(spi_desc, reg, &rd_val, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8);
    }

    *data = (uint8_t) rd_val;

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 32-bit register read */
static int32_t spi_regio_read32(void *desc, uint32_t reg, uint32_t *data)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;

    if ((reg >> 24U) == 0x47U) {
        /* spi direct */
        err = direct_read(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32);
    } else {
        /* spi indirect */
        err = indirect_read(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32);
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

static int32_t spi_regio_read_bf(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint64_t mask)
{
    int32_t err = (int32_t)API_CMS_ERROR_OK;

    if ((reg >> 24U) == 0x47U) {
        /* spi direct */
        err = spi_regio_read_bf_direct(desc, reg, data, data_len_bytes, mask);
    } else {
        /* spi indirect */
        err = spi_regio_read_bf_indirect(desc, reg, data, data_len_bytes, mask);
    }

    ADI_CMS_RETURN(err);
}

static int32_t spi_regio_read_bf_direct(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint64_t mask)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    uint32_t i;

    uint8_t in_data[16] = { 0 };
    uint8_t out_data[16] = { 0 };
    uint32_t xfer_sz_bytes;

    /* Set txn config info for bitfield */
    /* Note: data_len_bytes must fit in uint8_t and in_data/out_data buffers (max = sizeof(in_data) - 2) */
    err = txn_config_set(spi_desc, 2, 1, (uint8_t)data_len_bytes, mask, 1);        /* 2 addr, 1 data, data_len_bytes */
    ADI_APOLLO_ERROR_RETURN(err);

    /* Direct address (from 0x470000000) */
    in_data[0] = (uint8_t)(((reg >> 8) & 0x3FU) | 0x80U);
    in_data[1] = (uint8_t)((reg >> 0U) & 0xFFU);

    for (i = 0; i < data_len_bytes; i++) {
        in_data[2U + i] = data[i];
    }
    xfer_sz_bytes = 2U + data_len_bytes;

    err = spi_desc->read(spi_desc->dev_obj, in_data, out_data, xfer_sz_bytes, &spi_desc->txn_config);

    for (i = 0; i < data_len_bytes; i++) {
        data[i] = out_data[i + 2U];
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}


static int32_t spi_regio_read_bf_indirect(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint64_t mask)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    uint32_t dma_dat0;
    int32_t err;
    uint32_t i;

    uint8_t spidma_ahb_bus_select = 0;
    uint8_t spidma_auto_incr = 1;
    uint8_t spidma_bus_size = (uint8_t)ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8; // Size of read/write. 0 = byte, 1 = half-word, 2 = full-word, 3 = invalid.
    uint8_t spidma_sys_codeb = 0;
    uint8_t spidma_rd_wrb = 1;
    uint8_t spidma_ctl_data = (uint8_t)((spidma_rd_wrb << 7) | ((spidma_sys_codeb << 6) | ((spidma_bus_size << 2) | ((spidma_auto_incr << 1) | (spidma_ahb_bus_select << 0)))));
    uint8_t in_data[16] = { 0 };
    uint8_t out_data[16] = { 0 };
    uint32_t xfer_sz_bytes;

    err = indirect_addr_set(spi_desc, reg, spidma_ctl_data);
    ADI_APOLLO_ERROR_RETURN(err);

    /* FIFO mode is efficient if #bytes > 4 */
    if (data_len_bytes > FIFO_MODE_BYTE_CNT_EN_THRESH) {
        err = set_spi_fifo_mode(spi_desc, 1, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    /* Set txn config info for bitfield */
    /* Note: data_len_bytes must fit in uint8_t and in_data/out_data buffers (max = sizeof(in_data) - 2) */
    err = txn_config_set(spi_desc, 2, 1, (uint8_t)data_len_bytes, mask, 1);        /* 2 addr, 1 data, data_len_bytes */
    ADI_APOLLO_ERROR_RETURN(err);

    /* Set SPI address to spi dma lsb. Using auto increment mode */
    dma_dat0 = (spi_desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA0_ADDR : REG_SPIDMA0_DATA0_ADDR;
    in_data[0] = (uint8_t)(((dma_dat0 >> 8U) & 0x3FU) | 0x80U);
    in_data[1] = (uint8_t)((dma_dat0 >> 0U) & 0xFFU);

    for (i = 0; i < data_len_bytes; i++) {
        in_data[2U + i] = data[i];
    }
    xfer_sz_bytes = 2U + data_len_bytes;
    err = spi_desc->read(spi_desc->dev_obj, in_data, out_data, xfer_sz_bytes, &spi_desc->txn_config);

    for (i = 0; i < data_len_bytes; i++) {
        data[i] = out_data[i + 2U];
    }

    /* Reset txn spi fifo mode */
    if (data_len_bytes > FIFO_MODE_BYTE_CNT_EN_THRESH) {
        err = set_spi_fifo_mode(spi_desc, 0, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 8 bit stream write */
static int32_t spi_regio_stream_write(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint8_t is_cont)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    if (spi_desc->wr_stream_en != 0U) {
        err = stream_write(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, data_len_bytes, is_cont);
    } else {
        err = stream_writex(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, data_len_bytes, is_cont);
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 32 bit stream write */
static int32_t spi_regio_stream_write32(void *desc, uint32_t reg, uint32_t *data, uint32_t data_len_words, uint8_t is_cont)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;

    if (spi_desc->wr_stream_en != 0U) {
        err = stream_write(spi_desc, reg, (uint8_t *)data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32, data_len_words * 4U, is_cont);
    } else {
        err = stream_writex(spi_desc, reg, (uint8_t *)data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32, data_len_words * 4U, is_cont);
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 8 bit stream read */
static int32_t spi_regio_stream_read(void *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint8_t is_cont)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    if (spi_desc->rd_stream_en != 0U) {
        err = stream_read(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, data_len_bytes, is_cont);
    } else {
        err = stream_readx(spi_desc, reg, data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, data_len_bytes, is_cont);
    }
    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* 32 bit stream read */
static int32_t spi_regio_stream_read32(void *desc, uint32_t reg, uint32_t *data, uint32_t data_len_words, uint8_t is_cont)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    if (spi_desc->rd_stream_en != 0U) {
        err = stream_read(spi_desc, reg, (uint8_t *)data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32, data_len_words * 4U, is_cont);
    } else {
        err = stream_readx(spi_desc, reg, (uint8_t *)data, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32, data_len_words * 4U, is_cont);
    }

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

static int32_t indirect_poll_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t timeout_us, uint8_t reg_expect, uint8_t reg_mask)
{
    int32_t err;
    uint8_t spidma_ahb_bus_select = 0;
    uint8_t spidma_auto_incr = 0;
    uint8_t spidma_bus_size = (uint8_t)bus_size; // Size of read/write. 0 = byte, 1 = half-word, 2 = full-word, 3 = invalid.
    uint8_t spidma_sys_codeb = 0;
    uint8_t spidma_rd_wrb = 1;
    uint8_t spidma_ctl_data = (uint8_t)((spidma_rd_wrb << 7) | ((spidma_sys_codeb << 6) | ((spidma_bus_size << 2) | ((spidma_auto_incr << 1) | (spidma_ahb_bus_select << 0)))));
    uint8_t in_data[10] = {0};
    uint8_t out_data[10] = {0};
    uint32_t xfer_sz_bytes;

    /* Set the dma ctl and address */
    err = indirect_addr_set(desc, reg, spidma_ctl_data);
    ADI_APOLLO_ERROR_RETURN(err);

    /* in_data: [DMA address last 8 bits] [reg address] */
    in_data[0] = REG_SPIDMA0_ADDR3_ADDR & 0xFFU;
    in_data[1] = REG_SPIDMA0_ADDR2_ADDR & 0xFFU;
    in_data[2] = REG_SPIDMA0_ADDR1_ADDR & 0xFFU;
    in_data[3] = REG_SPIDMA0_ADDR0_ADDR & 0xFFU;

    in_data[4] = (uint8_t)((reg >> 24U) & 0xFFU);
    in_data[5] = (uint8_t)((reg >> 16U) & 0xFFU);
    in_data[6] = (uint8_t)((reg >> 8U) & 0xFFU);
    in_data[7] = (uint8_t)((reg >> 0U) & 0xFFU);

    xfer_sz_bytes = 8;

    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) {
        xfer_sz_bytes += 1U;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        xfer_sz_bytes += 4U;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
        xfer_sz_bytes += 2U;
    } else {
        ADI_CMS_RETURN(API_CMS_ERROR_SPI_REGIO_XFER);
    }

    // platform driver with ATE mods - needs work
    err = desc->poll_read(desc->dev_obj, in_data, out_data, xfer_sz_bytes, timeout_us, &reg_expect, &reg_mask, &default_txn_cfgs[bus_size]);
    ADI_APOLLO_ERROR_RETURN(err);

    *data = 0;
    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) {
        *data |= out_data[2];
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        *data |= (uint32_t)out_data[2] << 0;
        *data |= (uint32_t)out_data[3] << 8;
        *data |= (uint32_t)out_data[4] << 16;
        *data |= (uint32_t)out_data[5] << 24;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
        *data |= (uint32_t)out_data[2] << 0;
        *data |= (uint32_t)out_data[3] << 8;
    } else {
        ADI_CMS_RETURN(API_CMS_ERROR_SPI_REGIO_XFER);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/* 8-bit register polled read */
static int32_t spi_regio_poll_read(void *desc, uint32_t reg, uint8_t *data, uint32_t timeout_us, uint8_t reg_expect, uint8_t reg_mask)
{
    adi_apollo_hal_regio_spi_desc_t *spi_desc = (adi_apollo_hal_regio_spi_desc_t *)desc;
    int32_t err;
    uint32_t rd_val;

    if ((reg >> 24U) == 0x47U) {
        /* spi direct */
        err = direct_poll_read(spi_desc, reg, &rd_val, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, timeout_us, reg_expect, reg_mask);
    } else {
        /* spi indirect */
        err = indirect_poll_read(spi_desc, reg, &rd_val, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8, timeout_us, reg_expect, reg_mask);
    }

    *data = (uint8_t) rd_val;

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

/* MISRA Deviation: Rule 17.2 - Part of bounded recursion protection.
 * Justification: This function is called by spi_regio_write. When mask != NOMASK,
 *                it calls mask_reg_set which calls back to spi_regio_write.
 *                However, spi_regio_write always passes NOMASK, so this condition
 *                (line 568) evaluates to false on the recursive call, breaking the cycle. */
static int32_t direct_or_paged_write(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t mask, uint8_t is_paged)
{
    int32_t err;
    uint8_t in_data[6] = {0};
    uint8_t out_data[6] = {0};
    uint32_t xfer_sz_bytes;

    if ((desc->base_regio.rmw_enabled != 0U) && (mask != ADI_APOLLO_HAL_REGIO_NOMASK))
    {
        err = mask_reg_set(desc, bus_size, mask);   /* sets mask reg if necessary */
        ADI_APOLLO_ERROR_RETURN(err);
    }

    /* Direct address (from 0x470000000) */
    in_data[0] = (uint8_t)(((reg >> 8) & 0x3FU) | ((is_paged != 0U) ? 0x40U : 0U));
    in_data[1] = (uint8_t)((reg >> 0U) & 0xFFU);
    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) {
        in_data[2] = (uint8_t)(data >> 0);
        xfer_sz_bytes = 3;  // 2 addr + 1 data
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        in_data[2] = (uint8_t)(data >> 0);
        in_data[3] = (uint8_t)(data >> 8);
        in_data[4] = (uint8_t)(data >> 16);
        in_data[5] = (uint8_t)(data >> 24);
        xfer_sz_bytes = 6;  // 2 addr + 4 data
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
        in_data[2] = (uint8_t)(data >> 0);
        in_data[3] = (uint8_t)(data >> 8);
        xfer_sz_bytes = 4;  // 2 addr + 2 data
    } else {
        ADI_CMS_RETURN(API_CMS_ERROR_INVALID_PARAM);
    }

    /* write method takes precedence over xfer if both defined */
    if (desc->write != NULL) {
        err = desc->write(desc->dev_obj, in_data, xfer_sz_bytes, &default_txn_cfgs[bus_size]);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = desc->xfer(desc->dev_obj, in_data, out_data, xfer_sz_bytes);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(err);
}

static int32_t indirect_write(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t data, adi_apollo_hal_regio_spi_bus_size_e bus_size)
{
    if (desc->wr_stream_en != 0U)
    {
        return stream_write(desc, reg, (uint8_t *) &data, bus_size, (1U << (uint32_t)bus_size), 0);
    } else {
        return indirect_writex(desc, reg, data, bus_size);
    }
}

static int32_t mask_reg_set(adi_apollo_hal_regio_spi_desc_t *desc, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t mask)
{
    int32_t err;

    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        if (mask == 0xffffffffU) {
            ADI_CMS_RETURN(API_CMS_ERROR_OK);
        }
        err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPI1_MASK_31TO24_ADDR : REG_SPI0_MASK_31TO24_ADDR, (uint8_t)((mask >> 24U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
        err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPI1_MASK_23TO16_ADDR : REG_SPI0_MASK_23TO16_ADDR, (uint8_t)((mask >> 16U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
    }

    if ((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) || (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16)) {
        if ((mask & 0xffffU) == 0xffffU) {
            ADI_CMS_RETURN(API_CMS_ERROR_OK);
        }
        err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPI1_MASK_15TO8_ADDR : REG_SPI0_MASK_15TO8_ADDR, (uint8_t)((mask >> 8U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
    }

    if ((mask & 0xffU) != 0xffU) {
        err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPI1_MASK_7TO0_ADDR : REG_SPI0_MASK_7TO0_ADDR, (uint8_t)((mask >> 0U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
    }


    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t direct_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size)
{
    int32_t err;
    uint8_t in_data[6] = {0};
    uint8_t out_data[6] = {0};
    uint32_t xfer_sz_bytes;

    // ADI_APOLLO_NULL_POINTER_RETURN(desc);

    /* Direct address (from 0x470000000) */
    in_data[0] = (uint8_t)(((reg >> 8) & 0x3FU) | 0x80U);
    in_data[1] = (uint8_t)((reg >> 0U) & 0xFFU);

    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) {
        xfer_sz_bytes = 3;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        xfer_sz_bytes = 6;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
        xfer_sz_bytes = 4;
    } else {
        return -1;
    }

    /* read method takes precedence over xfer if both defined */
    if (desc->read != NULL) {
        err = desc->read(desc->dev_obj, in_data, out_data, xfer_sz_bytes, &default_txn_cfgs[bus_size]);
        ADI_APOLLO_ERROR_RETURN(err);
    } else {
        err = desc->xfer(desc->dev_obj, in_data, out_data, xfer_sz_bytes);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    *data = 0;
    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) {
        *data |= out_data[2];
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        *data |= (uint32_t)out_data[2] << 0;
        *data |= (uint32_t)out_data[3] << 8;
        *data |= (uint32_t)out_data[4] << 16;
        *data |= (uint32_t)out_data[5] << 24;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
        *data |= (uint32_t)out_data[2] << 0;
        *data |= (uint32_t)out_data[3] << 8;
    } else {
        ADI_CMS_RETURN(API_CMS_ERROR_SPI_REGIO_XFER);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t indirect_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size)
{
    if (desc->rd_stream_en != 0U) {
        return stream_read(desc, reg, (uint8_t *)data, bus_size, (1U << (uint32_t)bus_size), 0);
    } else {
        return indirect_readx(desc, reg, data, bus_size);
    }
}

static int32_t direct_poll_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t timeout_us, uint8_t reg_expect, uint8_t reg_mask)
{
    int32_t err;
    uint8_t in_data[6] = {0};
    uint8_t out_data[6] = {0};
    uint32_t xfer_sz_bytes;

    /* Direct address (from 0x470000000) */
    in_data[0] = (uint8_t)(((reg >> 8) & 0x3FU) | 0x80U);
    in_data[1] = (uint8_t)((reg >> 0U) & 0xFFU);

    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) {
        xfer_sz_bytes = 3;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        xfer_sz_bytes = 6;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
        xfer_sz_bytes = 4;
    } else {
        ADI_CMS_RETURN(API_CMS_ERROR_SPI_REGIO_XFER);
    }

    err = desc->poll_read(desc->dev_obj, in_data, out_data, xfer_sz_bytes, timeout_us, &reg_expect, &reg_mask, &default_txn_cfgs[bus_size]);
    ADI_APOLLO_ERROR_RETURN(err);

    *data = 0;
    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) {
        *data |= out_data[2];
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        *data |= (uint32_t)out_data[2] << 0;
        *data |= (uint32_t)out_data[3] << 8;
        *data |= (uint32_t)out_data[4] << 16;
        *data |= (uint32_t)out_data[5] << 24;
    } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
        *data |= (uint32_t)out_data[2] << 0;
        *data |= (uint32_t)out_data[3] << 8;
    } else {
        ADI_CMS_RETURN(API_CMS_ERROR_SPI_REGIO_XFER);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

static int32_t indirect_addr_set(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t dma_ctrl)
{
    int32_t err;

    if ((desc->indir_cache.enabled == 0U) || (desc->indir_cache.valid == 0U))
    {
        // The cache is invalid or is not enabled; Write all addr
        // bytes to the device.
        err = spi_regio_write(desc,
            (desc->is_spi1 != 0U) ? REG_SPIDMA1_CTL_ADDR
                          : REG_SPIDMA0_CTL_ADDR,
            dma_ctrl);
        ADI_APOLLO_ERROR_RETURN(err);
        desc->indir_cache.dma_ctrl = dma_ctrl;

        err = spi_regio_write(desc,
            (desc->is_spi1 != 0U) ? REG_SPIDMA1_ADDR3_ADDR
                          : REG_SPIDMA0_ADDR3_ADDR,
            (uint8_t)((reg >> 24U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
        desc->indir_cache.addr3 = (uint8_t)((reg >> 24U) & 0xFFU);

        err = spi_regio_write(desc,
            (desc->is_spi1 != 0U) ? REG_SPIDMA1_ADDR2_ADDR
                          : REG_SPIDMA0_ADDR2_ADDR,
            (uint8_t)((reg >> 16U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
        desc->indir_cache.addr2 = (uint8_t)((reg >> 16U) & 0xFFU);

        err = spi_regio_write(desc,
            (desc->is_spi1 != 0U) ? REG_SPIDMA1_ADDR1_ADDR
                          : REG_SPIDMA0_ADDR1_ADDR,
            (uint8_t)((reg >> 8U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
        desc->indir_cache.addr1 = (uint8_t)((reg >> 8U) & 0xFFU);

        err = spi_regio_write(desc,
            (desc->is_spi1 != 0U) ? REG_SPIDMA1_ADDR0_ADDR
                          : REG_SPIDMA0_ADDR0_ADDR,
            (uint8_t)((reg >> 0U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);

        // Mark cache as valid
        desc->indir_cache.valid = 1;
    } else {
        // Cache is valid and enabled. Only write the addr bytes
        // which are different to the cached addr to the device.
        if (desc->indir_cache.dma_ctrl != dma_ctrl) {
            err = spi_regio_write(desc,
                (desc->is_spi1 != 0U) ? REG_SPIDMA1_CTL_ADDR
                              : REG_SPIDMA0_CTL_ADDR,
                dma_ctrl);
            ADI_APOLLO_ERROR_RETURN(err);
            desc->indir_cache.dma_ctrl = dma_ctrl;
        }

        if (desc->indir_cache.addr3 != ((uint8_t)((reg >> 24U) & 0xFFU))) {
            err = spi_regio_write(desc,
                (desc->is_spi1 != 0U) ? REG_SPIDMA1_ADDR3_ADDR
                              : REG_SPIDMA0_ADDR3_ADDR,
                (uint8_t)((reg >> 24U) & 0xFFU));
            ADI_APOLLO_ERROR_RETURN(err);
            desc->indir_cache.addr3 = (uint8_t)((reg >> 24U) & 0xFFU);
        }

        if (desc->indir_cache.addr2 != ((uint8_t)((reg >> 16U) & 0xFFU))) {
            err = spi_regio_write(desc,
                (desc->is_spi1 != 0U) ? REG_SPIDMA1_ADDR2_ADDR
                              : REG_SPIDMA0_ADDR2_ADDR,
                (uint8_t)((reg >> 16U) & 0xFFU));
            ADI_APOLLO_ERROR_RETURN(err);
            desc->indir_cache.addr2 = (uint8_t)((reg >> 16U) & 0xFFU);
        }

        if (desc->indir_cache.addr1 != ((uint8_t)((reg >> 8U) & 0xFFU))) {
            err = spi_regio_write(desc,
                (desc->is_spi1 != 0U) ? REG_SPIDMA1_ADDR1_ADDR
                              : REG_SPIDMA0_ADDR1_ADDR,
                (uint8_t)((reg >> 8U) & 0xFFU));
            ADI_APOLLO_ERROR_RETURN(err);
            desc->indir_cache.addr1 = (uint8_t)((reg >> 8U) & 0xFFU);
        }

        // The DMA (indirect) register is not latched in until
        // the least-significant byte is written. So always write this last byte.
        err = spi_regio_write(desc,
            (desc->is_spi1 != 0U) ? REG_SPIDMA1_ADDR0_ADDR
                        : REG_SPIDMA0_ADDR0_ADDR,
            (uint8_t)((reg >> 0U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
    }

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/*
    This mode is introduced in SPI Slave to improve SPI Indirect DMA efficiency when it is in auto-increment/decrement mode.

    spi_fifo_mode[2:0]  Description
        000             SPI usual behavior (no change)
        001             8-bit access
        010             16-bit access
        011             32-bit access

*/
static __maybe_unused int32_t set_spi_fifo_mode(adi_apollo_hal_regio_spi_desc_t *desc, uint8_t is_fifo_enabled, adi_apollo_hal_regio_spi_bus_size_e bus_size)
{
    int32_t err;

    if (is_fifo_enabled != 0U) {
        if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
            err = spi_regio_write(desc, REG_SPI_FIFO_MODE_REG_ADDR, 0x03U);
        } else if(bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
            err = spi_regio_write(desc, REG_SPI_FIFO_MODE_REG_ADDR, 0x02U);
        } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) {
            err = spi_regio_write(desc, REG_SPI_FIFO_MODE_REG_ADDR, 0x01U);
        } else {
            /* Invalid bus size */
            ADI_CMS_RETURN(API_CMS_ERROR_INVALID_PARAM);
        }
    } else {
        err = spi_regio_write(desc, REG_SPI_FIFO_MODE_REG_ADDR, 0x00U);
    }

    ADI_CMS_RETURN(err);
}

static int32_t stream_write(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t stream_bytes, uint8_t is_cont)
{
    int32_t err;

    uint32_t pending_sz_bytes = stream_bytes;
    uint32_t data_index = 0;
    uint32_t data32;

    err = (int32_t)API_CMS_ERROR_OK;

    if(is_cont == 0U) {
        uint8_t spidma_ahb_bus_select = 0;
        uint8_t spidma_auto_incr = 1;
        uint8_t spidma_bus_size = (uint8_t)bus_size; // Size of read/write. 0 = byte, 1 = half-word, 2 = full-word, 3 = invalid.
        uint8_t spidma_sys_codeb = 0;
        uint8_t spidma_rd_wrb = 0;
        uint8_t spidma_ctl_data = (uint8_t)((spidma_rd_wrb << 7) | ((spidma_sys_codeb << 6) | ((spidma_bus_size << 2) | ((spidma_auto_incr << 1) | (spidma_ahb_bus_select << 0)))));

        //set_spi_fifo_mode(desc, 4);     // 4 = no fifo mode
        err = indirect_addr_set(desc, reg, spidma_ctl_data);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    while (pending_sz_bytes > 0U) {
        if ((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) && (pending_sz_bytes > 3U)) {
            data32 = (((uint32_t)data[data_index + 3U] << 0U) | (((uint32_t)data[data_index + 2U] << 8U) | (((uint32_t)data[data_index + 1U] << 16U) | ((uint32_t)data[data_index + 0U] << 24U))));
            err = direct_or_paged_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA3_ADDR : REG_SPIDMA0_DATA3_ADDR,
                                        data32, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32,
                                        ADI_APOLLO_HAL_REGIO_NOMASK, ADI_APOLLO_HAL_REGIO_NOT_PAGED);
            ADI_APOLLO_ERROR_RETURN(err);
            pending_sz_bytes -= 4U;
            data_index += 4U;
        } else if (((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) || (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16)) && (pending_sz_bytes > 1U)) {
            data32 = (((uint32_t)data[data_index + 1U] << 0U) | ((uint32_t)data[data_index + 0U] << 8U));
            err = direct_or_paged_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA1_ADDR : REG_SPIDMA0_DATA1_ADDR,
                                        data32 & 0x0000ffffU, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16,
                                        ADI_APOLLO_HAL_REGIO_NOMASK, ADI_APOLLO_HAL_REGIO_NOT_PAGED);
            ADI_APOLLO_ERROR_RETURN(err);
            pending_sz_bytes -= 2U;
            data_index += 2U;
        } else {
            data32 = data[data_index + 0U];
            err = direct_or_paged_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA0_ADDR : REG_SPIDMA0_DATA0_ADDR,
                                        data32 & 0x000000ffU, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8,
                                        ADI_APOLLO_HAL_REGIO_NOMASK, ADI_APOLLO_HAL_REGIO_NOT_PAGED);
            ADI_APOLLO_ERROR_RETURN(err);
            pending_sz_bytes -= 1U;
            data_index += 1U;
        }
    }

    ADI_CMS_RETURN(err);
}

static int32_t stream_read(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t stream_bytes, uint8_t is_cont)
{
    int32_t err;

    uint32_t pending_sz_bytes = stream_bytes;
    uint32_t data32;
    uint8_t *data_ptr;

    err = (int32_t)API_CMS_ERROR_OK;

    if(is_cont == 0U) {
        uint8_t spidma_ahb_bus_select = 0;
        uint8_t spidma_auto_incr = 1;
        uint8_t spidma_bus_size = (uint8_t)bus_size; // Size of read/write. 0 = byte, 1 = half-word, 2 = full-word, 3 = invalid.
        uint8_t spidma_sys_codeb = 0;
        uint8_t spidma_rd_wrb = 1;
        uint8_t spidma_ctl_data = (uint8_t)((spidma_rd_wrb << 7) | ((spidma_sys_codeb << 6) | ((spidma_bus_size << 2) | ((spidma_auto_incr << 1) | (spidma_ahb_bus_select << 0)))));

        err = indirect_addr_set(desc, reg, spidma_ctl_data);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    *data = 0;

    data_ptr = (uint8_t *) data;

    while (pending_sz_bytes > 0U) {
        if ((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) && (pending_sz_bytes > 3U)) {
            err = direct_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA3_ADDR : REG_SPIDMA0_DATA3_ADDR,
                                        &data32, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32);
            ADI_APOLLO_ERROR_RETURN(err);
            *(data_ptr + 0) = (uint8_t)((data32 >> 24U) & 0xFFU);
            *(data_ptr + 1) = (uint8_t)((data32 >> 16U) & 0xFFU);
            *(data_ptr + 2) = (uint8_t)((data32 >> 8U) & 0xFFU);
            *(data_ptr + 3) = (uint8_t)((data32 >> 0U) & 0xFFU);
            pending_sz_bytes -= 4U;
            data_ptr += 4U;
        } else if (((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) || (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16)) && (pending_sz_bytes > 1U)) {
            err = direct_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA1_ADDR : REG_SPIDMA0_DATA1_ADDR,
                                        &data32, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16);
            ADI_APOLLO_ERROR_RETURN(err);
            *(data_ptr + 0) = (uint8_t)((data32 >> 8U) & 0xFFU);
            *(data_ptr + 1) = (uint8_t)((data32 >> 0U) & 0xFFU);
            pending_sz_bytes -= 2U;
            data_ptr += 2U;
        } else {
            err = direct_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA0_ADDR : REG_SPIDMA0_DATA0_ADDR,
                                        &data32, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8);
            ADI_APOLLO_ERROR_RETURN(err);
            *(data_ptr + 0) = (uint8_t)((data32 >> 0U) & 0xFFU);
            pending_sz_bytes -= 1U;
            data_ptr += 1U;
        }
    }

    ADI_CMS_RETURN(err);
}

/* Checks paged address alignment and updates address if the byte range [reg, reg+range] is out of the active page. */
static int32_t align_page_base_address_range(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t range, uint32_t *reg_offset_p)
{
    int32_t err;
    int32_t reg_offset = (int32_t)reg - (int32_t)desc->page_base_addr;

    if ( (reg_offset < 0) || (((uint32_t)reg_offset + range) > (ADI_APOLLO_DIRECT_SPI_REGION_LEN - 1U)) )
    {
        err = spi_regio_paged_base_addr_set(desc, reg & 0xffffc000U);   // Page offset out of range for this transaction. Updates desc->page_base_addr.
        ADI_APOLLO_ERROR_RETURN(err);
        reg_offset = (int32_t)reg - (int32_t)desc->page_base_addr;
    }

    /* reg offset from page base */
    *reg_offset_p = (uint32_t)reg_offset;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/* Checks paged address alignment and updates address if out of active page range */
static int32_t align_page_base_address(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, adi_apollo_hal_regio_spi_bus_size_e size, uint32_t *reg_offset_p)
{
    uint32_t range =  (size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8) ? 0 : 3;   /* 8 bit, 32 bit */

    return align_page_base_address_range(desc, reg, range, reg_offset_p);
}

static __maybe_unused int32_t spi_write_stream_fifo_mode(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, uint32_t data_len_bytes, uint64_t mask, uint8_t is_bf)
{
    uint32_t dma_dat0;
    int32_t err;
    uint32_t i;
    uint8_t in_data[16] = { 0 };
    uint32_t xfer_sz_bytes;

    (void)reg;  // Mark as intentionally unused

    err = set_spi_fifo_mode(desc, 1, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8);
    ADI_APOLLO_ERROR_RETURN(err);

    /* Set txn config info for bitfield */
    /* Note: data_len_bytes must fit in uint8_t and in_data buffer (max = sizeof(in_data) - 2) */
    err = txn_config_set(desc, 2, 1, (uint8_t)data_len_bytes, mask, is_bf); /* 2 addr, 1 data, data_len_bytes */
    ADI_APOLLO_ERROR_RETURN(err);

    /* Write the SPI DMA control reg data with first byte */
    dma_dat0 = (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA0_ADDR : REG_SPIDMA0_DATA0_ADDR;
    in_data[0] = (uint8_t)(((dma_dat0 >> 8U) & 0x3FU) | 0x80U);
    in_data[1] = (uint8_t)((dma_dat0 >> 0U) & 0xFFU);

    for (i = 0; i < data_len_bytes; i++) {
        in_data[2U + i] = data[i];
    }
    xfer_sz_bytes = 2U + data_len_bytes;
    err = desc->write(desc->dev_obj, in_data, xfer_sz_bytes, &desc->txn_config);

    /* Reset txn spi fifo mode */
    err = set_spi_fifo_mode(desc, 0, ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D8);
    ADI_APOLLO_ERROR_RETURN(err);

    return (err == (int32_t)API_CMS_ERROR_OK) ? (int32_t)API_CMS_ERROR_OK : (int32_t)API_CMS_ERROR_SPI_REGIO_XFER;
}

static __maybe_unused int32_t txn_config_default_set(adi_apollo_hal_regio_spi_desc_t *desc, adi_apollo_hal_regio_spi_bus_size_e spi_bus_size)
{
    adi_apollo_hal_txn_config_t *txn_config_p = &desc->txn_config;
    adi_apollo_hal_txn_config_t *default_txn_config_p = &default_txn_cfgs[spi_bus_size];

    txn_config_p->addr_len = default_txn_config_p->addr_len;
    txn_config_p->data_len = default_txn_config_p->data_len;
    txn_config_p->stream_len = default_txn_config_p->stream_len;
    txn_config_p->mask = default_txn_config_p->mask;
    txn_config_p->is_bf_txn = default_txn_config_p->is_bf_txn;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}


static int32_t txn_config_set(adi_apollo_hal_regio_spi_desc_t *desc, uint8_t addr_siz_bytes, uint8_t data_siz_bytes, uint8_t stream_len, uint64_t mask, uint8_t is_bf_txn)
{
    adi_apollo_hal_txn_config_t *txn_config_p = &desc->txn_config;

    txn_config_p->addr_len = addr_siz_bytes;
    txn_config_p->data_len = data_siz_bytes;
    txn_config_p->stream_len = stream_len;
    txn_config_p->mask = mask;
    txn_config_p->is_bf_txn = is_bf_txn;

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/*
 * indirect_readx - Indirect register read that doesn't generate any streaming transactions. Use when rd_stream_en = 0.
 */
static int32_t indirect_readx(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size) {
    int32_t err;
    uint8_t spidma_ahb_bus_select = 0;
    uint8_t spidma_auto_incr = 0;
    uint8_t spidma_bus_size = (uint8_t)bus_size; // Size of read/write. 0 = byte, 1 = half-word, 2 = full-word, 3 = invalid.
    uint8_t spidma_sys_codeb = 0;
    uint8_t spidma_rd_wrb = 1;
    uint8_t spidma_ctl_data = (uint8_t)((spidma_rd_wrb << 7) | ((spidma_sys_codeb << 6) | ((spidma_bus_size << 2) | ((spidma_auto_incr << 1) | (spidma_ahb_bus_select << 0)))));
    uint8_t *data_ptr;

    /* Set the dma ctl and address */
    err = indirect_addr_set(desc, reg, spidma_ctl_data);
    ADI_APOLLO_ERROR_RETURN(err);

    *data = 0;

    data_ptr = (uint8_t *) data;

    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        err = spi_regio_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA3_ADDR : REG_SPIDMA0_DATA3_ADDR, data_ptr + 3); /* msb */
        ADI_APOLLO_ERROR_RETURN(err);
        err = spi_regio_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA2_ADDR : REG_SPIDMA0_DATA2_ADDR, data_ptr + 2);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    if ((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) || (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16)) {
        err = spi_regio_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA1_ADDR : REG_SPIDMA0_DATA1_ADDR, data_ptr + 1);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    err = spi_regio_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA0_ADDR : REG_SPIDMA0_DATA0_ADDR, data_ptr + 0); /* lsb */
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/*
 * indirect_writex - Indirect register write that doesn't generate any streaming transactions. Use when wr_stream_en = 0.
 */
static int32_t indirect_writex(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint32_t data, adi_apollo_hal_regio_spi_bus_size_e bus_size) {
    int32_t err;
    uint8_t spidma_ahb_bus_select = 0;
    uint8_t spidma_auto_incr = 0;
    uint8_t spidma_bus_size = (uint8_t)bus_size; // Size of read/write. 0 = byte, 1 = half-word, 2 = full-word, 3 = invalid.
    uint8_t spidma_sys_codeb = 0;
    uint8_t spidma_rd_wrb = 0;
    uint8_t spidma_ctl_data = (uint8_t)((spidma_rd_wrb << 7) | ((spidma_sys_codeb << 6) | ((spidma_bus_size << 2) | ((spidma_auto_incr << 1) | (spidma_ahb_bus_select << 0)))));

    /* Set the dma ctl and address */
    err = indirect_addr_set(desc, reg, spidma_ctl_data);
    ADI_APOLLO_ERROR_RETURN(err);

    if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
        err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA3_ADDR : REG_SPIDMA0_DATA3_ADDR, (uint8_t)((data >> 24U) & 0xFFU));      // MSB
        ADI_APOLLO_ERROR_RETURN(err);
        err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA2_ADDR : REG_SPIDMA0_DATA2_ADDR, (uint8_t)((data >> 16U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
    }

    if ((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) || (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16)) {
        err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA1_ADDR : REG_SPIDMA0_DATA1_ADDR, (uint8_t)((data >> 8U) & 0xFFU));
        ADI_APOLLO_ERROR_RETURN(err);
    }

    err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA0_ADDR : REG_SPIDMA0_DATA0_ADDR, (uint8_t)((data >> 0U) & 0xFFU));           // LSB
    ADI_APOLLO_ERROR_RETURN(err);

    ADI_CMS_RETURN(API_CMS_ERROR_OK);
}

/*
 * stream_readx - pseudo read stream function that doesn't generate any streaming transactions. Use when rd_stream_en = 0.
 */
static int32_t stream_readx(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t stream_bytes, uint8_t is_cont) {
    int32_t err = (int32_t)API_CMS_ERROR_OK;

    uint32_t pending_sz_bytes = stream_bytes;
    uint8_t *data_ptr;

    if (is_cont == 0U) {
        uint8_t spidma_ahb_bus_select = 0;
        uint8_t spidma_auto_incr = 1;
        uint8_t spidma_bus_size = (uint8_t)bus_size; // Size of read/write. 0 = byte, 1 = half-word, 2 = full-word, 3 = invalid.
        uint8_t spidma_sys_codeb = 0;
        uint8_t spidma_rd_wrb = 1;
        uint8_t spidma_ctl_data = (uint8_t)((spidma_rd_wrb << 7) | ((spidma_sys_codeb << 6) | ((spidma_bus_size << 2) | ((spidma_auto_incr << 1) | (spidma_ahb_bus_select << 0)))));

        err = indirect_addr_set(desc, reg, spidma_ctl_data);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    *data = 0;

    data_ptr = (uint8_t *) data;

    while (pending_sz_bytes > 0U) {
        if ((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) && (pending_sz_bytes > 3U)) {
            err = spi_regio_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA3_ADDR : REG_SPIDMA0_DATA3_ADDR, (data_ptr + 3));
            ADI_APOLLO_ERROR_RETURN(err);
            err = spi_regio_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA2_ADDR : REG_SPIDMA0_DATA2_ADDR, (data_ptr + 2));
            ADI_APOLLO_ERROR_RETURN(err);
            pending_sz_bytes -= 2U;
        }

        if (((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) || (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16)) && (pending_sz_bytes > 1U)) {
            err = spi_regio_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA1_ADDR : REG_SPIDMA0_DATA1_ADDR, (data_ptr + 1));
            ADI_APOLLO_ERROR_RETURN(err);
            pending_sz_bytes -= 1U;
        }

        err = spi_regio_read(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA0_ADDR : REG_SPIDMA0_DATA0_ADDR, (data_ptr + 0));
        ADI_APOLLO_ERROR_RETURN(err);
        pending_sz_bytes -= 1U;

        if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
            data_ptr += 4;
        } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
            data_ptr += 2;
        } else {
            data_ptr += 1;
        }
    }
    ADI_CMS_RETURN(err);
}

/*
 * stream_writex - pseudo write stream function that doesn't generate any streaming transactions. Use when wr_stream_en = 0.
 */
static int32_t stream_writex(adi_apollo_hal_regio_spi_desc_t *desc, uint32_t reg, uint8_t *data, adi_apollo_hal_regio_spi_bus_size_e bus_size, uint32_t stream_bytes, uint8_t is_cont) {
    int32_t err;

    uint32_t pending_sz_bytes = stream_bytes;
    uint32_t data_index = 0;

    err = (int32_t)API_CMS_ERROR_OK;

    if (is_cont == 0U) {
        uint8_t spidma_ahb_bus_select = 0;
        uint8_t spidma_auto_incr = 1;
        uint8_t spidma_bus_size = (uint8_t)bus_size; // Size of read/write. 0 = byte, 1 = half-word, 2 = full-word, 3 = invalid.
        uint8_t spidma_sys_codeb = 0;
        uint8_t spidma_rd_wrb = 0;
        uint8_t spidma_ctl_data = (uint8_t)((spidma_rd_wrb << 7) | ((spidma_sys_codeb << 6) | ((spidma_bus_size << 2) | ((spidma_auto_incr << 1) | (spidma_ahb_bus_select << 0)))));

        err = indirect_addr_set(desc, reg, spidma_ctl_data);
        ADI_APOLLO_ERROR_RETURN(err);
    }

    while (pending_sz_bytes > 0U) {
        if ((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) && (pending_sz_bytes > 3U)) {
            err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA3_ADDR : REG_SPIDMA0_DATA3_ADDR, (data[data_index + 3U]) & 0xFFU);
            ADI_APOLLO_ERROR_RETURN(err);
            err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA2_ADDR : REG_SPIDMA0_DATA2_ADDR, (data[data_index + 2U]) & 0xFFU);
            ADI_APOLLO_ERROR_RETURN(err);
            pending_sz_bytes -= 2U;
        }

        if (((bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) || (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16)) && (pending_sz_bytes > 1U)) {
            err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA1_ADDR : REG_SPIDMA0_DATA1_ADDR, (data[data_index + 1U]) & 0xFFU);
            ADI_APOLLO_ERROR_RETURN(err);
            pending_sz_bytes -= 1U;
        }

        err = spi_regio_write(desc, (desc->is_spi1 != 0U) ? REG_SPIDMA1_DATA0_ADDR : REG_SPIDMA0_DATA0_ADDR, (data[data_index]) & 0xFFU);
        ADI_APOLLO_ERROR_RETURN(err);
        pending_sz_bytes -= 1U;

        if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D32) {
            data_index += 4U;
        } else if (bus_size == ADI_APOLLO_HAL_REGIO_SPI_BUS_SIZE_D16) {
            data_index += 2U;
        } else {
            data_index += 1U;
        }
    }

    ADI_CMS_RETURN(err);
}
