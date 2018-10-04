/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Xtensa buffer pool API header
 *
 * Copyright 2018 NXP
 * Copyright (c) 2012-2013 by Tensilica Inc
 */
#ifndef FSL_DSP_POOL_H
#define FSL_DSP_POOL_H

#include <linux/types.h>
#include "fsl_dsp_proxy.h"

/* ...buffer pool type */
typedef u32 xf_pool_type_t;

/* ...previous declaration of struct */
struct xf_buffer;
struct xf_pool;
struct xf_handle;
struct xf_message;
struct xf_client;

/* ...response callback */
typedef void (*xf_response_cb)(struct xf_handle *h, struct xf_message *msg);

/* ...buffer pool type */
enum xf_pool_type {
        XF_POOL_AUX = 0,
        XF_POOL_INPUT = 1,
        XF_POOL_OUTPUT = 2
};

/* ...buffer link pointer */
union xf_buffer_link {
        /* ...pointer to next free buffer in a pool (for free buffer) */
        struct xf_buffer *next;
        /* ...reference to a buffer pool (for allocated buffer) */
        struct xf_pool *pool;
};

/* ...buffer descriptor */
struct xf_buffer {
        /* ...virtual address of contiguous buffer */
        void *address;
        /* ...link pointer */
        union xf_buffer_link link;
};

/* ...buffer pool */
struct xf_pool {
        /* ...reference to proxy data */
        struct xf_proxy *proxy;
        /* ...length of individual buffer in a pool */
        u32 length;
        /* ...number of buffers in a pool */
        u32 number;
        /* ...pointer to pool memory */
        void *p;
        /* ...pointer to first free buffer in a pool */
        struct xf_buffer *free;
        /* ...individual buffers */
        struct xf_buffer buffer[0];
};

/* component handle */
struct xf_handle {
        /* ...reference to proxy data */
        struct xf_proxy *proxy;
        /* ...auxiliary control buffer for control transactions */
        struct xf_buffer *aux;
        /* ...global client-id of the component */
        u32 id;
        /* ...local client number (think about merging into "id" field - tbd) */
        u32 client;
        /* ...response processing hook */
        xf_response_cb response;
};

/* ...accessor to buffer data */
static inline void *xf_buffer_data(struct xf_buffer *buffer)
{
        return buffer->address;
}

/* ...length of buffer data */
static inline size_t xf_buffer_length(struct xf_buffer *buffer)
{
        struct xf_pool *pool = buffer->link.pool;

        return (size_t)pool->length;
}

/* ...component client-id (global scope) */
static inline u32 xf_handle_id(struct xf_handle *handle)
{
        return handle->id;
}

/* ...pointer to auxiliary buffer */
static inline void *xf_handle_aux(struct xf_handle *handle)
{
        return xf_buffer_data(handle->aux);
}

int xf_pool_alloc(struct xf_client *client, struct xf_proxy *proxy, u32 number,
		  u32 length, xf_pool_type_t type, struct xf_pool **pool);
int xf_pool_free(struct xf_client *client, struct xf_pool *pool);

struct xf_buffer *xf_buffer_get(struct xf_pool *pool);
void xf_buffer_put(struct xf_buffer *buffer);

#endif /* FSL_DSP_POOL_H */
