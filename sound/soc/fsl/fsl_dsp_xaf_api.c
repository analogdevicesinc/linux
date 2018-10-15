// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
//
// Xtensa Audio Framework API for communication with DSP
//
// Copyright (C) 2017 Cadence Design Systems, Inc.
// Copyright 2018 NXP

#include "fsl_dsp.h"
#include "fsl_dsp_xaf_api.h"

/* ...send a command message to component */
int xf_command(struct xf_client *client, struct xf_handle *handle,
	       u32 port, u32 opcode, void *buffer, u32 length)
{
	struct xf_proxy *proxy = handle->proxy;
	struct xf_message  msg;

	/* ...fill-in message parameters */
	msg.id = __XF_MSG_ID(__XF_AP_CLIENT(0, 0),
				__XF_PORT_SPEC2(handle->id, port));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = opcode;
	msg.length = length;
	msg.buffer = buffer;
	msg.ret = 0;

	/* ...execute command synchronously */
	return xf_cmd_send(proxy, msg.id, msg.opcode, msg.buffer, msg.length);
}

int xaf_comp_set_config(struct xf_client *client, struct xaf_comp *p_comp,
			u32 num_param, void *p_param)
{
	struct xf_handle *p_handle;
	struct xf_message msg;
	struct xf_message *rmsg;
	struct xf_set_param_msg *smsg;
	struct xf_set_param_msg *param = (struct xf_set_param_msg *)p_param;
	struct xf_proxy *proxy;
	u32 i;

	p_handle = &p_comp->handle;
	proxy = p_handle->proxy;

	/* ...set persistent stream characteristics */
	smsg = xf_buffer_data(p_handle->aux);

	for (i = 0; i < num_param; i++) {
		smsg[i].id = param[i].id;
		smsg[i].value = param[i].value;
	}

	/* ...set command parameters */
	msg.id = __XF_MSG_ID(__XF_AP_CLIENT(0, 0),
				__XF_PORT_SPEC2(p_handle->id, 0));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = XF_SET_PARAM;
	msg.length = sizeof(*smsg) * num_param;
	msg.buffer = smsg;
	msg.ret = 0;

	/* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);

	if(IS_ERR(rmsg))
		return PTR_ERR(rmsg);
	/* ...save received component global client-id */
	/* TODO: review cleanup */
	/* xf_msg_free(proxy, rmsg);
	 * xf_unlock(&proxy->lock);
	 */

	/* ...make sure response is expected */
	if ((rmsg->opcode != XF_SET_PARAM) || (rmsg->buffer != smsg)) {
		return -EPIPE;
	}

	return 0;
}

int xaf_comp_get_config(struct xf_client *client, struct xaf_comp *p_comp,
			u32 num_param, void *p_param)
{

	struct xf_handle *p_handle;
	struct xf_message msg;
	struct xf_message *rmsg;
	struct xf_get_param_msg *smsg;
	struct xf_get_param_msg *param = (struct xf_get_param_msg *)p_param;
	struct xf_proxy *proxy;
	u32 i;

	p_handle = &p_comp->handle;
	proxy = p_handle->proxy;

	/* ...set persistent stream characteristics */
	smsg = xf_buffer_data(p_handle->aux);

	for (i = 0; i < num_param; i++)
		smsg[i].id = param[i].id;


	msg.id = __XF_MSG_ID(__XF_AP_CLIENT(0, 0),
				__XF_PORT_SPEC2(p_handle->id, 0));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = XF_GET_PARAM;
	msg.length = sizeof(*smsg) * num_param;
	msg.buffer = smsg;
	msg.ret = 0;

	/* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);

	/* ...save received component global client-id */
	if(IS_ERR(rmsg))
		return PTR_ERR(rmsg);

	/* TODO: review cleanup */
	/* xf_msg_free(proxy, rmsg);
	 * xf_unlock(&proxy->lock); */

	/* ...make sure response is expected */
	if ((rmsg->opcode != (u32)XF_GET_PARAM) || (rmsg->buffer != smsg)) {
		return -EPIPE;
	}

	for (i = 0; i < num_param; i++)
		param[i].value = smsg[i].value;

	return 0;
}

int xaf_comp_flush(struct xf_client *client, struct xaf_comp *p_comp)
{

	struct xf_handle       *p_handle;
	struct xf_proxy        *proxy;
	struct xf_message     msg;
	struct xf_message     *rmsg;

	p_handle = &p_comp->handle;
	proxy = p_handle->proxy;

	msg.id = __XF_MSG_ID(__XF_AP_CLIENT(0, 0),
				__XF_PORT_SPEC2(p_handle->id, 0));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = XF_FLUSH;
	msg.length = 0;
	msg.buffer = NULL;
	msg.ret = 0;


	/* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);

	if(IS_ERR(rmsg))
		return PTR_ERR(rmsg);
	
	/* ...make sure response is expected */
	if ((rmsg->opcode != (u32)XF_FLUSH) || rmsg->buffer) {
		return -EPIPE;
	}

	return 0;
}

int xaf_comp_create(struct xf_client *client, struct xf_proxy *proxy,
		    struct xaf_comp *p_comp, int comp_type)
{
	struct fsl_dsp *dsp_priv = container_of(proxy, struct fsl_dsp, proxy);
	char   lib_path[200];
	char   lib_wrap_path[200];
	struct xf_handle *p_handle;
	struct xf_buffer *buf;
	int    ret = 0;
	bool   loadlib = true;

	memset((void *)p_comp, 0, sizeof(struct xaf_comp));

	strcpy(lib_path, "/usr/lib/imx-mm/audio-codec/dsp/");
	strcpy(lib_wrap_path, "/usr/lib/imx-mm/audio-codec/dsp/");

	p_handle = &p_comp->handle;

	p_comp->comp_type = comp_type;

	if (comp_type == RENDER_ESAI)
		loadlib = false;

	if (loadlib) {
		p_comp->codec_lib.filename      = lib_path;
		p_comp->codec_wrap_lib.filename = lib_wrap_path;
		p_comp->codec_lib.lib_type      = DSP_CODEC_LIB;
	}

	switch (comp_type) {
	case CODEC_MP3_DEC:
		p_comp->dec_id = "audio-decoder/mp3";
		strcat(lib_path, "lib_dsp_mp3_dec.so");
		break;
	case CODEC_AAC_DEC:
		p_comp->dec_id = "audio-decoder/aac";
		strcat(lib_path, "lib_dsp_aac_dec.so");
		break;
	case RENDER_ESAI:
		p_comp->dec_id = "renderer/esai";
		break;

	default:
		return -EINVAL;
		break;
	}

	/* ...create decoder component instance (select core-0) */
	ret = xf_open(client, proxy, p_handle, p_comp->dec_id, 0, NULL);
	if (ret) {
		dev_err(dsp_priv->dev, "create (%s) component error: %d\n",
			p_comp->dec_id, ret);
		return ret;
	}

	if (loadlib) {
		strcat(lib_wrap_path, "lib_dsp_codec_wrap.so");
		p_comp->codec_wrap_lib.lib_type = DSP_CODEC_WRAP_LIB;

		/* ...load codec wrapper lib */
		ret = xf_load_lib(client, p_handle, &p_comp->codec_wrap_lib);
		if (ret) {
			dev_err(dsp_priv->dev, "load codec wrap lib error\n");
			return ret;
		}

		/* ...load codec lib */
		ret = xf_load_lib(client, p_handle, &p_comp->codec_lib);
		if (ret) {
			dev_err(dsp_priv->dev, "load codec lib error\n");
			return ret;
		}

		/* ...allocate input buffer */
		ret = xf_pool_alloc(client, proxy, 1, INBUF_SIZE,
				    XF_POOL_INPUT, &p_comp->inpool);
		if (ret) {
			dev_err(dsp_priv->dev, "alloc input buf error\n");
			return ret;
		}

		/* ...initialize input buffer pointer */
		buf = xf_buffer_get(p_comp->inpool);
		p_comp->inptr = xf_buffer_data(buf);
	}

	return ret;
}

int xaf_comp_delete(struct xf_client *client, struct xaf_comp *p_comp)
{

	struct xf_handle *p_handle;
	bool   loadlib = true;
	u32 ret = 0;

	if (p_comp->comp_type == RENDER_ESAI)
		loadlib = false;

	p_handle = &p_comp->handle;

	if (loadlib) {
		/* ...unload codec wrapper library */
		xf_unload_lib(client, p_handle, &p_comp->codec_wrap_lib);

		/* ...unload codec library */
		xf_unload_lib(client, p_handle, &p_comp->codec_lib);

		xf_pool_free(client, p_comp->inpool);
	}

	/* ...delete component */
	xf_close(client, p_handle);

	return ret;
}

int xaf_comp_process(struct xf_client *client, struct xaf_comp *p_comp, void *p_buf, u32 length, u32 flag)
{
	struct xf_handle *p_handle;
	u32 ret = 0;

	p_handle = &p_comp->handle;

	switch (flag) {
	case XF_FILL_THIS_BUFFER:
		/* ...send message to component output port (port-id=1) */
		ret = xf_command(client, p_handle, 1, XF_FILL_THIS_BUFFER,
				 p_buf, length);
		break;
	case XF_EMPTY_THIS_BUFFER:
		/* ...send message to component input port (port-id=0) */
		ret = xf_command(client, p_handle, 0, XF_EMPTY_THIS_BUFFER,
				 p_buf, length);
		break;
	default:
		break;
	}

	return ret;
}

/* ...port binding function */
int xf_route(struct xf_client *client, struct xf_handle *src, u32 src_port,
	     struct xf_handle *dst, u32 dst_port, u32 num, u32 size, u32 align)
{
	struct xf_proxy *proxy = src->proxy;
	struct xf_buffer *b;
	struct xf_route_port_msg *m;
	struct xf_message msg;
	struct xf_message *rmsg;

	/* ...sanity checks - proxy pointers are same */
	if (proxy != dst->proxy)
		return -EINVAL;

	/* ...buffer data is sane */
	if (!(num && size && xf_is_power_of_two(align)))
		return -EINVAL;

	/* ...get control buffer */
	if ((b = xf_buffer_get(proxy->aux)) == NULL)
		return -EBUSY;

	/* ...get message buffer */
	m = xf_buffer_data(b);

	/* ...fill-in message parameters */
	m->src = __XF_PORT_SPEC2(src->id, src_port);
	m->dst = __XF_PORT_SPEC2(dst->id, dst_port);
	m->alloc_number = num;
	m->alloc_size = size;
	m->alloc_align = align;

	/* ...set command parameters */
	msg.id = __XF_MSG_ID(__XF_AP_PROXY(0),
			     __XF_PORT_SPEC2(src->id, src_port));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = XF_ROUTE;
	msg.length = sizeof(*m);
	msg.buffer = m;
	msg.ret = 0;

	/* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);
	if(IS_ERR(rmsg))
		return PTR_ERR(rmsg);

	/* ...save received component global client-id */
	/* TODO: review cleanup */
	/* xf_msg_free(proxy, rmsg);
	 * xf_unlock(&proxy->lock); */


        /* ...synchronously execute command on remote DSP */
	/* XF_CHK_API(xf_proxy_cmd_exec(proxy, &msg)); */

	/* ...return buffer to proxy */
	xf_buffer_put(b);

        /* ...check result is successful */
	/* XF_CHK_ERR(msg.opcode == XF_ROUTE, -ENOMEM); */

	return 0;
}

/* ...port unbinding function */
int xf_unroute(struct xf_client *client, struct xf_handle *src, u32 src_port)
{
	struct xf_proxy        *proxy = src->proxy;
	struct xf_buffer       *b;
	struct xf_unroute_port_msg  *m;
	struct xf_message      msg;
	struct xf_message     *rmsg;
	int                  r = 0;

	/* ...get control buffer */
	if((b = xf_buffer_get(proxy->aux)) == NULL)
		return -EBUSY;

	/* ...get message buffer */
	m = xf_buffer_data(b);

	/* ...fill-in message parameters */
	m->src = __XF_PORT_SPEC2(src->id, src_port);

	/* ...set command parameters */
	msg.id = __XF_MSG_ID(__XF_AP_PROXY(0),
			__XF_PORT_SPEC2(src->id, src_port));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = XF_UNROUTE;
	msg.length = sizeof(*m);
	msg.buffer = m;
	msg.ret = 0;

	/* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);
	if (IS_ERR(rmsg))
		return PTR_ERR(rmsg);
	/* ...save received component global client-id */

	/*TODO: review cleanup */
	/* xf_msg_free(proxy, rmsg); */
	/* xf_unlock(&proxy->lock); */

        /* ...return buffer to proxy */
        xf_buffer_put(b);

        return r;
}

int xaf_connect(struct xf_client *client,
		struct xaf_comp *p_src,
		struct xaf_comp *p_dest,
		u32 num_buf,
		u32 buf_length)
{
	/* ...connect p_src output port with p_dest input port */
	return xf_route(client, &p_src->handle, 0, &p_dest->handle, 0,
			num_buf, buf_length, 8);
}

int xaf_disconnect(struct xf_client *client, struct xaf_comp *p_comp)
{
	/* ...disconnect p_src output port with p_dest input port */
	return xf_unroute(client, &p_comp->handle, 0);

}

int xaf_comp_add(struct xaf_pipeline *p_pipe, struct xaf_comp *p_comp)
{
	int ret = 0;

	p_comp->next = p_pipe->comp_chain;
	p_comp->pipeline = p_pipe;
	p_pipe->comp_chain = p_comp;

	return ret;
}

int xaf_pipeline_create(struct xaf_pipeline *p_pipe)
{
	int ret = 0;

	memset(p_pipe, 0, sizeof(struct xaf_pipeline));

	return ret;
}

int xaf_pipeline_delete(struct xaf_pipeline *p_pipe)
{
	int ret = 0;

	memset(p_pipe, 0, sizeof(struct xaf_pipeline));

	return ret;
}
