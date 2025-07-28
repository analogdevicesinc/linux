/* SPDX-License-Identifier: MIT */
#ifndef __NVKM_NVENC_PRIV_H__
#define __NVKM_NVENC_PRIV_H__
#include <engine/nvenc.h>

struct nvkm_nvenc_func {
	const struct nvkm_falcon_func *flcn;
};

struct nvkm_nvenc_fwif {
	int version;
	int (*load)(struct nvkm_nvenc *, int ver,
		    const struct nvkm_nvenc_fwif *);
	const struct nvkm_nvenc_func *func;
};

extern const struct nvkm_nvenc_fwif gm107_nvenc_fwif[];

int nvkm_nvenc_new_(const struct nvkm_nvenc_fwif *, struct nvkm_device *, enum nvkm_subdev_type,
		    int, struct nvkm_nvenc **pnvenc);

int r535_nvenc_new(const struct nvkm_engine_func *, struct nvkm_device *,
		   enum nvkm_subdev_type, int, struct nvkm_nvenc **);
#endif
