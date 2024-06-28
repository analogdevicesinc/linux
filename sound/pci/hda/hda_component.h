/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * HD audio Component Binding Interface
 *
 * Copyright (C) 2021 Cirrus Logic, Inc. and
 *                    Cirrus Logic International Semiconductor Ltd.
 */

#ifndef __HDA_COMPONENT_H__
#define __HDA_COMPONENT_H__

#include <linux/acpi.h>
#include <linux/component.h>
#include <sound/hda_codec.h>

#define HDA_MAX_COMPONENTS	4
#define HDA_MAX_NAME_SIZE	50

struct hda_component {
	struct device *dev;
	char name[HDA_MAX_NAME_SIZE];
	struct hda_codec *codec;
	struct acpi_device *adev;
	bool acpi_notifications_supported;
	void (*acpi_notify)(acpi_handle handle, u32 event, struct device *dev);
	void (*pre_playback_hook)(struct device *dev, int action);
	void (*playback_hook)(struct device *dev, int action);
	void (*post_playback_hook)(struct device *dev, int action);
};

#ifdef CONFIG_ACPI
void hda_component_acpi_device_notify(struct hda_component *comps, int num_comps,
				      acpi_handle handle, u32 event, void *data);
int hda_component_manager_bind_acpi_notifications(struct hda_codec *cdc,
						  struct hda_component *comps, int num_comps,
						  acpi_notify_handler handler, void *data);
void hda_component_manager_unbind_acpi_notifications(struct hda_codec *cdc,
						     struct hda_component *comps,
						     acpi_notify_handler handler);
#else
static inline void hda_component_acpi_device_notify(struct hda_component *comps,
						    int num_comps,
						    acpi_handle handle,
						    u32 event,
						    void *data)
{
}

static inline int hda_component_manager_bind_acpi_notifications(struct hda_codec *cdc,
								struct hda_component *comps,
								int num_comps,
								acpi_notify_handler handler,
								void *data)

{
	return 0;
}

static inline void hda_component_manager_unbind_acpi_notifications(struct hda_codec *cdc,
								   struct hda_component *comps,
								   acpi_notify_handler handler)
{
}
#endif /* ifdef CONFIG_ACPI */

void hda_component_manager_playback_hook(struct hda_component *comps, int num_comps,
					 int action);

int hda_component_manager_init(struct hda_codec *cdc,
			       struct hda_component *comps, int count,
			       const char *bus, const char *hid,
			       const char *match_str,
			       const struct component_master_ops *ops);

void hda_component_manager_free(struct hda_codec *cdc,
				const struct component_master_ops *ops);

int hda_component_manager_bind(struct hda_codec *cdc,
			       struct hda_component *comps, int count);

static inline void hda_component_manager_unbind(struct hda_codec *cdc,
					       struct hda_component *comps)
{
	component_unbind_all(hda_codec_dev(cdc), comps);
}

#endif /* ifndef __HDA_COMPONENT_H__ */
