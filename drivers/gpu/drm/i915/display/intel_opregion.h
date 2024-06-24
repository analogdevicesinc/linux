/*
 * Copyright © 2008-2017 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#ifndef _INTEL_OPREGION_H_
#define _INTEL_OPREGION_H_

#include <linux/pci.h>
#include <linux/types.h>

struct drm_i915_private;
struct intel_connector;
struct intel_encoder;

#ifdef CONFIG_ACPI

int intel_opregion_setup(struct drm_i915_private *dev_priv);
void intel_opregion_cleanup(struct drm_i915_private *i915);

void intel_opregion_register(struct drm_i915_private *dev_priv);
void intel_opregion_unregister(struct drm_i915_private *dev_priv);

void intel_opregion_resume(struct drm_i915_private *dev_priv);
void intel_opregion_suspend(struct drm_i915_private *dev_priv,
			    pci_power_t state);

bool intel_opregion_asle_present(struct drm_i915_private *i915);
void intel_opregion_asle_intr(struct drm_i915_private *dev_priv);
int intel_opregion_notify_encoder(struct intel_encoder *intel_encoder,
				  bool enable);
int intel_opregion_notify_adapter(struct drm_i915_private *dev_priv,
				  pci_power_t state);
int intel_opregion_get_panel_type(struct drm_i915_private *dev_priv);
const struct drm_edid *intel_opregion_get_edid(struct intel_connector *connector);

bool intel_opregion_vbt_present(struct drm_i915_private *i915);
const void *intel_opregion_get_vbt(struct drm_i915_private *i915, size_t *size);

bool intel_opregion_headless_sku(struct drm_i915_private *i915);

void intel_opregion_debugfs_register(struct drm_i915_private *i915);

#else /* CONFIG_ACPI*/

static inline int intel_opregion_setup(struct drm_i915_private *dev_priv)
{
	return 0;
}

static inline void intel_opregion_cleanup(struct drm_i915_private *i915)
{
}

static inline void intel_opregion_register(struct drm_i915_private *dev_priv)
{
}

static inline void intel_opregion_unregister(struct drm_i915_private *dev_priv)
{
}

static inline void intel_opregion_resume(struct drm_i915_private *dev_priv)
{
}

static inline void intel_opregion_suspend(struct drm_i915_private *dev_priv,
					  pci_power_t state)
{
}

static inline bool intel_opregion_asle_present(struct drm_i915_private *i915)
{
	return false;
}

static inline void intel_opregion_asle_intr(struct drm_i915_private *dev_priv)
{
}

static inline int
intel_opregion_notify_encoder(struct intel_encoder *intel_encoder, bool enable)
{
	return 0;
}

static inline int
intel_opregion_notify_adapter(struct drm_i915_private *dev, pci_power_t state)
{
	return 0;
}

static inline int intel_opregion_get_panel_type(struct drm_i915_private *dev)
{
	return -ENODEV;
}

static inline const struct drm_edid *
intel_opregion_get_edid(struct intel_connector *connector)
{
	return NULL;
}

static inline bool intel_opregion_vbt_present(struct drm_i915_private *i915)
{
	return false;
}

static inline const void *
intel_opregion_get_vbt(struct drm_i915_private *i915, size_t *size)
{
	return NULL;
}

static inline bool intel_opregion_headless_sku(struct drm_i915_private *i915)
{
	return false;
}

static inline void intel_opregion_debugfs_register(struct drm_i915_private *i915)
{
}

#endif /* CONFIG_ACPI */

#endif
