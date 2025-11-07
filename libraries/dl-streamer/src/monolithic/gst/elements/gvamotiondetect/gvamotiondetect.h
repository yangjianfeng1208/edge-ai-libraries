/*******************************************************************************
 * Copyright (C) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#pragma once
#include <gst/base/gstbasetransform.h>
#include <gst/gst.h>

G_BEGIN_DECLS

#define GST_TYPE_GVA_MOTION_DETECT (gst_gva_motion_detect_get_type())
G_DECLARE_FINAL_TYPE(GstGvaMotionDetect, gst_gva_motion_detect, GST, GVA_MOTION_DETECT, GstBaseTransform)

G_END_DECLS