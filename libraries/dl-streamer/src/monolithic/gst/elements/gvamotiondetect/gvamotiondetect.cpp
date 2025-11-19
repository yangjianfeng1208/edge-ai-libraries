/*******************************************************************************
 * Copyright (C) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "gvamotiondetect.h"
#include <gst/base/gstbasetransform.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <glib.h>

// Platform‑specific includes: VA path only for non-MSVC builds
#ifndef _MSC_VER
#include <gmodule.h>
#include <gst/va/gstvadisplay.h>
#include <gst/va/gstvautils.h>
#include <opencv2/core/va_intel.hpp>
#include <va/va.h>
#include <va/va.h>
#endif

#include <gst/video/video.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// Explicit OpenCL headers removed: element now uses only generic OpenCV UMat (which may internally use OpenCL).
#include <cmath> // for std::lround
#include <vector>
#include <algorithm>

#include <string>
#include <dlstreamer/gst/videoanalytics/video_frame.h> // analytics meta types (GstAnalyticsRelationMeta, GstAnalyticsODMtd)

// Removed legacy meta_attacher include: not needed for manual motion metadata attachment.

// (No dummy converter needed; motion ROIs attached directly as analytics relation + ROI metas.)

// Removed G_BEGIN_DECLS / G_END_DECLS to avoid forcing C linkage on C++ helper functions

GST_DEBUG_CATEGORY_STATIC(gst_gva_motion_detect_debug);
#define GST_CAT_DEFAULT gst_gva_motion_detect_debug

struct MotionRect {
    gint x;
    gint y;
    gint w;
    gint h;
};

/* Property identifiers */
enum {
    PROP_0,
    PROP_BLOCK_SIZE,
    PROP_MOTION_THRESHOLD
};

struct _GstGvaMotionDetect {
    GstBaseTransform parent;
    GstVideoInfo vinfo;
    gboolean caps_is_va; // retained for compatibility

    // Common (both platforms)
    int blur_kernel;      // odd size (used for potential future smoothing)
    double blur_sigma;    // gaussian sigma
    uint64_t frame_index; // running frame counter

#ifndef _MSC_VER
    VADisplay va_dpy;
    GstVaDisplay *va_display;
    cv::UMat scratch;
    cv::Mat overlay_cpu;  // host-side drawing buffer (BGRA)
    cv::UMat overlay_gpu; // device-side buffer used for blending
    bool overlay_ready;
    std::string last_text;
    VASurfaceID prev_sid; // simple 1‑frame history
#else
    // Windows (MSVC) build: no VAAPI. Provide stubs to satisfy logic.
    void *va_dpy;
    void *va_display;
    int prev_sid;
#endif

    /* Motion detection previous frame state */
    cv::UMat prev_small_gray;
    cv::UMat prev_luma;

    /* Grid detection parameters (properties) */
    int block_size;
    double motion_threshold;

    /* Debug controls */
    gboolean debug_enabled;
    guint debug_interval;
    guint64 last_debug_frame;
    gboolean tried_va_query;

    /* Concurrency guard for metadata operations */
    GMutex meta_mutex;
};

/* Forward declarations of property handlers */
static void gst_gva_motion_detect_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec);
static void gst_gva_motion_detect_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec);

static void gst_gva_motion_detect_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(object);
    switch (prop_id) {
    case PROP_BLOCK_SIZE:
        g_value_set_int(value, self->block_size);
        break;
    case PROP_MOTION_THRESHOLD:
        g_value_set_double(value, self->motion_threshold);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    }
}

static void gst_gva_motion_detect_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(object);
    switch (prop_id) {
    case PROP_BLOCK_SIZE:
        self->block_size = g_value_get_int(value);
        break;
    case PROP_MOTION_THRESHOLD:
        self->motion_threshold = g_value_get_double(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    }
}

/* Define class struct prior to G_DEFINE_TYPE so sizeof works */
struct _GstGvaMotionDetectClass {
    GstBaseTransformClass parent_class;
};

G_DEFINE_TYPE(GstGvaMotionDetect, gst_gva_motion_detect, GST_TYPE_BASE_TRANSFORM)

#ifndef _MSC_VER
static GstStaticPadTemplate sink_templ =
    GST_STATIC_PAD_TEMPLATE("sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS("video/x-raw(memory:VAMemory)"));
static GstStaticPadTemplate src_templ =
    GST_STATIC_PAD_TEMPLATE("src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS("video/x-raw(memory:VAMemory)"));
#else
// Windows build: system memory only
static GstStaticPadTemplate sink_templ =
    GST_STATIC_PAD_TEMPLATE("sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS("video/x-raw"));
static GstStaticPadTemplate src_templ =
    GST_STATIC_PAD_TEMPLATE("src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS("video/x-raw"));
#endif

static void gst_gva_motion_detect_set_context(GstElement *elem, GstContext *context) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(elem);
    const gchar *ctype = gst_context_get_context_type(context);
    const GstStructure *st = gst_context_get_structure(context);
#ifndef _MSC_VER
    if (g_strcmp0(ctype, "gst.va.display.handle") == 0 && !self->va_dpy) {
        if (gst_structure_has_field(st, "va-display")) {
            self->va_dpy = (VADisplay)g_value_get_pointer(gst_structure_get_value(st, "va-display"));
        } else if (gst_structure_has_field(st, "gst-display")) {
            GstObject *obj = nullptr;
            if (gst_structure_get(st, "gst-display", GST_TYPE_OBJECT, &obj, nullptr) && obj) {
                self->va_dpy = (VADisplay)gst_va_display_get_va_dpy(GST_VA_DISPLAY(obj));
                gst_object_unref(obj);
            }
        }
    }
#endif
    GST_ELEMENT_CLASS(gst_gva_motion_detect_parent_class)->set_context(elem, context);
}

static gboolean gst_gva_motion_detect_query(GstElement *elem, GstQuery *query) {
#ifndef _MSC_VER
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(elem);
    if (gst_va_handle_context_query(elem, query, self->va_display))
        return TRUE;
    return GST_ELEMENT_CLASS(gst_gva_motion_detect_parent_class)->query(elem, query);
#else
    return GST_ELEMENT_CLASS(gst_gva_motion_detect_parent_class)->query(elem, query);
#endif
}

static GstCaps *gst_gva_motion_detect_transform_caps(GstBaseTransform *, GstPadDirection /*direction*/, GstCaps *caps,
                                                     GstCaps *filter) {
    GstCaps *ret = gst_caps_ref(caps);
    if (filter) {
        GstCaps *intersect = gst_caps_intersect_full(filter, ret, GST_CAPS_INTERSECT_FIRST);
        gst_caps_unref(ret);
        ret = intersect;
    }
    return ret;
}

// -----------------------------------------------------------------------------
#ifndef _MSC_VER
// VA API helper functions (extracted from transform_ip for clarity)
// Map GST buffer to VA surface (using mapper + fallback) and return VASurfaceID
static VASurfaceID gva_motion_detect_get_surface(GstGvaMotionDetect *self, GstBuffer *buf) {
    if (!buf)
        return VA_INVALID_SURFACE;
    VASurfaceID sid = gst_va_buffer_get_surface(buf);
    if (sid != VA_INVALID_SURFACE)
        return sid;
    guint n = gst_buffer_n_memory(buf);
    for (guint i = 0; i < n; ++i) {
        GstMemory *m = gst_buffer_peek_memory(buf, i);
        if (m && gst_memory_is_type(m, "VAMemory")) {
            sid = gst_va_memory_get_surface(m);
            if (sid != VA_INVALID_SURFACE)
                return sid;
        }
    }
    return VA_INVALID_SURFACE;
}

// Convert VA surface to cv::UMat (GPU backed); returns false on failure
static bool gva_motion_detect_convert_from_surface(GstGvaMotionDetect *self, VASurfaceID sid, int width, int height,
                                                   cv::UMat &out) {
    if (sid == VA_INVALID_SURFACE || !self->va_dpy)
        return false;
    try {
        cv::va_intel::convertFromVASurface(self->va_dpy, sid, cv::Size(width, height), out);
        return true;
    } catch (const cv::Exception &e) {
        GST_WARNING_OBJECT(self, "convertFromVASurface failed: %s", e.what());
        return false;
    }
}

// Write cv::UMat back into the VA surface; returns false on failure
static bool gva_motion_detect_write_to_surface(GstGvaMotionDetect *self, const cv::UMat &src, VASurfaceID sid,
                                               int width, int height) {
    if (sid == VA_INVALID_SURFACE || !self->va_dpy)
        return false;
    try {
        cv::va_intel::convertToVASurface(self->va_dpy, src, sid, cv::Size(width, height));
        return true;
    } catch (const cv::Exception &e) {
        GST_WARNING_OBJECT(self, "convertToVASurface failed: %s", e.what());
        return false;
    }
}
#else
// Stubs for MSVC build (no VA available)
static inline int gva_motion_detect_get_surface(GstGvaMotionDetect *, GstBuffer *) {
    return -1;
}
static inline bool gva_motion_detect_convert_from_surface(GstGvaMotionDetect *, int, int, int, cv::UMat &) {
    return false;
}
static inline bool gva_motion_detect_write_to_surface(GstGvaMotionDetect *, const cv::UMat &, int, int, int) {
    return false;
}
#endif

static gboolean gst_gva_motion_detect_start(GstBaseTransform *trans) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(trans);
    gst_video_info_init(&self->vinfo);
    self->caps_is_va = FALSE;
    self->frame_index = 0;
    self->tried_va_query = FALSE;
#ifndef _MSC_VER
    self->va_dpy = nullptr;
    self->va_display = nullptr;
    gst_element_post_message(GST_ELEMENT(self),
                             gst_message_new_need_context(GST_OBJECT(self), "gst.va.display.handle"));
#endif
    return TRUE;
}

static gboolean gst_gva_motion_detect_set_caps(GstBaseTransform *trans, GstCaps *incaps, GstCaps *outcaps) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(trans);
    if (!gst_video_info_from_caps(&self->vinfo, incaps))
        return FALSE;
    // Stats probing removed: retain simple success path.
    return TRUE;
}

#ifndef _MSC_VER
// Helper to attach motion ROIs and associated analytics metadata (aggregated in a single relation meta)
static void gst_gva_motion_detect_attach_rois(GstGvaMotionDetect *self, GstBuffer *buf,
                                              const std::vector<MotionRect> &rois, int width, int height) {
    if (rois.empty())
        return;
    if (!gst_buffer_is_writable(buf)) {
        GstBuffer *writable = gst_buffer_make_writable(buf);
        if (writable != buf)
            buf = writable;
    }
    if (!gst_buffer_is_writable(buf)) {
        GST_WARNING_OBJECT(self, "Buffer not writable; skipping motion ROI attachment");
        return;
    }
    GstAnalyticsRelationMeta *relation_meta = gst_buffer_get_analytics_relation_meta(buf);
    if (!relation_meta) {
        relation_meta = gst_buffer_add_analytics_relation_meta(buf);
        GST_LOG_OBJECT(self, "Added new GstAnalyticsRelationMeta %p", relation_meta);
    } else {
        GST_LOG_OBJECT(self, "Reusing existing GstAnalyticsRelationMeta %p", relation_meta);
    }
    if (!relation_meta) {
        GST_WARNING_OBJECT(self, "Failed to add/get GstAnalyticsRelationMeta; skipping ROIs");
        return;
    }
    size_t attached = 0;
    for (const auto &r : rois) {
        double x = (double)r.x / (double)width;
        double y = (double)r.y / (double)height;
        double w = (double)r.w / (double)width;
        double h = (double)r.h / (double)height;
        if (!(x >= 0 && y >= 0 && w >= 0 && h >= 0 && x + w <= 1 && y + h <= 1)) {
            x = (x < 0) ? 0 : (x > 1 ? 1 : x);
            y = (y < 0) ? 0 : (y > 1 ? 1 : y);
            w = (w < 0) ? 0 : (w > 1 - x ? 1 - x : w);
            h = (h < 0) ? 0 : (h > 1 - y ? 1 - y : h);
        }
        double _x = x * width + 0.5;
        double _y = y * height + 0.5;
        double _w = w * width + 0.5;
        double _h = h * height + 0.5;
        GstStructure *detection = gst_structure_new("detection", "x_min", G_TYPE_DOUBLE, x, "x_max", G_TYPE_DOUBLE,
                                                  x + w, "y_min", G_TYPE_DOUBLE, y, "y_max", G_TYPE_DOUBLE, y + h,
                                                  "confidence", G_TYPE_DOUBLE, 1.0, NULL);
        GstAnalyticsODMtd od_mtd;
        if (!gst_analytics_relation_meta_add_od_mtd(relation_meta, g_quark_from_string("motion"),
                                                    (int)std::lround(_x), (int)std::lround(_y), (int)std::lround(_w),
                                                    (int)std::lround(_h), 1.0, &od_mtd)) {
            GST_WARNING_OBJECT(self, "Failed to add OD metadata for motion ROI");
            gst_structure_free(detection);
            continue;
        }
        GstVideoRegionOfInterestMeta *roi_meta = gst_buffer_add_video_region_of_interest_meta(
            buf, "motion", (guint)std::lround(_x), (guint)std::lround(_y), (guint)std::lround(_w),
            (guint)std::lround(_h));
        if (!roi_meta) {
            GST_WARNING_OBJECT(self, "Failed to add ROI meta for motion ROI");
            gst_structure_free(detection);
            continue;
        }
        roi_meta->id = od_mtd.id;
        gst_video_region_of_interest_meta_add_param(roi_meta, detection);
        GST_LOG_OBJECT(self, "Attached motion ROI id=%d rect=[%d,%d %dx%d]", od_mtd.id, (int)std::lround(_x),
                       (int)std::lround(_y), (int)std::lround(_w), (int)std::lround(_h));
        attached++;
    }
    // Enumerate OD metadata for debug correlation
    gpointer state_iter = nullptr;
    GstAnalyticsODMtd od_iter;
    size_t od_count = 0;
    while (gst_analytics_relation_meta_iterate(relation_meta, &state_iter, gst_analytics_od_mtd_get_mtd_type(),
                                               &od_iter)) {
        ++od_count;
    }
    GST_LOG_OBJECT(self, "Total OD metadata after attachment: %zu", od_count);
    GST_INFO_OBJECT(self, "Motion ROIs attached: %zu", attached);
    if (self->debug_enabled) {
        g_print("[gvamotiondetect] frame=%" G_GUINT64_FORMAT " ROIs=%zu (relation-meta aggregate)\n", self->frame_index,
                attached);
        fflush(stdout);
    }
}
#endif

#ifndef _MSC_VER
// Helper to merge overlapping motion rectangles in-place (simple O(n^2))
static void gst_gva_motion_detect_merge_rois(std::vector<MotionRect> &rois) {
    if (rois.empty())
        return;
    bool merged_any = true;
    while (merged_any) {
        merged_any = false;
        std::vector<MotionRect> out;
        std::vector<bool> used(rois.size(), false);
        for (size_t i = 0; i < rois.size(); ++i) {
            if (used[i])
                continue;
            MotionRect a = rois[i];
            for (size_t j = i + 1; j < rois.size(); ++j) {
                if (used[j])
                    continue;
                MotionRect b = rois[j];
                int ax2 = a.x + a.w, ay2 = a.y + a.h;
                int bx2 = b.x + b.w, by2 = b.y + b.h;
                bool overlap = !(bx2 < a.x || ax2 < b.x || by2 < a.y || ay2 < b.y);
                if (overlap) {
                    int nx = std::min(a.x, b.x);
                    int ny = std::min(a.y, b.y);
                    int nw = std::max(ax2, bx2) - nx;
                    int nh = std::max(ay2, by2) - ny;
                    a = MotionRect{nx, ny, nw, nh};
                    used[j] = true;
                    merged_any = true;
                }
            }
            out.push_back(a);
        }
        rois.swap(out);
    }
}
#endif

// In-place processing: get VASurfaceID via mapper
static GstFlowReturn gst_gva_motion_detect_transform_ip(GstBaseTransform *trans, GstBuffer *buf) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(trans);
#ifdef _MSC_VER
    // Windows software-only path: map buffer, build cv::Mat and run motion detection directly.
    ++self->frame_index;
    int width = GST_VIDEO_INFO_WIDTH(&self->vinfo);
    int height = GST_VIDEO_INFO_HEIGHT(&self->vinfo);
    if (!width || !height)
        return GST_FLOW_OK;
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ))
        return GST_FLOW_OK;
    // Assume packed RGB or BGR; fallback to gray if single channel
    cv::Mat frame_mat(height, width, CV_8UC3, (void *)map.data);
    cv::Mat gray;
    cv::cvtColor(frame_mat, gray, cv::COLOR_BGR2GRAY);
    cv::Mat prev;
    if (!self->prev_luma.empty())
        self->prev_luma.copyTo(prev);
    gray.copyTo(self->prev_luma);
    if (prev.empty()) {
        gst_buffer_unmap(buf, &map);
        return GST_FLOW_OK;
    }
    cv::Mat diff;
    cv::absdiff(gray, prev, diff);
    cv::GaussianBlur(diff, diff, cv::Size(3, 3), 0);
    cv::threshold(diff, diff, 15, 255, cv::THRESH_BINARY);
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(diff, diff, cv::MORPH_OPEN, k);
    cv::dilate(diff, diff, k);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<MotionRect> rois;
    for (auto &c : contours) {
        if (c.size() < 3)
            continue;
        cv::Rect r = cv::boundingRect(c);
        if (r.area() < width * height * 0.0005)
            continue;
        rois.push_back(MotionRect{r.x, r.y, r.width, r.height});
    }
    // For Windows build stripped of analytics meta system, just log ROI count.
    if (!rois.empty()) {
        GST_INFO_OBJECT(self, "Motion blocks detected: %zu (metadata attachment disabled)", rois.size());
    }
    gst_buffer_unmap(buf, &map);
    return GST_FLOW_OK;
#endif

#ifndef _MSC_VER
    // Acquire VA display via peer query if not yet set.
    if (!self->va_dpy) {
        if (!self->tried_va_query) {
            self->tried_va_query = TRUE;
            GstQuery *q = gst_query_new_context("gst.va.display.handle");
            if (gst_pad_peer_query(GST_BASE_TRANSFORM_SINK_PAD(trans), q)) {
                GstContext *ctx = nullptr;
                gst_query_parse_context(q, &ctx);
                if (ctx) {
                    GST_LOG_OBJECT(self, "Obtained VA context via peer query");
                    gst_gva_motion_detect_set_context(GST_ELEMENT(self), ctx);
                }
            }
            gst_query_unref(q);
        }
        if (!self->va_dpy) {
            GST_DEBUG_OBJECT(self, "No VADisplay (after peer query); pass-through frame=%" G_GUINT64_FORMAT,
                              self->frame_index);
            ++self->frame_index;
            return GST_FLOW_OK;
        }
    }

    // Map buffer to VA surface
    VASurfaceID sid = gva_motion_detect_get_surface(self, buf);
    if (sid == VA_INVALID_SURFACE) {
        GST_DEBUG_OBJECT(self, "Invalid VA surface; pass-through frame=%" G_GUINT64_FORMAT, self->frame_index);
        ++self->frame_index;
        return GST_FLOW_OK;
    }

    // Ensure surface ready
    VAStatus sync_st = vaSyncSurface(self->va_dpy, sid);
    if (sync_st != VA_STATUS_SUCCESS) {
        GST_WARNING_OBJECT(self, "vaSyncSurface failed sid=%u status=%d (%s)", sid, (int)sync_st, vaErrorStr(sync_st));
        ++self->frame_index;
        self->prev_sid = sid;
        return GST_FLOW_OK; // skip detection this frame
    }

    int width = GST_VIDEO_INFO_WIDTH(&self->vinfo);
    int height = GST_VIDEO_INFO_HEIGHT(&self->vinfo);
    if (!width || !height) {
        ++self->frame_index;
        self->prev_sid = sid;
        return GST_FLOW_OK;
    }

    // Convert VA surface to UMat
    cv::UMat frame_gpu;
    if (!gva_motion_detect_convert_from_surface(self, sid, width, height, frame_gpu)) {
        ++self->frame_index;
        self->prev_sid = sid;
        return GST_FLOW_OK;
    }

    // Extract grayscale (assume BGR)
    cv::UMat curr_gray;
    try {
        cv::cvtColor(frame_gpu, curr_gray, cv::COLOR_BGR2GRAY);
    } catch (const cv::Exception &e) {
        GST_WARNING_OBJECT(self, "cvtColor failed: %s", e.what());
        ++self->frame_index;
        self->prev_sid = sid;
        return GST_FLOW_OK;
    }

    // Downscale to small working resolution (keep aspect ratio). Target width ~320.
    int target_w = std::min(320, width);
    double scale = (double)target_w / (double)width;
    int small_w = target_w;
    int small_h = std::max(1, (int)std::lround(height * scale));
    cv::UMat curr_small;
    cv::resize(curr_gray, curr_small, cv::Size(small_w, small_h), 0, 0, cv::INTER_LINEAR);

    // If first frame, store and exit
    if (self->prev_small_gray.empty()) {
        curr_small.copyTo(self->prev_small_gray);
        curr_gray.copyTo(self->prev_luma);
        self->prev_sid = sid;
        ++self->frame_index;
        return GST_FLOW_OK;
    }

    // Motion mask generation (absdiff -> blur -> threshold -> morphology)
    const int PIXEL_DIFF_THR = 15; // absolute difference threshold
    cv::UMat diff_small; cv::absdiff(curr_small, self->prev_small_gray, diff_small);
    cv::UMat blurred_small; cv::GaussianBlur(diff_small, blurred_small, cv::Size(3,3), 0);
    cv::UMat thresh_small; cv::threshold(blurred_small, thresh_small, PIXEL_DIFF_THR, 255, cv::THRESH_BINARY);
    cv::UMat morph_small; {
        cv::UMat tmp; cv::Mat ksmall = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
        cv::morphologyEx(thresh_small, tmp, cv::MORPH_OPEN, ksmall);
        cv::dilate(tmp, morph_small, ksmall, cv::Point(-1,-1), 1);
    }

    // Block-based scan instead of contours
    std::vector<MotionRect> rois;
    const double MIN_REL_AREA = 0.0005; // fraction of full image (tunable)
    double full_area = (double)width * (double)height;
    double scale_x = (double)width / (double)small_w;
    double scale_y = (double)height / (double)small_h;

    int block_full = std::max(16, self->block_size); // clamp
    int block_small_w = std::max(4, (int)std::round(block_full / scale_x));
    int block_small_h = std::max(4, (int)std::round(block_full / scale_y));
    const double CHANGE_RATIO_THR =
        std::max(0.0, std::min(1.0, self->motion_threshold)); // fraction of changed pixels within block (property)
    cv::Mat morph_cpu = morph_small.getMat(cv::ACCESS_READ);

    for (int by = 0; by < small_h; by += block_small_h) {
        int h_small = std::min(block_small_h, small_h - by);
        if (h_small < 4)
            break;
        for (int bx = 0; bx < small_w; bx += block_small_w) {
            int w_small = std::min(block_small_w, small_w - bx);
            if (w_small < 4)
                break;
            cv::Rect r_small(bx, by, w_small, h_small);
            cv::Mat sub = morph_cpu(r_small);
            int changed = cv::countNonZero(sub);
            double ratio = (double)changed / (double)(r_small.width * r_small.height);
            if (ratio < CHANGE_RATIO_THR)
                continue; // insufficient motion in block

            // Scale to full-res
            int fx = (int)std::round(r_small.x * scale_x);
            int fy = (int)std::round(r_small.y * scale_y);
            int fw = (int)std::round(r_small.width * scale_x);
            int fh = (int)std::round(r_small.height * scale_y);

            double area_full = (double)fw * (double)fh;
            if (area_full / full_area < MIN_REL_AREA)
                continue; // too small

            // Pad a little
            const int PAD = 4;
            fx = std::max(0, fx - PAD);
            fy = std::max(0, fy - PAD);
            fw = std::min(width - fx, fw + 2 * PAD);
            fh = std::min(height - fy, fh + 2 * PAD);

            // Clamp
            if (fx + fw > width)
                fw = width - fx;
            if (fy + fh > height)
                fh = height - fy;

            rois.push_back(MotionRect{fx, fy, fw, fh});
        }
    }


    if (!rois.empty()) {
        // Merge overlapping ROIs via helper
        gst_gva_motion_detect_merge_rois(rois);
        gst_gva_motion_detect_attach_rois(self, buf, rois, width, height);
    }

    // Update previous frames
    // Update previous frames
    curr_small.copyTo(self->prev_small_gray);
    curr_gray.copyTo(self->prev_luma);
    self->prev_sid = sid;
    ++self->frame_index;
    return GST_FLOW_OK;
#endif // !_MSC_VER
}

static void gst_gva_motion_detect_finalize(GObject *obj) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(obj);
    g_mutex_clear(&self->meta_mutex);
    G_OBJECT_CLASS(gst_gva_motion_detect_parent_class)->finalize(obj);
}

static void gst_gva_motion_detect_class_init(GstGvaMotionDetectClass *klass) {
    GstElementClass *eclass = GST_ELEMENT_CLASS(klass);
    GstBaseTransformClass *bclass = GST_BASE_TRANSFORM_CLASS(klass);
    GObjectClass *oclass = G_OBJECT_CLASS(klass);

    GST_DEBUG_CATEGORY_INIT(gst_gva_motion_detect_debug, "gvamotiondetect", 0, "GVA motion detect filter");

    gst_element_class_set_static_metadata(
#ifndef _MSC_VER
        eclass, "VA GPU filter (VAMemory-only)", "Filter/Video",
        "Accepts/produces video/x-raw(memory:VAMemory) and reuses VA display via GstContext", "dlstreamer"
#else
        eclass, "Motion detect (software)", "Filter/Video",
        "Software motion detection (system memory frames) - VA skipped under MSVC", "dlstreamer"
#endif
    );

    gst_element_class_add_static_pad_template(eclass, &sink_templ);
    gst_element_class_add_static_pad_template(eclass, &src_templ);

    eclass->set_context = gst_gva_motion_detect_set_context;
    eclass->query = gst_gva_motion_detect_query;

    bclass->start = gst_gva_motion_detect_start;
    bclass->set_caps = gst_gva_motion_detect_set_caps;
    bclass->transform_caps = gst_gva_motion_detect_transform_caps;
    bclass->transform_ip = gst_gva_motion_detect_transform_ip;
    oclass->finalize = gst_gva_motion_detect_finalize;

    // Set property handlers before installing properties (required by GObject)
    oclass->set_property = gst_gva_motion_detect_set_property;
    oclass->get_property = gst_gva_motion_detect_get_property;

    g_object_class_install_property(
        oclass, PROP_BLOCK_SIZE,
        g_param_spec_int("block-size", "Block Size",
                         "Full-resolution block size (pixels) used for grid motion detection", 16, 512, 64,
                         (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(oclass, PROP_MOTION_THRESHOLD,
                                    g_param_spec_double("motion-threshold", "Motion Threshold",
                                                        "Per-block changed pixel ratio required to flag motion (0..1)",
                                                        0.0, 1.0, 0.05,
                                                        (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    /* OpenCL kernel property removed. */
}

static void gst_gva_motion_detect_init(GstGvaMotionDetect *self) {
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), FALSE);
    self->blur_kernel = 21;
    self->blur_sigma = 5.0;
    self->frame_index = 0;
    self->block_size = 64;         // default
    self->motion_threshold = 0.05; // default changed pixel ratio
#ifndef _MSC_VER
    self->va_dpy = nullptr;
    self->va_display = nullptr;
    self->prev_sid = VA_INVALID_SURFACE;
#else
    self->va_dpy = nullptr;
    self->va_display = nullptr;
    self->prev_sid = -1;
#endif
    // Debug environment parsing (simple, no logging here to avoid early flood)
    const gchar *env_dbg = g_getenv("GVA_MD_PRINT");
    self->debug_enabled = (env_dbg && env_dbg[0] != '\0' && g_strcmp0(env_dbg, "0") != 0);
    const gchar *env_int = g_getenv("GVA_MD_PRINT_INTERVAL");
    self->debug_interval = 30;
    if (env_int && env_int[0] != '\0') {
        gchar *endp = nullptr;
        unsigned long long v = g_ascii_strtoull(env_int, &endp, 10);
        if (endp && *endp == '\0' && v > 0 && v < 1000000UL)
            self->debug_interval = (guint)v;
    }
    self->last_debug_frame = (guint64)-1;
    self->tried_va_query = FALSE;
    g_mutex_init(&self->meta_mutex);
}

static gboolean plugin_init(GstPlugin *plugin) {
    return gst_element_register(plugin, "gvamotiondetect", GST_RANK_NONE, GST_TYPE_GVA_MOTION_DETECT);
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR, gvamotiondetect, "GVA motion detect filter", plugin_init, "1.0",
                  "MIT/X11", "dlstreamer", "https://example.com")