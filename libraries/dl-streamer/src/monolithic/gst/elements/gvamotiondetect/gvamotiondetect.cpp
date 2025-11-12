/*******************************************************************************
 * Copyright (C) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "gvamotiondetect.h"
#include <gst/base/gstbasetransform.h>
#include <gst/gst.h>
#include <gst/video/video.h>

// Platform‑specific includes: VA path only for non-MSVC builds
#ifndef _MSC_VER
#include <gst/va/gstvadisplay.h>
#include <gst/va/gstvautils.h>
#include <gmodule.h>
#include <opencv2/core/va_intel.hpp>
#include <va/va.h>
#include <va/va_fei.h>
#include <va/va_fei_h264.h>
#endif

#include <gst/video/video.h>
#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgproc.hpp>

#include "../../inference_elements/common/post_processor/meta_attacher.h" // ROIToFrameAttacher

namespace {
class DummyBlobToMetaConverter : public post_processing::BlobToMetaConverter {
  public:
    DummyBlobToMetaConverter() : BlobToMetaConverter(Initializer{}) {
    }
    post_processing::TensorsTable convert(const post_processing::OutputBlobs &) override {
        return {}; // never used; we already build TensorsTable manually
    }
};
} // namespace

// Removed G_BEGIN_DECLS / G_END_DECLS to avoid forcing C linkage on C++ helper functions

GST_DEBUG_CATEGORY_STATIC(gst_gva_motion_detect_debug);
#define GST_CAT_DEFAULT gst_gva_motion_detect_debug

struct MotionRect {
    gint x;
    gint y;
    gint w;
    gint h;
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
    bool overlay_ready = false;
    std::string last_text;
    VAConfigID stats_cfg = 0;
    VAContextID stats_ctx = 0;
    VASurfaceID prev_sid = VA_INVALID_SURFACE; // simple 1‑frame history
    gboolean stats_ready = FALSE;
#else
    // Windows (MSVC) build: no VAAPI. Provide stubs to satisfy logic.
    void *va_dpy = nullptr;
    void *va_display = nullptr;
    int prev_sid = -1;
#endif

    // Software motion detection state
    cv::UMat prev_luma;       // previous full-res grayscale (legacy path)
    cv::UMat prev_small_gray; // previous downscaled grayscale for coarse motion

    // Block-based motion detection parameters (coarse grid)
    int block_size;          // size in pixels at full resolution for grid blocks
    double motion_threshold; // ratio of changed pixels per block (0..1)
};

// Property identifiers
enum { PROP_0, PROP_BLOCK_SIZE, PROP_MOTION_THRESHOLD };

// GObject property handlers
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

static post_processing::TensorsTable build_motion_tensors(const std::vector<MotionRect> &rois) {
    post_processing::TensorsTable table(1); // one frame
    auto &frame_vec = table[0];
    frame_vec.reserve(rois.size());
    for (auto &r : rois) {
        GstStructure *s = gst_structure_new("motion", "x_abs", G_TYPE_UINT, (guint)r.x, "y_abs", G_TYPE_UINT,
                                            (guint)r.y, "w_abs", G_TYPE_UINT, (guint)r.w, "h_abs", G_TYPE_UINT,
                                            (guint)r.h, "label", G_TYPE_STRING, "motion", "confidence", G_TYPE_DOUBLE,
                                            1.0, "label_id", G_TYPE_INT, -1, "rotation", G_TYPE_DOUBLE, 0.0, NULL);
        // Use fully qualified DETECTION_TENSOR_ID
        std::vector<GstStructure *> tensor_slot(std::max((int)post_processing::DETECTION_TENSOR_ID + 1, 1), nullptr);
        tensor_slot[post_processing::DETECTION_TENSOR_ID] = s;
        frame_vec.push_back(std::move(tensor_slot));
    }
    return table;
}

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
static inline int gva_motion_detect_get_surface(GstGvaMotionDetect *, GstBuffer *) { return -1; }
static inline bool gva_motion_detect_convert_from_surface(GstGvaMotionDetect *, int, int, int, cv::UMat &) { return false; }
static inline bool gva_motion_detect_write_to_surface(GstGvaMotionDetect *, const cv::UMat &, int, int, int) { return false; }
#endif

static gboolean gst_gva_motion_detect_start(GstBaseTransform *trans) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(trans);
    gst_video_info_init(&self->vinfo);
    self->caps_is_va = FALSE;
    self->frame_index = 0;
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
#ifdef _MSC_VER
    // On Windows we skip VA stats initialization entirely
    return TRUE;
#endif

#ifndef _MSC_VER
    if (!self->va_dpy)
        return TRUE;

    if (!self->stats_ready) {
        VAProfile profiles[32];
        int nprof = 0;

        VAStatus query_status = vaQueryConfigProfiles(self->va_dpy, profiles, &nprof);
        if (query_status != VA_STATUS_SUCCESS) {
            GST_WARNING_OBJECT(self, "Failed to query profiles: %s", vaErrorStr(query_status));
            return TRUE;
        }

        GST_DEBUG_OBJECT(self, "Found %d profiles total", nprof);
        GST_DEBUG_OBJECT(self, "VAEntrypointStats constant value: %d", (int)VAEntrypointStats);
        GST_DEBUG_OBJECT(self, "VAProfileNone constant value: %d", (int)VAProfileNone);

        VAProfile selected_profile = VAProfileNone;
        gboolean found_stats_profile = FALSE;

        for (int i = 0; i < nprof && !found_stats_profile; ++i) {
            VAEntrypoint eps[32];
            int neps = 0;

            GST_DEBUG_OBJECT(self, "Checking profile index %d, profile value %d", i, (int)profiles[i]);

            // Skip VAProfileNone - it's not a real profile for creating contexts
            if (profiles[i] == VAProfileNone) {
                GST_DEBUG_OBJECT(self, "Skipping VAProfileNone");
                continue;
            }

            VAStatus ep_status = vaQueryConfigEntrypoints(self->va_dpy, profiles[i], eps, &neps);
            if (ep_status != VA_STATUS_SUCCESS) {
                GST_DEBUG_OBJECT(self, "Failed to query entrypoints for profile %d: %s", (int)profiles[i],
                                 vaErrorStr(ep_status));
                continue;
            }

            GST_DEBUG_OBJECT(self, "Profile %d has %d entrypoints", (int)profiles[i], neps);

            for (int j = 0; j < neps; ++j) {
                GST_DEBUG_OBJECT(self, "  Entrypoint %d: %d", j, (int)eps[j]);
                if (eps[j] == VAEntrypointStats) {
                    selected_profile = profiles[i];
                    found_stats_profile = TRUE;
                    GST_INFO_OBJECT(self, "Found VAEntrypointStats in profile %d", (int)selected_profile);
                    break;
                }
            }

            GST_DEBUG_OBJECT(self, "Profile %d (value=%d) stats_supported=%d", i, (int)profiles[i],
                             (found_stats_profile && selected_profile == profiles[i]));
        }

        // Alternative approach: If no specific profile supports stats, try VAProfileNone
        if (!found_stats_profile) {
            GST_INFO_OBJECT(self, "No specific profile supports stats, trying VAProfileNone");

            VAEntrypoint eps[32];
            int neps = 0;
            VAStatus ep_status = vaQueryConfigEntrypoints(self->va_dpy, VAProfileNone, eps, &neps);
            if (ep_status == VA_STATUS_SUCCESS) {
                for (int j = 0; j < neps; ++j) {
                    if (eps[j] == VAEntrypointStats) {
                        selected_profile = VAProfileNone;
                        found_stats_profile = TRUE;
                        GST_INFO_OBJECT(self, "VAProfileNone supports VAEntrypointStats");
                        break;
                    }
                }
            }
        }

        if (found_stats_profile) {
            GST_INFO_OBJECT(self, "Attempting to create config with profile %d", (int)selected_profile);

            VAConfigAttrib attribs[2];
            attribs[0].type = VAConfigAttribRTFormat;
            attribs[0].value = VA_RT_FORMAT_YUV420; // Set a default value
            attribs[1].type = VAConfigAttribStats;
            attribs[1].value = 0; // Let VA-API set this

            // For VAProfileNone, we might need different handling
            VAConfigAttrib *config_attribs = attribs;
            int num_attribs = 2;

            if (selected_profile == VAProfileNone) {
                // For VAProfileNone, try with minimal attributes
                config_attribs = &attribs[0]; // Only RT format
                num_attribs = 1;
                GST_INFO_OBJECT(self, "Using minimal config for VAProfileNone");
            }

            VAStatus st_cfg = vaCreateConfig(self->va_dpy, selected_profile, VAEntrypointStats, config_attribs,
                                             num_attribs, &self->stats_cfg);
            if (st_cfg == VA_STATUS_SUCCESS) {
                GST_INFO_OBJECT(self, "Successfully created config with profile %d", (int)selected_profile);

                VAStatus st_ctx =
                    vaCreateContext(self->va_dpy, self->stats_cfg, GST_VIDEO_INFO_WIDTH(&self->vinfo),
                                    GST_VIDEO_INFO_HEIGHT(&self->vinfo), VA_PROGRESSIVE, NULL, 0, &self->stats_ctx);

                if (st_ctx == VA_STATUS_SUCCESS) {
                    self->stats_ready = TRUE;
                    GST_INFO_OBJECT(self, "Stats context ready (profile=%d)", (int)selected_profile);
                } else {
                    GST_WARNING_OBJECT(self, "Stats context creation failed: %s", vaErrorStr(st_ctx));
                    self->stats_ready = FALSE;
                }
            } else {
                GST_WARNING_OBJECT(self, "Stats config failed (%s) for profile %d", vaErrorStr(st_cfg),
                                   (int)selected_profile);

                // Try with no attributes for VAProfileNone
                if (selected_profile == VAProfileNone) {
                    GST_INFO_OBJECT(self, "Trying VAProfileNone with no attributes");
                    VAStatus st_cfg2 =
                        vaCreateConfig(self->va_dpy, selected_profile, VAEntrypointStats, NULL, 0, &self->stats_cfg);
                    if (st_cfg2 == VA_STATUS_SUCCESS) {
                        VAStatus st_ctx = vaCreateContext(
                            self->va_dpy, self->stats_cfg, GST_VIDEO_INFO_WIDTH(&self->vinfo),
                            GST_VIDEO_INFO_HEIGHT(&self->vinfo), VA_PROGRESSIVE, NULL, 0, &self->stats_ctx);

                        self->stats_ready = (st_ctx == VA_STATUS_SUCCESS);
                        GST_INFO_OBJECT(self, "Stats context %s (VAProfileNone, no attrs)",
                                        self->stats_ready ? "ready" : "failed");
                    }
                }
            }
        } else {
            GST_INFO_OBJECT(self, "No profile exposes VAEntrypointStats");
        }
    }
    return TRUE;
#endif
}

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
    cv::Mat diff; cv::absdiff(gray, prev, diff);
    cv::GaussianBlur(diff, diff, cv::Size(3,3), 0);
    cv::threshold(diff, diff, 15, 255, cv::THRESH_BINARY);
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::morphologyEx(diff, diff, cv::MORPH_OPEN, k);
    cv::dilate(diff, diff, k);
    std::vector<std::vector<cv::Point>> contours; cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<MotionRect> rois;
    for (auto &c : contours) {
        if (c.size() < 3) continue;
        cv::Rect r = cv::boundingRect(c);
        if (r.area() < width*height*0.0005) continue;
        rois.push_back(MotionRect{r.x, r.y, r.width, r.height});
    }
    if (!rois.empty()) {
        post_processing::TensorsTable tensors = build_motion_tensors(rois);
        post_processing::FramesWrapper frames(buf, "gvamotiondetect", nullptr);
        post_processing::ROIToFrameAttacher attacher; DummyBlobToMetaConverter dummy; attacher.attach(tensors, frames, dummy);
    }
    gst_buffer_unmap(buf, &map);
    return GST_FLOW_OK;
#endif

#ifndef _MSC_VER
    if (!self->va_dpy) {
        GST_DEBUG_OBJECT(self, "No VADisplay; pass-through frame=%" G_GUINT64_FORMAT, self->frame_index);
        ++self->frame_index;
        return GST_FLOW_OK;
    }

    VASurfaceID sid = gva_motion_detect_get_surface(self, buf);
    if (sid == VA_INVALID_SURFACE) {
        GST_DEBUG_OBJECT(self, "No valid VA surface; pass-through frame=%" G_GUINT64_FORMAT, self->frame_index);
        ++self->frame_index;
        return GST_FLOW_OK;
    }

    // Lazy initialization (your existing code)
    if (!self->stats_ready && self->va_dpy && self->vinfo.width) {
        VAProfile profiles[32];
        int nprof = 0;
        vaQueryConfigProfiles(self->va_dpy, profiles, &nprof);
        VAProfile prof = VAProfileNone;
        for (int i = 0; i < nprof && prof == VAProfileNone; ++i) {
            VAEntrypoint eps[32];
            int neps = 0;
            vaQueryConfigEntrypoints(self->va_dpy, profiles[i], eps, &neps);
            for (int j = 0; j < neps; ++j)
                if (eps[j] == VAEntrypointStats) {
                    prof = profiles[i];
                    break;
                }
        }
        if (vaCreateConfig(self->va_dpy, prof, VAEntrypointStats, NULL, 0, &self->stats_cfg) == VA_STATUS_SUCCESS) {
            VAStatus st =
                vaCreateContext(self->va_dpy, self->stats_cfg, GST_VIDEO_INFO_WIDTH(&self->vinfo),
                                GST_VIDEO_INFO_HEIGHT(&self->vinfo), VA_PROGRESSIVE, NULL, 0, &self->stats_ctx);
            self->stats_ready = (st == VA_STATUS_SUCCESS);
            GST_INFO_OBJECT(self, "Lazy stats init: %s", self->stats_ready ? "ready" : "failed");
        }
    }

    // ----------------------------------------------------------------------
    // SOFTWARE MOTION DETECTION (OpenCV) replacing disabled HW stats path
    // ----------------------------------------------------------------------
    // Convert VA surface to UMat (convertFromVASurface currently delivers BGR/BGRA).
    cv::UMat current_bgr;
    if (!gva_motion_detect_convert_from_surface(self, sid, GST_VIDEO_INFO_WIDTH(&self->vinfo),
                                                GST_VIDEO_INFO_HEIGHT(&self->vinfo), current_bgr)) {
        GST_DEBUG_OBJECT(self, "convertFromVASurface failed; pass-through");
        self->prev_sid = sid;
        ++self->frame_index;
        return GST_FLOW_OK;
    }

    int width = GST_VIDEO_INFO_WIDTH(&self->vinfo);
    int height = GST_VIDEO_INFO_HEIGHT(&self->vinfo);
    if (current_bgr.empty()) {
        self->prev_sid = sid;
        ++self->frame_index;
        return GST_FLOW_OK;
    }

    // Ensure single-channel grayscale for motion diff (full resolution)
    cv::UMat curr_gray;
    int chs = current_bgr.channels();
    if (chs == 1) {
        curr_gray = current_bgr;
    } else if (chs == 3) {
        cv::cvtColor(current_bgr, curr_gray, cv::COLOR_BGR2GRAY);
    } else if (chs == 4) {
        cv::cvtColor(current_bgr, curr_gray, cv::COLOR_BGRA2GRAY);
    } else {
        GST_WARNING_OBJECT(self, "Unexpected channel count %d; skipping frame", chs);
        self->prev_sid = sid;
        ++self->frame_index;
        return GST_FLOW_OK;
    }

    // ---------------- Coarse motion path (downscale + block grid) -----------------
    const int TARGET_SMALL_W = 320; // coarse resolution target (could be property later)
    int small_w = std::min(TARGET_SMALL_W, width);
    int small_h = (int)std::round((double)height * small_w / (double)width);

    cv::UMat curr_small; // downscaled grayscale
    if (small_w != width) {
        cv::resize(curr_gray, curr_small, cv::Size(small_w, small_h), 0, 0, cv::INTER_AREA);
    } else {
        curr_small = curr_gray; // already small
    }

    if (self->prev_small_gray.empty()) {
        curr_small.copyTo(self->prev_small_gray);
        curr_gray.copyTo(self->prev_luma); // keep legacy for potential future use
        self->prev_sid = sid;
        ++self->frame_index;
        return GST_FLOW_OK;
    }

    cv::UMat diff_small;
    cv::absdiff(curr_small, self->prev_small_gray, diff_small);

    cv::UMat blurred_small;
    cv::GaussianBlur(diff_small, blurred_small, cv::Size(3, 3), 0);

    cv::UMat thresh_small;
    const int PIXEL_DIFF_THR = 15; // same threshold (on 0..255 grayscale)
    cv::threshold(blurred_small, thresh_small, PIXEL_DIFF_THR, 255, cv::THRESH_BINARY);

    // Light morphology to reduce noise
    cv::UMat morph_small;
    cv::Mat ksmall = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(thresh_small, morph_small, cv::MORPH_OPEN, ksmall);
    cv::dilate(morph_small, morph_small, ksmall, cv::Point(-1, -1), 1);

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

    // Merge overlapping ROIs (simple O(n^2) since counts expected low)
    if (!rois.empty()) {
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

    if (!rois.empty()) {
        post_processing::TensorsTable tensors = build_motion_tensors(rois);
        post_processing::FramesWrapper frames(buf, "gvamotiondetect", nullptr);
        post_processing::ROIToFrameAttacher attacher;
        DummyBlobToMetaConverter dummy;
        attacher.attach(tensors, frames, dummy);
        GST_INFO_OBJECT(self, "Coarse motion ROIs attached: %zu", rois.size());
    }

    // Update previous frames
    curr_small.copyTo(self->prev_small_gray);
    curr_gray.copyTo(self->prev_luma);

frame_done:

    self->prev_sid = sid;
    ++self->frame_index;
    return GST_FLOW_OK;
#endif
}

static void gst_gva_motion_detect_finalize(GObject *obj) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(obj);
#ifndef _MSC_VER
    if (self->stats_ctx)
        vaDestroyContext(self->va_dpy, self->stats_ctx);
    if (self->stats_cfg)
        vaDestroyConfig(self->va_dpy, self->stats_cfg);
#endif
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
}

static void gst_gva_motion_detect_init(GstGvaMotionDetect *self) {
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), FALSE);
    self->blur_kernel = 21;
    self->blur_sigma = 5.0;
    self->frame_index = 0;
    self->block_size = 64;         // default
    self->motion_threshold = 0.05; // default changed pixel ratio
}

static gboolean plugin_init(GstPlugin *plugin) {
    return gst_element_register(plugin, "gvamotiondetect", GST_RANK_NONE, GST_TYPE_GVA_MOTION_DETECT);
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR, gvamotiondetect, "GVA motion detect filter", plugin_init, "1.0",
                  "MIT/X11", "dlstreamer", "https://example.com")