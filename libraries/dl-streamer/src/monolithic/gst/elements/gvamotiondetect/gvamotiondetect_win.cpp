/*******************************************************************************
 * Copyright (C) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "gvamotiondetect.h"
#include <algorithm>
#include <cmath>
#include <glib.h>
#include <gst/base/gstbasetransform.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

GST_DEBUG_CATEGORY_STATIC(gst_gva_motion_detect_debug_win);
#define GST_CAT_DEFAULT gst_gva_motion_detect_debug_win

struct MotionRectWin {
    int x, y, w, h;
};

enum {
    PROP_0,
    PROP_BLOCK_SIZE,
    PROP_MOTION_THRESHOLD,
    PROP_MIN_PERSISTENCE,
    PROP_MAX_MISS,
    PROP_IOU_THRESHOLD,
    PROP_SMOOTH_ALPHA
};

struct _GstGvaMotionDetect {
    GstBaseTransform parent;
    GstVideoInfo vinfo;
    gboolean caps_is_va; // always FALSE on Windows
    int block_size;
    double motion_threshold;
    int min_persistence;
    int max_miss;
    double iou_threshold;
    double smooth_alpha;
    cv::UMat prev_small_gray;
    cv::UMat prev_luma;
    cv::Mat block_state; // CV_8U agreement counters
    struct Track {
        int x, y, w, h;
        double sx, sy, sw, sh;
        int age;
        int miss;
    };
    std::vector<Track> tracks;
    uint64_t frame_index;
};

struct _GstGvaMotionDetectClass {
    GstBaseTransformClass parent_class;
};

G_DEFINE_TYPE(GstGvaMotionDetect, gst_gva_motion_detect, GST_TYPE_BASE_TRANSFORM)

static void gst_gva_motion_detect_set_property(GObject *obj, guint id, const GValue *val, GParamSpec *pspec) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(obj);
    switch (id) {
    case PROP_BLOCK_SIZE:
        self->block_size = g_value_get_int(val);
        break;
    case PROP_MOTION_THRESHOLD:
        self->motion_threshold = g_value_get_double(val);
        break;
    case PROP_MIN_PERSISTENCE:
        self->min_persistence = std::max(1, g_value_get_int(val));
        break;
    case PROP_MAX_MISS:
        self->max_miss = std::max(0, g_value_get_int(val));
        break;
    case PROP_IOU_THRESHOLD:
        self->iou_threshold = std::clamp(g_value_get_double(val), 0.0, 1.0);
        break;
    case PROP_SMOOTH_ALPHA: {
        double a = g_value_get_double(val);
        self->smooth_alpha = a < 0 ? 0 : (a > 1 ? 1 : a);
        break;
    }
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(obj, id, pspec);
    }
}
static void gst_gva_motion_detect_get_property(GObject *obj, guint id, GValue *val, GParamSpec *pspec) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(obj);
    switch (id) {
    case PROP_BLOCK_SIZE:
        g_value_set_int(val, self->block_size);
        break;
    case PROP_MOTION_THRESHOLD:
        g_value_set_double(val, self->motion_threshold);
        break;
    case PROP_MIN_PERSISTENCE:
        g_value_set_int(val, self->min_persistence);
        break;
    case PROP_MAX_MISS:
        g_value_set_int(val, self->max_miss);
        break;
    case PROP_IOU_THRESHOLD:
        g_value_set_double(val, self->iou_threshold);
        break;
    case PROP_SMOOTH_ALPHA:
        g_value_set_double(val, self->smooth_alpha);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(obj, id, pspec);
    }
}

static inline double md_iou(const MotionRectWin &a, const MotionRectWin &b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.w, b.x + b.w);
    int y2 = std::min(a.y + a.h, b.y + b.h);
    int iw = std::max(0, x2 - x1);
    int ih = std::max(0, y2 - y1);
    int inter = iw * ih;
    if (!inter)
        return 0.0;
    return (double)inter / (double)(a.w * a.h + b.w * b.h - inter);
}

static gboolean gst_gva_motion_detect_start(GstBaseTransform *t) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(t);
    self->caps_is_va = FALSE;
    self->frame_index = 0;
    return TRUE;
}

static gboolean gst_gva_motion_detect_set_caps(GstBaseTransform *t, GstCaps *in, GstCaps *out) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(t);
    return gst_video_info_from_caps(&self->vinfo, in);
}

static GstFlowReturn gst_gva_motion_detect_transform_ip(GstBaseTransform *t, GstBuffer *buf) {
    GstGvaMotionDetect *self = GST_GVA_MOTION_DETECT(t);
    ++self->frame_index;
    int width = GST_VIDEO_INFO_WIDTH(&self->vinfo);
    int height = GST_VIDEO_INFO_HEIGHT(&self->vinfo);
    if (!width || !height)
        return GST_FLOW_OK;
    GstVideoFrame vframe;
    cv::UMat curr_luma;
    gboolean mapped = gst_video_frame_map(&vframe, &self->vinfo, buf, GST_MAP_READ);
    if (mapped) {
        guint8 *y = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(&vframe, 0);
        int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vframe, 0);
        cv::Mat y_mat(height, width, CV_8UC1, y, stride);
        y_mat.copyTo(curr_luma);
        gst_video_frame_unmap(&vframe);
    } else {
        return GST_FLOW_OK;
    }
    int target_w = std::min(320, width);
    double scale = (double)target_w / (double)width;
    int small_w = target_w;
    int small_h = std::max(1, (int)std::lround(height * scale));
    cv::UMat curr_small;
    cv::resize(curr_luma, curr_small, cv::Size(small_w, small_h));
    if (self->prev_small_gray.empty()) {
        curr_small.copyTo(self->prev_small_gray);
        curr_luma.copyTo(self->prev_luma);
        return GST_FLOW_OK;
    }
    const int PIX_THR = 15;
    cv::UMat diff;
    cv::absdiff(curr_small, self->prev_small_gray, diff);
    cv::UMat blur;
    cv::GaussianBlur(diff, blur, cv::Size(3, 3), 0);
    cv::UMat thr;
    cv::threshold(blur, thr, PIX_THR, 255, cv::THRESH_BINARY);
    cv::UMat morph;
    {
        cv::UMat tmp;
        cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(thr, tmp, cv::MORPH_OPEN, k);
        cv::dilate(tmp, morph, k);
    }
    std::vector<MotionRectWin> raw;
    double scale_x = (double)width / (double)small_w;
    double scale_y = (double)height / (double)small_h;
    double full_area = (double)width * height;
    const double MIN_REL = 0.0005;
    int block_full = std::max(16, self->block_size);
    int bs_w = std::max(4, (int)std::round(block_full / scale_x));
    int bs_h = std::max(4, (int)std::round(block_full / scale_y));
    double CHANGE_THR = std::clamp(self->motion_threshold, 0.0, 1.0);
    cv::Mat m_cpu = morph.getMat(cv::ACCESS_READ);
    int rows = (small_h + bs_h - 1) / bs_h;
    int cols = (small_w + bs_w - 1) / bs_w;
    if (self->block_state.empty() || self->block_state.rows != rows || self->block_state.cols != cols)
        self->block_state = cv::Mat(rows, cols, CV_8U, cv::Scalar(0));
    for (int by = 0, gy = 0; by < small_h; by += bs_h, ++gy) {
        int h_small = std::min(bs_h, small_h - by);
        if (h_small < 4)
            break;
        for (int bx = 0, gx = 0; bx < small_w; bx += bs_w, ++gx) {
            int w_small = std::min(bs_w, small_w - bx);
            if (w_small < 4)
                break;
            cv::Rect r(bx, by, w_small, h_small);
            cv::Mat sub = m_cpu(r);
            int changed = cv::countNonZero(sub);
            double ratio = (double)changed / (double)(r.width * r.height);
            unsigned char &state = self->block_state.at<unsigned char>(gy, gx);
            if (ratio >= CHANGE_THR) {
                if (state < 2)
                    state++;
            } else {
                if (state > 0)
                    state--;
            }
            if (state < 2)
                continue;
            int fx = (int)std::round(r.x * scale_x);
            int fy = (int)std::round(r.y * scale_y);
            int fw = (int)std::round(r.width * scale_x);
            int fh = (int)std::round(r.height * scale_y);
            double area_full = (double)fw * fh;
            if (area_full / full_area < MIN_REL)
                continue;
            const int PAD = 4;
            fx = std::max(0, fx - PAD);
            fy = std::max(0, fy - PAD);
            fw = std::min(width - fx, fw + 2 * PAD);
            fh = std::min(height - fy, fh + 2 * PAD);
            if (fx + fw > width)
                fw = width - fx;
            if (fy + fh > height)
                fh = height - fy;
            raw.push_back({fx, fy, fw, fh});
        }
    }
    // Merge overlaps
    bool merged = true;
    while (merged) {
        merged = false;
        std::vector<MotionRectWin> out;
        std::vector<char> used(raw.size(), 0);
        for (size_t i = 0; i < raw.size(); ++i) {
            if (used[i])
                continue;
            MotionRectWin a = raw[i];
            for (size_t j = i + 1; j < raw.size(); ++j) {
                if (used[j])
                    continue;
                MotionRectWin b = raw[j];
                bool overlap = !(b.x + b.w < a.x || a.x + a.w < b.x || b.y + b.h < a.y || a.y + a.h < b.y);
                if (overlap) {
                    int nx = std::min(a.x, b.x);
                    int ny = std::min(a.y, b.y);
                    int nw = std::max(a.x + a.w, b.x + b.w) - nx;
                    int nh = std::max(a.y + a.h, b.y + b.h) - ny;
                    a = {nx, ny, nw, nh};
                    used[j] = 1;
                    merged = true;
                }
            }
            out.push_back(a);
        }
        raw.swap(out);
    }
    // Tracking
    std::vector<char> matched(raw.size(), 0);
    for (auto &t : self->tracks)
        t.miss++;
    for (size_t i = 0; i < raw.size(); ++i) {
        auto &r = raw[i];
        double best = 0;
        int bi = -1;
        for (size_t j = 0; j < self->tracks.size(); ++j) {
            MotionRectWin tr{self->tracks[j].x, self->tracks[j].y, self->tracks[j].w, self->tracks[j].h};
            double iou = md_iou(r, tr);
            if (iou > best) {
                best = iou;
                bi = (int)j;
            }
        }
        if (bi >= 0 && best >= self->iou_threshold) {
            auto &t = self->tracks[bi];
            t.x = r.x;
            t.y = r.y;
            t.w = r.w;
            t.h = r.h;
            double a = self->smooth_alpha;
            t.sx = a * r.x + (1 - a) * t.sx;
            t.sy = a * r.y + (1 - a) * t.sy;
            t.sw = a * r.w + (1 - a) * t.sw;
            t.sh = a * r.h + (1 - a) * t.sh;
            t.age++;
            t.miss = 0;
            matched[i] = 1;
        }
    }
    for (size_t i = 0; i < raw.size(); ++i)
        if (!matched[i]) {
            auto &r = raw[i];
            self->tracks.push_back({r.x, r.y, r.w, r.h, (double)r.x, (double)r.y, (double)r.w, (double)r.h, 1, 0});
        }
    // (Windows build: metadata attachment omitted intentionally)
    curr_small.copyTo(self->prev_small_gray);
    curr_luma.copyTo(self->prev_luma);
    return GST_FLOW_OK;
}

static void gst_gva_motion_detect_finalize(GObject *obj) {
    G_OBJECT_CLASS(gst_gva_motion_detect_parent_class)->finalize(obj);
}

static void gst_gva_motion_detect_class_init(GstGvaMotionDetectClass *klass) {
    GstElementClass *eclass = GST_ELEMENT_CLASS(klass);
    GstBaseTransformClass *bclass = GST_BASE_TRANSFORM_CLASS(klass);
    GObjectClass *oclass = G_OBJECT_CLASS(klass);
    GST_DEBUG_CATEGORY_INIT(gst_gva_motion_detect_debug_win, "gvamotiondetect", 0, "Motion detect (Windows)");
    gst_element_class_set_static_metadata(eclass, "Motion detect (software)", "Filter/Video",
                                          "Windows software motion detection", "dlstreamer");
    static GstStaticPadTemplate sink_templ =
        GST_STATIC_PAD_TEMPLATE("sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS("video/x-raw, format=NV12"));
    static GstStaticPadTemplate src_templ =
        GST_STATIC_PAD_TEMPLATE("src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS("video/x-raw, format=NV12"));
    gst_element_class_add_static_pad_template(eclass, &sink_templ);
    gst_element_class_add_static_pad_template(eclass, &src_templ);
    bclass->start = gst_gva_motion_detect_start;
    bclass->set_caps = gst_gva_motion_detect_set_caps;
    bclass->transform_ip = gst_gva_motion_detect_transform_ip;
    oclass->finalize = gst_gva_motion_detect_finalize;
    oclass->set_property = gst_gva_motion_detect_set_property;
    oclass->get_property = gst_gva_motion_detect_get_property;
    g_object_class_install_property(
        oclass, PROP_BLOCK_SIZE,
        g_param_spec_int("block-size", "Block Size", "Full-res block size", 16, 512, 64, G_PARAM_READWRITE));
    g_object_class_install_property(oclass, PROP_MOTION_THRESHOLD,
                                    g_param_spec_double("motion-threshold", "Motion Threshold",
                                                        "Per-block changed pixel ratio", 0.0, 1.0, 0.05,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(oclass, PROP_MIN_PERSISTENCE,
                                    g_param_spec_int("min-persistence", "Min Persistence", "Frames track must persist",
                                                     1, 30, 2, G_PARAM_READWRITE));
    g_object_class_install_property(
        oclass, PROP_MAX_MISS,
        g_param_spec_int("max-miss", "Max Miss", "Grace frames before drop", 0, 30, 1, G_PARAM_READWRITE));
    g_object_class_install_property(oclass, PROP_IOU_THRESHOLD,
                                    g_param_spec_double("iou-threshold", "IoU Threshold", "Tracking IoU threshold", 0.0,
                                                        1.0, 0.3, G_PARAM_READWRITE));
    g_object_class_install_property(
        oclass, PROP_SMOOTH_ALPHA,
        g_param_spec_double("smooth-alpha", "Smooth Alpha", "EMA smoothing factor", 0.0, 1.0, 0.5, G_PARAM_READWRITE));
}

static void gst_gva_motion_detect_init(GstGvaMotionDetect *self) {
    self->block_size = 64;
    self->motion_threshold = 0.05;
    self->min_persistence = 2;
    self->max_miss = 1;
    self->iou_threshold = 0.3;
    self->smooth_alpha = 0.5;
    self->frame_index = 0;
}

static gboolean plugin_init(GstPlugin *plugin) {
    return gst_element_register(plugin, "gvamotiondetect", GST_RANK_NONE, GST_TYPE_GVA_MOTION_DETECT);
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR, gvamotiondetect, PRODUCT_FULL_NAME " gvamotiondetect element",
                  plugin_init, PLUGIN_VERSION, PLUGIN_LICENSE, PACKAGE_NAME, GST_PACKAGE_ORIGIN)
