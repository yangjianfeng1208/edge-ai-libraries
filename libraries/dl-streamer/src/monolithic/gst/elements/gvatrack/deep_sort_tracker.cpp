/*******************************************************************************
 * Copyright (C) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "deep_sort_tracker.h"
#include "mapped_mat.h"
#include <algorithm>
#include <cmath>
#include <numeric>

// Temporary flag to disable feature extraction and use only IoU-based tracking
// Set to 1 to disable features (IoU-only), set to 0 to enable full Deep SORT features
#define DISABLE_FEATURE_EXTRACTION 0
#define DISABLE_DBG_LOGS 1

namespace DeepSortWrapper {

// Track implementation
Track::Track(const cv::Rect_<float> &bbox, int track_id, int n_init, int max_age, const std::vector<float> &feature)
    : track_id_(track_id), hits_(1), age_(1), time_since_update_(0), state_(TrackState::Tentative), n_init_(n_init),
      max_age_(max_age), nn_budget_(DEFAULT_NN_BUDGET) {

    initiate(bbox);
    add_feature(feature);
}

void Track::initiate(const cv::Rect_<float> &bbox) {
    // Initialize Kalman filter with 8-dimensional state space (x, y, aspect_ratio, height, vx, vy, va, vh)
    mean_ = cv::Mat::zeros(8, 1, CV_32F);
    mean_.at<float>(0) = bbox.x + bbox.width / 2.0f;  // center_x
    mean_.at<float>(1) = bbox.y + bbox.height / 2.0f; // center_y
    mean_.at<float>(2) = bbox.width / bbox.height;    // aspect_ratio
    mean_.at<float>(3) = bbox.height;                 // height

    // Initialize covariance matrix
    covariance_ = cv::Mat::eye(8, 8, CV_32F);
    float std_weight_position = 1.0f / 20.0f;
    float std_weight_velocity = 1.0f / 160.0f;

    covariance_.at<float>(0, 0) = 2.0f * std_weight_position * bbox.height;
    covariance_.at<float>(1, 1) = 2.0f * std_weight_position * bbox.height;
    covariance_.at<float>(2, 2) = 1e-2;
    covariance_.at<float>(3, 3) = 2.0f * std_weight_position * bbox.height;
    covariance_.at<float>(4, 4) = 10.0f * std_weight_velocity * bbox.height;
    covariance_.at<float>(5, 5) = 10.0f * std_weight_velocity * bbox.height;
    covariance_.at<float>(6, 6) = 1e-5;
    covariance_.at<float>(7, 7) = 10.0f * std_weight_velocity * bbox.height;
}

void Track::predict() {
    // State transition matrix
    cv::Mat F = cv::Mat::eye(8, 8, CV_32F);
    F.at<float>(0, 4) = 1.0f; // x += vx
    F.at<float>(1, 5) = 1.0f; // y += vy
    F.at<float>(2, 6) = 1.0f; // aspect_ratio += va
    F.at<float>(3, 7) = 1.0f; // height += vh

    // Predict state
    mean_ = F * mean_;

    // Process noise
    cv::Mat Q = cv::Mat::eye(8, 8, CV_32F);
    float std_weight_position = 1.0f / 20.0f;
    float std_weight_velocity = 1.0f / 160.0f;
    float height = mean_.at<float>(3);

    Q.at<float>(0, 0) = std::pow(std_weight_position * height, 2);
    Q.at<float>(1, 1) = std::pow(std_weight_position * height, 2);
    Q.at<float>(2, 2) = 1e-2;
    Q.at<float>(3, 3) = std::pow(std_weight_position * height, 2);
    Q.at<float>(4, 4) = std::pow(std_weight_velocity * height, 2);
    Q.at<float>(5, 5) = std::pow(std_weight_velocity * height, 2);
    Q.at<float>(6, 6) = 1e-5;
    Q.at<float>(7, 7) = std::pow(std_weight_velocity * height, 2);

    // Update covariance
    covariance_ = F * covariance_ * F.t() + Q;
}

void Track::update(const Detection &detection) {
    predict();

    // Measurement model (we observe x, y, aspect_ratio, height)
    cv::Mat H = cv::Mat::zeros(4, 8, CV_32F);
    H.at<float>(0, 0) = 1.0f; // observe x
    H.at<float>(1, 1) = 1.0f; // observe y
    H.at<float>(2, 2) = 1.0f; // observe aspect_ratio
    H.at<float>(3, 3) = 1.0f; // observe height

    // Measurement noise
    cv::Mat R = cv::Mat::eye(4, 4, CV_32F);
    float std_weight_position = 1.0f / 20.0f;
    float height = detection.bbox.height;

    R.at<float>(0, 0) = std::pow(std_weight_position * height, 2);
    R.at<float>(1, 1) = std::pow(std_weight_position * height, 2);
    R.at<float>(2, 2) = 1e-1;
    R.at<float>(3, 3) = std::pow(std_weight_position * height, 2);

    // Measurement vector
    cv::Mat z = cv::Mat::zeros(4, 1, CV_32F);
    z.at<float>(0) = detection.bbox.x + detection.bbox.width / 2.0f;
    z.at<float>(1) = detection.bbox.y + detection.bbox.height / 2.0f;
    z.at<float>(2) = detection.bbox.width / detection.bbox.height;
    z.at<float>(3) = detection.bbox.height;

    // Kalman update
    cv::Mat S = H * covariance_ * H.t() + R;
    cv::Mat K = covariance_ * H.t() * S.inv();
    cv::Mat y = z - H * mean_;

    mean_ = mean_ + K * y;
    covariance_ = covariance_ - K * H * covariance_;

    add_feature(detection.feature);

    hits_++;
    time_since_update_ = 0;

    if (state_ == TrackState::Tentative && hits_ >= n_init_) {
        state_ = TrackState::Confirmed;
    }
}

void Track::mark_missed() {
    if (state_ == TrackState::Tentative) {
        // For tentative tracks, delete only after max_age misses, not immediately
        if (time_since_update_ >= max_age_) {
            state_ = TrackState::Deleted;
        }
    } else if (time_since_update_ >= max_age_) {
        state_ = TrackState::Deleted;
    }
    time_since_update_++;
    age_++;
}

cv::Rect_<float> Track::to_bbox() const {
    float center_x = mean_.at<float>(0);
    float center_y = mean_.at<float>(1);
    float aspect_ratio = mean_.at<float>(2);
    float height = mean_.at<float>(3);
    float width = aspect_ratio * height;

    return cv::Rect_<float>(center_x - width / 2.0f, center_y - height / 2.0f, width, height);
}

void Track::add_feature(const std::vector<float> &feature) {
    features_.push_back(feature);
    if (features_.size() > static_cast<size_t>(nn_budget_)) {
        features_.pop_front();
    }
}

// FeatureExtractor implementation
FeatureExtractor::FeatureExtractor(const std::string &model_path, const std::string &device) {
    // Load OpenVINO model
    auto model = core_.read_model(model_path);
    compiled_model_ = core_.compile_model(model, device);
    infer_request_ = compiled_model_.create_infer_request();

    // Get input dimensions - handle dynamic shapes
    auto input_port = compiled_model_.input();

    if (model_path.find("resnet18") != std::string::npos) {
        auto input_shape = input_port.get_shape();
        input_height_ = input_shape[2];
        input_width_ = input_shape[3];

        // Check if we have enough dimensions and handle dynamic dimensions
        if (input_shape.size() < 4) {
            throw std::runtime_error("Model input must have at least 4 dimensions (NCHW or NHWC)");
        }

        // Static batch size - standard format
        input_height_ = input_shape[2];
        input_width_ = input_shape[3];

    } else {
        GST_ERROR("Not recognized model");
    }

    // Validate extracted dimensions
    if (input_height_ <= 0 || input_width_ <= 0) {
        throw std::runtime_error("Invalid input dimensions detected from model: " + std::to_string(input_width_) + "x" +
                                 std::to_string(input_height_));
    }
}

std::vector<float> FeatureExtractor::extract(const cv::Mat &image, const cv::Rect &bbox) {
    // Validate bbox is completely within image bounds
    cv::Rect image_bounds(0, 0, image.cols, image.rows);

    // Check if bbox is valid and completely within image
    if (bbox.area() == 0) {
        GST_WARNING("Invalid bbox (zero area), returning zero feature");
        return std::vector<float>(128, 0.0f);
    }

    if (bbox.x < 0 || bbox.y < 0 || bbox.x + bbox.width > image.cols || bbox.y + bbox.height > image.rows) {
        GST_WARNING("Bbox extends beyond image bounds (%dx%d at (%d,%d) in %dx%d image), returning zero feature",
                    bbox.width, bbox.height, bbox.x, bbox.y, image.cols, image.rows);
        return std::vector<float>(128, 0.0f);
    }

    try {
        cv::Mat roi = image(bbox);
        if (roi.empty()) {
            GST_WARNING("Empty ROI, returning zero feature");
            return std::vector<float>(128, 0.0f);
        }

        cv::Mat preprocessed = preprocess(roi);
        if (preprocessed.empty()) {
            GST_ERROR("Preprocessing failed, returning zero feature");
            return std::vector<float>(128, 0.0f);
        }

        // Set input tensor
        auto input_tensor = infer_request_.get_input_tensor();
        float *input_data = input_tensor.data<float>();

        // Bounds check
        size_t expected_size = preprocessed.total();
        size_t tensor_size = input_tensor.get_size();

        if (expected_size != tensor_size) {
            GST_ERROR("Size mismatch: preprocessed=%lu, tensor=%lu", expected_size, tensor_size);
            return std::vector<float>(128, 0.0f);
        }

        std::memcpy(input_data, preprocessed.data, expected_size * sizeof(float));

        // Run inference
        infer_request_.infer();

        // Get output
        auto output_tensor = infer_request_.get_output_tensor();

        std::vector<float> feature = postprocess(output_tensor);

        return feature;

    } catch (const std::exception &e) {
        GST_ERROR("Exception in feature extraction: %s", e.what());
        return std::vector<float>(128, 0.0f);
    }
}

std::vector<std::vector<float>> FeatureExtractor::extract_batch(const cv::Mat &image,
                                                                const std::vector<cv::Rect> &bboxes) {
    std::vector<std::vector<float>> features;
    features.reserve(bboxes.size());

    for (const auto &bbox : bboxes) {
        features.push_back(extract(image, bbox));
    }

    return features;
}

cv::Mat FeatureExtractor::preprocess(const cv::Mat &image) {

    // Basic safety check
    if (image.empty()) {
        GST_ERROR("Input image is empty");
        return cv::Mat();
    }

    // Validate input has expected channels for RGB input
    if (image.channels() != 3) {
        GST_ERROR("Deep SORT feature extractor expects 3-channel RGB input, got %d channels", image.channels());
        return cv::Mat();
    }

    try {
        cv::Mat resized, normalized;
        cv::resize(image, resized, cv::Size(input_width_, input_height_));

        if (resized.empty()) {
            GST_ERROR("Resize operation failed");
            return cv::Mat();
        }

        resized.convertTo(normalized, CV_32F, 1.0f / 255.0f);
        if (normalized.empty()) {
            GST_ERROR("Normalization failed");
            return cv::Mat();
        }

        // Manual pixel-by-pixel copying with maximum safety
        int channels = normalized.channels();
        cv::Mat result = cv::Mat::zeros(1, channels * input_height_ * input_width_, CV_32F);
        float *result_data = result.ptr<float>();

        if (!result_data) {
            GST_ERROR("Failed to get result data pointer");
            return cv::Mat();
        }

        // Convert HWC to CHW format using memcpy for efficiency
        if (normalized.isContinuous()) {
            // Direct memory copy for continuous data
            size_t pixel_count = input_height_ * input_width_;
            float *src_data = normalized.ptr<float>();

            for (int c = 0; c < channels; ++c) {
                for (size_t i = 0; i < pixel_count; ++i) {
                    result_data[c * pixel_count + i] = src_data[i * channels + c];
                }
            }
        } else {
            GST_ERROR("Image data is not continuous, cannot use optimized copy");
            return cv::Mat();
        }

        return result;
    } catch (const cv::Exception &e) {
        GST_ERROR("OpenCV exception in preprocess: %s", e.what());
        return cv::Mat();
    } catch (const std::exception &e) {
        GST_ERROR("Standard exception in preprocess: %s", e.what());
        return cv::Mat();
    }
}

std::vector<float> FeatureExtractor::postprocess(const ov::Tensor &output) {
    const float *output_data = output.data<const float>();
    size_t feature_size = output.get_size();

    std::vector<float> feature(output_data, output_data + feature_size);

    // L2 normalize the feature
    float norm = std::sqrt(std::inner_product(feature.begin(), feature.end(), feature.begin(), 0.0f));
    if (norm > 0.0f) {
        for (float &f : feature) {
            f /= norm;
        }
    }

    return feature;
}

// DeepSortTracker implementation
DeepSortTracker::DeepSortTracker(const std::string &feature_model_path, const std::string &device,
                                 float max_iou_distance, float max_age, int n_init, float max_cosine_distance,
                                 int nn_budget, dlstreamer::MemoryMapperPtr mapper)
#if DISABLE_FEATURE_EXTRACTION
    : feature_extractor_(nullptr), next_id_(1),
#else
    : feature_extractor_(std::make_unique<FeatureExtractor>(feature_model_path, device)), next_id_(1),
#endif
      max_iou_distance_(max_iou_distance), max_age_(max_age), n_init_(n_init),
      max_cosine_distance_(max_cosine_distance), nn_budget_(nn_budget), buffer_mapper_(std::move(mapper)) {

#if DISABLE_FEATURE_EXTRACTION
    // Suppress unused parameter warnings when feature extraction is disabled
    (void)feature_model_path;
    (void)device;
    g_print("DeepSortTracker initialized with FEATURE EXTRACTION DISABLED: max_iou_distance=%.3f, max_age=%.3f, "
            "n_init=%d\n",
            max_iou_distance_, max_age_, n_init_);
#else
#if !DISABLE_DBG_LOGS
    g_print("DeepSortTracker initialized: max_iou_distance=%.3f, max_age=%.3f, n_init=%d, max_cosine_distance=%.3f\n",
            max_iou_distance_, max_age_, n_init_, max_cosine_distance_);
#endif
#endif
}

void DeepSortTracker::track(dlstreamer::FramePtr buffer, GVA::VideoFrame &frame_meta) {
    if (!buffer) {
        throw std::invalid_argument("DeepSortTracker: buffer is nullptr");
    }

    // Map buffer to system memory for OpenCV access
    dlstreamer::FramePtr sys_buffer = buffer_mapper_->map(buffer, dlstreamer::AccessMode::Read);
    MappedMat mapped_mat(sys_buffer);
    cv::Mat raw_image = mapped_mat.mat();

    // Convert to RGB if needed for feature extraction (Deep SORT requires 3-channel input)
    cv::Mat image;
    dlstreamer::ImageFormat format = static_cast<dlstreamer::ImageFormat>(sys_buffer->format());

    // Convert color space based on input format
    switch (format) {
    case dlstreamer::ImageFormat::BGR:
        // Already in correct format, just clone to avoid modifying original
        image = raw_image.clone();
        break;
    case dlstreamer::ImageFormat::NV12:
        cv::cvtColor(raw_image, image, cv::COLOR_YUV2BGR_NV12);
        break;
    case dlstreamer::ImageFormat::I420:
        cv::cvtColor(raw_image, image, cv::COLOR_YUV2BGR_I420);
        break;
    case dlstreamer::ImageFormat::BGRX:
        cv::cvtColor(raw_image, image, cv::COLOR_BGRA2BGR);
        break;
    case dlstreamer::ImageFormat::RGBX:
        cv::cvtColor(raw_image, image, cv::COLOR_RGBA2BGR);
        break;
    default:
        GST_ERROR("Unsupported video format %d for Deep SORT feature extraction", static_cast<int>(format));
        // Fallback: try to use as-is if it has 3 channels
        if (raw_image.channels() == 3) {
            image = raw_image.clone();
        } else {
            GST_ERROR("Cannot convert %d-channel image to RGB", raw_image.channels());
            return; // Early return on unsupported format
        }
        break;
    }

    auto regions = frame_meta.regions();

    // Convert GVA regions to detections with feature extraction
    std::vector<Detection> detections = convert_detections(image, regions);

    // Predict existing tracks
    for (auto &track : tracks_) {
        track->mark_missed();
    }

    // Associate detections to tracks
    std::vector<std::pair<int, int>> matches;
    std::vector<int> unmatched_dets, unmatched_trks;
    associate_detections_to_tracks(detections, matches, unmatched_dets, unmatched_trks);

    //  Update matched tracks and assign object IDs to existing regions
    for (const auto &match : matches) {
        tracks_[match.second]->update(detections[match.first]);

#if !DISABLE_DBG_LOGS
        auto &detection = detections[match.first];
        auto &track = tracks_[match.second];
        cv::Rect_<float> track_bbox = track->to_bbox();
        g_print("{%s} Updating matched tracks: det-bbox[%d][%.1f,%.1f,%.1fx%.1f], trk-bbox[%d][%.1f,%.1f,%.1fx%.1f], "
                "track_id=%d, track_state=%s\n",
                __FUNCTION__, match.first, detection.bbox.x, detection.bbox.y, detection.bbox.width,
                detection.bbox.height, match.second, track_bbox.x, track_bbox.y, track_bbox.width, track_bbox.height,
                track->track_id(), track->state_str().c_str());
#endif
        // Assign tracking ID to the existing region only if track is confirmed
        // This follows Deep SORT convention where only confirmed tracks get persistent IDs
        if (match.first < static_cast<int>(regions.size()) && tracks_[match.second]->is_confirmed()) {
            regions[match.first].set_object_id(tracks_[match.second]->track_id());
        }
    }

    // Create new tracks for unmatched detections
    for (int det_idx : unmatched_dets) {
#if DISABLE_FEATURE_EXTRACTION
        // Create track with dummy feature when feature extraction is disabled
        std::vector<float> dummy_feature(128, 0.0f);
        auto new_track =
            std::make_unique<Track>(detections[det_idx].bbox, next_id_++, n_init_, max_age_, dummy_feature);
#else
        auto new_track = std::make_unique<Track>(detections[det_idx].bbox, next_id_++, n_init_, max_age_,
                                                 detections[det_idx].feature);
#endif
        tracks_.push_back(std::move(new_track));
#if !DISABLE_DBG_LOGS
        int new_track_id = new_track->track_id();
        std::string track_state = new_track->state_str();
        g_print("{%s} New track created: ID=%d, bbox[%.1f, %.1f, %.1f x %.1f], state=%s\n", __FUNCTION__, new_track_id,
                detections[det_idx].bbox.x, detections[det_idx].bbox.y, detections[det_idx].bbox.width,
                detections[det_idx].bbox.height, track_state.c_str());
#endif
        // Note: For Deep SORT, we typically don't assign IDs to new tracks immediately
        // They need to be confirmed first (survive for n_init frames)
        // Testing immediate ID assignment:
        // if (det_idx < static_cast<int>(regions.size())) {
        // regions[det_idx].set_object_id(new_track_id);
        //}
    }

    // Remove deleted tracks
    // tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),[](const std::unique_ptr<Track> &track) { return
    // track->is_deleted(); }), tracks_.end());
}

std::vector<Detection> DeepSortTracker::convert_detections(const cv::Mat &image,
                                                           const std::vector<GVA::RegionOfInterest> &regions) {
    std::vector<Detection> detections;
    std::vector<cv::Rect> bboxes;

    for (const auto &region : regions) {
        cv::Rect bbox(region.rect().x, region.rect().y, region.rect().w, region.rect().h);
        bboxes.push_back(bbox);
    }

    // Extract features for all detections
#if DISABLE_FEATURE_EXTRACTION
    // Feature extraction disabled - create dummy zero features
    // Suppress unused parameter warning
    (void)image;
#if !DISABLE_DBG_LOGS
    g_print("=== FEATURE EXTRACTION DISABLED - Using IoU-only tracking ===\n");
#endif
    std::vector<std::vector<float>> features(regions.size(), std::vector<float>(128, 0.0f));
#else
    // Normal feature extraction
    if (!feature_extractor_) {
        throw std::runtime_error("Feature extractor not initialized but features are enabled");
    }
    auto features = feature_extractor_->extract_batch(image, bboxes);
#endif

    for (size_t i = 0; i < regions.size(); ++i) {
        const auto &region = regions[i];
        cv::Rect_<float> bbox(region.rect().x, region.rect().y, region.rect().w, region.rect().h);
        float confidence = region.confidence();

#if !DISABLE_DBG_LOGS
#if DISABLE_FEATURE_EXTRACTION
        g_print("{%s} Detection %zu: bbox[%.1f, %.1f, %.1f x %.1f], confidence=%.3f, feature_extraction=DISABLED\n",
                __FUNCTION__, i, bbox.x, bbox.y, bbox.width, bbox.height, confidence);
#else
        g_print("{%s} Detection %zu: bbox[%d,%d,%d,%d], confidence=%.3f, feature_size=%zu\n", __FUNCTION__, i,
                (int)bbox.x, (int)bbox.y, (int)bbox.width, (int)bbox.height, confidence, features[i].size());
#endif
#endif
        detections.emplace_back(bbox, confidence, features[i], -1);
    }

    return detections;
}

void DeepSortTracker::associate_detections_to_tracks(const std::vector<Detection> &detections,
                                                     std::vector<std::pair<int, int>> &matches,
                                                     std::vector<int> &unmatched_dets,
                                                     std::vector<int> &unmatched_trks) {
    matches.clear();
    unmatched_dets.clear();
    unmatched_trks.clear();

    if (tracks_.empty()) {
        for (size_t i = 0; i < detections.size(); ++i) {
            unmatched_dets.push_back(i);
        }
        return;
    }

    // Build cost matrix combining IoU and cosine distance
    std::vector<std::vector<float>> cost_matrix(detections.size(), std::vector<float>(tracks_.size(), 1.0f));

    for (size_t det_idx = 0; det_idx < detections.size(); ++det_idx) {
        for (size_t trk_idx = 0; trk_idx < tracks_.size(); ++trk_idx) {
            if (!tracks_[trk_idx]->is_confirmed()) {
                // continue;
            }

            cv::Rect_<float> track_bbox = tracks_[trk_idx]->to_bbox();
            float iou = calculate_iou(detections[det_idx].bbox, track_bbox);

#if !DISABLE_DBG_LOGS
            g_print(
                "{%s} Detection vs Track : det_bbox[%zu][%.1f, %.1f, %.1f, %.1f] vs track_bbox[%zu][%.1f, %.1f, %.1f, "
                "%.1f] ; iou=%.3f\n",
                __FUNCTION__, det_idx, detections[det_idx].bbox.x, detections[det_idx].bbox.y,
                detections[det_idx].bbox.width, detections[det_idx].bbox.height, trk_idx, track_bbox.x, track_bbox.y,
                track_bbox.width, track_bbox.height, iou);
#endif
            // Reject matches with IoU below threshold (poor overlap)
            if (iou < max_iou_distance_) {
                cost_matrix[det_idx][trk_idx] = 1.0f; // No match
                continue;
            }

#if DISABLE_FEATURE_EXTRACTION
            // Feature extraction disabled - use only IoU for matching
            cost_matrix[det_idx][trk_idx] = 1.0f - iou; // Convert IoU to cost (higher IoU = lower cost)
#else
            // Calculate minimum cosine distance to track features
            float min_cosine_dist = 1.0f;
            const auto &track_features = tracks_[trk_idx]->features();

            for (const auto &track_feature : track_features) {
                float cosine_dist = calculate_cosine_distance(detections[det_idx].feature, track_feature);
                min_cosine_dist = std::min(min_cosine_dist, cosine_dist);
            }

            if (min_cosine_dist > max_cosine_distance_) {
                cost_matrix[det_idx][trk_idx] = 1.0f; // No match
            } else {
                // Combine IoU and cosine distance
                cost_matrix[det_idx][trk_idx] = 0.5f * (1.0f - iou) + 0.5f * min_cosine_dist;
            }
#endif
        }
    }

    // Hungarian assignment
    hungarian_assignment(cost_matrix, matches);

    // Find unmatched detections and tracks
    std::vector<bool> matched_dets(detections.size(), false);
    std::vector<bool> matched_trks(tracks_.size(), false);

    for (const auto &match : matches) {
        matched_dets[match.first] = true;
        matched_trks[match.second] = true;
    }

    for (size_t i = 0; i < detections.size(); ++i) {
        if (!matched_dets[i]) {
            unmatched_dets.push_back(i);
        }
    }

    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (!matched_trks[i]) {
            unmatched_trks.push_back(i);
        }
    }
}

float DeepSortTracker::calculate_cosine_distance(const std::vector<float> &feat1, const std::vector<float> &feat2) {
    if (feat1.size() != feat2.size()) {
        return 1.0f; // Maximum distance for mismatched features
    }

    float dot_product = std::inner_product(feat1.begin(), feat1.end(), feat2.begin(), 0.0f);
    return 1.0f - dot_product; // Convert cosine similarity to distance
}

float DeepSortTracker::calculate_iou(const cv::Rect_<float> &bbox1, const cv::Rect_<float> &bbox2) {
    cv::Rect_<float> intersection_rect = bbox1 & bbox2;
    float intersection_area = intersection_rect.area();
    float union_area = bbox1.area() + bbox2.area() - intersection_area;

    float iou = union_area > 0.0f ? intersection_area / union_area : 0.0f;

#if !DISABLE_DBG_LOGS
    // Debug: Print detailed IoU calculation
    g_print("{%s} IoU calculation: bbox1[%.1f,%.1f,%.1fx%.1f] area=%.1f, bbox2[%.1f,%.1f,%.1fx%.1f] area=%.1f, "
            "intersection[%.1f,%.1f,%.1fx%.1f] area=%.1f, union=%.1f, iou=%.3f\n",
            __FUNCTION__, bbox1.x, bbox1.y, bbox1.width, bbox1.height, bbox1.area(), bbox2.x, bbox2.y, bbox2.width,
            bbox2.height, bbox2.area(), intersection_rect.x, intersection_rect.y, intersection_rect.width,
            intersection_rect.height, intersection_area, union_area, iou);
#endif
    return iou;
}

void DeepSortTracker::hungarian_assignment(const std::vector<std::vector<float>> &cost_matrix,
                                           std::vector<std::pair<int, int>> &assignments) {
    // Simple greedy assignment for now (can be replaced with proper Hungarian algorithm)
    assignments.clear();

    std::vector<bool> det_assigned(cost_matrix.size(), false);
    std::vector<bool> trk_assigned(cost_matrix.empty() ? 0 : cost_matrix[0].size(), false);

    for (size_t det_idx = 0; det_idx < cost_matrix.size(); ++det_idx) {
        if (det_assigned[det_idx])
            continue;

        float min_cost = 1.0f;
        int best_trk = -1;

        for (size_t trk_idx = 0; trk_idx < cost_matrix[det_idx].size(); ++trk_idx) {
            if (trk_assigned[trk_idx])
                continue;

            if (cost_matrix[det_idx][trk_idx] < min_cost) {
                min_cost = cost_matrix[det_idx][trk_idx];
                best_trk = trk_idx;
            }
        }

        if (best_trk >= 0 && min_cost < 0.5f) { // Threshold for assignment
            assignments.emplace_back(det_idx, best_trk);
            det_assigned[det_idx] = true;
            trk_assigned[best_trk] = true;
        }
    }
}

} // namespace DeepSortWrapper