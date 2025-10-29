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
        state_ = TrackState::Deleted;
    } else if (time_since_update_ > max_age_) {
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

    // Get input dimensions
    auto input_port = compiled_model_.input();
    auto input_shape = input_port.get_shape();
    input_height_ = input_shape[2];
    input_width_ = input_shape[3];

    GST_INFO("FeatureExtractor initialized: model=%s, device=%s, input_size=%dx%d", model_path.c_str(), device.c_str(),
             input_width_, input_height_);
}

std::vector<float> FeatureExtractor::extract(const cv::Mat &image, const cv::Rect &bbox) {
    // Extract region of interest
    cv::Rect safe_bbox = bbox & cv::Rect(0, 0, image.cols, image.rows);
    if (safe_bbox.area() == 0) {
        return std::vector<float>(128, 0.0f); // Return zero feature if invalid bbox
    }

    cv::Mat roi = image(safe_bbox);
    cv::Mat preprocessed = preprocess(roi);

    // Set input tensor
    auto input_tensor = infer_request_.get_input_tensor();
    float *input_data = input_tensor.data<float>();
    std::memcpy(input_data, preprocessed.data, preprocessed.total() * sizeof(float));

    // Run inference
    infer_request_.infer();

    // Get output
    auto output_tensor = infer_request_.get_output_tensor();
    return postprocess(output_tensor);
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
    cv::Mat resized, normalized;
    cv::resize(image, resized, cv::Size(input_width_, input_height_));
    resized.convertTo(normalized, CV_32F, 1.0f / 255.0f);

    // Convert HWC to CHW format
    cv::Mat channels[3];
    cv::split(normalized, channels);

    cv::Mat result(1, 3 * input_height_ * input_width_, CV_32F);
    float *result_data = result.ptr<float>();

    for (int c = 0; c < 3; ++c) {
        std::memcpy(result_data + c * input_height_ * input_width_, channels[c].data,
                    input_height_ * input_width_ * sizeof(float));
    }

    return result;
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
                                 float max_iou_distance, int max_age, int n_init, float max_cosine_distance,
                                 int nn_budget, dlstreamer::MemoryMapperPtr mapper)
    : feature_extractor_(std::make_unique<FeatureExtractor>(feature_model_path, device)), next_id_(1),
      max_iou_distance_(max_iou_distance), max_age_(max_age), n_init_(n_init),
      max_cosine_distance_(max_cosine_distance), nn_budget_(nn_budget), buffer_mapper_(std::move(mapper)) {

    GST_INFO("DeepSortTracker initialized: max_iou_distance=%.3f, max_age=%d, n_init=%d, max_cosine_distance=%.3f",
             max_iou_distance_, max_age_, n_init_, max_cosine_distance_);
}

void DeepSortTracker::track(dlstreamer::FramePtr buffer, GVA::VideoFrame &frame_meta) {
    if (!buffer) {
        throw std::invalid_argument("DeepSortTracker: buffer is nullptr");
    }

    // Map buffer to system memory for OpenCV access
    dlstreamer::FramePtr sys_buffer = buffer_mapper_->map(buffer, dlstreamer::AccessMode::Read);
    MappedMat mapped_mat(sys_buffer);
    cv::Mat image = mapped_mat.mat();

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

    // Update matched tracks
    for (const auto &match : matches) {
        tracks_[match.second]->update(detections[match.first]);
    }

    // Create new tracks for unmatched detections
    for (int det_idx : unmatched_dets) {
        auto new_track = std::make_unique<Track>(detections[det_idx].bbox, next_id_++, n_init_, max_age_,
                                                 detections[det_idx].feature);
        tracks_.push_back(std::move(new_track));
    }

    // Remove deleted tracks
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [](const std::unique_ptr<Track> &track) { return track->is_deleted(); }),
                  tracks_.end());

    // Update metadata with tracking results
    for (const auto &track : tracks_) {
        if (track->is_confirmed()) {
            cv::Rect_<float> bbox = track->to_bbox();

            // Convert to absolute coordinates
            cv::Rect abs_bbox(static_cast<int>(bbox.x), static_cast<int>(bbox.y), static_cast<int>(bbox.width),
                              static_cast<int>(bbox.height));

            // Add new region or update existing one
            auto new_region = frame_meta.add_region(abs_bbox.x, abs_bbox.y, abs_bbox.width, abs_bbox.height);
            new_region.set_object_id(track->track_id());
        }
    }
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
    auto features = feature_extractor_->extract_batch(image, bboxes);

    for (size_t i = 0; i < regions.size(); ++i) {
        const auto &region = regions[i];
        cv::Rect_<float> bbox(region.rect().x, region.rect().y, region.rect().w, region.rect().h);
        float confidence = region.confidence();

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
                continue;
            }

            cv::Rect_<float> track_bbox = tracks_[trk_idx]->to_bbox();
            float iou = calculate_iou(detections[det_idx].bbox, track_bbox);

            if (iou > max_iou_distance_) {
                cost_matrix[det_idx][trk_idx] = 1.0f; // No match
                continue;
            }

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
    float intersection_area = (bbox1 & bbox2).area();
    float union_area = bbox1.area() + bbox2.area() - intersection_area;

    return union_area > 0.0f ? intersection_area / union_area : 0.0f;
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