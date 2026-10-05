#pragma once

#include <apriltag/apriltag.h>
#include <array>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>
#include <vector>


typedef std::function<geometry_msgs::msg::Transform(apriltag_detection_t* const, const std::array<double, 4>&, const double&)> pose_estimation_f;

extern const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods;

geometry_msgs::msg::Transform bundle_pnp(
    const std::vector<apriltag_detection_t*>& detections,
    const std::array<double, 4>& intrinsics,
    const std::unordered_map<int, std::array<double, 3>>& bundle_tag_positions,
    const std::unordered_map<int, double>& tag_sizes,
    double default_size,
    const std::unordered_map<int, std::array<double, 3>>& bundle_tag_orientations = {});

// IPPE variants (best-of-2 by reprojection error, always publishes).
// Single tag uses IPPE_SQUARE (exactly 4 points); bundle uses IPPE (coplanar N points).
// ambiguity_ratio (err2/err1, 0 if unavailable) is optional; near 1.0 means flipped pose is likely.
geometry_msgs::msg::Transform ippe(
    apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize, double* ambiguity_ratio = nullptr);

geometry_msgs::msg::Transform bundle_ippe(
    const std::vector<apriltag_detection_t*>& detections,
    const std::array<double, 4>& intrinsics,
    const std::unordered_map<int, std::array<double, 3>>& bundle_tag_positions,
    const std::unordered_map<int, double>& tag_sizes,
    double default_size,
    double* ambiguity_ratio = nullptr,
    const std::unordered_map<int, std::array<double, 3>>& bundle_tag_orientations = {});
