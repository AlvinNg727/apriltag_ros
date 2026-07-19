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
    double default_size);
