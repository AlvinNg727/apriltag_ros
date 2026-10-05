#include "pose_estimation.hpp"
#include <Eigen/Geometry>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <tf2/convert.hpp>


geometry_msgs::msg::Transform
homography(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
{
    apriltag_detection_info_t info = {detection, tagsize, intr[0], intr[1], intr[2], intr[3]};

    apriltag_pose_t pose;
    estimate_pose_for_tag_homography(&info, &pose);

    // rotate frame such that z points in the opposite direction towards the camera
    for(int i = 0; i < 3; i++) {
        // swap x and y axes
        std::swap(MATD_EL(pose.R, 0, i), MATD_EL(pose.R, 1, i));
        // invert z axis
        MATD_EL(pose.R, 2, i) *= -1;
    }

    return tf2::toMsg<apriltag_pose_t, geometry_msgs::msg::Transform>(const_cast<const apriltag_pose_t&>(pose));
}

geometry_msgs::msg::Transform
pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
{
    const std::vector<cv::Point3d> objectPoints{
        {-tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, +tagsize / 2, 0},
        {-tagsize / 2, +tagsize / 2, 0},
    };

    const std::vector<cv::Point2d> imagePoints{
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);

    return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(std::make_pair(tvec, rvec));
}

// ponytail: best-of-2 only; node drops negative-height frames, add ratio threshold when flips persist
geometry_msgs::msg::Transform
ippe(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize, double* ambiguity_ratio)
{
    const double hs = tagsize / 2.0;
    // IPPE_SQUARE requires this corner order; image points permuted to preserve
    // the tag-frame convention used by pnp() (p0=(-,-), p1=(+,-), p2=(+,+), p3=(-,+)).
    const std::vector<cv::Point3d> objectPoints{
        {-hs, +hs, 0},
        {+hs, +hs, 0},
        {+hs, -hs, 0},
        {-hs, -hs, 0},
    };

    const std::vector<cv::Point2d> imagePoints{
        {detection->p[3][0], detection->p[3][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[0][0], detection->p[0][1]},
    };

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat errs;
    if(static_cast<int>(cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, {}, rvecs, tvecs, false,
                                            cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(), errs)) < 1 ||
       rvecs.empty() || tvecs.empty()) {
        if(ambiguity_ratio) *ambiguity_ratio = 0.0;
        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);
        return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(std::make_pair(tvec, rvec));
    }

    size_t best = 0;
    if(errs.rows >= 2 && errs.at<double>(1) < errs.at<double>(0)) best = 1;
    if(ambiguity_ratio) {
        const double e0 = errs.rows >= 1 ? errs.at<double>(0) : 0.0;
        const double e1 = errs.rows >= 2 ? errs.at<double>(1) : 0.0;
        *ambiguity_ratio = (e0 > 0.0 && std::isfinite(e0) && std::isfinite(e1)) ? std::max(e0, e1) / std::min(e0, e1) : 0.0;
    }

    return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(
        std::make_pair(cv::Mat_<double>(tvecs[best]), cv::Mat_<double>(rvecs[best])));
}

geometry_msgs::msg::Transform
bundle_pnp(
    const std::vector<apriltag_detection_t*>& detections,
    const std::array<double, 4>& intr,
    const std::unordered_map<int, std::array<double, 3>>& bundle_tag_positions,
    const std::unordered_map<int, double>& tag_sizes,
    double default_size)
{
    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> imagePoints;

    for(const auto& det : detections) {
        auto it = bundle_tag_positions.find(det->id);
        if(it == bundle_tag_positions.end()) continue;

        const auto& pos = it->second;
        const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : default_size;
        const double hs = size / 2.0;

        objectPoints.emplace_back(pos[0] - hs, pos[1] - hs, pos[2]);
        objectPoints.emplace_back(pos[0] + hs, pos[1] - hs, pos[2]);
        objectPoints.emplace_back(pos[0] + hs, pos[1] + hs, pos[2]);
        objectPoints.emplace_back(pos[0] - hs, pos[1] + hs, pos[2]);

        for(int i = 0; i < 4; i++) {
            imagePoints.emplace_back(det->p[i][0], det->p[i][1]);
        }
    }

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];
    cameraMatrix(1, 1) = intr[1];
    cameraMatrix(0, 2) = intr[2];
    cameraMatrix(1, 2) = intr[3];

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);

    return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(std::make_pair(tvec, rvec));
}

// ponytail: best-of-2 only; node drops negative-height frames, add ratio threshold when flips persist
geometry_msgs::msg::Transform
bundle_ippe(
    const std::vector<apriltag_detection_t*>& detections,
    const std::array<double, 4>& intr,
    const std::unordered_map<int, std::array<double, 3>>& bundle_tag_positions,
    const std::unordered_map<int, double>& tag_sizes,
    double default_size,
    double* ambiguity_ratio)
{
    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> imagePoints;

    for(const auto& det : detections) {
        auto it = bundle_tag_positions.find(det->id);
        if(it == bundle_tag_positions.end()) continue;

        const auto& pos = it->second;
        const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : default_size;
        const double hs = size / 2.0;

        objectPoints.emplace_back(pos[0] - hs, pos[1] - hs, pos[2]);
        objectPoints.emplace_back(pos[0] + hs, pos[1] - hs, pos[2]);
        objectPoints.emplace_back(pos[0] + hs, pos[1] + hs, pos[2]);
        objectPoints.emplace_back(pos[0] - hs, pos[1] + hs, pos[2]);

        for(int i = 0; i < 4; i++) {
            imagePoints.emplace_back(det->p[i][0], det->p[i][1]);
        }
    }

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];
    cameraMatrix(1, 1) = intr[1];
    cameraMatrix(0, 2) = intr[2];
    cameraMatrix(1, 2) = intr[3];

    if(objectPoints.size() < 4) {
        if(ambiguity_ratio) *ambiguity_ratio = 0.0;
        return bundle_pnp(detections, intr, bundle_tag_positions, tag_sizes, default_size);
    }

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat errs;
    if(static_cast<int>(cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, {}, rvecs, tvecs, false,
                                            cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), errs)) < 1 ||
       rvecs.empty() || tvecs.empty()) {
        if(ambiguity_ratio) *ambiguity_ratio = 0.0;
        return bundle_pnp(detections, intr, bundle_tag_positions, tag_sizes, default_size);
    }

    size_t best = 0;
    if(errs.rows >= 2 && errs.at<double>(1) < errs.at<double>(0)) best = 1;
    if(ambiguity_ratio) {
        const double e0 = errs.rows >= 1 ? errs.at<double>(0) : 0.0;
        const double e1 = errs.rows >= 2 ? errs.at<double>(1) : 0.0;
        *ambiguity_ratio = (e0 > 0.0 && std::isfinite(e0) && std::isfinite(e1)) ? std::max(e0, e1) / std::min(e0, e1) : 0.0;
    }

    return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(
        std::make_pair(cv::Mat_<double>(tvecs[best]), cv::Mat_<double>(rvecs[best])));
}

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"homography", homography},
    {"pnp", pnp},
    {"ippe", [](apriltag_detection_t* const d, const std::array<double, 4>& i, const double& s) { return ippe(d, i, s); }},
};
