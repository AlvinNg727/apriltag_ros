#include "pose_estimation.hpp"
#include <Eigen/Geometry>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
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

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"homography", homography},
    {"pnp", pnp},
};
