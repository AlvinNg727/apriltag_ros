// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <array>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>


#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

const static std::unordered_map<std::string, rmw_qos_profile_t> qos_profiles{
    {"default", rmw_qos_profile_default},
    {"sensor_data", rmw_qos_profile_sensor_data},
    {"system_default", rmw_qos_profile_system_default},
};

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;
    std::string world_frame_;
    std::unordered_map<int, std::array<double, 3>> tag_positions;

    // bundle parameters
    std::string bundle_frame_;
    std::unordered_map<int, std::array<double, 3>> bundle_tag_positions_;
    std::unordered_map<int, std::array<double, 3>> bundle_tag_orientations_;
    std::array<double, 3> bundle_world_position_;
    std::array<double, 3> bundle_world_orientation_;
    bool has_bundle_world_position_;
    bool has_bundle_world_orientation_;

    std::function<void(apriltag_family_t*)> tf_destructor;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    // topics
    sub_cam{
#ifdef image_transport_NODE_INTERFACE
        image_transport::RequiredInterfaces{*this},
#else
        this,
#endif
        this->get_node_topics_interface()->resolve_topic_name("image_rect"),
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        declare_parameter("image_transport", "raw", descr({}, true)),
#ifdef image_transport_QoS
        rclcpp::QoS{rclcpp::QoSInitialization::from_rmw(
#endif
            qos_profiles.at(declare_parameter("qos_profile", "default", descr("qos profile to use. 'default', 'sensor_data' or 'system_default'", true)))
#ifdef image_transport_QoS
                )}
#endif
    },
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    pub_pose(create_publisher<geometry_msgs::msg::PoseStamped>("pose", rclcpp::QoS(1))),
    tf_broadcaster(
#ifdef tf2_ros_NODE_INTERFACE
        tf2_ros::TransformBroadcaster::RequiredInterfaces { *this }
#else
        this
#endif
    )
{
    // read-only parameters
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // get method for estimating tag pose
    const std::string& pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp",
                          descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster), "
                                "set to \"\" (empty) to disable pose estimation",
                                true));

    if(!pose_estimation_method.empty()) {
        if(pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    world_frame_ = declare_parameter("world_frame", "map", descr("world frame name for tag positions", true));

    for(size_t i = 0; i < ids.size(); i++) {
        const auto key = std::string("tag.positions.") + std::to_string(ids[i]);
        const auto pos = declare_parameter(key, std::vector<double>{}, descr("tag position in world frame", true));
        if(pos.size() >= 3) {
            tag_positions[ids[i]] = {pos[0], pos[1], pos[2]};
        }
    }

    bundle_frame_ = declare_parameter("tag_bundle.frame", "", descr("tag bundle frame name (empty = disabled)", true));
    has_bundle_world_position_ = false;
    has_bundle_world_orientation_ = false;
    if(!bundle_frame_.empty()) {
        const auto bundle_ids = declare_parameter("tag_bundle.ids", std::vector<int64_t>{}, descr("tag bundle ids", true));
        for(const auto& id : bundle_ids) {
            const auto bundle_key = std::string("tag_bundle.positions.") + std::to_string(id);
            const auto pos = declare_parameter(bundle_key, std::vector<double>{}, descr("tag position in bundle frame", true));
            if(pos.size() >= 3) {
                bundle_tag_positions_[id] = {pos[0], pos[1], pos[2]};
            }
            const auto orient_key = std::string("tag_bundle.orientations.") + std::to_string(id);
            const auto orient = declare_parameter(orient_key, std::vector<double>{}, descr("tag orientation in bundle frame [roll, pitch, yaw] in radians", true));
            if(orient.size() >= 3) {
                bundle_tag_orientations_[id] = {orient[0], orient[1], orient[2]};
            }
        }
        const auto bundle_world_pos = declare_parameter("tag_bundle.position", std::vector<double>{}, descr("bundle origin in world frame", true));
        if(bundle_world_pos.size() >= 3) {
            has_bundle_world_position_ = true;
            bundle_world_position_ = {bundle_world_pos[0], bundle_world_pos[1], bundle_world_pos[2]};
        }
        const auto bundle_world_orient = declare_parameter("tag_bundle.orientation", std::vector<double>{}, descr("bundle orientation in world frame [roll, pitch, yaw] in radians", true));
        if(bundle_world_orient.size() >= 3) {
            has_bundle_world_orientation_ = true;
            bundle_world_orientation_ = {bundle_world_orient[0], bundle_world_orient[1], bundle_world_orient[2]};
        }
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }
}

AprilTagNode::~AprilTagNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // camera intrinsics for rectified images
    const std::array<double, 4> intrinsics = {msg_ci->p[0], msg_ci->p[5], msg_ci->p[2], msg_ci->p[6]};

    // check for valid intrinsics
    const bool calibrated = msg_ci->width && msg_ci->height &&
                            intrinsics[0] && intrinsics[1] && intrinsics[2] && intrinsics[3];

    if(estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN_STREAM(get_logger(), "The camera is not calibrated! Set 'pose_estimation_method' to \"\" (empty) to disable pose estimation and this warning.");
    }

    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    std::vector<apriltag_detection_t*> bundle_dets;

    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        if(estimate_pose != nullptr && calibrated) {
            // If bundle is active and this tag belongs to the bundle, collect it for joint estimation
            if(!bundle_frame_.empty() && bundle_tag_positions_.count(det->id)) {
                bundle_dets.push_back(det);
                continue;
            }

            geometry_msgs::msg::TransformStamped tf;
            geometry_msgs::msg::PoseStamped pose;
            // tf.header = msg_img->header;
            tf.header.frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            // pose.header = msg_img->header;
            pose.header.frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            // set child frame name by generic tag name or configured tag name
            tf.child_frame_id = msg_img->header.frame_id;
            // tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            geometry_msgs::msg::Transform transform = estimate_pose(det, intrinsics, size);
            tf2::Transform tf2_transform;
            tf2::convert(transform, tf2_transform);
            tf2::Transform tf2_transform_inv = tf2_transform.inverse();
            tf2::convert(tf2_transform_inv, transform);
            tf.transform = transform;
            pose.pose.position.x = transform.translation.x;
            pose.pose.position.y = transform.translation.y;
            pose.pose.position.z = transform.translation.z;
            pose.pose.orientation = transform.rotation;
            tfs.push_back(tf);

            if(tag_positions.count(det->id)) {
                geometry_msgs::msg::TransformStamped world_tf;
                world_tf.header.frame_id = world_frame_;
                world_tf.header.stamp = msg_img->header.stamp;
                world_tf.child_frame_id = tf.header.frame_id;
                world_tf.transform.translation.x = tag_positions[det->id][0];
                world_tf.transform.translation.y = tag_positions[det->id][1];
                world_tf.transform.translation.z = tag_positions[det->id][2];
                // world_tf.transform.rotation.w = 1.0;
                world_tf.transform.rotation.x = 0.0;
                world_tf.transform.rotation.y = 0.0;
                world_tf.transform.rotation.z = -0.707107;
                world_tf.transform.rotation.w = 0.707107;
                tfs.push_back(world_tf);
            }

            pub_pose->publish(pose);
        }
    }

    // Bundle pose estimation: use all detected bundle tags jointly
    if(!bundle_frame_.empty() && !bundle_dets.empty()) {
        geometry_msgs::msg::Transform transform = bundle_pnp(
            bundle_dets, intrinsics, bundle_tag_positions_, tag_sizes, tag_edge_size);

        geometry_msgs::msg::TransformStamped bundle_tf;
        bundle_tf.header.frame_id = bundle_frame_;
        bundle_tf.header.stamp = msg_img->header.stamp;
        bundle_tf.child_frame_id = msg_img->header.frame_id;

        tf2::Transform tf2_transform;
        tf2::convert(transform, tf2_transform);
        tf2::Transform tf2_transform_inv = tf2_transform.inverse();
        tf2::convert(tf2_transform_inv, transform);
        bundle_tf.transform = transform;
        tfs.push_back(bundle_tf);

        // world frame -> bundle frame static transform
        if(has_bundle_world_position_ || has_bundle_world_orientation_) {
            geometry_msgs::msg::TransformStamped world_tf;
            world_tf.header.frame_id = world_frame_;
            world_tf.header.stamp = msg_img->header.stamp;
            world_tf.child_frame_id = bundle_frame_;
            if(has_bundle_world_position_) {
                world_tf.transform.translation.x = bundle_world_position_[0];
                world_tf.transform.translation.y = bundle_world_position_[1];
                world_tf.transform.translation.z = bundle_world_position_[2];
            }
            if(has_bundle_world_orientation_) {
                tf2::Quaternion q;
                q.setRPY(bundle_world_orientation_[0], bundle_world_orientation_[1], bundle_world_orientation_[2]);
                world_tf.transform.rotation.x = q.x();
                world_tf.transform.rotation.y = q.y();
                world_tf.transform.rotation.z = q.z();
                world_tf.transform.rotation.w = q.w();
            }
            else {
                world_tf.transform.rotation.x = 0.0f;
                world_tf.transform.rotation.y = 0.0f;
                world_tf.transform.rotation.z = -0.707107f;
                world_tf.transform.rotation.w = 0.707107f;
            }
            tfs.push_back(world_tf);
        }

        // bundle frame -> individual tag frames (static, from configured positions)
        for(auto* det : bundle_dets) {
            const auto& pos = bundle_tag_positions_.at(det->id);
            geometry_msgs::msg::TransformStamped tag_tf;
            tag_tf.header.frame_id = bundle_frame_;
            tag_tf.header.stamp = msg_img->header.stamp;
            tag_tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            tag_tf.transform.translation.x = pos[0];
            tag_tf.transform.translation.y = pos[1];
            tag_tf.transform.translation.z = pos[2];
            if(bundle_tag_orientations_.count(det->id)) {
                const auto& ori = bundle_tag_orientations_.at(det->id);
                tf2::Quaternion q;
                q.setRPY(ori[0], ori[1], ori[2]);
                tag_tf.transform.rotation.x = q.x();
                tag_tf.transform.rotation.y = q.y();
                tag_tf.transform.rotation.z = q.z();
                tag_tf.transform.rotation.w = q.w();
            }
            else {
                tag_tf.transform.rotation.w = 1.0;
            }
            tfs.push_back(tag_tf);
        }

        // publish bundle pose (camera in bundle frame)
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = bundle_frame_;
        pose.header.stamp = msg_img->header.stamp;
        pose.pose.position.x = transform.translation.x;
        pose.pose.position.y = transform.translation.y;
        pose.pose.position.z = transform.translation.z;
        pose.pose.orientation = transform.rotation;
        pub_pose->publish(pose);
    }

    pub_detections->publish(msg_detections);

    if(estimate_pose != nullptr)
        tf_broadcaster.sendTransform(tfs);

    apriltag_detections_destroy(detections);
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}
