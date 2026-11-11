from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    device = DeclareLaunchArgument(
        "device",
        default_value="/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0",
    )
    config_path = DeclareLaunchArgument(
        "config_path",
        default_value="tags_36h11.yaml",
    )

    container = ComposableNodeContainer(
        name="apriltag_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="apriltag_ros",
                plugin="CameraPublisherNode",
                name="camera_publisher",
                namespace="down_camera",
                parameters=[{"video_device": LaunchConfiguration("device")}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
                namespace="down_camera",
                remappings=[
                    ("image", "/down_camera/image_raw"),
                    ("camera_info", "/down_camera/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                namespace="apriltag",
                remappings=[
                    ("image_rect", "/down_camera/image_rect"),
                    # ("image_rect", "/down_camera/image_raw"),
                    ("camera_info", "/down_camera/camera_info"),
                ],
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("apriltag_ros"),
                            "cfg",
                            LaunchConfiguration("config_path"),
                        ]
                    ),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription([device, config_path, container])
