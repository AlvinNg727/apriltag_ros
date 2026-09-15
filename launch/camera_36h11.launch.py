from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="0",
    )
    camera_info_url_arg = DeclareLaunchArgument(
        "camera_info_url",
        default_value="package://apriltag_ros/calibration/camera.yaml",
    )
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="tags_36h11.yaml",
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("apriltag_ros"), "cfg", LaunchConfiguration("config_path")]
    )

    container = ComposableNodeContainer(
        name="apriltag_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="usb_cam",
                plugin="usb_cam::UsbCamNode",
                name="camera_node",
                namespace="camera",
                parameters=[
                    {
                        "video_device": LaunchConfiguration("device"),
                        "camera_info_url": LaunchConfiguration("camera_info_url"),
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_node",
                namespace="camera",
                remappings=[
                    ("image", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag_node",
                namespace="apriltag",
                remappings=[
                    ("/apriltag/image_rect", "/camera/image_rect"),
                    ("/camera/camera_info", "/camera/camera_info"),
                ],
                parameters=[
                    config_path,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription(
        [
            device_arg,
            camera_info_url_arg,
            config_path_arg,
            container,
        ]
    )
