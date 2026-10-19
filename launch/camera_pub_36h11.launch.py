from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    device = DeclareLaunchArgument("device", default_value="/dev/video0")
    config_path = DeclareLaunchArgument(
        "config_path",
        default_value="tags_36h11.yaml",
    )

    # Raw V4L2 camera: publishes /down_camera/image_raw + /down_camera/camera_info
    camera = Node(
        package="apriltag_ros",
        executable="camera_pub",
        name="camera_publisher",
        namespace="down_camera",
        output="screen",
        parameters=[{"video_device": LaunchConfiguration("device")}],
    )

    # Undistort raw feed into /down_camera/image_rect using /down_camera/camera_info
    rectify = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify",
        namespace="down_camera",
        output="screen",
        remappings=[
            ("image", "/down_camera/image_raw"),
            ("camera_info", "/down_camera/camera_info"),
        ],
        parameters=[{
            "image_transport_plugins": ["raw"]
        }],
    )

    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        namespace="apriltag",
        output="screen",
        remappings=[
            # ("image_rect", "/down_camera/image_rect"),
            ("image_rect", "/down_camera/image_raw"),
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
    )

    # return LaunchDescription([device, config_path, camera, rectify, apriltag])
    return LaunchDescription([device, config_path, camera, apriltag])
