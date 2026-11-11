from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    device = DeclareLaunchArgument(
        "device",
        default_value="/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0",
    )
    size = DeclareLaunchArgument("size", default_value="8x6")
    square = DeclareLaunchArgument("square", default_value="0.025")

    camera = Node(
        package="apriltag_ros",
        executable="camera_pub",
        name="camera_publisher",
        namespace="down_camera",
        parameters=[{"video_device": LaunchConfiguration("device")}],
        output="screen",
    )

    calibrator = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        name="cameracalibrator",
        arguments=[
            "--size",
            LaunchConfiguration("size"),
            "--square",
            LaunchConfiguration("square"),
        ],
        remappings=[
            ("image", "/down_camera/image_raw"),
            ("camera", "/down_camera"),
        ],
        output="screen",
    )

    return LaunchDescription([device, size, square, camera, calibrator])
