from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    device = DeclareLaunchArgument("device", default_value="0")
    size = DeclareLaunchArgument("size", default_value="8x6")
    square = DeclareLaunchArgument("square", default_value="0.025")

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
                parameters=[{"video_device": LaunchConfiguration("device")}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
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
            ("image", "/camera/image_raw"),
            ("camera", "/camera/camera_info"),
        ],
        output="screen",
    )

    return LaunchDescription([device, size, square, container, calibrator])
