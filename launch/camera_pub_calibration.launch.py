from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def launch_nodes(context):
    camera_parameters = [LaunchConfiguration("camera_config")]
    device = LaunchConfiguration("device").perform(context)
    if device:
        camera_parameters.append({"video_device": ParameterValue(device, value_type=str)})

    camera = Node(
        package="apriltag_ros",
        executable="camera_pub",
        name="camera_publisher",
        namespace="down_camera",
        parameters=camera_parameters,
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

    return [camera, calibrator]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("apriltag_ros"), "cfg", "camera", "down_camera_params.yaml"]
                ),
                description="ROS parameters file for the down-facing camera publisher.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="",
                description="Override the video device in the camera parameters file.",
            ),
            DeclareLaunchArgument("size", default_value="8x6"),
            DeclareLaunchArgument("square", default_value="0.025"),
            OpaqueFunction(function=launch_nodes),
        ]
    )
