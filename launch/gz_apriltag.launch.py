import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _setup(context):
    world = context.launch_configurations["world"]
    return [
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            namespace="apriltag",
            output="screen",
            remappings=[
                ("image_rect", "/mono_cam_down/image"),
                ("camera_info", "/mono_cam_down/camera_info"),
            ],
            parameters=[
                os.path.join(
                    get_package_share_directory("apriltag_ros"),
                    "cfg",
                    f"{world}.yaml",
                ),
                {"use_sim_time": True},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="apriltag1",
                description="Gazebo world / apriltag bundle to detect (apriltag1, apriltag2, apriltag3)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
