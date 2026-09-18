from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
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
                    PathJoinSubstitution(
                        [
                            FindPackageShare("apriltag_ros"),
                            "cfg",
                            "apriltag2.yaml",
                        ]
                    ),
                    {"use_sim_time": True},
                ],
            ),
        ]
    )
