import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                )
            ]
        )
    )

    aruco_params = os.path.join(
        get_package_share_directory("franka_handeye_calibration_ros2"),
        "config",
        "aruco_parameters.yaml",
    )
    aruco_recognition_node = Node(
        package="ros2_aruco", executable="aruco_node", parameters=[aruco_params]
    )

    hand_eye_tf_publisher = Node(
        package="franka_handeye_calibration_ros2",
        executable="handeye_publisher",
        name="handeye_publisher",
        parameters=[{"calibration_name": "fr3_calibration"}],
    )

    follow_aruco_node = Node(
        package="franka_handeye_calibration_ros2",
        executable="follow_aruco_marker",
        name="follow_aruco_marker",
        output="screen",
    )

    return LaunchDescription(
        [
            realsense,
            hand_eye_tf_publisher,
            aruco_recognition_node,
            follow_aruco_node,
        ]
    )
