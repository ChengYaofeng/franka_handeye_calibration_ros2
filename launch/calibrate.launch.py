import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

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

    calibration_args = {
        "name": "fr3_calibration",
        "calibration_type": "eye_on_base",
        "robot_base_frame": "fr3_link0",
        "robot_effector_frame": "fr3_hand",
        "tracking_base_frame": "camera_color_frame",
        "tracking_marker_frame": "calibration_aruco",
    }

    calibration_aruco_publisher = Node(
        package="franka_handeye_calibration_ros2",
        executable="calibration_aruco_publisher",
        name="calibration_aruco_publisher",
        output="screen",
        parameters=[
            {
                "tracking_base_frame": calibration_args["tracking_base_frame"],
                "tracking_marker_frame": calibration_args["tracking_marker_frame"],
                "marker_id": 250, #change this marker_id to your own
            }
        ],
    )

    easy_handeye2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("easy_handeye2"),
                    "launch",
                    "calibrate.launch.py",
                )
            ]
        ),
        launch_arguments=calibration_args.items(),
    )

    # static transform publisher for camera_link to world
    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "camera_link"],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(realsense)
    ld.add_action(static_tf_publisher)
    ld.add_action(aruco_recognition_node)
    ld.add_action(calibration_aruco_publisher)
    ld.add_action(easy_handeye2)
    return ld