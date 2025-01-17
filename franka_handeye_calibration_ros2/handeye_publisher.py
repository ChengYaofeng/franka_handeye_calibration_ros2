#!/usr/bin/env python3
import numpy as np
import rclpy
import tf2_ros
from easy_handeye2.handeye_calibration import load_calibration
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import ParameterType, ParameterDescriptor
from transforms3d.quaternions import quat2mat, mat2quat


def transform_to_matrix(transform):
    """Converts a Transform message to a 4x4 transformation matrix using transforms3d."""
    # Extract translation
    translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    # Extract quaternion (note that transforms3d uses [w, x, y, z] order)
    rotation = np.array([transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z])
    # Convert quaternion to rotation matrix
    rotation_matrix = quat2mat(rotation)
    # Create the 4x4 transformation matrix
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = translation
    return matrix


def matrix_to_transform(matrix):
    """Converts a 4x4 transformation matrix to a Transform message using transforms3d."""
    # Extract rotation matrix and translation vector
    rotation_matrix = matrix[:3, :3]
    translation = matrix[:3, 3]
    # Convert rotation matrix to quaternion
    rotation = mat2quat(rotation_matrix)  # Returns [w, x, y, z]
    # Create Transform message
    transform = Transform()
    transform.translation.x = translation[0]
    transform.translation.y = translation[1]
    transform.translation.z = translation[2]
    transform.rotation.w = rotation[0]
    transform.rotation.x = rotation[1]
    transform.rotation.y = rotation[2]
    transform.rotation.z = rotation[3]
    return transform


class HandeyePublisher(Node):
    def __init__(self):
        super().__init__('handeye_publisher')

        calibration_name_p = self.declare_parameter(
            'calibration_name',
            '',
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        name = calibration_name_p.get_parameter_value().string_value

        self.get_logger().info(f'Loading the calibration with name {name}')
        self.calibration = load_calibration(name)
        parameters = self.calibration.parameters

        # ID of the aruco marker mounted on the robot
        self.marker_id = self.declare_parameter(
            "marker_id", 1).get_parameter_value().integer_value

        if parameters.calibration_type == 'eye_in_hand':
            orig = parameters.robot_effector_frame
        else:
            orig = parameters.robot_base_frame
        dest = "camera_link" # parameters.tracking_base_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.calibration_tf = TransformStamped()
        self.calibration_tf.header.frame_id = orig
        self.calibration_tf.child_frame_id = dest

        self.compute_transform_timer = self.create_timer(0.1, self.compute_transform)

    def compute_transform(self):
        try:
            color_to_link_tf = self.tf_buffer.lookup_transform(
                target_frame=self.calibration.parameters.tracking_base_frame,
                source_frame="camera_link",
                time=Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error('Could not get transform from tracking_base_frame to camera_link')
            return

        base_to_color_mat = transform_to_matrix(self.calibration.transform)
        color_to_link_mat = transform_to_matrix(color_to_link_tf.transform)
        base_to_link_mat = np.dot(base_to_color_mat, color_to_link_mat)
        self.calibration_tf.transform = matrix_to_transform(base_to_link_mat)
        self.get_logger().info('Computed the robot base to camera transform')

        self.compute_transform_timer.cancel()
        self.publish_transform_timer = self.create_timer(0.1, self.publish_transform)

    def publish_transform(self):
        self.calibration_tf.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.calibration_tf)


def main(args=None):
    rclpy.init(args=args)

    handeye_publisher = HandeyePublisher()

    try:
        rclpy.spin(handeye_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        handeye_publisher.destroy_node()


if __name__ == '__main__':
    main()
