from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    odometry_conversion_node_with_parameters = Node(
            package='odometry_conversion',
            executable='odometry_conversion_node',
            name='tracking_camera_odometry_conversion',
            parameters=[{
                "in_odom_frame": "tracking_camera_pose_frame",
                "in_sensor_frame": "tracking_camera_pose_frame",
                "out_odom_frame": "tracking_camera_odom",
                "out_base_frame": "base_link",
                "in_odom_topic": "/tracking_camera/odom/sample",
                "out_odom_topic": "/base_odom",
                "is_odom_child": True,
            }]
        )

    return LaunchDescription([
        odometry_conversion_node_with_parameters,
    ])