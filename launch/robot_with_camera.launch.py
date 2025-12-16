from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_control_node = Node(
        package='project',
        executable='motor_control',
        name='motor_and_servo_control',
        output='screen'
    )

    livestream_node=Node(
        package='project',
        executable='livestream',
        name='livestream',
        output='screen'
    )

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            '/home/amit/ros2_ws/src/project/config/camera_view.rviz'
        ],
        output='screen'
    )

    return LaunchDescription([
        motor_control_node,
        livestream_node,
        rviz_node
    ])