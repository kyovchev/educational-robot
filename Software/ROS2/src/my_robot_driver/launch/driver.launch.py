from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_driver',
            executable='my_robot_driver',
            name='my_robot_driver',
            output='screen',
            parameters=[
                # You can add params here later
                # {'use_sim_time': False}
            ],
            remappings=[
                # Example:
                # ('/old_topic', '/new_topic')
            ]
        )
    ])