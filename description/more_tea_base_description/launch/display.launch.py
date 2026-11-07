"""Show More Tea in RViz2. The joint_state_publisher_gui sliders spin the four wheel joints.

    ros2 launch more_tea_base_description display.launch.py            # with sliders
    ros2 launch more_tea_base_description display.launch.py gui:=false # static
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('more_tea_base_description')
    with open(os.path.join(share, 'urdf', 'more_tea.urdf')) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open joint_state_publisher_gui'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'display.rviz')]),
    ])
