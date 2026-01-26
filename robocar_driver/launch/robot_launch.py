#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Chemin du package
    package_dir = get_package_share_directory('robocar_driver')
    urdf_file = os.path.join(package_dir, 'resource', 'robocar.urdf')
    
    # Déclarer les arguments
    robot_name = LaunchConfiguration('robot_name', default='Robocar')
    
    # Lire le contenu de l'URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/opt/ros/jazzy/lib:' + env.get('LD_LIBRARY_PATH', '')
    env['WEBOTS_CONTROLLER_URL'] = 'ipc://1234/Robocar'


    # Node: Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )
    
    # Node: Webots ROS2 Driver
    webots_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'robot_description': urdf_file},
            {'set_robot_state_publisher': True},
        ],
        env=env,
    )
    
    return LaunchDescription([
        robot_state_publisher,
        webots_driver,
    ])

