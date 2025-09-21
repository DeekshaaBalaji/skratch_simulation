# Author: Chaitanya Gumudala

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    movable_joints = LaunchConfiguration('movable_joints')

    # Path to your xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('skratch_description'),
        'description',
        'skratch.urdf.xacro'
    ])

    # Run xacro via Command substitution
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' ',
        'movable_joints:=', movable_joints
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': use_sim_time
    }

    rviz_config_file = os.path.join(
        get_package_share_directory('skratch_gazebo'),
        'config',
        'rviz',
        'skratch_description.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument('movable_joints', default_value='true',
                              description='Enable or disable movable joints in URDF'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        Node(
            condition=UnlessCondition(gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            condition=IfCondition(gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
