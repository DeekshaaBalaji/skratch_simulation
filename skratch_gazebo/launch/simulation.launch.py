# Authors: Chaitanya Gumudala

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    movable_joints = LaunchConfiguration('movable_joints')

    pkg_path = get_package_share_directory('skratch_description')
    xacro_file = os.path.join(pkg_path, 'description', 'skratch.urdf.xacro')

    # Process xacro file
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

    # Gazebo launch
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg_path, 'worlds', 'simple_world.sdf')}.items()
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'skratch',
            '-x', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description['robot_description']}]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controller loading
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    controller_processes = [
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'skratch_base_controller'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'kinova_arm_controller'],
            output='screen'
        )
    ]

    # ROS-Gazebo Bridge for TF
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/simple_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/world/simple_world/dynamic_pose/info', '/link_states')
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge for clock
    ros_clock_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )

    # Skratch Platform Controller Node
    skratch_platform_controller_node = Node(
        package='skratch_gazebo',
        executable='skratch_gazebo_platform_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz node
    rviz_config_file = os.path.join(
        get_package_share_directory('skratch_gazebo'),
        'config', 'rviz', 'skratch_simulation.rviz'
    )
    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Enable joint_state_publisher_gui'),
        DeclareLaunchArgument('movable_joints', default_value='true',
                              description='Enable or disable movable joints in URDF'),
        gazebo,
        spawn,
        robot_state_publisher_node,   
        joint_state_publisher_node,
        ros_gz_bridge_node,
        ros_clock_node,
        load_joint_state_controller,
        *controller_processes,
        skratch_platform_controller_node,
        rviz_node
    ])
