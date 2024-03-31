# 
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler,DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument(
        'model',
        default_value='m1013',
        description='Robot Model'
    ),
    DeclareLaunchArgument(
        'color',
        default_value='white',
        description='Robot Color'
    )
    ]	

def generate_launch_description():
    
    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dsr_description2"),
                    "xacro",
                    LaunchConfiguration('model'),
                ]
            ),
            ".urdf.xacro",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dsr_controller2"),
            "config",
            "dsr_controller2.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )
    # robot_state_pub_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     remappings=[
    #         (
    #             "/joint_states",
    #             "/dsr/joint_states",
    #         ),
    #     ],
    #     parameters=[robot_description],
    # )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
        'robot_description': Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])           
    }])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dsr_controller2", "-c", "/controller_manager"],
    )
    
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dsr_joint_trajectory", "-c", "/controller_manager"],
    )

    # joint_state_publisher_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["dsr_joint_publisher", "--controller-manager", "/controller_manager"],
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # joint_state_publisher_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        # delay_robot_controller_spawner_after_joint_trajectory_controller_spawner,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
