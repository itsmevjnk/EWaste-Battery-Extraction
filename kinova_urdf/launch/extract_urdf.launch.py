# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
import subprocess
import os

def generate_urdf(context, *args, **kwargs):
    sim_gazebo = context.perform_substitution(LaunchConfiguration("sim_gazebo"))
    sim_isaac = context.perform_substitution(LaunchConfiguration("sim_isaac"))
    moveit = context.perform_substitution(LaunchConfiguration("moveit"))
    use_fake_hardware = context.perform_substitution(LaunchConfiguration("use_fake_hardware"))

    kortex_description_path = get_package_share_directory('kinova_urdf') # contains patched xacro files
    xacro_file = os.path.join(kortex_description_path, 'urdf', 'gen3_lite_gen3_lite_2f.xacro')

    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("kortex_description"), "robots", "gen3_lite_gen3_lite_2f.xacro"]
    #         ),
    #         " ", "sim_gazebo:=", sim_gazebo,
    #         " ", "sim_isaac:=", sim_isaac,
    #         " ", "moveit_active:=", moveit,
    #         " ", "use_fake_hardware:=", use_fake_hardware
    #     ]
    # )

    command = [
        'xacro', xacro_file,
        f'sim_gazebo:={sim_gazebo}',
        f'sim_isaac:={sim_isaac}',
        f'moveit_active:={moveit}',
        f'use_fake_hardware:={use_fake_hardware}'
    ]
    print('Running command:', ' '.join(command))
    robot_description_content = subprocess.run(
        command,
        capture_output=True, text=True
    ).stdout

    OUTPUT_FNAME = 'kinova_gen3_lite.urdf'
    with open(OUTPUT_FNAME, 'w') as f:
        f.write(robot_description_content)
    print('Wrote URDF file to', OUTPUT_FNAME)

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kortex_description"), "rviz", "view_robot.rviz"]
    )

    rviz = LaunchConfiguration("rviz")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(rviz)
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(rviz)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(rviz)
    )

    nodes_to_start = [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            description="Generate URDF file for Gazebo simulation",
            choices=["true", "false"],
            default_value="false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_isaac",
            description="Generate URDF file for Isaac Sim simulation",
            choices=["true", "false"],
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit",
            description="Configure URDF for MoveIt!",
            choices=["true", "false"],
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            description="Configure ros2_control to use fake hardware (passthrough)",
            choices=["true", "false"],
            default_value="false"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            description="Launch RViz to visualise robot",
            choices=["true", "false"],
            default_value="true"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=generate_urdf)])