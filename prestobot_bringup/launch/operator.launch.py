from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Declare the launch argument 'auto'
    auto_arg = DeclareLaunchArgument(
        "auto",
        default_value="true",
        description="Set to true for autonomous mode (launches hmi), false for manual mode (launches teleop).",
    )

    # 2. Define the path to the rviz configuration file
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("prestobot_navigation"),
            "rviz",
            "navigation_config.rviz",
        ]
    )

    # 3. Define the rviz2 node (launched unconditionally)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    # 4. Define the teleop include, to be launched only if 'auto' is false
    teleop_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("teleop_twist_joy"),
                    "launch",
                    "teleop-launch.py",
                ]
            )
        ),
        launch_arguments={"joy_config": "xbox"}.items(),
        condition=UnlessCondition(LaunchConfiguration("auto")),
    )

    # 5. Define the HMI node, to be launched only if 'auto' is true
    hmi_node = Node(
        package="prestobot_py_pkg",
        executable="hmi",
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto")),
    )

    # 6. Return the LaunchDescription
    return LaunchDescription(
        [
            auto_arg,
            rviz_node,
            teleop_include,
            hmi_node,
        ]
    )