from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Declare the is_sim launch argument
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Set to 'false' when launching on the real robot.",
    )

    # 2. Include robot_description.launch.xml (unconditionally)
    robot_description_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_description"),
                    "launch",
                    "robot_description.launch.xml",
                ]
            )
        )
    )

    # 3. Include ros2_control.launch.py and pass the is_sim argument to it
    ros2_control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_controller"),
                    "launch",
                    "ros2_control.launch.py",
                ]
            )
        ),
        # Pass the 'is_sim' argument down to the included launch file
        launch_arguments={"is_sim": LaunchConfiguration("is_sim")}.items(),
    )

    # 4. Include gazebo.launch.xml ONLY if is_sim is true
    gazebo_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_simulation"),
                    "launch",
                    "gazebo.launch.xml",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("is_sim")),
    )

    # 5. Include navigation.launch.xml (unconditionally)
    navigation_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("prestobot_navigation"),
                    "launch",
                    "navigation.launch.xml",
                ]
            )
        )
    )

    # 6. Include sllidar_c1_launch.py ONLY if is_sim is false
    sllidar_c1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sllidar_ros2"),
                    "launch",
                    "sllidar_c1_launch.py",
                ]
            )
        ),
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
    )


    # 7. Return the LaunchDescription
    return LaunchDescription(
        [
            is_sim_arg,
            robot_description_include,
            ros2_control_include,
            gazebo_include,
            navigation_include,
            sllidar_c1_include,
        ]
    )