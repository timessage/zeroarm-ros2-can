import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup_share = get_package_share_directory("zero_arm_bringup")
    moveit_share = get_package_share_directory("moveit_config")

    default_config_yaml = os.path.join(bringup_share, "config", "emm_v5_hw.yaml")
    config_yaml = LaunchConfiguration("config_yaml")

    declare_config_yaml = DeclareLaunchArgument(
        "config_yaml",
        default_value=default_config_yaml,
        description="YAML config for EmmV5 ros2_control hardware plugin",
    )

    bringup_urdf_xacro = os.path.join(bringup_share, "config", "zero_arm_can.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("zero_arm", package_name="moveit_config")
        .robot_description(file_path=bringup_urdf_xacro, mappings={"config_yaml": config_yaml})
        .to_moveit_configs()
    )

    controllers_yaml = os.path.join(moveit_share, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(moveit_share, "config", "moveit.rviz")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            controllers_yaml,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--param-file", controllers_yaml,
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--param-file", controllers_yaml,
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            declare_config_yaml,
            rsp,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group,
            rviz,
        ]
    )
