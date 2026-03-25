from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('emm_v5_mpc_controller'), 'config', 'mpc_controller.yaml')
    return LaunchDescription([
        Node(
            package='emm_v5_mpc_controller',
            executable='mpc_follow_joint_trajectory_server',
            name='mpc_controller',
            output='screen',
            parameters=[cfg],
        )
    ])
