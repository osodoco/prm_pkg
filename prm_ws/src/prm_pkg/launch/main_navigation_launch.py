import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory('prm_navigation')

    # --- Rutas ---
    coppelia_exec = '/home/osdoco/ROM/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/coppeliaSim.sh'
    scene_file = os.path.join(package_share_dir, 'coppelia_scenes', 'turtlebot3_burger_ROS2.ttt')
    map_file_path = os.path.join(package_share_dir, 'maps', 'coppeliasim_map.yaml')

    # --- Coppelia ---
    coppelia_cmd = ExecuteProcess(
        cmd=[coppelia_exec, scene_file],
        output='screen',
        name='coppeliasim_launcher',
        emulate_tty=True
    )

    # --- PRM Nodes ---
    prm_generator_node = Node(
        package='prm_navigation',
        executable='generate_prm',
        name='generate_prm',
        output='screen',
    )

    prm_navigator_node = Node(
        package='prm_navigation',
        executable='prm_navigation',
        name='prm_navigation',
        output='screen',
    )

    # --- Nav2 Bringup con tu mapa ---
    nav2_bringup_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
            f'map:={map_file_path}'
        ],
        output='screen',
        name='nav2_bringup_launcher',
        emulate_tty=True
    )

    return LaunchDescription([
        coppelia_cmd,
        nav2_bringup_cmd,
        prm_generator_node,
        prm_navigator_node,
    ])

