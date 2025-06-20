import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, ExecuteProcess,
                            RegisterEventHandler, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

# --- OpaqueFunction for PRM nodes sequential launch ---
def launch_prm_nodes_sequentially(context, *args, **kwargs):
    prm_pkg_share_dir = get_package_share_directory('prm_pkg')

    # 1. Launch PRM Navigator node first
    prm_navigator_node = Node(
        package='prm_pkg',
        executable='prm_navigation',
        name='prm_navigation_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        
    )

    # 2. Launch PRM Generator node after Navigator starts
    prm_generator_node = Node(
        package='prm_pkg',
        executable='generate_prm',
        name='generate_prm_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return [
        prm_navigator_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=prm_navigator_node,
                on_start=[
                    LogInfo(msg='PRM Navigator ready, launching PRM Generator.'),
                    prm_generator_node
                ]
            )
        )
    ]

# --- Main generate_launch_description function ---
def generate_launch_description():
    # Get share directories for required packages
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_pkg, 'launch')

    turtlebot3_coppeliasim_pkg = get_package_share_directory('turtlebot3_coppeliasim')
    turtlebot3_coppeliasim_launch_dir = os.path.join(turtlebot3_coppeliasim_pkg, 'launch')
    
    prm_pkg_share_dir = get_package_share_directory('prm_pkg')


    # --- Define Launch Arguments ---
    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_pkg, 'params', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )
    nav2_params_file = LaunchConfiguration('params_file')
    
    map_file_path = os.path.join(prm_pkg_share_dir, 'maps', 'coppeliasim_map.yaml')

    rviz_config_file = os.path.join(prm_pkg_share_dir, 'rviz', 'final_config.rviz')


    # --- Actions ---

    # 1. Launch CoppeliaSim with TurtleBot3 model 
    coppeliasim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_coppeliasim_launch_dir, 'turtlebot3_coppeliasim_no_rviz2.launch.py')
        ),

        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Launch Nav2 Bringup with your custom map
    nav2_bringup_cmd = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file_path,
                'params_file': nav2_params_file,
                'use_sim_time': 'true'
            }.items()
        ),
    ])

    # 3. Launch your custom RViz instance
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file], 
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )

    # 4. Launch PRM Generator and Navigator nodes sequentially
    prm_nodes_sequential_launch = OpaqueFunction(function=launch_prm_nodes_sequentially)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions in desired order
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(coppeliasim_launch)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_node) 
    ld.add_action(prm_nodes_sequential_launch)

    return ld
