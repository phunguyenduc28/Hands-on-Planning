import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Resolve paths
    pkg_dir = get_package_share_directory('turtlebot_simulation')
    
    launch_file = os.path.join(pkg_dir, 'launch', 'turtlebot_basic.launch.py')
    scenario_file = os.path.join(pkg_dir, 'scenarios', 'turtlebot_hoi_circuit2.scn')

    # 2. Define the individual actions
    base_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'scenario_description': scenario_file
        }.items()
    )

    occupancy_grid_node = Node(
        package='grid_mapping',
        executable='occupancy_grid',
        name='occupancy_grid',
        output='screen'
    )

    rrt_planner_node = Node(
        package='online_motion_planning',
        executable='rrt_tb',
        name='rrt_planner',
        output='screen'
    )

    delayed_nodes = TimerAction(
        period=15.0,
        actions=[
            occupancy_grid_node,
            rrt_planner_node
        ]
    )

    # 3. Return the Description
    return LaunchDescription([
        base_simulation,
        delayed_nodes
    ])