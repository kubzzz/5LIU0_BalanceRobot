import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    # Specify the name of the package
    pkg_name = 'balance_robot'
        
    gazebo_params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'gazebo_params.yaml')
    
    # Use rsp.launch.py file to launch the robot_state_publisher (rsp)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )
        
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', 'src/balance_robot/config/rvizView_config.rviz']
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file' + gazebo_params_file}.items()
    )
    
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',            
        arguments=['-topic', 'robot_description', '-entity', 'balance_robot'],
        output='screen'
    )
    
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_cont"]
    )
    
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )
    
    
    
    
    # Run the node
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz,
        diff_drive_spawner,
        joint_broad_spawner
    ])
    