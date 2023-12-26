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
    
    # Specify path to xacro file within the package
    file_subpath = 'robot_description/balance_robot.urdf.xacro'
    # rviz_path = 'config/rvizView_config.rviz'
    
    gazebo_params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'gazebo_params.yaml')
    
    # Use xacro to process the (URDF) file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    # robot_description_raw = xacro.process_file(xacro_file).toxml()
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
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
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file' + gazebo_params_file}.items()
    )
    
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',            
        arguments=['-topic', 'robot_description', '-entity', 'balance_robot'],
        output='screen'
    )
    
    diff_cont_spawner = Node(
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
        node_robot_state_publisher,
        spawn_entity,
        rviz,
        diff_cont_spawner,
        joint_broad_spawner
    ])
    