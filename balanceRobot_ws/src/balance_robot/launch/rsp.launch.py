from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_desc = LaunchDescription()
    
    launch_desc.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'balance_robot',
            'urdf_package_path': PathJoinSubstitution(['urdf', ''])}.items()
    ))
    return launch_desc