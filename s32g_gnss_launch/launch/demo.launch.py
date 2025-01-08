from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

ARGS=[
    DeclareLaunchArgument(
        'ntrip',
        default_value='false',
        description='Enable NTRIP client',
    ),
]


def generate_launch_description():

    gps_config_path = PathJoinSubstitution(
        [FindPackageShare('s32g_gnss_launch'), 'config', 'gnss.yaml']
    )
    
    return LaunchDescription(ARGS + [
        Node(
            condition=IfCondition(LaunchConfiguration('ntrip')),
            name='ntrip_client',
            package='ntrip_client',
            executable='ntrip_ros.py',
            parameters=[gps_config_path],
        ),  
        
        Node(
            name='ublox_gps_node',
            package='ublox_gps',
            executable='ublox_gps_node',
            output='both',
            parameters=[gps_config_path]
        ),
    ])