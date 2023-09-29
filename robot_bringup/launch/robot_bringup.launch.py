import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # URDF config
    urdf_file_name = 'robot.urdf'

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        urdf_file_name)
    ###

    # RVIZ config
    rviz_path = os.path.join(
        get_package_share_directory('robot_description'),
        'rviz',
        'rviz_config.rviz')
    ###

    return LaunchDescription([

        Node(
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
            # output='screen',
             parameters=[{'use_sim_time': False}],
             arguments=[urdf_path]
        ),

        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['d', rviz_path],
            output='screen'
        )
    ])
