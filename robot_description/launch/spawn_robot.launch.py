import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(get_package_share_directory('robot_description'), 'config/ekf.yaml'), {'use_sim_time': False}],
    #    parameters=[os.path.join(pkg_path, 'config/ekf.yaml'), {'use_sim_time': use_sim_time }]
    #    remappings=[
    #         ('/imu/data_raw', 'imu_data_raw'),
    #         # ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    #     ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
            # output='screen',
             parameters=[{'use_sim_time': False}],
             arguments=[urdf_path]),

        # Node(
        #     name='rplidar_composition',
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'serial_baudrate': 115200,  # A1 / A2
        #         # 'serial_baudrate': 57600,  # A1 / A2
        #         # 'serial_baudrate': 256000, # A3
        #         'frame_id': 'laser_frame',
        #         'inverted': False,
        #         'angle_compensate': True,
        #     }],
        # ),
        # Node(      # pub odom tf
        #     package='robot_navigation2',
        #     executable='py_node.py',
        #     name='py_node',
        #     parameters=[{'use_sim_time': False}],
        
        # ),
        
        # Node(
        #     package='robot_navigation2',
        #     executable='laser_filters.py',
        #     name='laser_filters',
        #     parameters=[{'use_sim_time': False}],
        
        # ),
        # robot_localization_node,
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['d', rviz_path],
        #     output='screen'
        # )
    ])
