import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():

    ld = LaunchDescription()

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), 'config/config', 'ekf.yaml')],
        #arguments=['--ros-args', '--log-level', 'debug'], for debug
        remappings=[('/odometry/filtered', '/odometry/filtered'),],
        )

    ld.add_action(ekf_node)


    state_estimator = Node(
        package='my_robot_localization',
        executable='state_estimator_node',
        name='state_estimator'
        )
    ld.add_action(state_estimator)


    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), 'config/config', 'navsat_transform_node.yaml')],
        remappings=[('gps/fix', '/gps/data_fixed'), ('/imu', '/imu/data')]
    )
    ld.add_action(navsat_node)



    # Add static transform publisher (fondamentale per il corretto funzionamento del sistema)
    
    static_tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    ld.add_action(static_tf2)

    #per fare un launch unico

    imu_merger = Node(
        package='state_estimation_py',
        executable='imu_merger_node',
        name='imu_merger',
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), 'config/config', 'imu_merger.yaml')],
        remappings=[('/imu/data', '/imu/data')]
    )

    ld.add_action(imu_merger)


    gps_header = Node(
        package= 'my_robot_localization',
        executable= 'fix_gps_header_node',
        name= 'fix_gps_header',
        remappings=[('/gps/data_fixed', '/gps/data_fixed')]
    )

    ld.add_action(gps_header)


    return ld
