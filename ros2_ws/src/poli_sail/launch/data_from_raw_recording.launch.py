from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Usage:
# ros2 launch <your_pkg> data_from_raw_recording.launch.py \
#   src_bag:=/path/to/src_bag \
#   out_bag:=/path/to/out_bag \
#   rate:=2.0

def generate_launch_description():
    src_bag = LaunchConfiguration('src_bag')
    out_bag = LaunchConfiguration('out_bag')
    rate = LaunchConfiguration('rate')

    # Topics to record – edit this list
    topics = [
        '/imu_data',
        '/fix',
        '/fix_velocity',  # <-- missing comma fixed
        '/wand_angle',    # keep as provided (verify topic name in your system)
        '/odometry/filtered',
        '/gps/filtered',
    ]

    # 1) Your processing nodes (brought in via included launches)
    
    params_file = os.path.join( # Path to mpu parameters
        get_package_share_directory('poli_sail'),
        'config', 
        'DataFromRawRecording.yaml'
    )

   # --------------- PARSERS -------------------- #

    pot_parser = Node(
        package='can_bus',
        executable='general_parser_node',
        name='pot_parser',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/raw_data', '/fromBrunilde'),
            ('/parsed_data', '/wand_angle'),
        ]
    )

    imu_parser = Node(
        package='can_bus',
        executable='imu_parser_node',
        name='imu_parser',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/serial_data', '/fromBrunilde'),
            ('/imu_data', '/imu_data'),
        ]
    )

    # -------------EKF ------------------- #
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[params_file],
        remappings=[('/myodometry/filtered', '/odometry/filtered')],
    )

    gps_node = Node(
        package= 'state_estimation',
        executable = 'gps_to_local',
        name = 'gps_to_local',
        parameters=[{'use_sim_time': True}],
        remappings=[('/gps_data', '/fix')],
    )
    
    visualization_node = Node(
        package = 'state_estimation',
        executable = 'local_to_gps',
        name = 'local_to_gps',
        parameters=[{'use_sim_time': True}],
        remappings=[('/gps_data', '/fix')],
    )
    
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_static_broadcaster',
        parameters=[{'use_sim_time': True}],
        # args: x y z roll pitch yaw frame_id child_frame_id
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    # ------------------------------------ #

    # 2) Recorder via CLI (Jazzy-friendly). Using ExecuteProcess instead of Node.
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time','--storage', 'mcap', '-o', out_bag] + topics,
        output='screen',
    )

    # 3) Player (start paused, publish /clock)
            # 3) Player (start paused, publish /clock). Use default node name 'rosbag2_player'.
    player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', src_bag, '--clock', '--rate', rate,
        ],
        output='screen',
    )

    # 5) When the player exits, shut down the launch (recorder & nodes stop cleanly)
    stop_when_player_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=player,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('src_bag'),
        DeclareLaunchArgument('out_bag'),
        DeclareLaunchArgument('rate', default_value='1.0'),

        pot_parser,
        imu_parser,
        ekf_node,
        gps_node,
        visualization_node,
        static_odom_to_base,
        
        recorder,
        player,
        stop_when_player_finishes,
    ])
