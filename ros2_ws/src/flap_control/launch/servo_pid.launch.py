from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    params_file = os.path.join( # Path to mpu parameters
        get_package_share_directory('flap_control'),
        'config', 
        'ServoPID.yaml'
    )

    # ---------------- WAND TO SERVO ------------------ #

    pid_node = Node(
        package='flap_control',
        executable='pid_node',
        name='pid_node',
        # parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/reference', '/wand_ref'),
            ('/measurement', '/wand_angle'),
            #---------------------------------#
            ('/control_value', '/servo_angle'),
        ]
    )

    ld.add_action(pid_node)

    servo_to_brunilde = Node(
        package='flap_control',
        executable='servo2brunilde',
        name='servo_to_brunilde',
        # parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/servo_angle', '/servo_angle'),
            #---------------------------------#
            ('/control_value', '/toBrunilde'),
        ]
    )

    ld.add_action(servo_to_brunilde)

    # ----------------   ------------------ #

    return ld
