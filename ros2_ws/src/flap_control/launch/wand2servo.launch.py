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
        'Wand2Servo.yaml'
    )

    # ---------------- WAND TO SERVO ------------------ #

    wand2servo = Node(
        package='flap_control',
        executable='wand_direct',
        name='wand2servo',
        # parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/wand_angle', '/wand_angle'),
            #---------------------------------#
            ('/toSerial', '/toBrunilde'),
        ]
    )

    ld.add_action(wand2servo)

    # ----------------   ------------------ #

    return ld

