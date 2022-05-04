from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pibot_bringup",
            executable="pibot_driver",
            name="pibot_driver",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"base_frame": "base_link"},
                {"port": "/dev/pibot"},
                {"baudrate": 115200},
                {"cmd_vel_topic": "cmd_vel"},
                {"odom_frame": "odom"},
                {"odom_topic": "odom"},
                {"out_pid_debug_enable": False},
                {"freq": 100}
            ]
        )
    ])
