from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

import os

def generate_launch_description():
    # args that can be set from the command line or a default will be used
    default_baudrate = os.getenv('PIBOT_DRIVER_BAUDRATE')
    if default_baudrate is None:
        default_baudrate = "115200"

    baudrate_arg = DeclareLaunchArgument(
        "baudrate", default_value=TextSubstitution(text=default_baudrate)
    )

    publish_odom_tf_arg = DeclareLaunchArgument(
        "publish_odom_tf", default_value=TextSubstitution(text="true")
    )

    use_imu_arg = DeclareLaunchArgument(
        "use_imu", default_value=TextSubstitution(text="true")
    )

    use_accelerometer_arg = DeclareLaunchArgument(
        "use_accelerometer", default_value=TextSubstitution(text="true")
    )
    
    use_gyroscope_arg = DeclareLaunchArgument(
        "use_gyroscope", default_value=TextSubstitution(text="true")
    )
    
    use_magnetometer_arg = DeclareLaunchArgument(
        "use_magnetometer", default_value=TextSubstitution(text="true")
    )

    cali_imu_startup_arg = DeclareLaunchArgument(
        "cali_imu_startup", default_value=TextSubstitution(text="true")
    )

    return LaunchDescription([
        baudrate_arg,
        publish_odom_tf_arg,
        use_imu_arg,
        use_accelerometer_arg,
        use_gyroscope_arg,
        use_magnetometer_arg,
        cali_imu_startup_arg,

        Node(
            package="pibot_bringup",
            executable="pibot_driver",
            name="pibot_driver",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"base_frame": "base_link"},
                {"port": "/dev/pibot"},
                {"baudrate": LaunchConfiguration('baudrate')},
                {"cmd_vel_topic": "cmd_vel"},
                {"odom_frame": "odom"},
                {"odom_topic": "odom"},
                {"out_pid_debug_enable": False},
                {"freq": 100},
                {"imu/accelerometer_bias": [0.005436, 0.014684, -0.395418]},
                {"imu/gyroscope_bias": [-0.035592, 0.080670, 0.001216]},
                {"imu/use_accelerometer": LaunchConfiguration('use_imu')},
                {"imu/use_gyroscope": LaunchConfiguration('use_imu')},
                {"imu/use_magnetometer": LaunchConfiguration('use_imu')},
                {"imu/perform_calibration": LaunchConfiguration('cali_imu_startup')},
            ]
        )
    ])
