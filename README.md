# PIBOT DRIVER ROS package
ROS node and test application for PIBOT DRIVER

# How to build pibot driver ros package
- 1) Clone this project to your catkin's workspace src folder
- 2) Running catkin_make to build pibot driver Node

# How to run pibot bringup ros package
- `ros2 launch  pibot_bringup bringup_launch.py`

# Configure pibot dirver board
`ros2 run rqt_reconfigure rqt_reconfigure`

# Control robot with pibot dirver
- with keyboard
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- with joystick
    > install dependent package
        `sudo apt-get install ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-teleop-twist-joy`
    - non-holonomic robot 
        `ros2 launch pibot_bringup joystick_launch.py`
    - holonomic robot 
        `ros2 launch pibot_bringup joystick_launch.py joy_config:=joystick-holonomic`

# Topic

- pub topic

| topic | type |
| :----: | :----: |
| `/odom` | [nav_msgs/Odometry](http://docs.ros.org/en/kinetic/api/nav_msgs/html/msg/Odometry.html) |
| `/imu/data_raw` | [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html) |
| `/imu/mag` | [sensor_msgs/MagneticField](http://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html) |

- sub topic

| topic | type |
| :----: | :----: |
| `/cmd_vel` | [geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) |

