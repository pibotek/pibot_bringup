#include "pibot_bringup/base_driver_config.h"

#include <memory>
#include "pibot_bringup/transport.h"
#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>

class Dataframe;
class BaseDriver : public rclcpp::Node {
 public:
  BaseDriver();
  ~BaseDriver();

  void work_loop();

 private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr vel_cmd);
  void init_cmd_odom();
  void init_pid_debug();
  void init_imu();
  void read_param();

  void update_param();
  void update_odom();
  void update_speed();
  void update_pid_debug();
  void update_imu();

 private:
  BaseDriverConfig config_;
  std::shared_ptr<Transport> trans_;
  std::shared_ptr<Dataframe> frame_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  nav_msgs::msg::Odometry odom_msg_;

  geometry_msgs::msg::TransformStamped odom_tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

#define MAX_MOTOR_COUNT 4
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pid_input_pub_[MAX_MOTOR_COUNT];
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pid_output_pub_[MAX_MOTOR_COUNT];

  std_msgs::msg::Int32 pid_input_msg_[MAX_MOTOR_COUNT];
  std_msgs::msg::Int32 pid_output_msg_[MAX_MOTOR_COUNT];

  bool need_update_speed_{false};

  // imu
  bool use_accelerometer_, use_gyroscope_, use_magnetometer_;
  bool use_mag_msg_;

  bool perform_calibration_, is_calibrated_;
  int calibration_samples_;
  std::vector<double> acceleration_bias_{0.0, 0.0, 0.0}, gyroscope_bias_{0.0, 0.0, 0.0};

  // Used for mag scaling
  double mag_x_min_, mag_x_max_;  //  [T]
  double mag_y_min_, mag_y_max_;
  double mag_z_min_, mag_z_max_;

  // imu pub/sub
  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // ros::Publisher mag_pub;
  sensor_msgs::msg::MagneticField mag_msg_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  // imu services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr imu_cal_srv_;

  void calibrateCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                         std::shared_ptr<std_srvs::srv::Empty::Response> response);
};
