#include "pibot_bringup/base_driver_config.h"

#include <memory>
#include "pibot_bringup/transport.h"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

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

  geometry_msgs::msg::TransformStamped odom_tf;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

#define MAX_MOTOR_COUNT 4
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pid_input_pub_[MAX_MOTOR_COUNT];
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pid_output_pub_[MAX_MOTOR_COUNT];

  std_msgs::msg::Int32 pid_input_msg_[MAX_MOTOR_COUNT];
  std_msgs::msg::Int32 pid_output_msg_[MAX_MOTOR_COUNT];

  bool need_update_speed_{false};

  // // imu
  // bool use_accelerometer, use_gyroscope, use_magnetometer;
  // bool use_mag_msg;

  // bool perform_calibration, is_calibrated;
  // int calibration_samples;
  // std::map<std::string,double> acceleration_bias, gyroscope_bias;

  // // Covariance
  // double linear_acc_stdev, angular_vel_stdev, magnetic_field_stdev;
  // boost::array<double, 9> linear_acc_covar;
  // boost::array<double, 9> angular_vel_covar;
  // boost::array<double, 9> magnetic_field_covar;

  // // Used for mag scaling
  // double mag_x_min, mag_x_max;  //  [T]
  // double mag_y_min, mag_y_max;
  // double mag_z_min, mag_z_max;

  // // imu pub/sub
  // ros::Publisher imu_pub;
  // ros::Publisher mag_pub;

  // // imu services
  // ros::ServiceServer imu_cal_srv;

  // bool calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  // void fillRowMajor(boost::array<double, 9> & covar, double stdev);
};
