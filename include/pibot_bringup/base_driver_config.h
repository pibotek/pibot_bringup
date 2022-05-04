#ifndef BASE_DRIVER_CONFIG_
#define BASE_DRIVER_CONFIG_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

#define USE_DYNAMIC_RECONFIG
#ifdef USE_DYNAMIC_RECONFIG
// #include "pibot_bringup/pibot_driverConfig.h"
#endif

class Robot_parameter;
class BaseDriverConfig {
 public:
  BaseDriverConfig();
  ~BaseDriverConfig();

  void init(rclcpp::Node* node);
  void SetRobotParameters(rclcpp::Node* node, Robot_parameter* r);

#ifdef USE_DYNAMIC_RECONFIG
  // void dynamic_callback(pibot_bringup::pibot_driverConfig& config, uint32_t level);
  // bool get_param_update_flag();
#endif
 public:
  Robot_parameter* rp;

  std::string port;
  int32_t baudrate;

  std::string base_frame;
  std::string odom_frame;

  bool publish_tf;

  bool use_imu;

  std::string cmd_vel_topic;
  std::string odom_topic;

  int32_t freq;

  bool out_pid_debug_enable;

 private:
#ifdef USE_DYNAMIC_RECONFIG
  bool param_update_flag_;
#endif
  bool set_flag;
};

#endif
