#ifndef PIBOT_BRINGUP_BASE_DRIVER_CONFIG_H_
#define PIBOT_BRINGUP_BASE_DRIVER_CONFIG_H_

#include <ros/ros.h>

#define USE_DYNAMIC_RECONFIG
#ifdef USE_DYNAMIC_RECONFIG
#include <dynamic_reconfigure/server.h>
#include "pibot_bringup/pibot_driverConfig.h"
#endif

namespace pibot {

class RobotParameter;
class BaseDriverConfig {
 public:
  BaseDriverConfig(ros::NodeHandle& p);
  ~BaseDriverConfig();

  void init(RobotParameter* r);
  void SetRobotParameters();

#ifdef USE_DYNAMIC_RECONFIG
  void dynamic_callback(pibot_bringup::pibot_driverConfig& config, uint32_t level);
  bool get_param_update_flag();

 private:
  dynamic_reconfigure::Server<pibot_bringup::pibot_driverConfig> server;
  dynamic_reconfigure::Server<pibot_bringup::pibot_driverConfig>::CallbackType f;
#endif
 public:
  RobotParameter* rp;

  std::string port;
  int32_t baudrate;

  std::string base_frame;
  std::string odom_frame;

  bool publish_odom_tf;

  bool use_imu;

  std::string cmd_vel_topic;
  std::string odom_topic;

  int32_t freq;

  bool out_pid_debug_enable;

 private:
#ifdef USE_DYNAMIC_RECONFIG
  bool param_update_flag;
#endif
  ros::NodeHandle& pn;
  ros::ServiceClient client;

  bool set_flag;
};

}  // namespace pibot

#endif
