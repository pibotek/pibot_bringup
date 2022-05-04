#include "pibot_bringup/base_driver_config.h"
#include "pibot_bringup/data_holder.h"

#define PI 3.1415926f

BaseDriverConfig::BaseDriverConfig() {
#ifdef USE_DYNAMIC_RECONFIG
  param_update_flag_ = false;
#endif

  set_flag = true;
}

BaseDriverConfig::~BaseDriverConfig() {
}

void BaseDriverConfig::init(rclcpp::Node* node) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  // comm param
  node->declare_parameter<std::string>("port", "/dev/ttyACM0", descriptor);
  node->declare_parameter<int32_t>("baudrate", 115200, descriptor);

  node->declare_parameter<std::string>("base_frame", "base_link", descriptor);
  node->declare_parameter<std::string>("odom_frame", "odom", descriptor);
  node->declare_parameter<bool>("publish_tf", true, descriptor);

  node->declare_parameter<bool>("out_pid_debug_enable", true, descriptor);

  // topic name param
  node->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel", descriptor);
  node->declare_parameter<std::string>("odom_topic", "odom", descriptor);

  node->declare_parameter<int32_t>("freq", 1000, descriptor);

  port = node->get_parameter("port").as_string();
  baudrate = node->get_parameter("baudrate").as_int();

  RCLCPP_INFO(node->get_logger(), "port:%s baudrate:%d", port.c_str(), baudrate);

  base_frame = node->get_parameter("base_frame").as_string();
  odom_frame = node->get_parameter("odom_frame").as_string();

  publish_tf = node->get_parameter("publish_tf").as_bool();

  out_pid_debug_enable = node->get_parameter("out_pid_debug_enable").as_bool();
  RCLCPP_INFO(node->get_logger(), "out_pid_debug_enable:%d", out_pid_debug_enable);

  // topic name param
  cmd_vel_topic = node->get_parameter("cmd_vel_topic").as_string();
  odom_topic = node->get_parameter("odom_topic").as_string();
  freq = node->get_parameter("freq").as_int();
}

void BaseDriverConfig::SetRobotParameters(rclcpp::Node* node, Robot_parameter* rp) {
  // #ifdef USE_DYNAMIC_RECONFIG
  //   static bool flag = true;
  //   if (flag) {
  //     flag = false;
  //     f = boost::bind(&BaseDriverConfig::dynamic_callback, this, _1, _2);
  //     server.setCallback(f);
  //   }
  // #endif
}

// #ifdef USE_DYNAMIC_RECONFIG
// void BaseDriverConfig::dynamic_callback(pibot_bringup::pibot_driverConfig& config, uint32_t level) {
//   if (set_flag) {
//     set_flag = false;
//     config.wheel_diameter = rp->wheel_diameter;
//     config.wheel_track = rp->wheel_track;
//     config.do_pid_interval = rp->do_pid_interval;
//     config.encoder_resolution = rp->encoder_resolution;
//     config.kp = rp->kp;
//     config.ki = rp->ki;
//     config.kd = rp->kd;
//     config.ko = rp->ko;
//     config.cmd_last_time = rp->cmd_last_time;
//     config.max_v_liner_x = rp->max_v_liner_x;
//     config.max_v_liner_y = rp->max_v_liner_y;
//     config.max_v_angular_z = rp->max_v_angular_z;
//     config.imu_type = rp->imu_type;
//     config.motor_ratio = rp->motor_ratio;
//     config.model_type = rp->model_type;

//     config.motor1_exchange_flag = (~rp->motor_nonexchange_flag & MOTOR_ENCODER_1_FLAG);
//     config.motor2_exchange_flag = (~rp->motor_nonexchange_flag & MOTOR_ENCODER_2_FLAG) >> 1;
//     config.motor3_exchange_flag = (~rp->motor_nonexchange_flag & MOTOR_ENCODER_3_FLAG) >> 2;
//     config.motor4_exchange_flag = (~rp->motor_nonexchange_flag & MOTOR_ENCODER_4_FLAG) >> 3;

//     config.encoder1_exchange_flag = (~rp->encoder_nonexchange_flag & MOTOR_ENCODER_1_FLAG);
//     config.encoder2_exchange_flag = (~rp->encoder_nonexchange_flag & MOTOR_ENCODER_2_FLAG) >> 1;
//     config.encoder3_exchange_flag = (~rp->encoder_nonexchange_flag & MOTOR_ENCODER_3_FLAG) >> 2;
//     config.encoder4_exchange_flag = (~rp->encoder_nonexchange_flag & MOTOR_ENCODER_4_FLAG) >> 3;

//     return;
//   }

//   rp->wheel_diameter = config.wheel_diameter;
//   rp->wheel_track = config.wheel_track;
//   rp->do_pid_interval = config.do_pid_interval;
//   rp->encoder_resolution = config.encoder_resolution;
//   rp->kp = config.kp;
//   rp->ki = config.ki;
//   rp->kd = config.kd;
//   rp->ko = config.ko;
//   rp->cmd_last_time = config.cmd_last_time;
//   rp->max_v_liner_x = config.max_v_liner_x;
//   rp->max_v_liner_y = config.max_v_liner_y;
//   rp->max_v_angular_z = config.max_v_angular_z;
//   rp->imu_type = config.imu_type;
//   rp->motor_ratio = config.motor_ratio;
//   rp->model_type = config.model_type;

//   rp->motor_nonexchange_flag = (!config.motor4_exchange_flag << 3) |
//                                (!config.motor3_exchange_flag << 2) |
//                                (!config.motor2_exchange_flag << 1) |
//                                (!config.motor1_exchange_flag);
//   rp->encoder_nonexchange_flag = (!config.encoder4_exchange_flag << 3) |
//                                  (!config.encoder3_exchange_flag << 2) |
//                                  (!config.encoder2_exchange_flag << 1) |
//                                  (!config.encoder1_exchange_flag);
//   Data_holder::dump_params(rp);

//   param_update_flag_ = true;
// }

// bool BaseDriverConfig::get_param_update_flag() {
//   bool tmp = param_update_flag;
//   param_update_flag_ = false;

//   return tmp;
// }
// #endif
