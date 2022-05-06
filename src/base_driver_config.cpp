#include "pibot_bringup/base_driver_config.h"
#include <bitset>
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

  node->declare_parameter<bool>("use_imu", true, descriptor);

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

  use_imu = node->get_parameter("use_imu").as_bool();

  base_frame = node->get_parameter("base_frame").as_string();
  odom_frame = node->get_parameter("odom_frame").as_string();

  publish_tf = node->get_parameter("publish_tf").as_bool();

  out_pid_debug_enable = node->get_parameter("out_pid_debug_enable").as_bool();
  RCLCPP_INFO(node->get_logger(), "out_pid_debug_enable:%d", out_pid_debug_enable);

  // topic name param
  cmd_vel_topic = node->get_parameter("cmd_vel_topic").as_string();
  odom_topic = node->get_parameter("odom_topic").as_string();
  freq = node->get_parameter("freq").as_int();

  // udeclare static parameter (load on init)
  // node->undeclare_parameter("port");
  // node->undeclare_parameter("baudrate");
  // node->undeclare_parameter("base_frame");
  // node->undeclare_parameter("odom_frame");
  // node->undeclare_parameter("publish_tf");
  // node->undeclare_parameter("out_pid_debug_enable");
  // node->undeclare_parameter("cmd_vel_topic");
  // node->undeclare_parameter("odom_topic");
  // node->undeclare_parameter("freq");

#ifdef USE_DYNAMIC_RECONFIG
#endif
}

void BaseDriverConfig::SetRobotParameters(rclcpp::Node* node, Robot_parameter* rp) {
#ifdef USE_DYNAMIC_RECONFIG
  static bool f = false;
  if (!f) {
    f = true;

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "";
    descriptor.name = "name";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = 255;
    descriptor.integer_range[0].step = 1;
    node->declare_parameter<int8_t>("model_type", rp->model_type, descriptor);

    node->declare_parameter<bool>("motor1_exchange_flag", (~rp->motor_nonexchange_flag & MOTOR_ENCODER_1_FLAG));
    node->declare_parameter<bool>("motor2_exchange_flag", (~rp->motor_nonexchange_flag & MOTOR_ENCODER_2_FLAG) >> 1);
    node->declare_parameter<bool>("motor3_exchange_flag", (~rp->motor_nonexchange_flag & MOTOR_ENCODER_3_FLAG) >> 2);
    node->declare_parameter<bool>("motor4_exchange_flag", (~rp->motor_nonexchange_flag & MOTOR_ENCODER_4_FLAG) >> 3);

    node->declare_parameter<bool>("encoder1_exchange_flag", (~rp->encoder_nonexchange_flag & MOTOR_ENCODER_1_FLAG));
    node->declare_parameter<bool>("encoder2_exchange_flag", (~rp->encoder_nonexchange_flag & MOTOR_ENCODER_2_FLAG) >> 1);
    node->declare_parameter<bool>("encoder3_exchange_flag", (~rp->encoder_nonexchange_flag & MOTOR_ENCODER_3_FLAG) >> 2);
    node->declare_parameter<bool>("encoder4_exchange_flag", (~rp->encoder_nonexchange_flag & MOTOR_ENCODER_4_FLAG) >> 3);

    descriptor.integer_range[0].from_value = 10;
    descriptor.integer_range[0].to_value = 1000;
    node->declare_parameter<uint16_t>("wheel_diameter", rp->wheel_diameter, descriptor);

    descriptor.integer_range[0].from_value = 10;
    descriptor.integer_range[0].to_value = 1000;
    node->declare_parameter<uint16_t>("wheel_track", rp->wheel_track, descriptor);

    node->declare_parameter<uint16_t>("encoder_resolution", rp->encoder_resolution);
    node->declare_parameter<uint8_t>("do_pid_interval", rp->do_pid_interval);

    node->declare_parameter<uint16_t>("kp", rp->kp);
    node->declare_parameter<uint16_t>("ki", rp->ki);
    node->declare_parameter<uint16_t>("kd", rp->kd);
    node->declare_parameter<uint16_t>("ko", rp->ko);

    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = 1000;
    node->declare_parameter<uint16_t>("cmd_last_time", rp->cmd_last_time, descriptor);

    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = 200;
    node->declare_parameter<uint16_t>("max_v_liner_x", rp->max_v_liner_x, descriptor);

    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = 200;
    node->declare_parameter<uint16_t>("max_v_liner_y", rp->max_v_liner_y, descriptor);

    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = 5000;
    node->declare_parameter<uint16_t>("max_v_angular_z", rp->max_v_angular_z, descriptor);

    node->declare_parameter<uint8_t>("imu_type", rp->imu_type);

    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = 1000;
    node->declare_parameter<uint16_t>("motor_ratio", rp->motor_ratio, descriptor);

    callback_handle_ = node->add_on_set_parameters_callback(std::bind(&BaseDriverConfig::SetParametersCallback,
                                                                      this,
                                                                      std::placeholders::_1,
                                                                      node,
                                                                      rp));
  }
#endif
}

#ifdef USE_DYNAMIC_RECONFIG
rcl_interfaces::msg::SetParametersResult BaseDriverConfig::SetParametersCallback(const std::vector<rclcpp::Parameter>& parameters, rclcpp::Node* node, Robot_parameter* rp) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  bool f = false;
  for (auto& param : parameters) {
    if (param.get_name() == "motor1_exchange_flag") {
      RCLCPP_INFO(node->get_logger(), "++param %d", rp->motor_nonexchange_flag);
      std::bitset<8> val(rp->motor_nonexchange_flag);
      val[0] = !param.as_bool();
      rp->motor_nonexchange_flag = val.to_ulong();
      RCLCPP_INFO(node->get_logger(), "--param %d", rp->motor_nonexchange_flag);
    } else if (param.get_name() == "motor2_exchange_flag") {
      std::bitset<8> val(rp->motor_nonexchange_flag);
      val[1] = !param.as_bool();
      rp->motor_nonexchange_flag = val.to_ulong();
    } else if (param.get_name() == "motor3_exchange_flag") {
      std::bitset<8> val(rp->motor_nonexchange_flag);
      val[2] = !param.as_bool();
      rp->motor_nonexchange_flag = val.to_ulong();
    } else if (param.get_name() == "motor4_exchange_flag") {
      std::bitset<8> val(rp->motor_nonexchange_flag);
      val[3] = !param.as_bool();
      rp->motor_nonexchange_flag = val.to_ulong();
    } else if (param.get_name() == "encoder1_exchange_flag") {
      std::bitset<8> val(rp->encoder_nonexchange_flag);
      val[0] = !param.as_bool();
      rp->encoder_nonexchange_flag = val.to_ulong();
    } else if (param.get_name() == "encoder2_exchange_flag") {
      std::bitset<8> val(rp->encoder_nonexchange_flag);
      val[1] = !param.as_bool();
      rp->encoder_nonexchange_flag = val.to_ulong();
    } else if (param.get_name() == "encoder3_exchange_flag") {
      std::bitset<8> val(rp->encoder_nonexchange_flag);
      val[2] = !param.as_bool();
      rp->encoder_nonexchange_flag = val.to_ulong();
    } else if (param.get_name() == "encoder4_exchange_flag") {
      std::bitset<8> val(rp->encoder_nonexchange_flag);
      val[3] = !param.as_bool();
      rp->encoder_nonexchange_flag = val.to_ulong();
    } else if (param.get_name() == "model_type") {
      rp->model_type = param.as_int();
    } else if (param.get_name() == "wheel_diameter") {
      rp->wheel_diameter = param.as_int();
    } else if (param.get_name() == "wheel_track") {
      rp->wheel_track = param.as_int();
    } else if (param.get_name() == "encoder_resolution") {
      rp->encoder_resolution = param.as_int();
    } else if (param.get_name() == "do_pid_interval") {
      rp->do_pid_interval = param.as_int();
    } else if (param.get_name() == "kp") {
      rp->kp = param.as_int();
    } else if (param.get_name() == "ki") {
      rp->ki = param.as_int();
    } else if (param.get_name() == "kd") {
      rp->kd = param.as_int();
    } else if (param.get_name() == "ko") {
      rp->ko = param.as_int();
    } else if (param.get_name() == "cmd_last_time") {
      rp->cmd_last_time = param.as_int();
    } else if (param.get_name() == "max_v_liner_x") {
      rp->max_v_liner_x = param.as_int();
    } else if (param.get_name() == "max_v_liner_y") {
      rp->max_v_liner_y = param.as_int();
    } else if (param.get_name() == "max_v_angular_z") {
      rp->max_v_angular_z = param.as_int();
    } else if (param.get_name() == "imu_type") {
      rp->imu_type = param.as_int();
    } else if (param.get_name() == "motor_ratio") {
      rp->motor_ratio = param.as_int();
    } else {
      continue;
    }

    f = true;

    RCLCPP_INFO(node->get_logger(), "param %s update", param.get_name().c_str());
  }

  if (f)
    DataHolder::dump_params(rp);

  param_update_flag_ = true;
  return result;
}

bool BaseDriverConfig::get_param_update_flag() {
  bool tmp = param_update_flag_;
  param_update_flag_ = false;

  return tmp;
}
#endif
