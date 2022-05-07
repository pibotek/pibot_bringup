#include "pibot_bringup/base_driver.h"
#include "pibot_bringup/data_holder.h"
#include "pibot_bringup/dataframe.h"
#include "pibot_bringup/serial_transport.h"
#include "pibot_bringup/simple_dataframe_master.h"

#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

static double GRAVITY = -9.81;                       // [m/s/s]
static double MILIGAUSS_TO_TESLA_SCALE = 0.0000001;  // From Milligauss [mG] to Tesla [T]

BaseDriver::BaseDriver()
    : Node("pibot_dirver"),
      use_accelerometer_{true},
      use_gyroscope_{true},
      use_magnetometer_{false},
      use_mag_msg_{true},
      perform_calibration_{true},
      is_calibrated_{false},
      calibration_samples_{500} {
  // init config
  config_.init(this);

  trans_ = std::make_shared<SerialTransport>(config_.port, config_.baudrate);

  frame_ = std::make_shared<SimpleDataframe>(trans_);

  RCLCPP_INFO(this->get_logger(), "BaseDriver startup...");
  if (trans_->init()) {
    RCLCPP_INFO(this->get_logger(), "connected to main board");
  } else {
    RCLCPP_ERROR(this->get_logger(), "can't connect to main board, please check the usb connection or baudrate!");
    return;
  }

  frame_->init();

  for (int i = 0; i < 3; i++) {
    RCLCPP_INFO(this->get_logger(), "get version");
    if (frame_->interact(ID_GET_VERSION))
      break;

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
  }

  RCLCPP_INFO(this->get_logger(), "robot version:%s build time:%s",
              DataHolder::get()->firmware_info.version,
              DataHolder::get()->firmware_info.time);

  init_cmd_odom();

  init_pid_debug();

  read_param();

  if (config_.use_imu) {
    init_imu();
  }

  RCLCPP_INFO(this->get_logger(), "init ok");
}

BaseDriver::~BaseDriver() {
}

void BaseDriver::init_cmd_odom() {
  frame_->interact(ID_INIT_ODOM);

  RCLCPP_INFO(this->get_logger(), "subscribe cmd topic on [%s]", config_.cmd_vel_topic.c_str());

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(config_.cmd_vel_topic,
                                                                      10,
                                                                      std::bind(&BaseDriver::cmd_vel_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "advertise odom topic on [%s]", config_.odom_topic.c_str());
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(config_.odom_topic, 10);

  odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // init odom_tf
  odom_tf_.header.frame_id = config_.odom_frame;
  odom_tf_.child_frame_id = config_.base_frame;
  odom_tf_.transform.translation.z = 0;

  // init odom
  odom_msg_.header.frame_id = config_.odom_frame;
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.child_frame_id = config_.base_frame;
  odom_msg_.twist.twist.linear.y = 0;

  if (!config_.publish_tf) {
    for (unsigned int i = 0; i < odom_msg_.pose.covariance.size(); ++i) {
      odom_msg_.pose.covariance[i] = 0.0;
      odom_msg_.twist.covariance[i] = 0.0;
    }

    odom_msg_.pose.covariance[0] = 0.001;
    odom_msg_.pose.covariance[7] = 0.001;
    odom_msg_.pose.covariance[14] = 0.001;
    odom_msg_.pose.covariance[21] = 0.001;
    odom_msg_.pose.covariance[28] = 0.001;
    odom_msg_.pose.covariance[35] = 0.03;
    odom_msg_.twist.covariance[0] = 0.001;
    odom_msg_.twist.covariance[7] = 0.001;
    odom_msg_.twist.covariance[14] = 0.001;
    odom_msg_.twist.covariance[21] = 0.001;
    odom_msg_.twist.covariance[28] = 0.001;
    odom_msg_.twist.covariance[35] = 0.03;
  }

  need_update_speed_ = false;
}

void BaseDriver::init_pid_debug() {
  if (config_.out_pid_debug_enable) {
    const char* input_topic_name[MAX_MOTOR_COUNT] = {"motor1_input", "motor2_input", "motor3_input", "motor4_input"};
    const char* output_topic_name[MAX_MOTOR_COUNT] = {"motor1_output", "motor2_output", "motor3_output", "motor4_output"};
    for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
      pid_input_pub_[i] = this->create_publisher<std_msgs::msg::Int32>(input_topic_name[i], 10);
      pid_output_pub_[i] = this->create_publisher<std_msgs::msg::Int32>(output_topic_name[i], 10);
    }
  }
}

void BaseDriver::init_imu() {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  rclcpp::Parameter param;

  this->declare_parameter("imu/use_accelerometer", use_accelerometer_);
  if (get_parameter("imu/use_accelerometer", param)) {
    use_accelerometer_ = param.as_bool();
  }

  this->declare_parameter("imu/use_gyroscope", use_gyroscope_);
  if (get_parameter("imu/use_gyroscope", param)) {
    use_gyroscope_ = param.as_bool();
  }

  this->declare_parameter("imu/use_magnetometer", use_magnetometer_);
  if (get_parameter("imu/use_magnetometer", param)) {
    use_magnetometer_ = param.as_bool();
  }

  this->declare_parameter("imu/perform_calibration", perform_calibration_);
  if (get_parameter("imu/perform_calibration", param)) {
    perform_calibration_ = param.as_bool();
  }

  // 创建一个服务,用于接收运行中的校准请求, 可以ros2命令查看
  imu_cal_srv_ = create_service<std_srvs::srv::Empty>("imu/calibrate_imu",
                                                      std::bind(&BaseDriver::calibrateCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  this->declare_parameter("imu/accelerometer_bias", acceleration_bias_);
  this->declare_parameter("imu/gyroscope_bias", gyroscope_bias_);

  // 是否使能加速度计或陀螺仪
  if (use_accelerometer_ || use_gyroscope_) {
    // 根据配置决定是否进行校准
    rclcpp::Parameter accelerometer_bias_param, gyroscope_bias_param;
    if (!get_parameter("imu/accelerometer_bias", accelerometer_bias_param) || !get_parameter("imu/gyroscope_bias", gyroscope_bias_param)) {
      RCLCPP_WARN(this->get_logger(), "IMU calibration NOT found.");
      is_calibrated_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "IMU calibration found.");
      acceleration_bias_ = accelerometer_bias_param.as_double_array();
      gyroscope_bias_ = gyroscope_bias_param.as_double_array();

      RCLCPP_INFO(this->get_logger(), "acceleration_bias: %f %f %f, gyroscope_bias: %f %f %f",
                  acceleration_bias_[0], acceleration_bias_[1], acceleration_bias_[2],
                  gyroscope_bias_[0], gyroscope_bias_[1], gyroscope_bias_[2]);

      is_calibrated_ = true;
    }

    declare_parameter<uint16_t>("imu/calibration_samples", 500, descriptor);
    // 校准采样点数
    if (get_parameter("imu/calibration_samples", param)) {
      calibration_samples_ = param.get_value<int>();
    }

    // 转换为ROS标准的sensor_msgs::Imu类型的topic
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());

    imu_msg_.header.frame_id = config_.base_frame;

    // 线加速度协方差
    declare_parameter<double>("imu/linear_acc_stdev", 0.0, descriptor);
    if (get_parameter("imu/linear_acc_stdev", param)) {
      const double linear_acceleration_stdev = param.get_value<double>();
      const double linear_acceleration_variance = linear_acceleration_stdev * linear_acceleration_stdev;
      imu_msg_.linear_acceleration_covariance[0] = linear_acceleration_variance;
      imu_msg_.linear_acceleration_covariance[4] = linear_acceleration_variance;
      imu_msg_.linear_acceleration_covariance[8] = linear_acceleration_variance;
    }

    // 角速度协方差
    declare_parameter<double>("imu/angular_vel_stdev", 0.0, descriptor);
    if (get_parameter("imu/angular_vel_stdev", param)) {
      const double angular_velocity_stdev = param.get_value<double>();
      const double angular_velocity_variance = angular_velocity_stdev * angular_velocity_stdev;
      imu_msg_.angular_velocity_covariance[0] = angular_velocity_variance;
      imu_msg_.angular_velocity_covariance[4] = angular_velocity_variance;
      imu_msg_.angular_velocity_covariance[8] = angular_velocity_variance;
    }
  }

  // // 是否使能磁力计
  if (use_magnetometer_) {
    // Magnetometer calibration values.

    declare_parameter<double>("mag/x/min", -0.000078936, descriptor);
    if (get_parameter("mag/x/min", param)) {
      mag_x_min_ = param.get_value<double>();
    }

    declare_parameter<double>("mag/x/max", 0.000077924, descriptor);
    if (get_parameter("mag/x/max", param)) {
      mag_x_max_ = param.get_value<double>();
    }

    declare_parameter<double>("mag/y/min", -0.000075532, descriptor);
    if (get_parameter("mag/y/min", param)) {
      mag_y_min_ = param.get_value<double>();
    }

    declare_parameter<double>("mag/y/max", 0.000076360, descriptor);
    if (get_parameter("mag/y/max", param)) {
      mag_y_max_ = param.get_value<double>();
    }

    declare_parameter<double>("mag/z/min", -0.000079948, descriptor);
    if (get_parameter("mag/z/min", param)) {
      mag_z_min_ = param.get_value<double>();
    }

    declare_parameter<double>("mag/z/max", 0.000064216, descriptor);
    if (get_parameter("mag/z/max", param)) {
      mag_z_max_ = param.get_value<double>();
    }

    this->declare_parameter("imu/use_mag_msg", use_mag_msg_);
    if (get_parameter("imu/use_mag_msg", param)) {
      use_mag_msg_ = param.as_bool();
    }

    if (use_mag_msg_) {
      // 转换为ROS标准的sensor_msgs::MagneticField类型的topic
      mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());

      declare_parameter<double>("imu/magnetic_field_stdev", 0.0, descriptor);
      if (get_parameter("imu/angular_vel_stdev", param)) {
        const double magnetic_field_stdev = param.get_value<double>();
        const double magnetic_field_variance = magnetic_field_stdev * magnetic_field_stdev;
        mag_msg_.magnetic_field_covariance[0] = magnetic_field_variance;
        mag_msg_.magnetic_field_covariance[4] = magnetic_field_variance;
        mag_msg_.magnetic_field_covariance[8] = magnetic_field_variance;
      }

      mag_msg_.header.frame_id = config_.base_frame;
    }
  }
}

void BaseDriver::read_param() {
  Robot_parameter* param = &DataHolder::get()->parameter;
  memset(param, 0, sizeof(Robot_parameter));

  frame_->interact(ID_GET_ROBOT_PARAMTER);

  DataHolder::dump_params(param);

  config_.SetRobotParameters(this, &DataHolder::get()->parameter);
}

void BaseDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr vel_cmd) {
  RCLCPP_INFO(this->get_logger(), "cmd_vel:[%.3f %.3f %.3f]", vel_cmd->linear.x, vel_cmd->linear.y, vel_cmd->angular.z);

  DataHolder::get()->velocity.v_liner_x = vel_cmd->linear.x * 100;
  DataHolder::get()->velocity.v_liner_y = vel_cmd->linear.y * 100;
  DataHolder::get()->velocity.v_angular_z = vel_cmd->angular.z * 100;

  need_update_speed_ = true;
}

void BaseDriver::work_loop() {
  rclcpp::WallRate loop(100);
  rclcpp::Node::SharedPtr node(this);
  while (rclcpp::ok()) {
    update_param();

    update_odom();

    update_pid_debug();

    update_speed();

    if (config_.use_imu) {
      if (DataHolder::get()->parameter.imu_type == IMU_TYPE_GY65 || DataHolder::get()->parameter.imu_type == IMU_TYPE_GY85 || DataHolder::get()->parameter.imu_type == IMU_TYPE_GY87) {
        update_imu();
      }
    }

    rclcpp::spin_some(node);

    loop.sleep();
  }
}

void BaseDriver::update_param() {
#ifdef USE_DYNAMIC_RECONFIG
  if (config_.get_param_update_flag()) {
    frame_->interact(ID_SET_ROBOT_PARAMTER);
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(200ms);
  }
#endif
}

void BaseDriver::update_odom() {
  frame_->interact(ID_GET_ODOM);

  auto now = this->get_clock()->now();

  float x = DataHolder::get()->odom.x * 0.01;
  float y = DataHolder::get()->odom.y * 0.01;
  float th = DataHolder::get()->odom.yaw * 0.01;

  float vxy = DataHolder::get()->odom.v_liner_x * 0.01;
  float vth = DataHolder::get()->odom.v_angular_z * 0.01;

  // RCLCPP_INFO(this->get_logger(), "odom: x=%.2f y=%.2f th=%.2f vxy=%.2f vth=%.2f", x, y ,th, vxy,vth);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, th);

  // publish_tf
  if (config_.publish_tf) {
    odom_tf_.header.stamp = now;
    odom_tf_.transform.translation.x = x;
    odom_tf_.transform.translation.y = y;

    odom_tf_.transform.rotation.x = q.x();
    odom_tf_.transform.rotation.y = q.y();
    odom_tf_.transform.rotation.z = q.z();
    odom_tf_.transform.rotation.w = q.w();

    odom_broadcaster_->sendTransform(odom_tf_);
  }

  // publish the message
  odom_msg_.header.stamp = now;
  odom_msg_.pose.pose.position.x = x;
  odom_msg_.pose.pose.position.y = y;
  odom_msg_.pose.pose.orientation.x = q.x();
  odom_msg_.pose.pose.orientation.y = q.y();
  odom_msg_.pose.pose.orientation.z = q.z();
  odom_msg_.pose.pose.orientation.w = q.w();
  odom_msg_.twist.twist.linear.x = vxy;
  odom_msg_.twist.twist.angular.z = vth;

  odom_pub_->publish(odom_msg_);
}

void BaseDriver::update_speed() {
  if (need_update_speed_) {
    RCLCPP_INFO(this->get_logger(), "update_speed");
    need_update_speed_ = !(frame_->interact(ID_SET_VELOCITY));
  }
}

void BaseDriver::update_pid_debug() {
  if (config_.out_pid_debug_enable) {
    frame_->interact(ID_GET_PID_DATA);

    for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
      pid_input_msg_[i].data = DataHolder::get()->pid_data.input[i];
      pid_output_msg_[i].data = DataHolder::get()->pid_data.output[i];

      pid_input_pub_[i]->publish(pid_input_msg_[i]);
      pid_output_pub_[i]->publish(pid_output_msg_[i]);
    }
  }
}

void BaseDriver::update_imu() {
  if (!use_accelerometer_ && !use_gyroscope_ && !use_magnetometer_) {
    return;
  }

  frame_->interact(ID_GET_IMU_DATA);
  if (!use_accelerometer_) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Accelerometer not found!");
  }
  if (!use_gyroscope_) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Gyroscope not found!");
  }
  if (!use_magnetometer_) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Magnetometer not found!");
  }

  if (perform_calibration_ || !is_calibrated_) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");

    static int taken_samples;

    if (taken_samples < calibration_samples_) {
      acceleration_bias_[0] += DataHolder::get()->imu_data[0];
      acceleration_bias_[1] += DataHolder::get()->imu_data[1];
      acceleration_bias_[2] += DataHolder::get()->imu_data[2];

      gyroscope_bias_[0] += DataHolder::get()->imu_data[3];
      gyroscope_bias_[1] += DataHolder::get()->imu_data[4];
      gyroscope_bias_[2] += DataHolder::get()->imu_data[5];

      taken_samples++;
    } else {
      acceleration_bias_[0] /= calibration_samples_;
      acceleration_bias_[1] /= calibration_samples_;
      acceleration_bias_[2] = acceleration_bias_[2] / calibration_samples_ + GRAVITY;

      gyroscope_bias_[0] /= calibration_samples_;
      gyroscope_bias_[1] /= calibration_samples_;
      gyroscope_bias_[2] /= calibration_samples_;

      RCLCPP_INFO(this->get_logger(), "Calibrating accelerometer and gyroscope complete.");
      RCLCPP_INFO(this->get_logger(), "Bias values can be saved for reuse.");
      RCLCPP_INFO(this->get_logger(), "   Accelerometer: x: %f, y: %f, z: %f", acceleration_bias_[0], acceleration_bias_[1], acceleration_bias_[2]);
      RCLCPP_INFO(this->get_logger(), "   Gyroscope: x: %f, y: %f, z: %f", gyroscope_bias_[0], gyroscope_bias_[1], gyroscope_bias_[2]);

      set_parameter(rclcpp::Parameter{"imu/accelerometer_bias", acceleration_bias_});
      set_parameter(rclcpp::Parameter{"imu/gyroscope_bias", gyroscope_bias_});

      is_calibrated_ = true;
      perform_calibration_ = false;
      taken_samples = 0;
    }
  } else {
    auto now = this->get_clock()->now();
    if (use_accelerometer_ || use_gyroscope_) {
      imu_msg_.header.stamp = now;

      imu_msg_.angular_velocity.x = DataHolder::get()->imu_data[3] - gyroscope_bias_[0];
      imu_msg_.angular_velocity.y = DataHolder::get()->imu_data[4] - gyroscope_bias_[1];
      imu_msg_.angular_velocity.z = DataHolder::get()->imu_data[5] - gyroscope_bias_[2];

      imu_msg_.linear_acceleration.x = DataHolder::get()->imu_data[0] - acceleration_bias_[0];
      imu_msg_.linear_acceleration.y = DataHolder::get()->imu_data[1] - acceleration_bias_[1];
      imu_msg_.linear_acceleration.z = DataHolder::get()->imu_data[2] - acceleration_bias_[2];

      imu_pub_->publish(imu_msg_);
    }

    if (use_magnetometer_) {
      if (use_mag_msg_) {
        mag_msg_.header.stamp = now;

        mag_msg_.magnetic_field.x = (DataHolder::get()->imu_data[6] * MILIGAUSS_TO_TESLA_SCALE - (mag_x_max_ - mag_x_min_) / 2 - mag_x_min_);
        mag_msg_.magnetic_field.y = (DataHolder::get()->imu_data[7] * MILIGAUSS_TO_TESLA_SCALE - (mag_y_max_ - mag_y_min_) / 2 - mag_y_min_);
        mag_msg_.magnetic_field.z = (DataHolder::get()->imu_data[8] * MILIGAUSS_TO_TESLA_SCALE - (mag_z_max_ - mag_z_min_) / 2 - mag_z_min_);

        mag_pub_->publish(mag_msg_);
      }
    }
  }
}

void BaseDriver::calibrateCallback(const std::shared_ptr<rmw_request_id_t> __attribute__((unused)) request_header,
                                   const std::shared_ptr<std_srvs::srv::Empty::Request> __attribute__((unused)) request,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> __attribute__((unused)) response) {
  RCLCPP_INFO(this->get_logger(), "Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");
  perform_calibration_ = true;
}