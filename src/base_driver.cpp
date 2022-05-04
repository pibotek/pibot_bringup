#include "pibot_bringup/base_driver.h"
#include "pibot_bringup/data_holder.h"
#include "pibot_bringup/dataframe.h"
#include "pibot_bringup/serial_transport.h"
#include "pibot_bringup/simple_dataframe_master.h"

#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

// static double GRAVITY = -9.81;                       // [m/s/s]
// static double MILIGAUSS_TO_TESLA_SCALE = 0.0000001;  // From Milligauss [mG] to Tesla [T]

BaseDriver::BaseDriver()
    : Node("pibot_dirver")
//                         , use_accelerometer(true)
//                         , use_gyroscope(true)
//                         , use_magnetometer(true)
//                         , use_mag_msg(true)
//                         , perform_calibration(true)
//                         , is_calibrated(false)
{
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
  odom_tf.header.frame_id = config_.odom_frame;
  odom_tf.child_frame_id = config_.base_frame;
  odom_tf.transform.translation.z = 0;

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
  // pn.param<bool>("imu/use_accelerometer", use_accelerometer, use_accelerometer);
  // pn.param<bool>("imu/use_gyroscope", use_gyroscope, use_gyroscope);
  // pn.param<bool>("imu/use_magnetometer", use_magnetometer, use_magnetometer);
  // pn.param<bool>("imu/perform_calibration", perform_calibration, perform_calibration);

  // // 创建一个服务,用于接收运行中的校准请求, 可以rosservice命令查看
  // imu_cal_srv = nh.advertiseService("imu/calibrate_imu", &BaseDriver::calibrateCallback, this);

  // // 是否使能加速度计或陀螺仪
  // if (use_accelerometer || use_gyroscope) {
  // 	// 根据配置决定是否进行校准
  // 	if (!pn.getParam("imu/accelerometer_bias", acceleration_bias) ||
  // 		!pn.getParam("imu/gyroscope_bias", gyroscope_bias)) {
  // 		ROS_WARN("IMU calibration NOT found.");
  // 		is_calibrated = false;
  // 	} else {
  // 		ROS_INFO("IMU calibration found.");
  // 		pn.getParam("imu/accelerometer_bias", acceleration_bias);
  // 		pn.getParam("imu/gyroscope_bias", gyroscope_bias);
  // 		is_calibrated = true;
  // 	}

  // 	// 校准采样点数
  // 	pn.param<int>("imu/calibration_samples", calibration_samples, 500);

  // 	// 转换为ROS标准的sensor_msgs::Imu类型的topic
  // 	imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  // 	// 线加速度协方差
  // 	pn.param<double>("imu/linear_acc_stdev", linear_acc_stdev, 0.0);
  // 	fillRowMajor(linear_acc_covar, linear_acc_stdev);

  // 	// 角速度协方差
  // 	pn.param<double>("imu/angular_vel_stdev", angular_vel_stdev, 0.0);
  // 	fillRowMajor(angular_vel_covar, angular_vel_stdev);
  // }

  // // 是否使能磁力计
  // if (use_magnetometer) {
  // 	// Magnetometer calibration values.
  // 	pn.param<double>("mag/x/min", mag_x_min, -0.000078936);
  // 	pn.param<double>("mag/x/max", mag_x_max,  0.000077924);
  // 	pn.param<double>("mag/y/min", mag_y_min, -0.000075532);
  // 	pn.param<double>("mag/y/max", mag_y_max,  0.000076360);
  // 	pn.param<double>("mag/z/min", mag_z_min, -0.000079948);
  // 	pn.param<double>("mag/z/max", mag_z_max,  0.000064216);

  // 	pn.param<bool>("imu/use_mag_msg", use_mag_msg, use_mag_msg);

  // 	if (use_mag_msg) {
  // 		// 转换为ROS标准的sensor_msgs::MagneticField类型的topic
  // 		mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 5);

  // 		pn.param<double>("imu/magnetic_field_stdev", magnetic_field_stdev, 0.0);
  // 		fillRowMajor(magnetic_field_covar, magnetic_field_stdev);
  // 	} else {
  // 		// 使用Vector3Stamped类型的topic
  // 		mag_pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 5);
  // 	}
  // }
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
    odom_tf.header.stamp = now;
    odom_tf.transform.translation.x = x;
    odom_tf.transform.translation.y = y;

    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    odom_broadcaster_->sendTransform(odom_tf);
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
  frame_->interact(ID_GET_IMU_DATA);
  // if (!use_accelerometer) {
  // 	ROS_ERROR_ONCE("Accelerometer not found!");
  // }
  // if (!use_gyroscope) {
  // 	ROS_ERROR_ONCE("Gyroscope not found!");
  // }
  // if (!use_magnetometer) {
  // 	ROS_ERROR_ONCE("Magnetometer not found!");
  // }

  // if (perform_calibration || !is_calibrated) {
  // 	ROS_WARN_ONCE("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");

  // 	static int taken_samples;

  // 	if (taken_samples < calibration_samples) {
  // 		acceleration_bias["x"] += DataHolder::get()->imu_data[0];
  // 		acceleration_bias["y"] += DataHolder::get()->imu_data[1];
  // 		acceleration_bias["z"] += DataHolder::get()->imu_data[2];

  // 		gyroscope_bias["x"] += DataHolder::get()->imu_data[3];
  // 		gyroscope_bias["y"] += DataHolder::get()->imu_data[4];
  // 		gyroscope_bias["z"] += DataHolder::get()->imu_data[5];

  // 		taken_samples++;
  // 	} else {
  // 		acceleration_bias["x"] /= calibration_samples;
  // 		acceleration_bias["y"] /= calibration_samples;
  // 		acceleration_bias["z"] = acceleration_bias["z"] / calibration_samples + GRAVITY;

  // 		gyroscope_bias["x"] /= calibration_samples;
  // 		gyroscope_bias["y"] /= calibration_samples;
  // 		gyroscope_bias["z"] /= calibration_samples;

  // 		ROS_INFO("Calibrating accelerometer and gyroscope complete.");
  // 		ROS_INFO("Bias values can be saved for reuse.");
  // 		ROS_INFO("Accelerometer: x: %f, y: %f, z: %f", acceleration_bias["x"], acceleration_bias["y"], acceleration_bias["z"]);
  // 		ROS_INFO("Gyroscope: x: %f, y: %f, z: %f", gyroscope_bias["x"], gyroscope_bias["y"], gyroscope_bias["z"]);

  // 		nh.setParam("imu/accelerometer_bias", acceleration_bias);
  // 		nh.setParam("imu/gyroscope_bias", gyroscope_bias);

  // 		is_calibrated = true;
  // 		perform_calibration = false;
  // 		taken_samples = 0;
  // 	}
  // } else {
  // 	if (use_accelerometer || use_gyroscope) {
  // 		sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
  // 		imu_msg->header.stamp = ros::Time::now();
  //         imu_msg->header.frame_id = bdg.base_frame;

  // 		imu_msg->angular_velocity.x = DataHolder::get()->imu_data[3] - gyroscope_bias["x"];
  // 		imu_msg->angular_velocity.y = DataHolder::get()->imu_data[4] - gyroscope_bias["y"];
  // 		imu_msg->angular_velocity.z = DataHolder::get()->imu_data[5] - gyroscope_bias["z"];
  // 		imu_msg->orientation_covariance = angular_vel_covar;

  // 		imu_msg->linear_acceleration.x = DataHolder::get()->imu_data[0] - acceleration_bias["x"];
  // 		imu_msg->linear_acceleration.y = DataHolder::get()->imu_data[1] - acceleration_bias["y"];
  // 		imu_msg->linear_acceleration.z = DataHolder::get()->imu_data[2] - acceleration_bias["z"];
  // 		imu_msg->linear_acceleration_covariance = linear_acc_covar;

  // 		imu_pub.publish(imu_msg);
  // 	}

  // 	if (use_magnetometer) {
  // 		if (use_mag_msg) {
  // 			sensor_msgs::MagneticFieldPtr mag_msg = boost::make_shared<sensor_msgs::MagneticField>();
  //             mag_msg->header.stamp = ros::Time::now();
  //             mag_msg->header.frame_id = bdg.base_frame;

  // 			mag_msg->magnetic_field.x = (DataHolder::get()->imu_data[6] * MILIGAUSS_TO_TESLA_SCALE - (mag_x_max - mag_x_min) / 2 - mag_x_min);
  // 			mag_msg->magnetic_field.y = (DataHolder::get()->imu_data[7] * MILIGAUSS_TO_TESLA_SCALE - (mag_y_max - mag_y_min) / 2 - mag_y_min);
  // 			mag_msg->magnetic_field.z = (DataHolder::get()->imu_data[8] * MILIGAUSS_TO_TESLA_SCALE - (mag_z_max - mag_z_min) / 2 - mag_z_min);
  // 			mag_msg->magnetic_field_covariance = magnetic_field_covar;

  // 			mag_pub.publish(mag_msg);
  // 		} else {
  // 			geometry_msgs::Vector3StampedPtr mag_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
  //             mag_msg->header.stamp = ros::Time::now();
  //             mag_msg->header.frame_id = bdg.base_frame;

  // 			mag_msg->vector.x = (DataHolder::get()->imu_data[6] - (mag_x_max - mag_x_min) / 2 - mag_x_min) * MILIGAUSS_TO_TESLA_SCALE;
  // 			mag_msg->vector.y = (DataHolder::get()->imu_data[7] - (mag_y_max - mag_y_min) / 2 - mag_y_min) * MILIGAUSS_TO_TESLA_SCALE;
  // 			mag_msg->vector.z = (DataHolder::get()->imu_data[8] - (mag_z_max - mag_z_min) / 2 - mag_z_min) * MILIGAUSS_TO_TESLA_SCALE;

  // 			mag_pub.publish(mag_msg);
  // 		}
  // 	}
  // }
}

// bool BaseDriver::calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
// {
// ROS_WARN("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");
// perform_calibration = true;
// return true;
// }

// void BaseDriver::fillRowMajor(boost::array<double, 9> & covar, double stdev)
// {
// std::fill(covar.begin(), covar.end(), 0.0);
// covar[0] = pow(stdev, 2);  // X(roll)
// covar[4] = pow(stdev, 2);  // Y(pitch)
// covar[8] = pow(stdev, 2);  // Z(yaw)
// }
