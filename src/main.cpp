#include "pibot_bringup/base_driver.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto driver = std::make_shared<BaseDriver>();
  driver->work_loop();

  rclcpp::shutdown();

  return 0;
}