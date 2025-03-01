#include <odometry_conversion/OdometryConversion.hpp>

using namespace odometry_conversion;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  std::shared_ptr<OdometryConversion> odometryConversion = std::make_shared<OdometryConversion>(node_options);
  rclcpp::spin(odometryConversion);
  return 0;
}
