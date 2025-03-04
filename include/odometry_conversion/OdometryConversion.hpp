#pragma once

#include "Eigen/Eigen"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace odometry_conversion {
class OdometryConversion : public rclcpp::Node {
 public:
  OdometryConversion(const rclcpp::NodeOptions & options);

 private:
  void odometryInCallback(const nav_msgs::msg::Odometry::SharedPtr odomIn);

  Eigen::Matrix4d toHomTransform(const geometry_msgs::msg::Transform& transform) const;
  Eigen::Matrix4d toHomTransform(const geometry_msgs::msg::Pose& transform) const;
  geometry_msgs::msg::Transform fromHomTransform(const Eigen::Matrix4d& homTransform) const;
  geometry_msgs::msg::Pose fromHomTransformToPose(const Eigen::Matrix4d& homTransform) const;

  std::string inOdomFrame_ = "camera_pose_frame";
  std::string inSensorFrame_ = "camera_pose_frame";
  std::string outOdomFrame_ = "tracking_camera_odom";
  std::string outBaseFrame_ = "base_link";
  std::string inOdomTopic_ = "/tracking_camera/odom/sample";
  std::string outOdomTopic_ = "/base_odom";
  bool odomChild_ = true;

  Eigen::Matrix4d odomTransformHom_;
  Eigen::Matrix4d sensorTransformHom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometryInSubscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> odomCameraOdomTransformPublisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odomPublisher_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener transformListener_;
};
}  // namespace odometry_conversion