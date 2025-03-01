#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <odometry_conversion/OdometryConversion.hpp>

using namespace odometry_conversion;

OdometryConversion::OdometryConversion(const rclcpp::NodeOptions & options) 
: rclcpp::Node("odometry_conversion", options), buffer_(this->get_clock()), transformListener_(buffer_) {
  
  // Declare and get parameters
  this->declare_parameter("in_odom_frame", inOdomFrame_);
  this->declare_parameter("out_odom_frame", outOdomFrame_);
  this->declare_parameter("in_sensor_frame", inSensorFrame_);
  this->declare_parameter("out_base_frame", outBaseFrame_);
  this->declare_parameter("in_odom_topic", inOdomTopic_);
  this->declare_parameter("out_odom_topic", outOdomTopic_);
  this->declare_parameter("is_odom_child", odomChild_);

  this->get_parameter("in_odom_frame", inOdomFrame_);
  this->get_parameter("out_odom_frame", outOdomFrame_);
  this->get_parameter("in_sensor_frame", inSensorFrame_);
  this->get_parameter("out_base_frame", outBaseFrame_);
  this->get_parameter("in_odom_topic", inOdomTopic_);
  this->get_parameter("out_odom_topic", outOdomTopic_);
  this->get_parameter("is_odom_child", odomChild_);

  // Lookup initial transforms
  auto sensorTransform = buffer_.lookupTransform(outBaseFrame_, inSensorFrame_, tf2::TimePointZero);
  sensorTransformHom_ = toHomTransform(sensorTransform.transform);

  auto odomTransform = buffer_.lookupTransform(outBaseFrame_, inOdomFrame_, tf2::TimePointZero);
  odomTransformHom_ = toHomTransform(odomTransform.transform);

  // Subscribers and Publishers
  odometryPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(outOdomTopic_, 10);
  odometryInSubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    inOdomTopic_, 10, std::bind(&OdometryConversion::odometryInCallback, this, std::placeholders::_1)
  );
}

Eigen::Matrix4d OdometryConversion::toHomTransform(const geometry_msgs::msg::Transform& transform) const {
  Eigen::Matrix4d homTransform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation = {transform.translation.x, transform.translation.y, transform.translation.z};
  homTransform.block<3, 1>(0, 3) = translation;

  Eigen::Quaterniond rotation = {transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w};
  homTransform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  return homTransform;
}

Eigen::Matrix4d OdometryConversion::toHomTransform(const geometry_msgs::msg::Pose& transform) const {
  Eigen::Matrix4d homTransform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation = {transform.position.x, transform.position.y, transform.position.z};
  homTransform.block<3, 1>(0, 3) = translation;

  Eigen::Quaterniond rotation = {transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w};
  homTransform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  return homTransform;
}

geometry_msgs::msg::Transform OdometryConversion::fromHomTransform(const Eigen::Matrix4d& homTransform) const {
  geometry_msgs::msg::Transform transform;
  Eigen::Vector3d translation = homTransform.block<3, 1>(0, 3);
  transform.translation.x = translation(0);
  transform.translation.y = translation(1);
  transform.translation.z = translation(2);

  Eigen::Quaterniond rotation(homTransform.block<3, 3>(0, 0));
  transform.rotation = tf2::toMsg(rotation);
  return transform;
}

geometry_msgs::msg::Pose OdometryConversion::fromHomTransformToPose(const Eigen::Matrix4d& homTransform) const {
  geometry_msgs::msg::Pose pose;
  Eigen::Vector3d translation = homTransform.block<3, 1>(0, 3);
  pose.position = tf2::toMsg(translation);

  Eigen::Quaterniond rotation(homTransform.block<3, 3>(0, 0));
  pose.orientation = tf2::toMsg(rotation);
  return pose;
}

void OdometryConversion::odometryInCallback(const nav_msgs::msg::Odometry::SharedPtr odomIn) {
  auto odomOut = std::make_shared<nav_msgs::msg::Odometry>();
  odomOut->header.stamp = odomIn->header.stamp;
  odomOut->header.frame_id = outOdomFrame_;
  odomOut->child_frame_id = outBaseFrame_;

  Eigen::Matrix4d inHom = toHomTransform(odomIn->pose.pose);
  Eigen::Matrix4d outHom = odomTransformHom_ * inHom * sensorTransformHom_.inverse();
  odomOut->pose.pose = fromHomTransformToPose(outHom);

  // Angular velocity transformation
  Eigen::Vector3d inRotVel = {odomIn->twist.twist.angular.x, odomIn->twist.twist.angular.y, odomIn->twist.twist.angular.z};
  Eigen::Vector3d outRotVel = sensorTransformHom_.block<3, 3>(0, 0) * inRotVel;

  // Linear velocity transformation
  Eigen::Vector3d inLinVel = {odomIn->twist.twist.linear.x, odomIn->twist.twist.linear.y, odomIn->twist.twist.linear.z};
  Eigen::Vector3d inLinVelOutFrame = sensorTransformHom_.block<3, 3>(0, 0) * inLinVel;
  Eigen::Vector3d outLinVel = inLinVelOutFrame + outRotVel.cross(sensorTransformHom_.inverse().block<3, 1>(0, 3));

  odomOut->twist.twist.angular.x = outRotVel(0);
  odomOut->twist.twist.angular.y = outRotVel(1);
  odomOut->twist.twist.angular.z = outRotVel(2);
  odomOut->twist.twist.linear.x = outLinVel(0);
  odomOut->twist.twist.linear.y = outLinVel(1);
  odomOut->twist.twist.linear.z = outLinVel(2);

  // Copy covariance
  odomOut->pose.covariance = odomIn->pose.covariance;
  odomOut->twist.covariance = odomIn->twist.covariance;

  // Publish the converted odometry
  odometryPublisher_->publish(*odomOut);

  // Publish transformed odometry frame via TF2
  geometry_msgs::msg::TransformStamped odomTransform;
  odomTransform.header.stamp = odomIn->header.stamp;

  if (odomChild_) {
    odomTransform.header.frame_id = outBaseFrame_;
    odomTransform.child_frame_id = outOdomFrame_;
    odomTransform.transform = fromHomTransform(outHom.inverse());
  } else {
    odomTransform.header.frame_id = outOdomFrame_;
    odomTransform.child_frame_id = outBaseFrame_;
    odomTransform.transform = fromHomTransform(outHom);
  }

  odomPublisher_->sendTransform(odomTransform);
}
