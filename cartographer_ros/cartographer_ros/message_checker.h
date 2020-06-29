/**
 * Copyright 2020 Bossa Nova Robotics, Inc.
 * @author Ernesto Corbellini
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>

namespace cartographer_ros {

class MessageChecker {
 public:
  /**
   * Indicates if a pose is valid
   * @param pose Pose to check
   * @return True if the pose is valid
   */
  bool IsValidPose(const geometry_msgs::Pose& pose);

  /**
   * Checks a the type string corresponds to a laser scan message type
   * @param data_type String containing the data type
   * @return True if the message is a laser scan
   */
  bool IsPointDataType(const std::string& data_type);

  /**
   * Check if the accelerations for the IMU are in range
   * @param imu_message Message to check
   * @param error_msg Reference to a string to store the error message output
   * @return True if the values are ok
   */
  bool CheckImuMessage(const sensor_msgs::Imu& imu_message, std::string& error_msg);

  /**
   * Check if the odometry pose is valid
   * @param message Message to check
   * @param error_msg Reference to a string to store the error message output
   * @return True if the message is ok
   */
  bool CheckOdometryMessage(const nav_msgs::Odometry& message, std::string& error_msg);

  /**
   * Check if the transformation are well defined
   * @param message Message to check
   * @param error_msg Reference to a string to store the error message output
   * @return True if the message is ok
   */
  bool CheckTfMessage(const tf2_msgs::TFMessage& message, std::string& error_msg);

 private:
  const double kMinLinearAcceleration = 3.;
  const double kMaxLinearAcceleration = 30.;
  const std::set<std::string> kPointDataTypes = {
      std::string(
          ros::message_traits::DataType<sensor_msgs::PointCloud2>::value()),
      std::string(ros::message_traits::DataType<
                  sensor_msgs::MultiEchoLaserScan>::value()),
      std::string(
          ros::message_traits::DataType<sensor_msgs::LaserScan>::value())};
};

}  // namespace cartographer_ros