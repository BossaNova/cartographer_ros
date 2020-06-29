/**
 * Copyright 2020 Bossa Nova Robotics, Inc.
 * @author Ernesto Corbellini
 */

#include <string>
#include <sstream>

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"

#include "message_checker.h"


namespace cartographer_ros {

bool MessageChecker::IsPointDataType(const std::string& data_type) {
  return (kPointDataTypes.count(data_type) != 0);
}

bool MessageChecker::IsValidPose(const geometry_msgs::Pose& pose) {
  return ToRigid3d(pose).IsValid();
}

bool MessageChecker::CheckImuMessage(const sensor_msgs::Imu& imu_message, std::string& error_msg) {
  bool is_ok{true};
  std::stringstream msg_buf;
  auto linear_acceleration = ToEigen(imu_message.linear_acceleration);
  if (std::isnan(linear_acceleration.norm()) ||
      linear_acceleration.norm() < kMinLinearAcceleration ||
      linear_acceleration.norm() > kMaxLinearAcceleration) {
    is_ok = false;
    msg_buf << "frame_id " << imu_message.header.frame_id << " time "
            << imu_message.header.stamp.toNSec() << ": IMU linear acceleration is "
            << linear_acceleration.norm() << " m/s^2,"
            << " expected is [" << kMinLinearAcceleration << ", "
            << kMaxLinearAcceleration << "] m/s^2."
            << " (It should include gravity and be given in m/s^2.)"
            << " linear_acceleration " << linear_acceleration.transpose();
  }
  error_msg = msg_buf.str();
  return is_ok;
}

bool MessageChecker::CheckOdometryMessage(const nav_msgs::Odometry& message, std::string& error_msg) {
  bool is_ok{true};
  std::stringstream msg_buf;
  const auto& pose = message.pose.pose;
  if (!IsValidPose(pose)) {
    is_ok = false;
    msg_buf << "frame_id " << message.header.frame_id << " time "
            << message.header.stamp.toNSec()
            << ": Odometry pose is invalid."
            << " pose " << pose;
  }
  error_msg = msg_buf.str();
  return is_ok;
}

bool MessageChecker::CheckTfMessage(const tf2_msgs::TFMessage& message, std::string& error_msg) {
  bool is_ok{true};
  std::stringstream msg_buf;
  for (const auto& transform : message.transforms) {
    if (transform.header.frame_id == "map") {
      is_ok = false;
      msg_buf << "Input contains transform message from frame_id \""
              << transform.header.frame_id << "\" to child_frame_id \""
              << transform.child_frame_id
              << "\". This is almost always output published by cartographer and "
                 "should not appear as input. (Unless you have some complex "
                 "remove_frames configuration, this is will not work. Simplest "
                 "solution is to record input without cartographer running.)";
      break;
    }
  }
  error_msg = msg_buf.str();
  return is_ok;
}

}  // namespace cartographer_ros