/**
 * Copyright 2020 Bossa Nova Robotics, Inc.
 * @author Ernesto Corbellini
 */
#pragma once

#include <map>
#include <sstream>
#include <string>

#include "cartographer/common/time.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "glog/logging.h"
#include "range_data_checker.h"
#include "tf2_eigen/tf2_eigen.h"


namespace cartographer_ros {

/**
 * A class for checking the health of a sequence of ROS sensor scan messages messages.
 * Checks that messages are sequential in time and that scans don't overlap in time.
 * Keeps an internal state of the last message processed and compares with the new.
 */
class RangeDataChecker {
 public:
  using FrameToOverlapMap = std::map<std::string, double>;

  struct RangeErrorReport {
    bool non_sequential;
    bool overlaps;
    bool repeated;
    std::string error_msg;
  };

  /**
   * Checks that the provided message is good.
   * Updates the internal state with statistics from the message
   * @param message the message to be processed
   * @param error_report A reference to a RangeErrorReport structure that will hold the error status
   * @return True if the message if good, false if there is an error
   */
  template <typename MessageType>
  bool CheckMessage(const MessageType& message, RangeErrorReport& error_report);

  /**
   * Returns a map containing the maximum overlap duration for each frame id
   * @return A mapping between frame ids and maximum overlap duration
   */
  FrameToOverlapMap get_max_overlap_durations() const {
    return frame_id_to_max_overlap_duration_;
  }

 private:
  typedef std::pair<size_t /* num_points */, Eigen::Vector4f /* points_sum */>
      RangeChecksum;

  std::map<std::string, RangeChecksum> frame_id_to_range_checksum_;
  std::map<std::string, cartographer::common::Time>
      frame_id_to_previous_time_to_;
  FrameToOverlapMap frame_id_to_max_overlap_duration_;

  template <typename MessageType>
  static void ReadRangeMessage(const MessageType& message,
                               RangeChecksum* range_checksum,
                               cartographer::common::Time* from,
                               cartographer::common::Time* to);
};

// The implementation needs to be in the header file to allow template instantiation

template <typename MessageType>
bool RangeDataChecker::CheckMessage(const MessageType& message, RangeErrorReport& error_report) {
  const std::string& frame_id = message.header.frame_id;
  ros::Time current_time_stamp = message.header.stamp;
  RangeChecksum current_checksum;
  cartographer::common::Time time_from, time_to;
  error_report.non_sequential = false;
  error_report.overlaps = false;
  error_report.repeated = false;
  error_report.error_msg = "";
  ReadRangeMessage(message, &current_checksum, &time_from, &time_to);
  auto previous_time_to_it = frame_id_to_previous_time_to_.find(frame_id);
  if (previous_time_to_it != frame_id_to_previous_time_to_.end() &&
      previous_time_to_it->second >= time_from) {
    if (previous_time_to_it->second >= time_to) {
      error_report.non_sequential = true;
      std::stringstream msg_buffer;
      msg_buffer << "Sensor with frame_id \"" << frame_id
                 << "\" is not sequential in time."
                 << "Previous range message ends at time "
                 << previous_time_to_it->second
                 << ", current one at time " << time_to;
      error_report.error_msg = msg_buffer.str();
    } else {
      error_report.overlaps = true;
      std::stringstream msg_buffer;
      msg_buffer << "Sensor with frame_id \"" << frame_id
                 << "\" measurements overlap in time. "
                 << "Previous range message, ending at time stamp "
                 << previous_time_to_it->second
                 << ", must finish before current range message, "
                 << "which ranges from " << time_from << " to " << time_to;
      error_report.error_msg = msg_buffer.str();
    }
    double overlap = cartographer::common::ToSeconds(
        previous_time_to_it->second - time_from);
    auto it = frame_id_to_max_overlap_duration_.find(frame_id);
    if (it == frame_id_to_max_overlap_duration_.end() ||
        overlap > frame_id_to_max_overlap_duration_.at(frame_id)) {
      frame_id_to_max_overlap_duration_[frame_id] = overlap;
    }
  }
  frame_id_to_previous_time_to_[frame_id] = time_to;
  if (current_checksum.first != 0) {
    auto it = frame_id_to_range_checksum_.find(frame_id);
    if (it != frame_id_to_range_checksum_.end()) {
      RangeChecksum previous_checksum = it->second;
      if (previous_checksum == current_checksum) {
        error_report.repeated = true;
        std::stringstream msg_buffer;
        msg_buffer << "Sensor with frame_id \"" << frame_id
                   << "\" sends exactly the same range measurements multiple times. "
                   << "Range data at time " << current_time_stamp
                   << " equals preceding data with " << current_checksum.first
                   << " points.";
        error_report.error_msg = msg_buffer.str();
      }
    }
    frame_id_to_range_checksum_[frame_id] = current_checksum;
  }
  return !error_report.non_sequential && !error_report.overlaps && !error_report.repeated;
}

template <typename MessageType>
void RangeDataChecker::ReadRangeMessage(const MessageType& message,
                              RangeChecksum* range_checksum,
                              cartographer::common::Time* from,
                              cartographer::common::Time* to) {
  auto point_cloud_time = ToPointCloudWithIntensities(message);
  const cartographer::sensor::TimedPointCloud& point_cloud =
      std::get<0>(point_cloud_time).points;
  *to = std::get<1>(point_cloud_time);
  *from = *to + cartographer::common::FromSeconds(point_cloud[0].time);
  Eigen::Vector4f points_sum = Eigen::Vector4f::Zero();
  for (const auto& point : point_cloud) {
    points_sum.head<3>() += point.position;
    points_sum[3] += point.time;
  }
  if (point_cloud.size() > 0) {
    double first_point_relative_time = point_cloud[0].time;
    double last_point_relative_time = (*point_cloud.rbegin()).time;
    if (first_point_relative_time > 0) {
      LOG_FIRST_N(ERROR, 1)
          << "Sensor with frame_id \"" << message.header.frame_id
          << "\" has range data that has positive relative time "
          << first_point_relative_time << " s, must negative or zero.";
    }
    if (last_point_relative_time != 0) {
      LOG_FIRST_N(INFO, 1)
          << "Sensor with frame_id \"" << message.header.frame_id
          << "\" has range data whose last point has relative time "
          << last_point_relative_time << " s, should be zero.";
    }
  }
  *range_checksum = {point_cloud.size(), points_sum};
}

}  // namespace cartographer_ros
