/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/histogram.h"
#include "cartographer_ros/msg_conversion.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "message_checker.h"
#include "nav_msgs/Odometry.h"
#include "range_data_checker.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/buffer.h"
#include "urdf/model.h"


DEFINE_string(bag_filename, "", "Bag to process.");
DEFINE_bool(dump_timing, false,
            "Dump per-sensor timing information in files called "
            "timing_<frame_id>.csv in the current directory.");

namespace cartographer_ros {
namespace {

const double kMinAverageAcceleration = 9.5;
const double kMaxAverageAcceleration = 10.5;
const double kMaxGapPointsData = 0.1;
const double kMaxGapImuData = 0.05;
const double kTimeDeltaSerializationSensorWarning = 0.1;
const double kTimeDeltaSerializationSensorError = 0.5;

struct FrameProperties {
  ros::Time last_timestamp;
  std::string topic;
  std::vector<float> time_deltas;
  std::unique_ptr<std::ofstream> timing_file;
  std::string data_type;
};

std::unique_ptr<std::ofstream> CreateTimingFile(const std::string& frame_id) {
  auto timing_file = absl::make_unique<std::ofstream>(
      std::string("timing_") + frame_id + ".csv", std::ios_base::out);

  (*timing_file)
      << "# Timing information for sensor with frame id: " << frame_id
      << std::endl
      << "# Columns are in order" << std::endl
      << "# - packet index of the packet in the bag, first packet is 1"
      << std::endl
      << "# - timestamp when rosbag wrote the packet, i.e. "
         "rosbag::MessageInstance::getTime().toNSec()"
      << std::endl
      << "# - timestamp when data was acquired, i.e. "
         "message.header.stamp.toNSec()"
      << std::endl
      << "#" << std::endl
      << "# The data can be read in python using" << std::endl
      << "# import numpy" << std::endl
      << "# np.loadtxt(<filename>, dtype='uint64')" << std::endl;

  return timing_file;
}

void logRangeError(const RangeDataChecker::RangeErrorReport& error_report) {
  if (error_report.error_msg.empty()) {
    return;
  }
  if (error_report.non_sequential || error_report.repeated) {
    LOG_FIRST_N(ERROR, 3) << error_report.error_msg;
  } else {
    LOG_FIRST_N(WARNING, 3) << error_report.error_msg;
  }
}

void PrintOverlapReport(const RangeDataChecker::FrameToOverlapMap& map) {
  for (auto& it : map) {
    LOG(WARNING) << "Sensor with frame_id \"" << it.first
                  << "\" range measurements have longest overlap of "
                  << it.second << " s";
  }
}


void Run(const std::string& bag_filename, const bool dump_timing) {
  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag);

  std::map<std::string, FrameProperties> frame_id_to_properties;
  size_t message_index = 0;
  int num_imu_messages = 0;
  double sum_imu_acceleration = 0.;
  MessageChecker message_checker;
  RangeDataChecker range_data_checker;
  RangeDataChecker::RangeErrorReport error_report;
  for (const rosbag::MessageInstance& message : view) {
    ++message_index;
    std::string frame_id;
    ros::Time time;
    if (message.isType<sensor_msgs::PointCloud2>()) {
      auto msg = message.instantiate<sensor_msgs::PointCloud2>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
      if (!range_data_checker.CheckMessage(*msg, error_report)) {
        logRangeError(error_report);
      }
    } else if (message.isType<sensor_msgs::MultiEchoLaserScan>()) {
      auto msg = message.instantiate<sensor_msgs::MultiEchoLaserScan>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
      if (!range_data_checker.CheckMessage(*msg, error_report)) {
        logRangeError(error_report);
      }
    } else if (message.isType<sensor_msgs::LaserScan>()) {
      auto msg = message.instantiate<sensor_msgs::LaserScan>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
      if (!range_data_checker.CheckMessage(*msg, error_report)) {
        logRangeError(error_report);
      }
    } else if (message.isType<sensor_msgs::Imu>()) {
      auto msg = message.instantiate<sensor_msgs::Imu>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
      std::string error_msg;
      if (!message_checker.CheckImuMessage(*msg, error_msg)) {
        LOG_FIRST_N(WARNING, 3) << error_msg;
      }
      num_imu_messages++;
      sum_imu_acceleration += ToEigen(msg->linear_acceleration).norm();
    } else if (message.isType<nav_msgs::Odometry>()) {
      auto msg = message.instantiate<nav_msgs::Odometry>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
      std::string error_msg;
      if (!message_checker.CheckOdometryMessage(*msg, error_msg)) {
        LOG_FIRST_N(ERROR, 3) << error_msg;
      }
    } else if (message.isType<tf2_msgs::TFMessage>()) {
      auto msg = message.instantiate<tf2_msgs::TFMessage>();
      std::string error_msg;
      if (!message_checker.CheckTfMessage(*msg, error_msg)) {
        LOG_FIRST_N(ERROR, 1) << error_msg;
      }
      continue;
    } else {
      continue;
    }

    bool first_packet = false;
    if (!frame_id_to_properties.count(frame_id)) {
      frame_id_to_properties.emplace(
          frame_id,
          FrameProperties{time, message.getTopic(), std::vector<float>(),
                          dump_timing ? CreateTimingFile(frame_id) : nullptr,
                          message.getDataType()});
      first_packet = true;
    }

    auto& entry = frame_id_to_properties.at(frame_id);
    if (!first_packet) {
      const double delta_t_sec = (time - entry.last_timestamp).toSec();
      if (delta_t_sec <= 0) {
        LOG_FIRST_N(ERROR, 3)
            << "Sensor with frame_id \"" << frame_id
            << "\" jumps backwards in time, i.e. timestamps are not strictly "
               "increasing. Make sure that the bag contains the data for each "
               "frame_id sorted by header.stamp, i.e. the order in which they "
               "were acquired from the sensor.";
      }
      entry.time_deltas.push_back(delta_t_sec);
    }

    if (entry.topic != message.getTopic()) {
      LOG_FIRST_N(ERROR, 3)
          << "frame_id \"" << frame_id
          << "\" is send on multiple topics. It was seen at least on "
          << entry.topic << " and " << message.getTopic();
    }
    entry.last_timestamp = time;

    if (dump_timing) {
      CHECK(entry.timing_file != nullptr);
      (*entry.timing_file) << message_index << "\t"
                           << message.getTime().toNSec() << "\t"
                           << time.toNSec() << std::endl;
    }

    double duration_serialization_sensor = (time - message.getTime()).toSec();
    if (std::abs(duration_serialization_sensor) >
        kTimeDeltaSerializationSensorWarning) {
      std::stringstream stream;
      stream << "frame_id \"" << frame_id << "\" on topic "
             << message.getTopic() << " has serialization time "
             << message.getTime() << " but sensor time " << time
             << " differing by " << duration_serialization_sensor << " s.";
      if (std::abs(duration_serialization_sensor) >
          kTimeDeltaSerializationSensorError) {
        LOG_FIRST_N(ERROR, 3) << stream.str();
      } else {
        LOG_FIRST_N(WARNING, 1) << stream.str();
      }
    }
  }
  bag.close();

  PrintOverlapReport(range_data_checker.get_max_overlap_durations());

  if (num_imu_messages > 0) {
    double average_imu_acceleration = sum_imu_acceleration / num_imu_messages;
    if (std::isnan(average_imu_acceleration) ||
        average_imu_acceleration < kMinAverageAcceleration ||
        average_imu_acceleration > kMaxAverageAcceleration) {
      LOG(ERROR) << "Average IMU linear acceleration is "
                 << average_imu_acceleration << " m/s^2,"
                 << " expected is [" << kMinAverageAcceleration << ", "
                 << kMaxAverageAcceleration
                 << "] m/s^2. Linear acceleration data "
                    "should include gravity and be given in m/s^2.";
    }
  }

  constexpr int kNumBucketsForHistogram = 10;
  for (const auto& entry_pair : frame_id_to_properties) {
    const FrameProperties& frame_properties = entry_pair.second;
    float max_time_delta =
        *std::max_element(frame_properties.time_deltas.begin(),
                          frame_properties.time_deltas.end());
    if (message_checker.IsPointDataType(frame_properties.data_type) &&
        max_time_delta > kMaxGapPointsData) {
      LOG(ERROR) << "Point data (frame_id: \"" << entry_pair.first
                 << "\") has a large gap, largest is " << max_time_delta
                 << " s, recommended is [0.0005, 0.05] s with no jitter.";
    }
    if (frame_properties.data_type ==
            ros::message_traits::DataType<sensor_msgs::Imu>::value() &&
        max_time_delta > kMaxGapImuData) {
      LOG(ERROR) << "IMU data (frame_id: \"" << entry_pair.first
                 << "\") has a large gap, largest is " << max_time_delta
                 << " s, recommended is [0.0005, 0.005] s with no jitter.";
    }

    cartographer::common::Histogram histogram;
    for (float time_delta : frame_properties.time_deltas) {
      histogram.Add(time_delta);
    }
    LOG(INFO) << "Time delta histogram for consecutive messages on topic \""
              << frame_properties.topic << "\" (frame_id: \""
              << entry_pair.first << "\"):\n"
              << histogram.ToString(kNumBucketsForHistogram);
  }

  if (dump_timing) {
    for (const auto& entry_pair : frame_id_to_properties) {
      entry_pair.second.timing_file->close();
      CHECK(*entry_pair.second.timing_file)
          << "Could not write timing information for \"" << entry_pair.first
          << "\"";
    }
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  ::cartographer_ros::Run(FLAGS_bag_filename, FLAGS_dump_timing);
}
