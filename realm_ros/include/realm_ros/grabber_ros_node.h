/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPENREALM_GRABBER_ROS_NODE_H
#define OPENREALM_GRABBER_ROS_NODE_H

#include <iostream>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
// #include <ros/package.h>
// #include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <OpenREALM/realm_io/realm_import.h>
#include <OpenREALM/realm_io/exif_import.h>
#include <OpenREALM/realm_io/utilities.h>

#include <realm_ros/conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <realm_msgs/msg/frame.hpp>
#include <realm_msgs/msg/pinhole.hpp>

namespace realm
{

class RosGrabberNode : public rclcpp::Node
{
  using ApproxTimePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::NavSatFix>;

public:
  explicit RosGrabberNode();

  void spin();

  // bool isOkay();

private:

  // name identifier for the node
  std::string _id_node;

  // chosen processing profile
  std::string _profile;

  // topics
  std::string _topic_image;
  std::string _topic_gnss;
  std::string _topic_heading;
  std::string _topic_relative_altitude;
  std::string _topic_orientation;
  std::string _topic_out_frame;
  std::string _topic_out_imu;

  // paths
  std::string _path_working_directory;
  std::string _path_profile;

  // filepaths
  std::string _file_settings_camera;

  bool _do_imu_passthrough;

  int _nrof_frames_received;

  double _fps;

  std::mutex _mutex_heading;
  double _heading;

  std::mutex _mutex_relative_altitude;
  double _relative_altitude;

  std::mutex _mutex_orientation;
  cv::Mat _orientation;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub_heading;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub_relative_altitude;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_orientation;

  message_filters::Subscriber<sensor_msgs::msg::Image> _sub_input_image;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> _sub_input_gnss;
  message_filters::Synchronizer<ApproxTimePolicy> _sync_topics;

  rclcpp::Publisher<realm_msgs::msg::Frame>::SharedPtr _pub_frame;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _pub_imu;

  camera::Pinhole::Ptr _cam;
  realm_msgs::msg::Pinhole _cam_msg;

  void createParams();
  void readParameters();
  void setPaths();

  void subHeading(const std_msgs::msg::Float64::SharedPtr msg);
  void subRelativeAltitude(const std_msgs::msg::Float64::SharedPtr msg);
  void subOrientation(const sensor_msgs::msg::Imu::SharedPtr msg);
  void subImageGnss(const sensor_msgs::msg::Image::SharedPtr msg_img, const sensor_msgs::msg::NavSatFix::SharedPtr msg_gnss);

  void publish(const Frame::Ptr &frame);
};

} // namespace realm

#endif //OPENREALM_GRABBER_ROS_NODE_H
