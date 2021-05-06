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

#include <realm_ros/grabber_ros_node.h>

using namespace realm;

RosGrabberNode::RosGrabberNode()
  : Node("realm_ros_grabber_node"), _do_imu_passthrough(false),
    _nrof_frames_received(0),
    _fps(1.0),
    _cam(nullptr),
    _sync_topics(ApproxTimePolicy(1000), _sub_input_image, _sub_input_gnss)
{
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(10)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      .avoid_ros_namespace_conventions(false);
  createParams();
  readParameters();
  setPaths();

  if (io::fileExists(_file_settings_camera))
    _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_file_settings_camera, &_fps));
  else
    throw(std::invalid_argument("Error loading camera file: Provided path does not exist."));
  _cam_msg = to_ros::pinhole(_cam);

  RCLCPP_INFO_STREAM(this->get_logger(),
          "ROS Grabber Node successfully loaded camera: "
                  << "\n\tfps = " << _fps
                  << "\n\tcx = " << _cam->cx()
                  << "\n\tcy = " << _cam->cy()
                  << "\n\tfx = " << _cam->fx()
                  << "\n\tfy = " << _cam->fy()
                  << "\n\tk1 = " << _cam->k1()
                  << "\n\tk2 = " << _cam->k2()
                  << "\n\tp1 = " << _cam->p1()
                  << "\n\tp2 = " << _cam->p2()
                  << "\n\tk3 = " << _cam->k3());
  _sub_heading = this->create_subscription<std_msgs::msg::Float64>(_topic_heading, qos_profile,
                std::bind(&RosGrabberNode::subHeading, this, std::placeholders::_1));

  _sub_relative_altitude = this->create_subscription<std_msgs::msg::Float64>(_topic_relative_altitude, qos_profile,
                std::bind(&RosGrabberNode::subRelativeAltitude, this, std::placeholders::_1));
  _sub_orientation = this->create_subscription<sensor_msgs::msg::Imu>(_topic_orientation, qos_profile,
                std::bind(&RosGrabberNode::subOrientation, this, std::placeholders::_1));

  
  _sub_input_image.subscribe(this, _topic_image);
  _sub_input_gnss.subscribe(this,_topic_gnss);

  // _sync_topics.registerCallback(boost::bind(RosGrabberNode::subImageGnss, this, std::placeholders::_1, std::placeholders::_2));
  _sync_topics.registerCallback(&RosGrabberNode::subImageGnss, this);
  //_sync_topics.registerCallback(RosGrabberNode::subImageGnss);

  _pub_frame = this->create_publisher<realm_msgs::msg::Frame>(_topic_out_frame, qos_profile);
  
  if (_do_imu_passthrough)
    _pub_imu = this->create_publisher<sensor_msgs::msg::Imu>(_topic_out_imu, qos_profile); 

  RCLCPP_INFO_STREAM(this->get_logger(),"ROS Grabber Node subscribed to topics:\n"
                  "- Image topic:\t\t"  << _topic_image   << "\n"
                  "- GNSS topic:\t\t"   << _topic_gnss    << "\n"
                  "- Heading topic:\t"  << _topic_heading << "\n"
                  "- Rel. Altitude topic:  "  << _topic_relative_altitude << "\n"
                  "- Orientation topic:\t"  << _topic_orientation << "\n"
                  "- Output topic IMU:\t"  << _topic_out_imu << "\n"
                  "- Output topic Frame:\t\t" << _topic_out_frame     << "\n");
}

void RosGrabberNode::createParams()
{  
  this->declare_parameter<std::string>("config/id", std::string("uninitialised"));
  this->declare_parameter<std::string>("config/profile", std::string("uninitialised"));
  this->declare_parameter<std::string>("config/opt/working_directory", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/image", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/gnss", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/heading", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/relative_altitude", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/orientation", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/out/frame", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/out/imu", std::string("uninitialised"));
}

void RosGrabberNode::readParameters()
{
  this->get_parameter("config/id", _id_node);
  this->get_parameter("config/profile", _profile);
  this->get_parameter("config/opt/working_directory", _path_working_directory);
  this->get_parameter("topic/image", _topic_image);
  this->get_parameter("topic/gnss", _topic_gnss);
  this->get_parameter("topic/heading", _topic_heading);
  this->get_parameter("topic/relative_altitude", _topic_relative_altitude);
  this->get_parameter("topic/orientation", _topic_orientation);
  this->get_parameter("topic/out/frame", _topic_out_frame);
  this->get_parameter("topic/out/imu", _topic_out_imu);

  if (_topic_out_imu != "uninitialised")
    _do_imu_passthrough = true;

  if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
    throw(std::invalid_argument("Error: Working directory does not exist!"));
}

void RosGrabberNode::setPaths()
{
  if (_path_working_directory == "uninitialised")
    _path_working_directory = "~/realm_ws/src/OpenREALM_ROS2_bridge/realm_ros";
  _path_profile = _path_working_directory + "/profiles/" + _profile;

  _file_settings_camera = _path_profile + "/camera/calib.yaml";

  if (!io::dirExists(_path_profile))
    throw(std::invalid_argument("Error: Config folder path '" + _path_profile + "' does not exist!"));
}

void RosGrabberNode::spin()
{
  rclcpp::Rate rate(_fps);
  rate.sleep();
}

/* bool RosGrabberNode::isOkay()
{
  return _nh.ok();
}
 */
void RosGrabberNode::subHeading(const std_msgs::msg::Float64::SharedPtr msg)
{
  _mutex_heading.lock();
  _heading = msg->data;
  _mutex_heading.unlock();
}

void RosGrabberNode::subRelativeAltitude(const std_msgs::msg::Float64::SharedPtr msg)
{
  _mutex_relative_altitude.lock();
  _relative_altitude = msg->data;
  _mutex_relative_altitude.unlock();
}

void RosGrabberNode::subOrientation(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  _mutex_orientation.lock();
  _orientation = to_realm::orientation(msg->orientation);
  _mutex_orientation.unlock();

  if (_do_imu_passthrough)
    _pub_imu->publish(*msg);
}

void RosGrabberNode::subImageGnss(const sensor_msgs::msg::Image::SharedPtr msg_img, const sensor_msgs::msg::NavSatFix::SharedPtr msg_gnss)
{
  cv::Mat img = to_realm::image(*msg_img);

  _mutex_relative_altitude.lock();
  _mutex_heading.lock();

  WGSPose wgs;
  wgs.latitude = msg_gnss->latitude;
  wgs.longitude = msg_gnss->longitude;
  wgs.altitude = msg_gnss->altitude;
  wgs.heading = _heading;

  UTMPose utm = gis::convertToUTM(wgs);

  _mutex_heading.unlock();
  _mutex_relative_altitude.unlock();

  cv::Mat orientation;
  if (_topic_orientation == "uninitialised" || _orientation.empty())
    orientation = io::computeOrientationFromHeading(utm.heading);
  else
    orientation = _orientation;

  auto frame = std::make_shared<Frame>(_id_node, _nrof_frames_received, msg_img->header.stamp.sec, img, utm, _cam, orientation);

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "/realm";
  _pub_frame->publish(to_ros::frame(header, frame));

  _nrof_frames_received++;
}