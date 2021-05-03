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

#include <realm_ros/grabber_exiv2_node.h>

using namespace realm;

Exiv2GrabberNode::Exiv2GrabberNode(std::string name)
: Node(name),
  _do_set_all_keyframes(false),
  _use_apriori_pose(false),
  _use_apriori_georeference(false),
  _use_apriori_surface_pts(false),
  _fps(0.0),
  _id_curr_file(0),
  _cam(nullptr)
{
  readParams();
  setPaths();

  _exiv2_reader = io::Exiv2FrameReader(io::Exiv2FrameReader::FrameTags::loadFromFile(_path_profile + "/config/exif.yaml"));

  if (io::fileExists(_file_settings_camera))
    _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_file_settings_camera));
  else
    throw(std::invalid_argument("Error loading camera file: Provided path does not exist."));
  _cam_msg = to_ros::pinhole(_cam);

  // Loading poses from file if provided
  if (_use_apriori_pose)
  {
    _poses = io::loadTrajectoryFromTxtTUM(_filepath_poses);
    RCLCPP_INFO_STREAM(node->get_logger(),"Succesfully loaded " << _poses.size() << " external poses.");
  }

  // Loading georeference from file if provided
  if (_use_apriori_georeference)
  {
    _georeference = io::loadGeoreferenceFromYaml(_filepath_georeference);
    RCLCPP_INFO_STREAM(node->get_logger(),"Succesfully loaded external georeference:\n" << _georeference);
  }

  // Loading surface points from file if provided
  if (_use_apriori_surface_pts)
  {
    _surface_pts = io::loadSurfacePointsFromTxt(_filepath_surface_pts);
    RCLCPP_INFO_STREAM(node->get_logger(),"Succesfully loaded " << _surface_pts.rows << " external surface points.");
  }

  RCLCPP_INFO_STREAM(node->get_logger(),
      "Exiv2 Grabber Node [Ankommen]: Successfully loaded camera: "
          << "\n\tcx = " << _cam->cx()
          << "\n\tcy = " << _cam->cy()
          << "\n\tfx = " << _cam->fx()
          << "\n\tfy = " << _cam->fy()
          << "\n\tk1 = " << _cam->k1()
          << "\n\tk2 = " << _cam->k2()
          << "\n\tp1 = " << _cam->p1()
          << "\n\tp2 = " << _cam->p2());

  // ROS related inits
  _topic_prefix = "/realm/" + _id_node;

  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(5)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      .avoid_ros_namespace_conventions(false);

  _pub_frame = this->create_publisher<realm_msgs::Frame>(_topic_prefix+ "/input", qos_profile);
  _pub_image = this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "/img", qos_profile);

  // Start grabbing images
  _file_list = getFileList(_path_grab);

  // Check if the exif tags in the config exist
  RCLCPP_INFO(node->get_logger(),"Scanning input image for provided meta tags...");
  std::map<std::string, bool> tag_existence = _exiv2_reader.probeImage(_file_list[0]);
  for (const auto &tag : tag_existence)
  {
    if (tag.second)
      RCLCPP_INFO(node->get_logger(),"[FOUND]\t\t'%s'", tag.first.c_str());
    else
      ROS_WARN("[NOT FOUND]\t'%s'", tag.first.c_str());
  }
}

void Exiv2GrabberNode::readParams()
{
  rclcpp::parameter_client::SyncParameterClient client(node);
  
  _id_node = client.get_parameter("config/id", std::string("uninitialised"));
  _path_grab = client.get_parameter("config/input", std::string("uninitialised"));
  _fps = client.get_parameter("config/rate", 0.0);
  _profile = client.get_parameter("config/profile", std::string("uninitialised"));
  _filepath_poses = client.get_parameter("config/opt/poses", std::string("uninitialised"));
  _filepath_georeference = client.get_parameter("config/opt/georeference", std::string("uninitialised"));
  _filepath_surface_pts = client.get_parameter("config/opt/surface_pts", std::string("uninitialised"));
  _do_set_all_keyframes = client.get_parameter("config/opt/set_all_keyframes", false);
  _path_working_directory = client.get_parameter("config/opt/working_directory", std::string("uninitialised"));

  if (_fps < 0.01)
    throw(std::invalid_argument("Error reading exiv2 grabber parameters: Frame rate is too low!"));
  if (_filepath_poses != "uninitialised")
    _use_apriori_pose = true;
  if (_filepath_georeference != "uninitialised")
    _use_apriori_georeference = true;
  if (_filepath_surface_pts != "uninitialised")
    _use_apriori_surface_pts = true;
  if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
    throw(std::invalid_argument("Error: Working directory does not exist!"));
}

void Exiv2GrabberNode::setPaths()
{
  if (_path_working_directory == "uninitialised")
    _path_working_directory = ros::package::getPath("realm_ros");
  _path_profile = _path_working_directory + "/profiles/" + _profile;

  _file_settings_camera = _path_profile + "/camera/calib.yaml";

  if (!io::dirExists(_path_profile))
    throw(std::invalid_argument("Error: Config folder path '" + _path_profile + "' does not exist!"));
  if (!io::dirExists(_path_grab))
    throw(std::invalid_argument("Error grabbing Exiv2 images: Folder '" + _path_grab + "' does not exist!"));
}

void Exiv2GrabberNode::spin()
{
  rclcpp::Rate rate(_fps);
  if (_id_curr_file < _file_list.size())
  {
    RCLCPP_INFO_STREAM(node->get_logger(),"Image #" << _id_curr_file << ", image Path: " << _file_list[_id_curr_file]);
    Frame::Ptr frame = _exiv2_reader.loadFrameFromExiv2(_id_node, _cam, _file_list[_id_curr_file]);

    // External pose can be provided
    if (_use_apriori_pose)
    {
      cv::Mat pose = _poses[frame->getTimestamp()];
      if (pose.empty())
        throw(std::runtime_error("Error adding external pose informations: No pose was found. Maybe images or provided pose file do not match?"));
      frame->setVisualPose(pose);
      if (_do_set_all_keyframes)
        frame->setKeyframe(true);
    }

    if (_use_apriori_georeference)
    {
      frame->updateGeoreference(_georeference);
    }

    // External surface points can be provided
    if (_use_apriori_surface_pts)
    {
      if (_surface_pts.empty())
        throw(std::runtime_error("Error adding external surface points: No points were found!"));
      frame->setSparseCloud(_surface_pts, false);
    }

    pubFrame(frame);
    _id_curr_file++;
  }

  std::vector<std::string> new_file_list = getFileList(_path_grab);
  if (new_file_list.size() != _file_list.size())
  {
    RCLCPP_INFO_STREAM(node->get_logger(),"Processed images in folder: " << _file_list.size() << " / " << new_file_list.size());
    std::set_difference(new_file_list.begin(), new_file_list.end(), _file_list.begin(), _file_list.end(), std::back_inserter(_file_list));
    std::sort(_file_list.begin(), _file_list.end());
  }

  rate.sleep();
}

void Exiv2GrabberNode::pubFrame(const Frame::Ptr &frame)
{
  // Create message header
  std_msgs::msg::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "map";

  // Create message informations
  realm_msgs::Frame msg_frame = to_ros::frame(header, frame);
  sensor_msgs::msg::Image msg_img = *to_ros::imageDisplay(header, frame->getImageRaw()).toImageMsg();

  _pub_frame.publish(msg_frame);
  _pub_image.publish(msg_img);
}

/* bool Exiv2GrabberNode::isOkay()  // dont have a replacement for this
{
  return _nh.ok();
} */

std::vector<std::string> Exiv2GrabberNode::getFileList(const std::string& path)
{
  std::vector<std::string> file_names;
  if (!path.empty())
  {
    boost::filesystem::path apk_path(path);
    boost::filesystem::recursive_directory_iterator end;

    for (boost::filesystem::recursive_directory_iterator it(apk_path); it != end; ++it)
    {
      const boost::filesystem::path cp = (*it);
      file_names.push_back(cp.string());
    }
  }
  std::sort(file_names.begin(), file_names.end());
  return file_names;
}