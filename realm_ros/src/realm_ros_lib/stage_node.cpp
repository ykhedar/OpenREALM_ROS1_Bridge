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

#include <realm_ros/stage_node.h>

using namespace realm;

StageNode::StageNode(int argc, char **argv)
: Node("realm_stage_node"), _nrof_msgs_rcvd(0),
  _is_master_stage(false),
  _do_shutdown(false),
  _is_tf_base_initialized(false),
  _is_tf_stage_initialized(false)
{
  
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      .keep_last(10)
      .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      .avoid_ros_namespace_conventions(false);

  // Read basic launch file inputs
  readParams();

  // Specify stage
  setPaths();
  readStageSettings();

  // Set naming conventions
  _topic_prefix = "/realm/" + _id_camera + "/" + _type_stage + "/";
  _tf_base_frame_name = "realm_base";
  _tf_stage_frame_name = "realm_" + _id_camera + "_" + _type_stage;

  // Set ros subscriber according to launch input
  _sub_input_frame = this->create_subscription<realm_msgs::msg::Frame>(_topic_frame_in, qos_profile, std::bind(&StageNode::subFrame, this, std::placeholders::_1));
  // _pub_frame = this->create_publisher<realm_msgs::msg::Frame>(_topic_out_frame, qos_profile);
  if (_is_master_stage)
  {
    _publisher.insert({"general/output_dir", this->create_publisher<std_msgs::msg::String>("/realm/" + _id_camera + "/general/output_dir", qos_profile)});
    _publisher.insert({"general/gnss_base", this->create_publisher<sensor_msgs::msg::NavSatFix>("/realm/" + _id_camera + "/general/gnss_base", qos_profile)});
  }
  else
    _sub_output_dir = _this->create_subscription<realm_msgs::msg::Frame>("/realm/"+ _id_camera +"/general/output_dir", qos_profile, 
    std::bind(&StageNode::subOutputPath, this, std::placeholders::_1));

  // Set ros services for stage handling
  _srv_req_finish = this->create_service<std_srvs::srv::Trigger>(_topic_prefix + "request_finish", std::bind(&StageNode::srvFinish, this, std::placeholders::_1, std::placeholders::_2), ::rmw_qos_profile_default);
  _srv_req_stop = this->create_service<std_srvs::srv::Trigger>(_topic_prefix + "request_stop", std::bind(&StageNode::srvStop, this, std::placeholders::_1, std::placeholders::_2), ::rmw_qos_profile_default);
  _srv_req_resume = this->create_service<std_srvs::srv::Trigger>(_topic_prefix + "request_resume", std::bind(&StageNode::srvResume, this, std::placeholders::_1, std::placeholders::_2), ::rmw_qos_profile_default);
  _srv_req_reset = this->create_service<std_srvs::srv::Trigger>(_topic_prefix + "request_reset", std::bind(&StageNode::srvReset, this, std::placeholders::_1, std::placeholders::_2), ::rmw_qos_profile_default);
  _srv_change_param = this->create_service<std_srvs::srv::Trigger>(_topic_prefix + "change_param", std::bind(&StageNode::srvChangeParam, this, std::placeholders::_1, std::placeholders::_2), ::rmw_qos_profile_default);

  // Provide camera information a priori to all stages
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: : Loading camera from path:\n\t%s", _type_stage.c_str(),_file_settings_camera.c_str());
  _settings_camera = CameraSettingsFactory::load(_file_settings_camera);
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: : Detected camera model: '%s'", _type_stage.c_str(), (*_settings_camera)["type"].toString().c_str());

  // Create stages
  if (_type_stage == "pose_estimation")
    createStagePoseEstimation();
  if (_type_stage == "densification")
    createStageDensification();
  if (_type_stage == "surface_generation")
    createStageSurfaceGeneration();
  if (_type_stage == "ortho_rectification")
    createStageOrthoRectification();
  if (_type_stage == "mosaicing")
    createStageMosaicing();
  if (_type_stage == "tileing")
    createStageTileing();

  // set stage path if master stage
  if (_is_master_stage)
    _stage->initStagePath(_path_output + "/" + _dir_date_time);

  // Start the thread for processing
  _stage->start();
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Started stage node successfully!", _type_stage.c_str());
}

StageNode::~StageNode()
{
  // In case of an unproper shutdown, at least call the finish procedure
  if (!_do_shutdown)
  {
    _stage->requestFinish();
    _stage->join();
  }
}

void StageNode::spin()
{
  if (_is_master_stage)
  {
    // Share output folder with slaves
    std_msgs::msg::String msg;
    msg.data = _dir_date_time;
    _publisher["general/output_dir"]->publish(msg);
  }

  static tf2_ros::TransformBroadcaster br;
  if (_is_tf_base_initialized && _is_master_stage)
  {
    // Master stage sends first tf as mission reference
    br.sendTransform(tf2::Stamped<tf2::Transform>(_tf_base, this->now(), "utm", _tf_base_frame_name));
    _publisher["general/gnss_base"]->publish(_gnss_base);
  }

  if (_is_tf_stage_initialized)
  {
    // Publish of current mission reference
    br.sendTransform(tf2::Stamped<tf2::Transform>(_tf_stage, this->now(), _tf_base_frame_name, _tf_stage_frame_name));
  }
}

/* bool StageNode::isOkay()
{
  std::unique_lock<std::mutex> lock(_mutex_do_shutdown);
  return (!_do_shutdown && _nh.ok());
} */

void StageNode::createStagePoseEstimation()
{
  // Pose estimation uses external frameworks, therefore load settings for that
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: : Loading vslam settings from path:\n\t%s", _type_stage.c_str(), _file_settings_method.c_str());
  VisualSlamSettings::Ptr settings_vslam = VisualSlamSettingsFactory::load(_file_settings_method, _path_profile + "/" + _type_stage + "/method");
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: : Detected vslam type: '%s'", _type_stage.c_str(), (*settings_vslam)["type"].toString().c_str());

  ImuSettings::Ptr settings_imu = nullptr;
  if ((*_settings_stage)["use_imu"].toInt() > 0)
  {
    settings_imu = std::make_shared<ImuSettings>();
    settings_imu->loadFromFile(_file_settings_imu);
  }

  // Topic and stage creation
  _stage = std::make_shared<stages::PoseEstimation>(_settings_stage, settings_vslam, _settings_camera, settings_imu, (*_settings_camera)["fps"].toDouble());
  
  _publisher.insert({"output/frame", this->create_publisher<realm_msgs::msg::Frame>(_topic_frame_out, 5)});
  _publisher.insert({"output/pose/visual/utm", this->create_publisher<realm_msgs::msg::PoseStamped>(_topic_prefix + "pose/visual/utm", 5)});
  _publisher.insert({"output/pose/visual/wgs", this->create_publisher<realm_msgs::msg::PoseStamped>(_topic_prefix + "pose/visual/wgs", 5)});
  _publisher.insert({"output/pose/visual/traj", this->create_publisher<nav_msgs::msg::Path>(_topic_prefix + "pose/visual/traj", 5)});
  _publisher.insert({"output/pose/gnss/utm", this->create_publisher<realm_msgs::msg::PoseStamped>(_topic_prefix + "pose/gnss/utm", 5)});
  _publisher.insert({"output/pose/gnss/wgs", this->create_publisher<realm_msgs::msg::PoseStamped>(_topic_prefix + "pose/gnss/wgs", 5)});
  _publisher.insert({"output/pose/gnss/traj", this->create_publisher<nav_msgs::msg::Path>(_topic_prefix + "pose/gnss/traj", 5)});
  _publisher.insert({"output/pointcloud", this->create_publisher<sensor_msgs::msg::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  _publisher.insert({"debug/tracked", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "tracked", 5)});
  linkStageTransport();

  if (_topic_imu_in != "uninitialised")
  {
    _sub_input_imu = _nh.subscribe(_topic_imu_in, 100, &StageNode::subImu, this, ros::TransportHints());
    //_sub_output_dir = _this->create_subscription<realm_msgs::msg::Frame>("/realm/"+ _id_camera +"/general/output_dir", qos_profile, 
    //std::bind(&StageNode::subOutputPath, this, std::placeholders::_1));
  }
}

void StageNode::createStageDensification()
{
  // Densification uses external frameworks, therefore load settings for that
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: : Loading densifier settings from path:\n\t%s", _type_stage.c_str(), _file_settings_method.c_str());
  DensifierSettings::Ptr settings_densifier = DensifierSettingsFactory::load(_file_settings_method, _path_profile + "/" + _type_stage + "/method");
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: : Detected densifier type: '%s'", _type_stage.c_str(), (*settings_densifier)["type"].toString().c_str());

  // Topic and stage creation
  _stage = std::make_shared<stages::Densification>(_settings_stage, settings_densifier, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/frame", this->create_publisher<realm_msgs::msg::Frame>(_topic_frame_out, 5)});
  _publisher.insert({"output/pose/utm", this->create_publisher<realm_msgs::msg::PoseStamped>(_topic_prefix + "pose/utm", 5)});
  _publisher.insert({"output/pose/wgs", this->create_publisher<realm_msgs::msg::PoseStamped>(_topic_prefix + "pose/wgs", 5)});
  _publisher.insert({"output/pointcloud", this->create_publisher<sensor_msgs::msg::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  _publisher.insert({"output/img_rectified", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "img", 5)});
  _publisher.insert({"output/depth", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "depth", 5)});
  _publisher.insert({"output/depth_display", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "depth_display", 5)});
  linkStageTransport();
}

void StageNode::createStageSurfaceGeneration()
{
  _stage = std::make_shared<stages::SurfaceGeneration>(_settings_stage, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/frame", this->create_publisher<realm_msgs::msg::Frame>(_topic_frame_out, 5)});
  _publisher.insert({"output/elevation_map", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "elevation_map", 5)});
  linkStageTransport();
}

void StageNode::createStageOrthoRectification()
{
  _stage = std::make_shared<stages::OrthoRectification>(_settings_stage, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/frame", this->create_publisher<realm_msgs::msg::Frame>(_topic_frame_out, 5)});
  _publisher.insert({"output/rectified", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "rectified", 5)});
  _publisher.insert({"output/pointcloud", this->create_publisher<sensor_msgs::msg::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  linkStageTransport();
}

void StageNode::createStageMosaicing()
{
  _stage = std::make_shared<stages::Mosaicing>(_settings_stage, (*_settings_camera)["fps"].toDouble());
  _publisher.insert({"output/rgb", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "rgb", 5)});
  _publisher.insert({"output/elevation", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "elevation", 5)});
  _publisher.insert({"output/pointcloud", this->create_publisher<sensor_msgs::msg::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  _publisher.insert({"output/mesh", this->create_publisher<visualization_msgs::Marker>(_topic_prefix + "mesh", 5)});
  _publisher.insert({"output/update/ortho", this->create_publisher<realm_msgs::msg::GroundImageCompressed>(_topic_prefix + "update/ortho", 5)});
  //_publisher.insert({"output/update/elevation", this->create_publisher<realm_msgs::msg::GroundImageCompressed>(_topic_prefix + "update/elevation", 5)});
  linkStageTransport();
}

void StageNode::createStageTileing()
{
  _stage = std::make_shared<stages::Tileing>(_settings_stage, (*_settings_camera)["fps"].toDouble());
  //_publisher.insert({"output/rgb", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "rgb", 5)});
  //_publisher.insert({"output/elevation", this->create_publisher<sensor_msgs::msg::Image>(_topic_prefix + "elevation", 5)});
  //_publisher.insert({"output/pointcloud", this->create_publisher<sensor_msgs::msg::PointCloud2>(_topic_prefix + "pointcloud", 5)});
  //_publisher.insert({"output/mesh", this->create_publisher<visualization_msgs::Marker>(_topic_prefix + "mesh", 5)});
  //_publisher.insert({"output/update/ortho", this->create_publisher<realm_msgs::msg::GroundImageCompressed>(_topic_prefix + "update/ortho", 5)});
  linkStageTransport();
}

void StageNode::linkStageTransport()
{
  namespace ph = std::placeholders;
  auto transport_frame = std::bind(&StageNode::pubFrame, this, ph::_1, ph::_2);
  auto transport_pose = std::bind(&StageNode::pubPose, this, ph::_1, ph::_2, ph::_3, ph::_4);
  auto transport_pointcloud = std::bind(&StageNode::pubPointCloud, this, ph::_1, ph::_2);
  auto transport_img = std::bind(&StageNode::pubImage, this, ph::_1, ph::_2);
  auto transport_depth = std::bind(&StageNode::pubDepthMap, this, ph::_1, ph::_2);
  auto transport_mesh = std::bind(&StageNode::pubMesh, this, ph::_1, ph::_2);
  auto transport_cvgridmap = std::bind(&StageNode::pubCvGridMap, this, ph::_1, ph::_2, ph::_3, ph::_4);
  _stage->registerFrameTransport(transport_frame);
  _stage->registerPoseTransport(transport_pose);
  _stage->registerPointCloudTransport(transport_pointcloud);
  _stage->registerImageTransport(transport_img);
  _stage->registerDepthMapTransport(transport_img);
  _stage->registerMeshTransport(transport_mesh);
  _stage->registerCvGridMapTransport(transport_cvgridmap);
}

/* void StageNode::reset()
{
  // TODO: Currently reset of stage via service seems to not suite the node reset
  _nrof_msgs_rcvd = 0;
  _is_tf_base_initialized = false;
  _is_tf_stage_initialized = false;
} */

void StageNode::subFrame(const realm_msgs::msg::Frame &msg)
{
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Received frame.", _type_stage.c_str());
  if (msg.do_reset.data)
  {
    _stage->requestReset();
    _publisher["output/geoimg"]this->publish(msg);
    ROS_WARN("STAGE_NODE [%s]: Mission has triggered reset. Stage resetting...", _type_stage.c_str());
    return;
  }

  Frame::Ptr frame = to_realm::frame(msg);
  if (_is_master_stage)
  {
    if (!_is_tf_base_initialized)
      setTfBaseFrame(frame->getGnssUtm());
  }

  _stage->addFrame(std::move(frame));
  _nrof_msgs_rcvd++;
}

void StageNode::subImu(const sensor_msgs::msg::Imu &msg)
{
  VisualSlamIF::ImuData imu;
  imu.timestamp = msg.header.stamp.sec;
  imu.acceleration.x = msg.linear_acceleration.x;
  imu.acceleration.y = msg.linear_acceleration.y;
  imu.acceleration.z = msg.linear_acceleration.z;
  imu.gyroscope.x = msg.angular_velocity.x;
  imu.gyroscope.y = msg.angular_velocity.y;
  imu.gyroscope.z = msg.angular_velocity.z;
  reinterpret_cast<stages::PoseEstimation*>(_stage.get())->queueImuData(imu);
}

void StageNode::subOutputPath(const std_msgs::msg::String &msg)
{
  // check if output directory has changed
  if (_dir_date_time != msg.data)
  {
    // Note: master privilege is not to create folder, but to set the name of the folder
    _dir_date_time = msg.data;
    if (!io::dirExists(_path_output + "/" + _dir_date_time))
      io::createDir(_path_output + "/" + _dir_date_time);
    _stage->initStagePath(_path_output + "/" + _dir_date_time);

    // Debug info
    RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Received output directory, set to:\n\t%s",
             _type_stage.c_str(),
             (_path_output + "/" + _dir_date_time).c_str());
  }
}

void StageNode::pubFrame(const Frame::Ptr &frame, const std::string &topic)
{
  auto publisher = _publisher[topic];
  if (publisher->get_subscription_count() == 0)
    return;

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "utm";

  realm_msgs::msg::Frame msg = to_ros::frame(header, frame);
  publisher->publish(msg);
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Published frame.", _type_stage.c_str());
}

void StageNode::pubPose(const cv::Mat &pose, uint8_t zone, char band, const std::string &topic)
{
  std_msgs::msg::Header header;
  header.stamp = this->now();

  // utm
  realm_msgs::msg::PoseStamped utm_msg;
  utm_msg.header = header;
  utm_msg.header.frame_id = "utm";
  utm_msg.pose = to_ros::pose(pose);

  // wgs
  realm_msgs::msg::PoseStamped wgs_msg;
  wgs_msg.header = header;
  wgs_msg.header.frame_id = "wgs";
  wgs_msg.pose = to_ros::poseWgs84(pose, zone, band);

  _publisher[topic + "/utm"]->publish(utm_msg);
  _publisher[topic + "/wgs"]->publish(wgs_msg);

  // trajectory
  std::vector<realm_msgs::msg::PoseStamped>* trajectory = &_trajectories[topic];
  trajectory->push_back(utm_msg);
  pubTrajectory(*trajectory, topic + "/traj");

  // transform
  _tf_stage = to_ros::tf(pose);
  if (!_is_tf_stage_initialized)
    _is_tf_stage_initialized = true;
}

void StageNode::pubPointCloud(const cv::Mat &pts, const std::string &topic)
{
  auto publisher = _publisher[topic];
  if (publisher->get_subscription_count() == 0)
    return;

  std_msgs::msg::Header header;
  header.frame_id = "utm";
  header.stamp = this->now();
  sensor_msgs::msg::PointCloud2 msg = to_ros::pointCloud(header, pts);
  publisher->publish(msg);
}

void StageNode::pubDepthMap(const cv::Mat &img, const std::string &topic)
{
  auto publisher = _publisher[topic];
  if (publisher->get_subscription_count() == 0)
    return;
 
  std_msgs::msg::Header header;
  header.frame_id = "utm";
  header.stamp = this->now();

  sensor_msgs::msg::Image msg;
  msg = *to_ros::image(header, img).toImageMsg();
  publisher->publish(msg);
}

void StageNode::pubImage(const cv::Mat &img, const std::string &topic)
{
  auto publisher = _publisher[topic];
  if (publisher->get_subscription_count() == 0)
    return;

  std_msgs::msg::Header header;
  header.frame_id = "utm";
  header.stamp = this->now();

  sensor_msgs::msg::Image msg;
  msg = *to_ros::imageDisplay(header, img).toImageMsg();
  publisher->publish(msg);
}

void StageNode::pubMesh(const std::vector<Face> &faces, const std::string &topic)
{
  std::unique_lock<std::mutex> lock(_mutex_do_shutdown);
  auto publisher = _publisher[topic];
  if (publisher->get_subscription_count() == 0)
    return;

  std_msgs::msg::Header header;
  header.frame_id = _tf_base_frame_name;
  header.stamp = this->now();

  visualization_msgs::Marker msg = to_ros::meshMarker(header, faces, "Global Map", 0,
                                                      visualization_msgs::Marker::TRIANGLE_LIST,
                                                      visualization_msgs::Marker::ADD, _tf_base.inverse());
  publisher->publish(msg);
}

void StageNode::pubCvGridMap(const CvGridMap &map, uint8_t zone, char band, const std::string &topic)
{
  auto publisher = _publisher[topic];
  if (publisher->get_subscription_count() == 0)
    return; 

  std_msgs::msg::Header header;
  header.frame_id = "utm";
  header.stamp = this->now();

  cv::Rect2d roi = map.roi();
  realm::UTMPose utm;
  utm.easting = roi.x + roi.width/2;
  utm.northing = roi.y + roi.height/2;
  utm.altitude = 0.0;
  utm.zone = zone;
  utm.band = band;

  realm_msgs::msg::GroundImageCompressed msg;
  std::vector<std::string> layer_names = map.getAllLayerNames();
  if (layer_names.size() == 1)
    msg = to_ros::groundImage(header, map[layer_names[0]], utm, map.resolution());
  else if (layer_names.size() == 2)
    msg = to_ros::groundImage(header, map[layer_names[0]], utm, map.resolution(), map[layer_names[1]]); // TODO: nobody understands that layer to is "valid"
  else
    throw(std::invalid_argument("Error publishing CvGridMap: More than one layer provided!"));

  publisher->publish(msg);
}

void StageNode::pubTrajectory(const std::vector<realm_msgs::msg::PoseStamped> &traj, const std::string &topic)
{
  auto publisher = _publisher[topic];
  if (publisher->get_subscription_count() == 0)
    return;

  nav_msgs::msg::Path msg;
  msg.header = traj.back().header;
  msg.poses = traj;
  publisher->publish(msg);
}

bool StageNode::srvFinish(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res)
{
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Requesting stage finishCallback...!", _type_stage.c_str());
  _stage->requestFinish();
  _stage->join();
  res.success = 1;
  res.message = "Successfully finished stage!";
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Successfully finished stage!", _type_stage.c_str());
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Shutting stage node down...", _type_stage.c_str());
  std::unique_lock<std::mutex> lock(_mutex_do_shutdown);
  _do_shutdown = true;
  return true;
}

bool StageNode::srvStop(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res)
{
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Requesting stage stop...!", _type_stage.c_str());
  _stage->requestStop();
  res.success = 1;
  res.message = "Successfully stopped stage!";
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Successfully stopped stage!", _type_stage.c_str());
  return true;
}

bool StageNode::srvResume(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res)
{
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Requesting stage resume...!", _type_stage.c_str());
  _stage->resume();
  res.success = 1;
  res.message = "Successfully resumed stage!";
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Successfully resumed stage!", _type_stage.c_str());
  return true;
}

bool StageNode::srvReset(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res)
{
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Requesting stage reset...!", _type_stage.c_str());
  _stage->requestReset();
  res.success = 1;
  res.message = "Successfully reset stage!";
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Successfully reset stage!", _type_stage.c_str());
  return true;
}

bool StageNode::srvChangeParam(realm_msgs::srv::ParameterChange::Request &req, realm_msgs::srv::ParameterChange::Response &res)
{
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Changing stage parameter %s to value %s...", _type_stage.c_str(), req.name.c_str(), req.val.c_str());
  if (_stage->changeParam(req.name, req.val))
  {
    RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Successfully changed parameter!", _type_stage.c_str());
    res.success = 1;
    res.message = "Successfully changed parameter!";
    return true;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Failed to change parameter!", _type_stage.c_str());
    res.success = 0;
    res.message = "Failed to change parameter!";
    return false;
  }
}

void StageNode::readStageSettings()
{
  // Load stage settings
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Loading stage settings from path:\n\t%s", _type_stage.c_str(), _file_settings_stage.c_str());
  _settings_stage = StageSettingsFactory::load(_type_stage, _file_settings_stage);
  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Detected stage type: '%s'", _type_stage.c_str(), (*_settings_stage)["type"].toString().c_str());
}

void RosGrabberNode::createParams()
{  
  this->declare_parameter<std::string>("stage/master", false);

  this->declare_parameter<std::string>("config/id", std::string("uninitialised"));
  this->declare_parameter<std::string>("config/profile", std::string("uninitialised"));
  this->declare_parameter<std::string>("config/opt/working_directory", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/input/frame", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/input/imu", std::string("uninitialised"));

  this->declare_parameter<std::string>("config/method", std::string("uninitialised"));
  this->declare_parameter<std::string>("config/opt/output_directory", std::string("uninitialised"));
  this->declare_parameter<std::string>("stage/type", std::string("uninitialised"));
  this->declare_parameter<std::string>("stage/output_dir", std::string("uninitialised"));
  this->declare_parameter<std::string>("topic/output", std::string("uninitialised"));



}

void StageNode::readParams()
{
  // Read parameters from launch file
  this->get_parameter("stage/master", _is_master_stage);
  this->get_parameter("topics/input/frame", _topic_frame_in);
  this->get_parameter("topics/input/imu", _topic_imu_in);
  this->get_parameter("config/id", _id_camera);
  this->get_parameter("config/profile", _profile);
  this->get_parameter("config/opt/working_directory", _path_working_directory);
  this->get_parameter("config/method", _method);
  this->get_parameter("config/opt/output_directory", _path_output);
  this->get_parameter("stage/type", _type_stage);
  this->get_parameter("stage/output_dir", _path_output);
  this->get_parameter("topics/output", _topic_frame_out);


  // Set specific config file paths
  if (_profile == "uninitialised")
    throw(std::invalid_argument("Error: Stage settings profile must be provided in launch file."));
  if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
    throw(std::invalid_argument("Error: Working directory does not exist!"));
}

void StageNode::setPaths()
{
  if (_path_working_directory == "uninitialised")
    _path_working_directory = "~/realm_ws/src/OpenREALM_ROS2_bridge/realm_ros";

  _path_profile = _path_working_directory + "/profiles/" + _profile;

  if (_path_output == "uninitialised")
    _path_output = _path_working_directory + "/output";

  // Set settings filepaths
  _file_settings_camera = _path_profile + "/camera/calib.yaml";
  _file_settings_imu    = _path_profile + "/config/imu.yaml";
  _file_settings_stage = _path_profile + "/" + _type_stage + "/stage_settings.yaml";
  _file_settings_method = _path_profile + "/" + _type_stage + "/method/" + _method + "_settings.yaml";

  if (!io::dirExists(_path_profile))
    throw(std::runtime_error("Error: Profile folder '" + _path_profile + "' was not found!"));
  if (!io::dirExists(_path_output))
    io::createDir(_path_output);

  // Master priviliges
  if (_is_master_stage)
  {
    // Create sub directory with timestamp
    _dir_date_time = io::getDateTime();
    if (!io::dirExists(_path_output + "/" + _dir_date_time))
      io::createDir(_path_output + "/" + _dir_date_time);
  }
}

void StageNode::setTfBaseFrame(const UTMPose &utm)
{
  tf2::Vector3 origin(utm.easting, utm.northing, 0.0);
  tf2::Quaternion q(0.0, 0.0, 0.0, 1.0);
  _tf_base = tf2::Transform(q, origin);
  _is_tf_base_initialized = true;

  geographic_msgs::msg::GeoPoint wgs = to_ros::wgs84(utm);

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = _tf_base_frame_name;

  _gnss_base.header = header;
  _gnss_base.latitude = wgs.latitude;
  _gnss_base.longitude = wgs.longitude;
  _gnss_base.altitude = wgs.altitude;

  RCLCPP_INFO(this->get_logger(),"STAGE_NODE [%s]: Frame reference set at: %f, %f", _type_stage.c_str(), _gnss_base.latitude, _gnss_base.longitude);
}