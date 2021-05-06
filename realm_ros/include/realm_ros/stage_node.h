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

#ifndef PROJECT_STAGE_NODE_H
#define PROJECT_STAGE_NODE_H

#include <mutex>
#include <unordered_map>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <ros/package.h>

#include <OpenREALM/realm_core/structs.h>
#include <OpenREALM/realm_core/camera_settings_factory.h>
#include <OpenREALM/realm_vslam_base/visual_slam_settings_factory.h>
#include <OpenREALM/realm_densifier_base/densifier_settings_factory.h>
#include <OpenREALM/realm_io/utilities.h>

#include <OpenREALM/realm_stages/stage_settings_factory.h>
#include <OpenREALM/realm_stages/pose_estimation.h>
#include <OpenREALM/realm_stages/densification.h>
#include <OpenREALM/realm_stages/surface_generation.h>
#include <OpenREALM/realm_stages/ortho_rectification.h>
#include <OpenREALM/realm_stages/mosaicing.h>
#include <OpenREALM/realm_stages/tileing.h>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realm_ros/conversions.h>
#include <realm_msgs/msg/frame.hpp>
#include <realm_msgs/msg/cv_grid_map.hpp>
#include <realm_msgs/msg/ground_image_compressed.hpp>
#include <realm_msgs/msg/pose_stamped.hpp>


#include <std_srvs/srv/trigger.hpp>
#include <realm_msgs/srv/parameter_change.hpp>

namespace realm
{

class StageNode : public rclcpp::Node
{
  public:
    StageNode(int argc, char **argv);
    ~StageNode();
    void spin();
    //bool isOkay();
  private:
    // Master stage has several privileges,
    // e.g. creating output folder, ...
    bool _is_master_stage;

    // Set to true, to shut node down
    std::mutex _mutex_do_shutdown;
    bool _do_shutdown;

    // number of msgs that triggere ros CB
    uint32_t _nrof_msgs_rcvd;

    // type of stage
    std::string _type_stage;
    std::string _type_method;

    // camera info
    std::string _id_camera;

    // ros handle
    // auto _nh = rclcpp::Node::make_shared("talker");

    // ros communication handles
    rclcpp::Subscription<realm_msgs::msg::Frame>::SharedPtr _sub_input_frame;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_input_imu;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_output_dir;
    std::unordered_map<std::string, rclcpp::Publisher<>::SharedPtr> _publisher;

    // ros service handles
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srv_req_finish;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srv_req_stop;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srv_req_resume;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srv_req_reset;
    rclcpp::Service<realm_msgs::srv::ParameterChange>::SharedPtr _srv_change_param;


    // chosen profile related to all settings set
    std::string _profile;

    // chosen method related to the specific framework implementation
    std::string _method;

    // working paths
    std::string _path_working_directory;
    std::string _path_profile;
    std::string _path_output;

    // working directories
    std::string _dir_date_time;

    // filename of settings
    std::string _file_settings_stage;
    std::string _file_settings_method;
    std::string _file_settings_camera;
    std::string _file_settings_imu;

    // topics
    std::string _topic_prefix;
    std::string _topic_frame_in;
    std::string _topic_frame_out;
    std::string _topic_imu_in;

    // transforms
    bool _is_tf_base_initialized;
    bool _is_tf_stage_initialized;
    std::string _tf_base_frame_name;
    std::string _tf_stage_frame_name;
    tf2::Transform _tf_base;
    tf2::Transform _tf_stage;
    sensor_msgs::msg::NavSatFix _gnss_base;

    // trajectories
    std::unordered_map<std::string, std::vector<realm_msgs::msg::PoseStamped>> _trajectories;

    // Settings of the stage
    StageSettings::Ptr _settings_stage;

    CameraSettings::Ptr _settings_camera;

    // Handle for stage
    StageBase::Ptr _stage;

    // Initialization
    void readParams();
    void readStageSettings();
    void setPaths();
    void setTfBaseFrame(const UTMPose &utm);
    void createStagePoseEstimation();
    void createStageDensification();
    void createStageSurfaceGeneration();
    void createStageOrthoRectification();
    void createStageMosaicing();
    void createStageTileing();
    void linkStageTransport();

    // Functionalities
    // void reset();

    // ros communication functions
    void subFrame(const realm_msgs::msg::Frame &msg);
    void subImu(const sensor_msgs::msg::Imu &msg);
    void subOutputPath(const std_msgs::msg::String &msg);

    // stage callbacks
    void pubFrame(const Frame::Ptr &frame, const std::string &topic);
    void pubPose(const cv::Mat &pose, uint8_t zone, char band, const std::string &topic);
    void pubPointCloud(const cv::Mat &pts, const std::string &topic);
    void pubImage(const cv::Mat &img, const std::string &topic);
    void pubDepthMap(const cv::Mat &img, const std::string &topic);
    void pubMesh(const std::vector<Face> &faces, const std::string &topic);

    // master publish
    void pubTrajectory(const std::vector<realm_msgs::msg::PoseStamped> &traj, const std::string &topic);

    /*!
     * @brief Publisher for CvGridMaps as GroundImages. Should either contain one or two layers. The first layer
     *        represents the visual informations and the second (optional) the mask or valid elements of the visual info.
     * @param map Either 1- or 2-layer grid map containing visual informations (and valid elements of visual img)
     * @param zone UTM zone of the published CvGridMap
     * @param band UTM band of the published CvGridMap
     * @param topic Topic is NOT the ros topic, but the topic for REALM to identify which publisher should be triggered.
     */
    void pubCvGridMap(const CvGridMap &map, uint8_t zone, char band, const std::string &topic);

    bool srvFinish(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
    bool srvStop(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
    bool srvResume(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
    bool srvReset(std_srvs::srv::Trigger::Request &req, std_srvs::srv::Trigger::Response &res);
    bool srvChangeParam(realm_msgs::srv::ParameterChange::Request &req, realm_msgs::srv::ParameterChange::Response &res);
};

} // namespace realm

#endif //PROJECT_STAGE_NODE_H
