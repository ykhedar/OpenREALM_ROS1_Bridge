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

#ifndef PROJECT_CONVERSION_H
#define PROJECT_CONVERSION_H

#include <fstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>

#include <OpenREALM/realm_core/enums.h>
#include <OpenREALM/realm_core/frame.h>
#include <OpenREALM/realm_core/utm32.h>
#include <OpenREALM/realm_core/structs.h>
#include <OpenREALM/realm_core/camera.h>
#include <OpenREALM/realm_core/cv_grid_map.h>
#include <OpenREALM/realm_core/analysis.h>
#include <OpenREALM/realm_core/depthmap.h>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <realm_msgs/msg/frame.hpp>
#include <realm_msgs/msg/pinhole.hpp>
#include <realm_msgs/msg/cv_grid_map.hpp>
#include <realm_msgs/msg/pose_stamped.hpp>
#include <realm_msgs/msg/georeference.hpp>
#include <realm_msgs/msg/ground_image_compressed.hpp>

namespace realm
{
namespace to_ros
{
std::vector<Face> fixRvizMeshFlickerBug(const std::vector<Face> &faces, const tf2::Transform &T);
cv_bridge::CvImage image(const std_msgs::msg::Header &header, const cv::Mat &cv_img);
cv_bridge::CvImage imageDisplay(const std_msgs::msg::Header &header, const cv::Mat &cv_img);
geodesy::UTMPoint utm(const realm::UTMPose &r_utm);
geographic_msgs::msg::GeoPoint wgs84(const realm::UTMPose &r_utm);
tf2::Transform tf(const geometry_msgs::msg::Pose &msg);
tf2::Transform tf(const cv::Mat &cv_pose);
tf2::Quaternion quaternion(const cv::Mat &R);
realm_msgs::msg::Georeference georeference(const cv::Mat &mat);
geometry_msgs::msg::Transform tfMsg(const cv::Mat &T);
tf2::Stamped<tf2::Transform> tfStamped(const realm_msgs::msg::PoseStamped &msg);
sensor_msgs::msg::PointCloud2 pointCloud(const std_msgs::msg::Header &header, const cv::Mat &points);
realm_msgs::msg::Pinhole pinhole(const realm::camera::Pinhole::ConstPtr &cam);
realm_msgs::msg::GroundImageCompressed groundImage(const std_msgs::msg::Header &header,
                                              const cv::Mat &img,
                                              const realm::UTMPose &ulc,
                                              double GSD,
                                              const cv::Mat &mask = cv::Mat());
realm_msgs::msg::Depthmap depthmap(const std_msgs::msg::Header &header, const realm::Depthmap::Ptr &depthmap);
geometry_msgs::msg::Pose pose(const cv::Mat &cv_pose);
geometry_msgs::msg::Pose pose(const tf2::Transform &transform);
geometry_msgs::msg::Pose poseWgs84(const cv::Mat &cv_pose, uint8_t zone, char band);
realm_msgs::msg::CvGridMap cvGridMap(const std_msgs::msg::Header &header, const realm::CvGridMap::Ptr &map);
realm_msgs::msg::Frame frame(const std_msgs::msg::Header &header, const realm::Frame::Ptr &frame);
visualization_msgs::msg::Marker meshMarker(const std_msgs::msg::Header &header,
                                      const std::vector<Face> &faces,
                                      const std::string &ns,
                                      int32_t id,
                                      int32_t type,
                                      int32_t action,
                                      const tf2::Transform &T);

} // namespace to_ros

namespace to_realm
{
UTMPose utm(const sensor_msgs::msg::NavSatFix &gnss, const std_msgs::msg::Float32 &heading = std_msgs::msg::Float32());
Frame::Ptr frame(const realm_msgs::msg::Frame &msg);
camera::Pinhole::Ptr pinhole(const realm_msgs::msg::Pinhole &msg);
cv::Mat pose(const geometry_msgs::msg::Pose &ros_pose);
cv::Mat pose(const tf2::Transform &transform);
cv::Mat tf(const geometry_msgs::msg::Transform &msg);
cv::Mat orientation(const geometry_msgs::msg::Quaternion &msg);
cv::Mat georeference(const realm_msgs::msg::Georeference &msg);
cv::Mat pointCloud(const sensor_msgs::msg::Image &msg);
cv::Mat image(const sensor_msgs::msg::Image &msg);
cv::Mat imageCompressed(const sensor_msgs::msg::CompressedImage &msg);
realm::CvGridMap::Ptr cvGridMap(const realm_msgs::msg::CvGridMap &msg);
realm::Depthmap::Ptr depthmap(const realm_msgs::msg::Depthmap &msg);

} // namespace to_realm

namespace tf2_compat
{
void quaternionMsgToTF(const geometry_msgs::msg::Quaternion& msg, tf2::Quaternion& bt);
void quaternionTFToMsg(const tf2::Quaternion& bt, geometry_msgs::msg::Quaternion& msg);

}
} // namespace realm

#endif