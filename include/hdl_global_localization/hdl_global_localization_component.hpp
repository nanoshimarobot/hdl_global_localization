#pragma once

#include <iostream>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <hdl_global_localization/engines/global_localization_bbs.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>
// #include <hdl_global_localization/engines/global_localization_fpfh_teaser.hpp>

#include "hdl_global_localization_msgs/srv/set_global_map.hpp"
#include "hdl_global_localization_msgs/srv/set_global_localization_engine.hpp"
#include "hdl_global_localization_msgs/srv/query_global_localization.hpp"

namespace hdl_global_localization {
using namespace hdl_global_localization_msgs;
class GlobalLocalizationNode : public rclcpp::Node {
private:
  std_msgs::msg::Header globalmap_header;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
  std::unique_ptr<GlobalLocalizationEngine> engine;

public:
  GlobalLocalizationNode(const rclcpp::NodeOptions& options) : GlobalLocalizationNode("", options) {}
  GlobalLocalizationNode(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("hdl_global_localization_node", name_space, options) {
    set_engine(this->declare_parameter<std::string>("global_localization_engine", "FPFH_RANSAC"));
  }

  bool set_engine(const std::string& engine_name) {
    if (engine_name == "BBS") {
      engine.reset(new GlobalLocalizationBBS(this->shared_from_this()));
    } else if (engine_name == "FPFH_RANSAC") {
      engine.reset(new GlobalLocalizationEngineFPFH_RANSAC(this->shared_from_this()));
    }
#ifdef TEASER_ENABLED
    else if (engine_name == "FPFH_TEASER") {
      engine.reset(new GlobalLocalizationEngineFPFH_Teaser(private_nh));
    }
#endif
    else {
      RCLCPP_INFO_STREAM(this->get_logger(), "Unknown Global Localization Engine:" << engine_name);
      return false;
    }

    if (global_map) {
      engine->set_global_map(global_map);
    }

    return true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(resolution, resolution, resolution);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*filtered);
    return filtered;
  }

  bool set_engine(srv::SetGlobalLocalizationEngine::Request& req, srv::SetGlobalLocalizationEngine::Response& res) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Set Global Localization Engine");
    return set_engine(req.engine_name.data);
  }

  bool set_global_map(srv::SetGlobalMap::Request& req, srv::SetGlobalMap::Response& res) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Global Map Received");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.global_map, *cloud);
    cloud = downsample(cloud, this->declare_parameter<double>("globalmap_downsample_resolution", 0.5));

    globalmap_header = req.global_map.header;
    global_map = cloud;
    engine->set_global_map(global_map);

    RCLCPP_INFO_STREAM(this->get_logger(), "DONE");

    return true;
  }

  bool query(srv::QueryGlobalLocalization::Request& req, srv::QueryGlobalLocalization::Response& res) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Query Global Localization");
    if (global_map == nullptr) {
      RCLCPP_WARN_STREAM(this->get_logger(), "No Globalmap");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.cloud, *cloud);
    cloud = downsample(cloud, this->declare_parameter<double>("query_downsample_resolution", 0.5));

    auto results = engine->query(cloud, req.max_num_candidates);

    res.inlier_fractions.resize(results.results.size());
    res.errors.resize(results.results.size());
    res.poses.resize(results.results.size());

    res.header = req.cloud.header;
    res.globalmap_header = globalmap_header;

    for (int i = 0; i < results.results.size(); i++) {
      const auto& result = results.results[i];
      Eigen::Quaternionf quat(result->pose.linear());
      Eigen::Vector3f trans(result->pose.translation());

      res.inlier_fractions[i] = result->inlier_fraction;
      res.errors[i] = result->error;
      res.poses[i].orientation.x = quat.x();
      res.poses[i].orientation.y = quat.y();
      res.poses[i].orientation.z = quat.z();
      res.poses[i].orientation.w = quat.w();

      res.poses[i].position.x = trans.x();
      res.poses[i].position.y = trans.y();
      res.poses[i].position.z = trans.z();
    }

    return !results.results.empty();
  }
};
}  // namespace hdl_global_localization