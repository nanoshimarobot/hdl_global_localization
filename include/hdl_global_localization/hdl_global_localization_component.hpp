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

  rclcpp::Service<hdl_global_localization_msgs::srv::SetGlobalLocalizationEngine>::SharedPtr set_engine_service_;
  rclcpp::Service<hdl_global_localization_msgs::srv::SetGlobalMap>::SharedPtr set_global_map_service_;
  rclcpp::Service<hdl_global_localization_msgs::srv::QueryGlobalLocalization>::SharedPtr query_service_;

public:
  GlobalLocalizationNode(const rclcpp::NodeOptions& options) : GlobalLocalizationNode("", options) {}
  GlobalLocalizationNode(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("hdl_global_localization_node", name_space, options) {
    set_engine(this->declare_parameter<std::string>("global_localization_engine", "FPFH_RANSAC"));

    set_engine_service_ = this->create_service<srv::SetGlobalLocalizationEngine>(
      "set_engine",
      std::bind(&GlobalLocalizationNode::set_engine_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_global_map_service_ =
      this->create_service<srv::SetGlobalMap>("set_global_map", std::bind(&GlobalLocalizationNode::set_global_map_callback, this, std::placeholders::_1, std::placeholders::_2));
    query_service_ =
      this->create_service<srv::QueryGlobalLocalization>("query", std::bind(&GlobalLocalizationNode::query_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  bool set_engine(const std::string& engine_name) {
    if (engine_name == "BBS") {
      engine.reset(new GlobalLocalizationBBS(this));
    } else if (engine_name == "FPFH_RANSAC") {
      engine.reset(new GlobalLocalizationEngineFPFH_RANSAC(this));
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

  void set_engine_callback(const srv::SetGlobalLocalizationEngine::Request::SharedPtr req, const srv::SetGlobalLocalizationEngine::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Set Global Localization Engine");
    set_engine(req->engine_name.data);
  }

  void set_global_map_callback(srv::SetGlobalMap::Request::SharedPtr req, srv::SetGlobalMap::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Global Map Received");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req->global_map, *cloud);
    cloud = downsample(cloud, this->declare_parameter<double>("globalmap_downsample_resolution", 0.5));

    globalmap_header = req->global_map.header;
    global_map = cloud;
    engine->set_global_map(global_map);

    RCLCPP_INFO_STREAM(this->get_logger(), "DONE");
  }

  void query_callback(srv::QueryGlobalLocalization::Request::SharedPtr req, srv::QueryGlobalLocalization::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Query Global Localization");
    if (global_map == nullptr) {
      RCLCPP_WARN_STREAM(this->get_logger(), "No Globalmap");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req->cloud, *cloud);
    cloud = downsample(cloud, this->declare_parameter<double>("query_downsample_resolution", 0.5));

    auto results = engine->query(cloud, req->max_num_candidates);

    res->inlier_fractions.resize(results.results.size());
    res->errors.resize(results.results.size());
    res->poses.resize(results.results.size());

    res->header = req->cloud.header;
    res->globalmap_header = globalmap_header;

    for (int i = 0; i < results.results.size(); i++) {
      const auto& result = results.results[i];
      Eigen::Quaternionf quat(result->pose.linear());
      Eigen::Vector3f trans(result->pose.translation());

      res->inlier_fractions[i] = result->inlier_fraction;
      res->errors[i] = result->error;
      res->poses[i].orientation.x = quat.x();
      res->poses[i].orientation.y = quat.y();
      res->poses[i].orientation.z = quat.z();
      res->poses[i].orientation.w = quat.w();

      res->poses[i].position.x = trans.x();
      res->poses[i].position.y = trans.y();
      res->poses[i].position.z = trans.z();
    }

    // return !results.results.empty();
  }
};
}  // namespace hdl_global_localization