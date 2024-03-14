#include <hdl_global_localization/engines/global_localization_bbs.hpp>

#include <hdl_global_localization/bbs/bbs_localization.hpp>
#include <hdl_global_localization/bbs/occupancy_gridmap.hpp>

namespace hdl_global_localization {

GlobalLocalizationBBS::GlobalLocalizationBBS(rclcpp::Node* node_ptr) : node_ptr_(node_ptr) {
  gridmap_pub = node_ptr_->create_publisher<nav_msgs::msg::OccupancyGrid>("bbs.gridmap", 1);
  map_slice_pub = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("bbs.map_slice", 1);
  scan_slice_pub = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("bbs.scan_slice", 1);
}

GlobalLocalizationBBS ::~GlobalLocalizationBBS() {}

void GlobalLocalizationBBS::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  BBSParams params;
  params.max_range = node_ptr_->declare_parameter<double>("bbs.max_range", 15.0);
  params.min_tx = node_ptr_->declare_parameter<double>("bbs.min_tx", -10.0);
  params.max_tx = node_ptr_->declare_parameter<double>("bbs.max_tx", 10.0);
  params.min_ty = node_ptr_->declare_parameter<double>("bbs.min_ty", -10.0);
  params.max_ty = node_ptr_->declare_parameter<double>("bbs.max_ty", 10.0);
  params.min_theta = node_ptr_->declare_parameter<double>("bbs.min_theta", -3.15);
  params.max_theta = node_ptr_->declare_parameter<double>("bbs.max_theta", 3.15);
  bbs.reset(new BBSLocalization(params));

  double map_min_z = node_ptr_->declare_parameter<double>("bbs.map_min_z", 2.0);
  double map_max_z = node_ptr_->declare_parameter<double>("bbs.map_max_z", 2.4);
  auto map_2d = slice(*cloud, map_min_z, map_max_z);
  RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Set Map " << map_2d.size() << " points");

  if (map_2d.size() < 128) {
    RCLCPP_WARN_STREAM(node_ptr_->get_logger(), "Num points in the sliced map is too small!!");
    RCLCPP_WARN_STREAM(node_ptr_->get_logger(), "Change the slice range parameters!!");
  }

  int map_width = node_ptr_->declare_parameter<int>("bbs.map_width", 1024);
  int map_height = node_ptr_->declare_parameter<int>("bbs.map_height", 1024);
  double map_resolution = node_ptr_->declare_parameter<double>("bbs.map_resolution", 0.5);
  int map_pyramid_level = node_ptr_->declare_parameter<int>("bbs.map_pyramid_level", 6);
  int max_points_per_cell = node_ptr_->declare_parameter<int>("bbs.max_points_per_cell", 5);
  bbs->set_map(map_2d, map_resolution, map_width, map_height, map_pyramid_level, max_points_per_cell);

  auto map_3d = unslice(map_2d);
  sensor_msgs::msg::PointCloud2 map_3d_msg;
  pcl::toROSMsg(*map_3d, map_3d_msg);
  map_3d_msg.header.frame_id = "map";
  map_slice_pub->publish(map_3d_msg);
  gridmap_pub->publish(*(bbs->gridmap()->to_rosmsg()));
}

GlobalLocalizationResults GlobalLocalizationBBS::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  double scan_min_z = node_ptr_->declare_parameter<double>("bbs.scan_min_z", -0.2);
  double scan_max_z = node_ptr_->declare_parameter<double>("bbs.scan_max_z", 0.2);
  auto scan_2d = slice(*cloud, scan_min_z, scan_max_z);

  std::vector<GlobalLocalizationResult::Ptr> results;

  RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Query " << scan_2d.size() << " points");
  if (scan_2d.size() < 32) {
    RCLCPP_WARN_STREAM(node_ptr_->get_logger(), "Num points in the sliced scan is too small!!");
    RCLCPP_WARN_STREAM(node_ptr_->get_logger(), "Change the slice range parameters!!");
    return GlobalLocalizationResults(results);
  }

  double best_score = 0.0;
  auto trans_2d = bbs->localize(scan_2d, 0.0, &best_score);
  if (!trans_2d) {
    return GlobalLocalizationResults(results);
  }

  if (scan_slice_pub->get_subscription_count() > 0) {
    auto scan_3d = unslice(scan_2d);
    sensor_msgs::msg::PointCloud2 scan_3d_msg;
    pcl::toROSMsg(*scan_3d, scan_3d_msg);
    scan_3d_msg.header.frame_id = cloud->header.frame_id;
    scan_3d_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(cloud->header.stamp * 1000));
    scan_slice_pub->publish(scan_3d_msg);
  }

  Eigen::Isometry3f trans_3d = Eigen::Isometry3f::Identity();
  trans_3d.linear().block<2, 2>(0, 0) = trans_2d->linear();
  trans_3d.translation().head<2>() = trans_2d->translation();

  results.resize(1);
  results[0].reset(new GlobalLocalizationResult(best_score, best_score, trans_3d));

  return GlobalLocalizationResults(results);
}

GlobalLocalizationBBS::Points2D GlobalLocalizationBBS::slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const {
  Points2D points_2d;
  points_2d.reserve(cloud.size());
  for (int i = 0; i < cloud.size(); i++) {
    if (min_z < cloud.at(i).z && cloud.at(i).z < max_z) {
      points_2d.push_back(cloud.at(i).getVector3fMap().head<2>());
    }
  }
  return points_2d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalLocalizationBBS::unslice(const Points2D& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(points.size());
  for (int i = 0; i < points.size(); i++) {
    cloud->at(i).getVector3fMap().head<2>() = points[i];
    cloud->at(i).z = 0.0f;
  }

  return cloud;
}
}  // namespace hdl_global_localization