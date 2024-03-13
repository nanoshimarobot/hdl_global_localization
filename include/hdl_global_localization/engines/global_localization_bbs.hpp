#ifndef HDL_GLOBAL_LOCALIZATION_BBS_HPP
#define HDL_GLOBAL_LOCALIZATION_BBS_HPP

#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <hdl_global_localization/engines/global_localization_engine.hpp>

namespace hdl_global_localization {

class BBSLocalization;

class GlobalLocalizationBBS : public GlobalLocalizationEngine {
public:
  GlobalLocalizationBBS(rclcpp::Node::SharedPtr node_ptr);
  virtual ~GlobalLocalizationBBS() override;

  virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;

  virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;

private:
  using Points2D = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
  Points2D slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr unslice(const Points2D& points);

protected:
  rclcpp::Node::SharedPtr node_ptr_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridmap_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_slice_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_slice_pub;

  std::unique_ptr<BBSLocalization> bbs;
};

}  // namespace hdl_global_localization

#endif