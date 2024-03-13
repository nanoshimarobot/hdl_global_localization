#include <hdl_global_localization/hdl_global_localization_component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hdl_global_localization::GlobalLocalizationNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}