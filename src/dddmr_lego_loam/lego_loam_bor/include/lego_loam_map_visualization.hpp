#ifndef LEGO_LOAM_VISUALIZATION_HPP_
#define LEGO_LOAM_VISUALIZATION_HPP_


#include <dddmr_sys_core/srv/get_key_frame_cloud.hpp>
#include "utility.h"
//optimized pcl transform
#include "transforms.hpp"

class LegoLoamVisualization : public rclcpp::Node
{
public:
  LegoLoamVisualization(std::string name);

private:
  
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudKeyPoses6D_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m2ci_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGround;
  
  void m2ci_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
  void cloudKeyPoses6D_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void sync_ground_timer_callback();
  void pubMapTimer();
  pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

  rclcpp::TimerBase::SharedPtr sync_ground_timer_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  rclcpp::Client<dddmr_sys_core::srv::GetKeyFrameCloud>::SharedPtr get_key_frame_cloud_client_;
  rclcpp::CallbackGroup::SharedPtr cbs_group_;

  std::vector<pcl::PointCloud<PointType>> key_frame_clouds_;
  std::vector<pcl::PointCloud<PointType>> key_frame_ground_clouds_;
  std::vector<pcl::PointCloud<PointType>> key_frame_ground_edge_clouds_;
  std::string base_frame_;

  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  Eigen::Affine3d trans_m2ci_af3_;
  bool has_m2ci_;
  double voxel_leaf_size_;
  
};

#endif  // LEGO_LOAM_VISUALIZATION_HPP_
