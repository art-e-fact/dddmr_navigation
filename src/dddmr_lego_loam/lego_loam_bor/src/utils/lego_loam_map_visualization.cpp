#include <cmath>
#include "lego_loam_map_visualization.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <pcl/filters/voxel_grid.h>


using namespace std::chrono_literals;
LegoLoamVisualization::LegoLoamVisualization(std::string name) : Node(name)
{
  has_m2ci_ = false;
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
  clock_ = this->get_clock();
  key_frame_clouds_.clear();

  this->declare_parameter<double>("voxel_leaf_size", 0.2);
  this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

  pubMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_map", 1);  
  pubGround = this->create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_ground", 1);  

  rclcpp::SubscriptionOptions sub_options;
  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_options.callback_group = cbs_group_;
  // Initialize PoseArray subscriber
  cloudKeyPoses6D_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_keypose_6d", 1, std::bind(&LegoLoamVisualization::cloudKeyPoses6D_callback, this, std::placeholders::_1), sub_options);
  m2ci_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "lego_loam/m2ci", 1, std::bind(&LegoLoamVisualization::m2ci_callback, this, std::placeholders::_1), sub_options);

  // Initialize Service Client
 
  get_key_frame_cloud_client_ = this->create_client<dddmr_sys_core::srv::GetKeyFrameCloud>("get_key_frame_cloud", rmw_qos_profile_services_default, cbs_group_);

  // Initialize Timer
  sync_ground_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&LegoLoamVisualization::sync_ground_timer_callback, this), cbs_group_);

  // Initialize Ground Publish Timer
  map_publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&LegoLoamVisualization::pubMapTimer, this), cbs_group_);

}

void LegoLoamVisualization::m2ci_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  trans_m2ci_af3_ = tf2::transformToEigen(*msg);
  has_m2ci_ = true;
}

void LegoLoamVisualization::cloudKeyPoses6D_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
  pcl::fromROSMsg(*msg, *cloudKeyPoses6D);
}

void LegoLoamVisualization::sync_ground_timer_callback()
{

  auto request = std::make_shared<dddmr_sys_core::srv::GetKeyFrameCloud::Request>();
  request->key_frame_number = key_frame_clouds_.size();
  
  if(request->key_frame_number>=cloudKeyPoses6D->size())
  {
   return; 
  }

  if (!get_key_frame_cloud_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Service get_key_frame_cloud not available");
    return;
  }

  get_key_frame_cloud_client_->async_send_request(request, 
    [this](rclcpp::Client<dddmr_sys_core::srv::GetKeyFrameCloud>::SharedFuture future) {
      try {
        auto result = future.get();
        pcl::PointCloud<PointType> pcl_cloud;
        pcl::PointCloud<PointType> pcl_ground_cloud;
        pcl::PointCloud<PointType> pcl_ground_edge_cloud;

        pcl::fromROSMsg(result->key_frame_cloud, pcl_cloud);
        if(pcl_cloud.points.size()<1){
          RCLCPP_INFO(this->get_logger(), "Empty key frame1");
          return;
        }
        pcl::fromROSMsg(result->key_frame_ground, pcl_ground_cloud);
        if(pcl_ground_cloud.points.size()<1){
          RCLCPP_INFO(this->get_logger(), "Empty key frame2");
          return;
        }

        pcl::fromROSMsg(result->key_frame_ground_edge, pcl_ground_edge_cloud);
        if(pcl_ground_edge_cloud.points.size()<1){
          RCLCPP_INFO(this->get_logger(), "Empty key frame3");
          return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Sync key frame number: %lu with total size: %lu", key_frame_clouds_.size(), cloudKeyPoses6D->size());
        key_frame_clouds_.push_back(pcl_cloud);
        key_frame_ground_clouds_.push_back(pcl_ground_cloud);
        key_frame_ground_edge_clouds_.push_back(pcl_ground_edge_cloud);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }
    });

}

void LegoLoamVisualization::pubMapTimer()
{
  if(!has_m2ci_)
    return;

  pcl::PointCloud<PointType> map_cloud;
  pcl::PointCloud<PointType> ground_cloud;
  size_t cnt = 0;
  for(auto it=key_frame_clouds_.begin();it!=key_frame_clouds_.end();it++){
    pcl::PointCloud<PointType> one_frame_map_cloud;
    one_frame_map_cloud= *transformPointCloud((*it).makeShared(), &cloudKeyPoses6D->points[cnt]);
    pcl::transformPointCloud(one_frame_map_cloud, one_frame_map_cloud, trans_m2ci_af3_);
    map_cloud+=one_frame_map_cloud;
    cnt++;
  }
  cnt = 0;
  for(auto it=key_frame_ground_clouds_.begin();it!=key_frame_ground_clouds_.end();it++){
    pcl::PointCloud<PointType> one_frame_map_cloud;
    one_frame_map_cloud= *transformPointCloud((*it).makeShared(), &cloudKeyPoses6D->points[cnt]);
    pcl::transformPointCloud(one_frame_map_cloud, one_frame_map_cloud, trans_m2ci_af3_);
    ground_cloud+=one_frame_map_cloud;
    cnt++;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg_map;
  pcl::toROSMsg(map_cloud, cloud_msg_map);
  cloud_msg_map.header.stamp = clock_->now();
  cloud_msg_map.header.frame_id = "map";
  pubMap->publish(cloud_msg_map);

  sensor_msgs::msg::PointCloud2 cloud_msg_ground;
  pcl::PointCloud<PointType>::Ptr content_ground_ptr(new pcl::PointCloud<PointType>(ground_cloud));
  pcl::PointCloud<PointType> content_ground_filtered;
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(content_ground_ptr);
  sor.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  sor.filter(content_ground_filtered);

  pcl::toROSMsg(content_ground_filtered, cloud_msg_ground);
  cloud_msg_ground.header.stamp = clock_->now();
  cloud_msg_ground.header.frame_id = "map";
  pubGround->publish(cloud_msg_ground);
}


pcl::PointCloud<PointType>::Ptr LegoLoamVisualization::transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn) {

  pcl::PointCloud<PointType>::Ptr cloudOut2(new pcl::PointCloud<PointType>());
  
  Eigen::Affine3f af3_yaw = Eigen::Affine3f::Identity();
  af3_yaw.rotate (Eigen::AngleAxisf (transformIn->yaw, Eigen::Vector3f::UnitZ()));
  Eigen::Affine3f af3_roll = Eigen::Affine3f::Identity();
  af3_roll.rotate (Eigen::AngleAxisf (transformIn->roll, Eigen::Vector3f::UnitX()));
  Eigen::Affine3f af3_pitch = Eigen::Affine3f::Identity();
  af3_pitch.rotate (Eigen::AngleAxisf (transformIn->pitch, Eigen::Vector3f::UnitY()));
  Eigen::Affine3f af3_translation = Eigen::Affine3f::Identity();
  af3_translation.translation() << transformIn->x, transformIn->y, transformIn->z;
  pcl_opt::transformPointCloudSequentially(*cloudIn, *cloudOut2, af3_yaw.matrix(), af3_roll.matrix(), af3_pitch.matrix(), af3_translation.matrix());

  return cloudOut2;
}