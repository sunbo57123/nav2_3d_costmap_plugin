

#include "pcl_conversions/pcl_conversions.h"
#include "nav2_3d_costmap_plugin/nav2_3d_buffer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/observation.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/PCLPointCloud2.h"

namespace buffer
{
    nav23dBuffer::nav23dBuffer(
        tf2_ros::Buffer & tf2_buffer,
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
        double observation_keep_time,
        double expected_update_rate,
        std::string global_frame,
        std::string topic_name,
        std::list<nav2_costmap_2d::Observation> observation_list,
        double min_obstacle_height,
        double max_obstacle_height,
        double obstacle_range,
        double tf_tolerance
        )
    : tf2_buffer_(tf2_buffer),
      nh_(nh),
      observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),
      expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),
      last_updated_(nh_->now()),
      global_frame_(global_frame),
      topic_name_(topic_name),
      min_obstacle_height_(min_obstacle_height),
      max_obstacle_height_(max_obstacle_height),
      obstacle_range_(obstacle_range),
      tf_tolerance_(tf_tolerance)
    {
    }

    nav23dBuffer::~nav23dBuffer()
    {
    }

    void nav23dBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud)
    {
        // use nav2 default observation, pc2 reading is supported
        observation_list_.push_front(nav2_costmap_2d::Observation());

        min_obstacle_height_ = 0;
//      The reference is STVL
//      Is there any other function in openvdb could be used for downsampling?
//      TODO: Trying pruneGrid in openvdb and making comparison with pcl-voxelgrid.

        try
        {
            // point_cloud_ptr ----std unique pointer <point cloud 2>
            sensor_msgs::msg::PointCloud2 global_frame_cloud;
            point_cloud_ptr cld_global(new sensor_msgs::msg::PointCloud2);

            pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
            pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

            pcl_conversions::toPCL( *cld_global, *cloud_pcl);
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud_pcl);
            sor.setFilterFieldName("z");
            sor.setFilterLimits(min_obstacle_height_, max_obstacle_height_);
            sor.setDownsampleAllData(false);
            sor.setLeafSize(0.01f, 0.01f, 0.01f);
            sor.filter(*cloud_filtered);
            pcl_conversions::fromPCL(*cloud_filtered, *cld_global);
        }
        catch (tf2::TransformException & ex)
        {
            observation_list_.pop_front();
            return;
        }
        last_updated_ = nh_->now();
    }
}





