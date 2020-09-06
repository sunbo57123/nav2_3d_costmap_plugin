

#include "pcl_conversions/pcl_conversions.h"
#include "nav2_3d_costmap_plugin/nav2_3d_buffer.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "pcl/common/transforms.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/PCLPointCloud2.h"
#include "geometry_msgs/msg/point_stamped.h"


namespace buffer
{
    nav23dBuffer::nav23dBuffer(
        tf2_ros::Buffer & tf2_buffer,
        std::shared_ptr<rclcpp::Node> nh,
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
        // use nav2 default observation, pc2 reading is suppo  rted
        observation_list_.push_front(nav2_costmap_2d::Observation());

        // const std::string origin_frame =_sensor_frame == "" ? cloud.header.frame_id : _sensor_frame;

        min_obstacle_height_ = 0;

        try
        {
            // point_cloud_ptr ----std unique pointer <point cloud 2>
            sensor_msgs::msg::PointCloud2 global_frame_cloud;
            point_cloud_ptr cld_global(new sensor_msgs::msg::PointCloud2);

            // filter part
            pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
            pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

            pcl_conversions::toPCL( *cld_global, *cloud_pcl);
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud_pcl);
            sor.setFilterFieldName("z");
            sor.setFilterLimits(min_obstacle_height_, max_obstacle_height_);
            sor.setDownsampleAllData(false);

//            float v_s = static_cast<float>(voxel_size_);

            sor.setLeafSize(0.01f, 0.01f, 0.01f);
//            sor.setLeafSize(v_s, v_s, v_s);

//            sor.setMinimumPointsNumberPerVoxel(static_cast<unsigned int>(voxel_min_points_));
            sor.filter(*cloud_filtered);
            pcl_conversions::fromPCL(*cloud_filtered, *cld_global);
//            observation_list_.front()._cloud.reset(global_frame_cloud.release());

        }
        catch (tf2::TransformException & ex)
        {
            observation_list_.pop_front();
            return;
        }
        last_updated_ = nh_->now();
//        purgeStaleObservations();
    }
//    // same with obstacle; need to be changed
//    void nav23dBuffer::purgeStaleObservations()
//    {
//    if (!observation_list_.empty()) {
//        std::list<Observation>::iterator obs_it = observation_list_.begin();
//        // if we're keeping observations for no time... then we'll only keep one observation
//        if (observation_keep_time_ == rclcpp::Duration(0.0)) {
//        observation_list_.erase(++obs_it, observation_list_.end());
//        return;
//        }
//
//        // otherwise... we'll have to loop through the observations to see which ones are stale
//        for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
//        Observation & obs = *obs_it;
//        // check if the observation is out of date... and if it is,
//        // remove it and those that follow from the list
//        if ((last_updated_ - obs.cloud_->header.stamp) > observation_keep_time_) {
//            observation_list_.erase(obs_it, observation_list_.end());
//            return;
//        }
//        }
//    }
}





