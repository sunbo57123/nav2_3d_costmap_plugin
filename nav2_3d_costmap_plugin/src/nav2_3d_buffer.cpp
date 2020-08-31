// set filter here voxel grid or passthrough in pcl filters
// make pointcloud processing  easy
#include "include\nav2_3d_buffer.hpp"
namespace buffer
{
    nav2_3d_buffer::nav23dBuffer(
        tf2_ros::Buffer & tf2_buffer,
        nav2_util::LifecycleNode::SharedPtr nh,
        const rclcpp::Duration observation_keep_time, 
        const rclcpp::Duration expected_update_rate,
        std::string global_frame,
        std::string topic_name,
        std::list<Observation> observation_list,
        std::recursive_mutex lock,
        double min_obstacle_height,
        double max_obstacle_height,
        double obstacle_range,
        double raytrace_range,
        double tf_tolerance)
    : tf2_buffer_(tf2_buffer),
      nh_(nh),
      observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),
      expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),
      last_updated_(nh->now()),
      global_frame_(global_frame),
      topic_name_(topic_name),
      min_obstacle_height_(min_obstacle_height),
      max_obstacle_height_(max_obstacle_height),
      obstacle_range_(obstacle_range),
      raytrace_range_(raytrace_range),
      tf_tolerance_(tf_tolerance)
    {
    }

    nav2_3d_buffer::~nav23dBuffer()
    {

    }

    void buffer::bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud)
    {
        // use nav2 default observation, pc2 reading is supported
        observation_list_.push_front(Observation());

        // const std::string origin_frame =_sensor_frame == "" ? cloud.header.frame_id : _sensor_frame;

        try
        {
            // transform into global frame
            geometry_msgs::msg::PointStamped global_origin, local_origin;
            local_origin.header.stamp = cloud.header.stamp;
            local_origin.header.frame_id = origin_frame_;
            local_origin.point.x = 0;
            local_origin.point.y = 0;
            local_origin.point.z = 0;
            tf2_buffer_.transform(local_origin, global_origin, global_frame_);
            tf2::convert(global_origin.point, observation_list_.front().origin_);

            // std::list<Observation> observation_list_;
            // uncertain
            observation_list_.front().raytrace_range_ = raytrace_range_;
            observation_list_.front().obstacle_range_ = obstacle_range_;

            sensor_msgs::msg::PointCloud2 global_frame_cloud;

            // filter part
            pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
            pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

            pcl_conversions::toPCL(*global_frame_cloud, *cloud_pcl);
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud_pcl);
            sor.setFilterFieldName("z");
            sor.setFilterLimits(min_obstacle_height_, max_obstacle_height_);
            sor.setDownsampleAllData(false);
            float v_s = static_cast<float>(voxel_size_);
            sor.setLeafSize(v_s, v_s, v_s);
            sor.setMinimumPointsNumberPerVoxel(static_cast<unsigned int>(voxel_min_points_));
            sor.filter(*cloud_filtered);
            pcl_conversions::fromPCL(*cloud_filtered, *cld_global);

            observation_list_.front()._cloud.reset(global_frame_cloud.release());

        }catch (tf2::TransformException & ex){
            observation_list_.pop_front();
            return;
        }
        last_updated_ = nh_->now();
        purgeStaleObservations();
    }
    // same with obstacle; need to be changed
    void ObservationBuffer::purgeStaleObservations()
    {
    if (!observation_list_.empty()) {
        std::list<Observation>::iterator obs_it = observation_list_.begin();
        // if we're keeping observations for no time... then we'll only keep one observation
        if (observation_keep_time_ == rclcpp::Duration(0.0)) {
        observation_list_.erase(++obs_it, observation_list_.end());
        return;
        }

        // otherwise... we'll have to loop through the observations to see which ones are stale
        for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
        Observation & obs = *obs_it;
        // check if the observation is out of date... and if it is,
        // remove it and those that follow from the list
        if ((last_updated_ - obs.cloud_->header.stamp) > observation_keep_time_) {
            observation_list_.erase(obs_it, observation_list_.end());
            return;
        }
        }
    }
}

    


}
