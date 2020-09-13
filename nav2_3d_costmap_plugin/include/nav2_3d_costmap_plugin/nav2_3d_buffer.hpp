
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "tf2_ros/buffer.h"

// These part will be changed, this class should be a sub-class of nav2_costmap_2d:Observationbuffer.
namespace buffer
{
    typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> point_cloud_ptr;

    class nav23dBuffer
    {
        public:
            nav23dBuffer(
//                    TODO: parameters needed tobe changed, some are useless
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
            );
            ~nav23dBuffer();
            void bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud);

            inline void lock()
            {
                lock_.lock();
            }
            inline void unlock()
            {
                lock_.unlock();
            }


        private:

            rclcpp::Clock::SharedPtr clock_;
            tf2_ros::Buffer & tf2_buffer_;
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh_;

            const rclcpp::Duration observation_keep_time_, expected_update_rate_;
            rclcpp::Time last_updated_;

            std::string global_frame_;
            std::string topic_name_;
            std::list<nav2_costmap_2d::Observation> observation_list_;
            
            std::recursive_mutex lock_;
            
            // uncertain
            double min_obstacle_height_;
            double max_obstacle_height_;
            double obstacle_range_;
            double tf_tolerance_;

    };
}