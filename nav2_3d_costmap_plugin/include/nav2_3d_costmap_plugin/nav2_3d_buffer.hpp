
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "tf2_ros/buffer.h"

namespace buffer
{
    typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> point_cloud_ptr;
    // voxel grid
    class nav23dBuffer
    {
        public:
            nav23dBuffer(
                // paramters initialization
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
//            nav23dBuffer();
            ~nav23dBuffer();
            // buffer PointCloud2
            void bufferCloud(const sensor_msgs::msg::PointCloud2 & cloud);

            // lock the observation buffer
            inline void lock()
            {
                lock_.lock();
            }
            inline void unlock()
            {
                lock_.unlock();
            }

            // uncertain
            // check if the obs buffer is being update at its expected rate
            bool isCurrent() const;
            void getobservation(std::vector<buffer::nav23dBuffer> & observations);
            void resetLastUpdated();
            bool setGlobalFrame(const std::string new_global_frame);

        private:
            // uncertain 
            void purgeStaleObservations();

            rclcpp::Clock::SharedPtr clock_;
            tf2_ros::Buffer & tf2_buffer_;
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh_;
//            rclcpp::Node::SharedPtr nh_;

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