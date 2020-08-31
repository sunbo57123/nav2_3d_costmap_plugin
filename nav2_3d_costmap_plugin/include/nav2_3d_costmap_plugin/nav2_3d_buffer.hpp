#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace buffer
{

    // voxel grid
    class nav23dBuffer
    {
        public:
            nav23dBuffer(
                // paramters initialization
            );
            ~nav23dBuffer(void);
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
            void getobservation(std::vector<Observation> & observations);
            void resetLastUpdated();
            bool setGlobalFrame(const std::string new_global_frame);

        private:
            // uncertain 
            void purgeStaleObservations();

            tf2_ros::Buffer & tf2_buffer_;
            nav2_util::LifecycleNode::SharedPtr nh_;

            const rclcpp::Duration observation_keep_time_, expected_update_rate_;
            rclcpp::Time last_updated_;

            std::string global_frame_;
            std::string topic_name_;
            std::list<Observation> observation_list_;
            
            std::recursive_mutex lock_;
            
            // uncertain
            double min_obstacle_height, max_obstacle_height;
            double obstacle_range_, raytrace_range_;
            double tf_tolerance_;

    };
}