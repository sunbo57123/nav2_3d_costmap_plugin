#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/PCLPointCloud2.h"

namespace nav2_3d_costmap_plugin
{
class Nav23dStaticLayer : public nav2_costmap_2d::ObstacleLayer
    {
        public:
            Nav23dStaticLayer();
            virtual ~Nav23dStaticLayer();

            virtual void onInitialize();
            virtual void activate();
            virtual void deactivate();
            virtual void reset();

            virtual void convertTo2D(sensor_msgs::msg::PointCloud2);

            virtual void updateBounds(
                double robot_x, double robot_y, double robot_yaw, double * min_x,
                double * min_y, double * max_x, double * max_y);

//            virtual void updateCosts(
//                nav2_costmap_2d::Costmap2D & master_grid,
//                int min_i, int min_j, int max_i, int max_j);

            virtual void matchSize();

            void PointCloud2Callback(
                sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
                const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);
        private:
            bool rolling_window_;
            std::string global_frame_;
            std::string topic_name_;
//            nav2_voxel_grid::VoxelGrid map_3d_;
//            std::shared_ptr<nav2_costmap_2d::Costmap2D> map_2d_;
            nav2_costmap_2d::Costmap2D map_2d_;
            int mark_threshold_;
            std::vector<std::shared_ptr<message_filters::SubscriberBase>> observation_subscribers_;
            std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
            std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> observation_buffers_;


        void getParameters();
    };

}
