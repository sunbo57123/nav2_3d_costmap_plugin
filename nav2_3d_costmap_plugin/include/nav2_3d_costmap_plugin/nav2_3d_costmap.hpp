#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace nav2_3d_costmap_plugin
{
    class Nav23dStaticLayer : public nav2_costmap_2d::CostmapLayer
    {
        public:
            Nav23dStaticLayer();
            virtual ~Nav23dStaticLayer();

            virtual void onInitialize();
            virtual void activate();
            virtual void deactivate();
            virtual void reset();

            virtual void updateBounds(
                double robot_x, double robot_y, double robot_yaw, double * min_x,
                double * min_y, double * max_x, double * max_y);

            virtual void updateCosts(
                nav2_costmap_2d::Costmap2D & master_grid,
                int min_i, int min_j, int max_i, int max_j);

            virtual void matchSize();
        private:
            std::vector<std::shared_ptr<message_filters::SubscriberBase>> observation_subscribers_;
            std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
            std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> observation_buffers_;
            std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> marking_buffers_;
            std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> clearing_buffers_;
        
    };

}
