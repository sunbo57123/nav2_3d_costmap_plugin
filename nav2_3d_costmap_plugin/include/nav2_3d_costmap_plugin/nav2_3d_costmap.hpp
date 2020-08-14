#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

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
        
    };

}
