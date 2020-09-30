//
// Created by sun on 2020/9/25.
//



#ifndef SIMPLE_PLUGIN_SIMPLE_PLUGIN_HPP
#define SIMPLE_PLUGIN_SIMPLE_PLUGIN_HPP

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
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/PCLPointCloud2.h"

namespace simple_plugin{

        class SimpleLayer : public nav2_costmap_2d::ObstacleLayer
        {
        public:
            SimpleLayer();
            virtual ~SimpleLayer();

            virtual void onInitialize();

            virtual void convertTo2D(sensor_msgs::msg::PointCloud2 cloud);
            virtual void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud);

            virtual void showParamers();

            virtual void updateBounds(
                    double robot_x, double robot_y, double robot_yaw, double * min_x,
                    double * min_y, double * max_x, double * max_y);

            virtual void updateCosts(
                    nav2_costmap_2d::Costmap2D & master_grid,
                    int min_i, int min_j, int max_i, int max_j);

        private:
            std::string file_path_;
            std::string topic_name_;

            nav2_costmap_2d::Costmap2D map_2d_;
            unsigned int map_size_x;
            unsigned int map_size_y;

            int mark_threshold_;
            unsigned char lethal_threshold_;
//            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr msg_subscriber_;
            double map_resolution_;

        };



}

#endif //SIMPLE_PLUGIN_SIMPLE_PLUGIN_HPP

