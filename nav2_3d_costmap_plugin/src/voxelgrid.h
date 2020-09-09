//
// Created by sun on 2020/9/7.
//

#ifndef NAV2TEST_VOXELGRID_H
#define NAV2TEST_VOXELGRID_H

#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "openvdb/openvdb.h"
#include "openvdb/tools/GridTransformer.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud2.h"

//TODO:change namespace name for clear specification
namespace grid
{
    class nav2_3d_grid
    {
    public:
        nav2_3d_grid(
            std::shared_ptr<rclcpp::Node> node,
            const double & bg_value,
            const float & voxel_size
                );
        ~nav2_3d_grid(void);
        void mark(sensor_msgs::msg::PointCloud2 & pc2);
        sensor_msgs::msg::PointCloud2 convert();

    protected:
        void onInitializeGrid(void);
        std::shared_ptr<rclcpp::Node> node_;
        mutable openvdb::DoubleGrid::Ptr voxel_grid_;
        std::unique_ptr<std::vector<geometry_msgs::msg::Point32>> grid_points_;
        double bg_value_;
        float voxel_size_;

};
}

#endif //NAV2TEST_VOXELGRID_H
