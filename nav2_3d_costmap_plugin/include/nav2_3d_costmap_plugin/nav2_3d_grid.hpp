//
// Created by sun on 2020/9/12.
//

#ifndef NAV2_3D_COSTMAP_PLUGIN_NAV2_3D_GRID_HPP
#define NAV2_3D_COSTMAP_PLUGIN_NAV2_3D_GRID_HPP

#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "openvdb/openvdb.h"
#include "openvdb/tools/GridTransformer.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud2.h"

namespace nav2_3d_grid
{
    class vdbBasicGrid{
    public:
        vdbBasicGrid(
                std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                const double & bg_value,
                const float & voxel_size
        );
        ~vdbBasicGrid(void);
        void mark(sensor_msgs::msg::PointCloud2 & pc2);
        void convert();
    protected:
        void onInitializeGrid(void);
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        mutable openvdb::DoubleGrid::Ptr voxel_grid_;
        std::shared_ptr<std::map<openvdb::Vec2d, uint>> map_2d_;
        std::unique_ptr<std::vector<geometry_msgs::msg::Point32>> grid_points_;
        double bg_value_;
        float voxel_size_;
    };
}

#endif //NAV2_3D_COSTMAP_PLUGIN_NAV2_3D_GRID_HPP
