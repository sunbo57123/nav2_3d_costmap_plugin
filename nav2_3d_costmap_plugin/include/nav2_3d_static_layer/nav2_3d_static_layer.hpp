//
// Created by sun on 2020/10/10.
//

#ifndef NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP
#define NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace nav2_3d_static_layer
{

    class Nav23dStaticLayer : public nav2_costmap_2d::ObstacleLayer
    {
        /*
         *
         */
    public:
        Nav23dStaticLayer();
        virtual ~Nav23dStaticLayer();

        virtual void onInitialize();

        virtual void updateBounds(
                double robot_x, double robot_y, double robot_yaw, double * min_x,
                double * min_y, double * max_x, double * max_y);

        virtual void updateCosts(
                nav2_costmap_2d::Costmap2D & master_grid,
                int min_i, int min_j, int max_i, int max_j);

        virtual void readPC(std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_pc2);

        virtual void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud);

        virtual void convertTo2d(sensor_msgs::msg::PointCloud2 cloud);
    private:
        nav2_costmap_2d::Costmap2D map_2d_;
        unsigned int map_size_x_;
        unsigned int map_size_y_;
        double map_resolution_;
        double origin_x_;
        double origin_y_;
        /*
         *  mark_threshold_ : >> ?? <<
         */
        unsigned char lethal_threshold_;
        std::string topic_name_;

        /*
         * voxel grid parameters
         */
        float voxel_leafsize_;
        double min_z_height_;
        double max_z_height_;
    };

}

#endif //NAV2_3D_STATIC_LAYER_NAV2_3D_STATIC_LAYER_HPP
