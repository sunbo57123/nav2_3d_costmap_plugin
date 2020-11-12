//
// Created by sun on 2020/11/4.
//

#ifndef NAV2_3D_STATIC_LAYER_TEST_HEADER_HPP
#define NAV2_3D_STATIC_LAYER_TEST_HEADER_HPP

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/observation.hpp"

void printMap(nav2_costmap_2d::Costmap2D & costmap)
{
    printf("map:\n");
    for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
        for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
            printf("%4d", static_cast<int>(costmap.getCost(j, i)));
        }
        printf("\n\n");
    }
}

void addStaticLayer(
        nav2_costmap_2d::LayeredCostmap & layers,
        tf2_ros::Buffer & tf, nav2_util::LifecycleNode::SharedPtr node,
        std::shared_ptr<nav2_costmap_2d::StaticLayer> & slayer)
{
    slayer = std::make_shared<nav2_costmap_2d::StaticLayer>();
    layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(slayer));
    slayer->initialize(&layers, "static", &tf, node, nullptr, nullptr /*TODO*/);
}

void addObservation(
        std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer, double x, double y, double z = 0.0,
        double ox = 0.0, double oy = 0.0, double oz = 1.0, bool marking = true, bool clearing = true)
{
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    *iter_x = x;
    *iter_y = y;
    *iter_z = z;

    geometry_msgs::msg::Point p;
    p.x = ox;
    p.y = oy;
    p.z = oz;

    // obstacle range = raytrace range = 100.0
    nav2_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);
    olayer->addStaticObservation(obs, marking, clearing);
}

#endif //NAV2_3D_STATIC_LAYER_TEST_HEADER_HPP
