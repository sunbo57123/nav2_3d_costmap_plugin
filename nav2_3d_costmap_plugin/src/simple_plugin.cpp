//
// Created by sun on 2020/9/25.
//

#include <string>

#include "simple_plugin/simple_plugin.hpp"

#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "octomap/octomap.h"
#include "nav2_costmap_2d/layer.hpp"

#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/compression/octree_pointcloud_compression.h"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace simple_plugin
{
    SimpleLayer::SimpleLayer(){
        map_2d_ = nav2_costmap_2d::Costmap2D();
    }

    SimpleLayer::~SimpleLayer(){}

// read parameters in initialize func
    void
    SimpleLayer::onInitialize()
    {
        nav2_costmap_2d::ObstacleLayer::onInitialize();
        RCLCPP_INFO(
                logger_,
                "humuhumunukunukuapuaa is loading 3d static map"
        );

        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("mark_threshold", rclcpp::ParameterValue(0));
        declareParameter("topic_name", rclcpp::ParameterValue("humuhumunukunukuapuaa"));
        declareParameter("lethal_threshold", rclcpp::ParameterValue(0));
        declareParameter("map_resolution", rclcpp::ParameterValue(0.0));
//  <<<<<<<for test>>>>>>
        declareParameter("file_path", rclcpp::ParameterValue(""));

        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        node ->get_parameter(name_ + "." + "enabled", enabled_);
        node ->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);
        node ->get_parameter(name_ + "." + "topic_name", topic_name_);
        node ->get_parameter(name_ + "." + "lethal_threshold", lethal_threshold_);
        node ->get_parameter(name_ + "." + "map_resolution", map_resolution_);

        node ->get_parameter(name_ + "." + "file_path", file_path_);

        rolling_window_ = layered_costmap_->isRolling();
        default_value_ = NO_INFORMATION;
        global_frame_ = layered_costmap_->getGlobalFrameID();

//<<<<< qos settings should be specified >>>>>>
        rclcpp::QoS map_qos(10);  // initialize to default
//<<<<< creating pc2 messages part as a map_server replacement >>>>>>
        RCLCPP_INFO(
                logger_,
                "load pc"
        );
        pcl::PCLPointCloud2::Ptr cloud_file (new pcl::PCLPointCloud2 ());
        pcl::PCDReader reader;
        reader.read(
                "filepath/origin.pcd",
                * cloud_file
        );
        std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_pc2(new sensor_msgs::msg::PointCloud2());
        pcl_conversions::fromPCL(*cloud_file, *cloud_pc2);
        cloudCallback(cloud_pc2);
//            read from buffer once not continuous
//        msg_subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
//                topic_name_,
//                map_qos,
//                std::bind(&SimpleLayer::cloudCallback, this, std::placeholders::_1));
    }

    void
    SimpleLayer::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud)
    {
        RCLCPP_INFO(
                logger_,
                "loading callback"
        );
        showParamers();
        Costmap2D * master = layered_costmap_->getCostmap();
        map_size_x = master->getSizeInCellsX();
        map_size_y = master->getSizeInCellsY();
        double resolution = master->getResolution();
//origin 0,0 should be changed

        map_2d_ = nav2_costmap_2d::Costmap2D(map_size_x, map_size_y, resolution, 0, 0);
        convertTo2D(*pointcloud);
        map_2d_.saveMap("filepath/map_2d.pmg");
        if (master->saveMap("filepath/humuhumunukunukuapuaa.pmg")){
            RCLCPP_INFO(
                    logger_,
                    "humuhumunukunukuapuaa saved "
            );
        }
    }

    void SimpleLayer::convertTo2D(sensor_msgs::msg::PointCloud2 cloud)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        pcl::PCLPointCloud2 cloud_pcl;
        pcl_conversions::toPCL(cloud, cloud_pcl);
        pcl::fromPCLPointCloud2(cloud_pcl, cloud_xyz);

        octomap::OcTree octotree (0.1);
        RCLCPP_INFO(
                logger_,
                "show information of pointcloud: width:%d, height: %d",
                cloud_pcl.width, cloud_pcl.height
        );
//!!!!!----------!!!!!!!!!----------
        float octo_resolution = 0.8;
//!!!!!----------!!!!!!!!!----------

        for (auto p:cloud_xyz.points)
        {
            octotree.updateNode(octomap::point3d(p.x*octo_resolution, p.y*octo_resolution, p.z*octo_resolution), true);
        }
        octotree.updateInnerOccupancy();
        int oct_max_x, oct_max_y, oct_max_z, oct_min_x, oct_min_y, oct_min_z= 0;
        for(octomap::OcTree::iterator it = octotree.begin(); it != octotree.end(); ++it) {
            if (octotree.isNodeOccupied(*it)) {
                auto coord = it.getCoordinate();
                oct_max_x = std::max(oct_max_x,  (int) coord.x());
                oct_max_y = std::max(oct_max_y,  (int) coord.y());
                oct_max_z = std::max(oct_max_z,  (int) coord.z());
                oct_min_x = std::min(oct_min_x,  (int) coord.x());
                oct_min_y = std::min(oct_min_y,  (int) coord.y());
                oct_min_z = std::min(oct_min_z,  (int) coord.z());
            }
        }
        RCLCPP_INFO(
                logger_,
                "<<<<!!!!>>>>show information of octomap: max_x: %d,min_x: %d,max_y: %d,min_y: %d, max_z: %d, min_z: %d",
                oct_max_x,oct_min_x, oct_max_y, oct_min_y, oct_max_z, oct_min_z
        );

        double pc_resolution_x = map_size_x / (oct_max_x - oct_min_x);
        double pc_resolution_y = map_size_y / (oct_max_y - oct_min_y);
        RCLCPP_INFO(
                logger_,
                "<<<<!!!!>>>>show information of octomap: pc_resolution_x: %d,pc_resolution_y: %d",
                pc_resolution_x,pc_resolution_y
        );

        for(octomap::OcTree::iterator it = octotree.begin(); it != octotree.end(); ++it){
            if(octotree.isNodeOccupied(*it))
            {
                auto coord = it.getCoordinate();
                int point_x = (int)coord.x() - oct_min_x;
                int point_y = (int)coord.y() - oct_min_y;
                int point_z = (int)coord.z();
                unsigned int x = point_x * pc_resolution_x;
                unsigned int y = point_y * pc_resolution_y;
                if (x > map_size_x || y >map_size_y ){
                    RCLCPP_INFO(
                            logger_,
                            "<<<<!!!!>>>>over: x: %d,y: %d",
                            x,y
                    );
                    break;
                }
                else{
                    if (point_z > 1) {
                        map_2d_.setCost(x,y,LETHAL_OBSTACLE);
                    }else if(point_z < 1){
                        map_2d_.setCost(x,y,FREE_SPACE);
                    }
                }
            }
        }
    }

    void
    SimpleLayer::showParamers()
    {
        RCLCPP_INFO(
                logger_,
                "show some parameters"
        );
//        Costmap2D::resetMaps();
        rolling_window_ = layered_costmap_->isRolling();
        global_frame_ = layered_costmap_->getGlobalFrameID();
        Costmap2D * master = layered_costmap_->getCostmap();
        unsigned int size_x = master->getSizeInCellsX();
        unsigned int size_y = master->getSizeInCellsY();
        double resolution = master->getResolution();
        double meterx = master->getSizeInMetersX();
        double metery = master->getSizeInMetersY();
        unsigned char devalue = master->getDefaultValue();
        double originx= master->getOriginX();
        double originy = master->getOriginY();
        RCLCPP_INFO(
                logger_,
                "show some parameters: global_frame: %s, size x: %d, size y: %d ",
                global_frame_.c_str(), size_x, size_y
        );
        RCLCPP_INFO(
                logger_,
                "show some parameters: resolution: %d, Msize x: %d, Msize y: %d, default value: %s ",
                resolution, meterx, metery, devalue
        );
        RCLCPP_INFO(
                logger_,
                "show some parameters: originx: %d, originy: %d",
                originx, originy
        );

        //log:
//[controller_server-8] [INFO] [1601384403.991757568] [local_costmap.local_costmap]: show some parameters
//[controller_server-8] [INFO] [1601384403.993547181] [local_costmap.local_costmap]: show some parameters: global_frame: odom, size x: 60, size y: 60
//[controller_server-8] [INFO] [1601384403.993616945] [local_costmap.local_costmap]: show some parameters: resolution: 0, Msize x: -1205715725, Msize y: 0, default value: ��#�
//[controller_server-8] [INFO] [1601384403.993675415] [local_costmap.local_costmap]: show some parameters: originx: -1205715725, originy: -1083615920
//i dont understand this extremely large number
    }

    void
    SimpleLayer::updateBounds(
            double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
            double * min_y, double * max_x, double * max_y)
    {
        RCLCPP_INFO(
                logger_,
                ">>>>>>>>>>>>>>>>>>>>>>>>>>>updateBounds"
        );
//        RCLCPP_INFO(
//                logger_,
//                "show robot position: x: %d, y:%d ",
//                robot_x, robot_y
//        );

//        useExtraBounds(min_x, min_y, max_x, max_y);

        for (unsigned int i = 0; i <map_size_x; ++i){
            for (unsigned int j = 0; j <map_size_y; ++j){
                costmap_[getIndex(i, j)] = map_2d_.getCost(i, j);
                touch(i, j,  min_x, min_y, max_x, max_y);
            }
        }
        Costmap2D * master = layered_costmap_->getCostmap();

        double resolution = master->getResolution();
        double meterx = master->getSizeInMetersX();
        double metery = master->getSizeInMetersY();
        unsigned char devalue = master->getDefaultValue();

        RCLCPP_INFO(
                logger_,
                "show some parameters: resolution: %d, Msize x: %d, Msize y: %d, default value: %s ",
                resolution, meterx, metery, devalue
        );

//        RCLCPP_INFO(
//                logger_,
//                "show robot position: %s %s  %s %s ",
//                std::to_string(*min_x), std::to_string(*min_y), std::to_string(*max_x), std::to_string(*max_y)
//        );

    }


    void
    SimpleLayer::updateCosts(
            nav2_costmap_2d::Costmap2D & master_grid,
            int min_i, int min_j, int max_i, int max_j)
    {
        RCLCPP_INFO(
                logger_,
                ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>updateCosts"
        );
//        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);

    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(simple_plugin::SimpleLayer, nav2_costmap_2d::Layer)
