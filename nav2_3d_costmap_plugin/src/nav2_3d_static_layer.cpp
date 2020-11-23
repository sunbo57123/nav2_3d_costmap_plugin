//
// Created by sun on 2020/10/10.
//

#include "nav2_3d_static_layer/nav2_3d_static_layer.hpp"

#include "octomap/octomap.h"
#include "pcl/compression/octree_pointcloud_compression.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_3d_static_layer
{
    StaticLayer3D::StaticLayer3D() {
        map_2d_ = nav2_costmap_2d::Costmap2D();
    }
// TODO What 'noexcept' means in CPP?
    StaticLayer3D::~StaticLayer3D() noexcept {}

    void
    StaticLayer3D::onInitialize()
    {
        RCLCPP_INFO(
                logger_,
                "humuhumunukunukuapuaa is loading 3d static map"
        );
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("topic_name", rclcpp::ParameterValue("pc2_info"));
        declareParameter("lethal_threshold", rclcpp::ParameterValue(2.0));
        declareParameter("voxel_leafsize", rclcpp::ParameterValue(0.2));
        declareParameter("min_z_height", rclcpp::ParameterValue(0.0));
        declareParameter("max_z_height", rclcpp::ParameterValue(3.0));

        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        node ->get_parameter(name_ + "." + "enabled", enabled_);
        node ->get_parameter(name_ + "." + "topic_name", topic_name_);
        node ->get_parameter(name_ + "." + "lethal_threshold", lethal_threshold_);
        node ->get_parameter(name_ + "." + "voxel_leafsize", voxel_leafsize_);
        node ->get_parameter(name_ + "." + "min_z_height", min_z_height_);
        node ->get_parameter(name_ + "." + "max_z_height", max_z_height_);

        RCLCPP_INFO(
                logger_,
                "subcribing to: %s ",topic_name_.c_str()
        );

        rolling_window_ = layered_costmap_->isRolling();
        default_value_ = NO_INFORMATION;
        global_frame_ = layered_costmap_->getGlobalFrameID();
        map_received_ = false;
        /*
         * TODO:QoS part should be more specific
         * the observation function could be test when map server ready
         */
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_qos.reliable();
        map_qos.keep_last(1);
        std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_pc2(new sensor_msgs::msg::PointCloud2());

        _subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_name_, map_qos,
                std::bind(&StaticLayer3D::cloudCallback, this, std::placeholders::_1));
    }

    void
    StaticLayer3D::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud)
    {
        /*
         *TODO: add observation part
         */
        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        if(!map_received_)
        {
            map_received_ = true;
            RCLCPP_INFO(
                    logger_,
                    "here"
                    );
        }

        if (layered_costmap_->getCostmap()){
            Costmap2D * master = layered_costmap_->getCostmap();
            map_resolution_ = master->getResolution();
            map_size_x_ = master->getSizeInMetersX()/map_resolution_;
            map_size_y_ = master->getSizeInMetersY()/map_resolution_;
            origin_x_ = master->getOriginX();
            origin_y_ = master->getOriginY();

            map_2d_ = nav2_costmap_2d::Costmap2D(map_size_x_,
                                                 map_size_y_,
                                                 map_resolution_,
                                                 origin_x_,
                                                 origin_y_);
            RCLCPP_INFO(
                    logger_,
                    "<<<<humuhumunukunukuapuaa>>>>created 2d map with size: %d X %d, resolution: %lf, origin: (%lf, %lf)",
                    map_size_x_, map_size_y_, map_resolution_, origin_x_, origin_y_
            );
        }
        else{
            map_resolution_ = 0.05;
            map_size_x_ = 100;
            map_size_y_ = 100;
            origin_x_ = 0.0;
            origin_y_ = 0.0;

        };
//        convertTo2d(*pointcloud);
        filteredPoints(*pointcloud);
        map_2d_.saveMap("/home/sun/navigation2/src/navigation2/simple_bringup/2_simple_result.pmg");
    }

    bool
    StaticLayer3D::receivedMap(){
        if(map_received_){
            RCLCPP_INFO(
                    logger_,
                    "map has been received"
                    );
            return true;
        }
        else{
            RCLCPP_INFO(
                    logger_,
                    "NOT received"
            );
            return false;
        }
    }

    void
    StaticLayer3D::filteredPoints(sensor_msgs::msg::PointCloud2 cloud){
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        float scale = 0.1;
        for (; iter_x != iter_x.end();){
            for (; iter_y != iter_y.end(); ){
                if (*iter_z >= lethal_threshold_){
                    unsigned int map_x = (unsigned int) (*iter_x) * scale;
                    unsigned int map_y = (unsigned int) (*iter_y) * scale;
                    if (map_2d_.getCost(map_x, map_y) != LETHAL_OBSTACLE){
                        map_2d_.setCost(map_x, map_y, LETHAL_OBSTACLE);
                        RCLCPP_INFO(
                                logger_,
                                "set x %d, y %d is z %d", map_x, map_y, LETHAL_OBSTACLE
                                );
                    }
                }
                ++iter_z;
                ++iter_y;
                ++iter_x;
            }

        }

    }

    void
    StaticLayer3D::convertTo2d(sensor_msgs::msg::PointCloud2 cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(cloud, *cloud_pcl);
        pcl::fromPCLPointCloud2(*cloud_pcl, *cloud_xyz);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor_voxelgrid;
        sor_voxelgrid.setInputCloud(cloud_xyz);
        sor_voxelgrid.setLeafSize(0.2f, 0.2f, 0.2f);
        sor_voxelgrid.setFilterFieldName("z");
        sor_voxelgrid.setFilterLimits(0.0, 6.0);
        sor_voxelgrid.setFilterLimitsNegative(false);
        sor_voxelgrid.filter(*filtered_cloud);

        octomap::OcTree octotree (map_resolution_);

        for (auto p:filtered_cloud->points)
        {
            octotree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
        }
        octotree.updateInnerOccupancy();
        for(octomap::OcTree::iterator it = octotree.begin(), end=octotree.end();
        it != end; ++it){
            if(octotree.isNodeOccupied(*it))
            {
                auto coord = it.getCoordinate();
                /*
                 *
                 */
                // TODO CPP recommends static_cast<int>
                // TODO Be aware of the implicit accuracy loss
                // TODO Make it clear why the forum is like this:
                // Ref: https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/src/costmap_2d.cpp#L264
                int point_x = (int) (coord.x() - (origin_x_ - 0.5) / map_resolution_);
                int point_y = (int) (-(origin_y_ - 0.5) / map_resolution_ - coord.y());
                int point_z = (int) coord.z();

                // TODO What does the implicit conversion means?
                unsigned int x = point_x;
                unsigned int y = point_y;
                // TODO More conditions to validate the result?
                if (x > map_size_x_ || y > map_size_y_ ){
                    // TODO If the map is invalid, make a warning or error
                    // then break
                    RCLCPP_INFO(
                            logger_,
                            "<<<<humuhumunukunukuapuaa  the world map coord transformation>>>>over: x: %lf,y: %lf",
                            x,y
                    );
                    break;
                }
                else{
                    if (point_z > lethal_threshold_) {
                        map_2d_.setCost(x,y,LETHAL_OBSTACLE);
                    }else if(point_z < lethal_threshold_){
                        map_2d_.setCost(x,y,FREE_SPACE);
                    }
                }
            }
        }
    }
    void
    StaticLayer3D::updateBounds(
            double robot_x, double robot_y, double /*robot_yaw*/,
            double * min_x, double * min_y, double * max_x, double * max_y) {
        RCLCPP_INFO(
                logger_,
                ">>>>>>>>>>>>>>>>>>>>>>>>>>>updateBounds"
        );
        RCLCPP_INFO(
                logger_,
                "show robot position: x: %lf, y:%lf ",
                robot_x, robot_y
        );
        useExtraBounds(min_x, min_y, max_x, max_y);
        /*
         *
         */
        *min_x = - (map_size_x_ * map_resolution_);
        *min_y = - (map_size_y_ * map_resolution_);
        *max_x = map_size_x_ * map_resolution_;
        *max_y = map_size_y_ * map_resolution_;


    }

    void
    StaticLayer3D::updateCosts(
            nav2_costmap_2d::Costmap2D & master_grid,
            int min_i, int min_j, int max_i, int max_j)
    {
        RCLCPP_INFO(
                logger_,
                ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>updateCosts"
        );
        RCLCPP_INFO(
                logger_,
                "show cost bbox: %d %d  %d %d ",
                min_i, min_j, max_i, max_j
        );

        // TODO Only update the cost within the bounds
        for (unsigned int i = 0; i <max_i; ++i){
            for (unsigned int j = 0; j <max_j; ++j){
                master_grid.setCost(i, j, map_2d_.getCost(i, j));
            }
        }
    }

    void
    StaticLayer3D::activate() {


    }
    void
    StaticLayer3D::deactivate() {


    }
    void
    StaticLayer3D::reset() {


    }

}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_3d_static_layer::StaticLayer3D, nav2_costmap_2d::Layer)