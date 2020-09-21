

#include "nav2_3d_costmap_plugin/nav2_3d_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "octomap/octomap.h"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"


using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_3d_costmap_plugin
{
    Nav23dStaticLayer::Nav23dStaticLayer(){
        map_2d_ = nav2_costmap_2d::Costmap2D();
    }

Nav23dStaticLayer::~Nav23dStaticLayer(){}

// read parameters in initialize func
void
Nav23dStaticLayer::onInitialize()
{
    nav2_costmap_2d::ObstacleLayer::onInitialize();

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("mark_threshold", rclcpp::ParameterValue(0));
    declareParameter("topic_name", rclcpp::ParameterValue(""));
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
    node ->get_parameter(name_ + "." +"map_resolution", map_resolution_);

    node ->get_parameter(name_ + "." + "file_path", file_path_);

    rolling_window_ = layered_costmap_->isRolling();
    default_value_ = NO_INFORMATION;
    global_frame_ = layered_costmap_->getGlobalFrameID();
//<<<<< qos settings should be changed >>>>>>

    rclcpp::QoS map_qos(10);  // initialize to default

    //    read from buffer once not continuous
    msg_subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name_,
            map_qos,
            std::bind(&Nav23dStaticLayer::cloudCallback, this, std::placeholders::_1));
}

void
Nav23dStaticLayer::cloudCallback(sensor_msgs::msg::PointCloud2 pointcloud)
{
//        add mutex?
    nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
//    get master size
    unsigned  int master_x = master->getSizeInCellsX();
    unsigned  int master_y = master->getSizeInCellsY();

    map_2d_ = nav2_costmap_2d::Costmap2D(master_x, master_y, map_resolution_, origin_x_, origin_y_);
    convertTo2D(pointcloud);
    updateMap();
    }


void Nav23dStaticLayer::convertTo2D(sensor_msgs::msg::PointCloud2 cloud)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::PCLPointCloud2 cloud_pcl;
    pcl_conversions::toPCL(cloud, cloud_pcl);
    pcl::fromPCLPointCloud2(cloud_pcl, cloud_xyz);

    octomap::OcTree treeTest (0.1);
    for (auto p:cloud_xyz.points)
    {
        treeTest.updateNode(octomap::point3d(p.x*0.8f, p.y*0.8f, p.z*0.8f), true);
    }
    treeTest.updateInnerOccupancy();

//    map_2d_ = nav2_costmap_2d::Costmap2D(300,300,0.05,0,0);

    std::vector<octomap::point3d> data;
    for(octomap::OcTree::iterator it = treeTest.begin(); it != treeTest.end(); ++it){
        if(treeTest.isNodeOccupied(*it))
        {
            auto coord = it.getCoordinate();

            int point_x = (int)coord.x() + 100;
            int point_y = (int)coord.y() + 100;
            int point_z = (int)coord.z();
            if (point_z > 5) {
                map_2d_.setCost(point_x, point_y, LETHAL_OBSTACLE);
            }else if(point_z < 5){
                map_2d_.setCost(point_x, point_y, FREE_SPACE);
            }
            data.push_back(coord);
        }
    }
}

void
Nav23dStaticLayer::updateMap()
{
    costmap_ = map_2d_.getCharMap();
}

void
Nav23dStaticLayer::updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
    double * min_y, double * max_x, double * max_y)
{
    if (!enabled_) {
        return;
    }
}

void
Nav23dStaticLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
{

}

}



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_3d_costmap_plugin::Nav23dStaticLayer, nav2_costmap_2d::ObstacleLayer)