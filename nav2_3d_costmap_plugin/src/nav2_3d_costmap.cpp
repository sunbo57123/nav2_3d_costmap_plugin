

#include "nav2_3d_costmap_plugin/nav2_3d_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_3d_costmap_plugin
{
    Nav23dStaticLayer::Nav23dStaticLayer(
            std::make_shared<nav2_costmap_2d::CostmapLayer>() map_2d
            ):map_2d_(map_2d) {}

Nav23dStaticLayer::~Nav23dStaticLayer(){}

// parameters declares
void
Nav23dStaticLayer::getParameters()
{
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("mark_threshold", rclcpp::ParameterValue(0));
    declareParameter("topic_name", rclcpp::ParameterValue(""));
//    declareParameter("", rclcpp::ParameterValue());

// user node_ as replacement
// <<<<< put try-catch here >>>>>
    auto node = node_.lock();

//    std::string topics_string;

    node ->get_parameter(name_ + "." + "enabled", enabled_);
    node ->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);
    node ->get_parameter(name_ + "." + "topic_name", topic_name_);
//        node ->get_parameter(name_ + "." + "observation_sources", topics_string);

    rolling_window_ = layered_costmap_->isRolling();
    default_value_ = NO_INFORMATION;
    global_frame_ = layered_costmap_->getGlobalFrameID();

//    std::stringstream ss(topics_string);
//    std::string source;
//    while (ss >> source)
//    {
//        std::string topic;
//        declareParameter(source + "." + "topic",rclcpp::ParameterValue(source));
//        node->get_parameter(name_ + "." + source + "." + "  topic", topic);
//    }

}
// read parameters in initialize func
void
Nav23dStaticLayer::onInitialize()
{
    nav2_costmap_2d::ObstacleLayer::onInitialize();
    getParameters();
//<<<<< qos settings should be changed >>>>>>
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

//    subscriber part
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub(
        new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(rclcpp_node_, topic_name_, custom_qos_profile));

    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> filter(
        new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(*sub, *tf_, global_frame_, 50, rclcpp_node_));

    filter->registerCallback(
        std::bind(&Nav23dStaticLayer::PointCloud2Callback, this, std::placeholders::_1, observation_buffers_.back()));

}

void
Nav23dStaticLayer::PointCloud2Callback(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
        const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
{
    pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL( *message, *cloud_pcl);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_pcl);
    sor.setFilterFieldName("z");
    sor.setFilterLimits(0, max_obstacle_height_);
    sor.setDownsampleAllData(true);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
    sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg;
    pcl_conversions::fromPCL(*cloud_filtered, *filtered_msg);

    buffer->lock();
    buffer->bufferCloud(*filtered_msg);
    buffer->unlock();
}

void Nav23dStaticLayer::convertTo2D(sensor_msgs::msg::PointCloud2 cloud)
{
//
// the process of convert sensor_msgs -> costmap2d
    unsigned int pc_width = cloud.width;
    unsigned int pc_length = cloud.height;
//    resize layered costmap


// iterator
    int mark_threshold_ = 10;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    unsigned int point_x = static_cast<unsigned int>(*iter_x);
    unsigned int point_y = static_cast<unsigned int>(*iter_y);
    unsigned int point_z = static_cast<unsigned int>(*iter_z);
//    simple convert process
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        unsigned int index = getIndex(point_x, point_y);
        if (point_z >= mark_threshold_){
            costmap_[index] = LETHAL_OBSTACLE;
            map_2d_.setCost(point_x, point_y, LETHAL_OBSTACLE);
        }else if(point_z < mark_threshold_){
            costmap_[index] = FREE_SPACE;
            map_2d_.setCost(point_x, point_y, FREE_SPACE);
        }
    }
}



void
Nav23dStaticLayer::updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
    double * min_y, double * max_x, double * max_y)
{

    if (!enabled_) {
        return;
    }
    std::vector<nav2_costmap_2d::Observation> observation;
    if (getMarkingObservations(observation))
    {
        for (std::vector<nav2_costmap_2d::Observation>::const_iterator it = observation.begin(); it != observation.end();
             ++it)
        {
            const nav2_costmap_2d::Observation & obs = *it;
            const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);
            convertTo2D(cloud);
        }

    }
}

//void
//Nav23dStaticLayer::updateCosts(
//    nav2_costmap_2d::Costmap2D & master_grid,
//    int min_i, int min_j, int max_i, int max_j)
//{
//}

void
Nav23dStaticLayer::matchSize()
{

}

}



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_3d_costmap_plugin::Nav23dStaticLayer, nav2_costmap_2d::ObstacleLayer)