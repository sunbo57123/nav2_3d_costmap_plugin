#include "nav2_3d_costmap_plugin/nav2_3d_costmap.hpp"
#include "nav2_3d_costmap_plugin/nav2_3d_buffer.hpp"


#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_3d_costmap_plugin
{
Nav23dStaticLayer::Nav23dStaticLayer()
{
}

Nav23dStaticLayer::~Nav23dStaticLayer()
{
}
void
Nav23dStaticLayer::getParameters()
{
    declareParameter("", rclcpp::ParameterValue());
//   ??? no lock function in node_(lifecyclenode) ???
// user node_ as replacement
    auto node = node_.lock();
    std::string topics_string;
    node ->get_parameter(name_ + "." + "observation_sources", topics_string);
    rolling_window_ = layered_costmap_->isRolling();

    default_value_ = NO_INFORMATION;
    global_frame_ = layered_costmap_->getGlobalFrameID();

    std::stringstream ss(topics_string);
    std::string source;
    while (ss >> source)
    {
        std::string topic;
        declareParameter(source + "." + "topic",rclcpp::ParameterValue(source));
        node->get_parameter(name_ + "." + source + "." + "topic", topic);
    }

}
// read parameters in initialize func
void
Nav23dStaticLayer::onInitialize()
{
    // get parameters
//    getParameters();
//    parameters pass to buffer
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = node_.lock();
    tf2_ros::Buffer * tf2_;
//    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
    std::string global_frame;
    std::string topic_name = "pc2test";
    std::list<nav2_costmap_2d::Observation> observation_list;
    double observation_keep_time = 10;
    double expected_update_rate = 1;
    double min_obstacle_height = 0;
    double max_obstacle_height = 0;
    double obstacle_range = 0;
    double tf_tolerance = 0;
    // set QoS
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;
    // create an observation buffer
    observation_buffers_.push_back(
      std::shared_ptr<buffer::nav23dBuffer>(
        new buffer::nav23dBuffer(
                *tf2_,
                node,
                observation_keep_time,
                expected_update_rate,
                global_frame,
                topic_name,
                observation_list,
                min_obstacle_height,
                max_obstacle_height,
                obstacle_range,
                tf_tolerance
          )));

    // subscribe data 
    // 传入数据：command line node; topic, qos
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub(
        new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(rclcpp_node_, topic_name, custom_qos_profile));
    // message filter :filter
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> filter(
        new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(*sub, *tf_, global_frame, 50, rclcpp_node_));

    // 预设值函数
    // _observation_buffers.back() 存储获取到的信息值 pc2 信息
    filter->registerCallback(
        std::bind(&Nav23dStaticLayer::PointCloud2Callback, this, std::placeholders::_1, observation_buffers_.back()));

}

void Nav23dStaticLayer::PointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<buffer::nav23dBuffer> & buffer)
{
  // buffer the point cloud
  buffer->lock();
  // 将点云信息预处理
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void
Nav23dStaticLayer::activate()
{
}

void
Nav23dStaticLayer::deactivate()
{
}

void 
Nav23dStaticLayer::reset()
{
}
// 更新bounds
//void
//Nav23dStaticLayer::updateBounds(
//    double robot_x, double robot_y, double robot_yaw, double * min_x,
//    double * min_y, double * max_x, double * max_y)
//{
//    // 初始化中对observation进行了定义
//    // 此处读取 observation接收到的点云信息
//    bool current = true;
//    std::vector<Observation> observations, clearing_observations;
//
//    // get the marking observations
//    current = current && getMarkingObservations(observations);
//
//    // get the clearing observations
//    current = current && getClearingObservations(clearing_observations);
//
//    // update the global current status
//    current_ = current;
//
//    // 遍历observation 完成update
//
//
//}
// 更新bounds内的cost
void
Nav23dStaticLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
{
  // 遍历observation 完成update
}
// 更新static map
void
Nav23dStaticLayer::matchSize()
{
  // 从observations获取到地图信息（occupancy grid）更新static layer信息
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_3d_costmap_plugin::Nav23dStaticLayer, nav2_costmap_2d::CostmapLayer)