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
        node_->get_parameter(name_ + "." + source + "." + "topic", topic);
    }

}
// read parameters in initialize func
void
Nav23dStaticLayer::onInitialize()
{
    // get parameters
    getParameters();
    // set QoS

    // create an observation buffer
    observation_buffers_.push_back(
      std::shared_ptr<>(
        new  (
          node_, topic, observation_keep_time, expected_update_rate,
          min_obstacle_height,
          max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
          sensor_frame, transform_tolerance)));

    // check if we'll add this buffer to our marking observation buffers
    if (marking) {
      marking_buffers_.push_back(observation_buffers_.back());
    }

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing) {
      clearing_buffers_.push_back(observation_buffers_.back());
    }   
     
    // instance new buffer
    observation_buffers_.push_back(new buffer::nav23dBuffer)
    // subscribe data 
    // 传入数据：command line node; topic, qos
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub(
        new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(rclcpp_node_, topic, custom_qos_profile));
    // message filter :filter
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> filter(
        new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(*sub, *tf_, _global_frame, 50, rclcpp_node_));

    // 预设值函数
    // _observation_buffers.back() 存储获取到的信息值 pc2 信息
    filter->registerCallback(
        std::bind(&Nav23dStaticLayer::PointCloud2Callback, this, _1, _observation_buffers.back()));

    _observation_subscribers.push_back(sub);
    _observation_notifiers.push_back(filter);

}



/*****************************************************************************/
void Nav23dStaticLayer::PointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<buffer::MeasurementBuffer> & buffer)
/*****************************************************************************/
{
  // buffer the point cloud
  buffer->Lock();
  // 将点云信息预处理
  buffer->BufferROSCloud(*message);
  buffer->Unlock();
}

void
Nav23dStaticLayer::getParameters()
{
// get node parameters here 
// declareparameter and get parameters

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
void
Nav23dStaticLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y)
{
    // 初始化中对observation进行了定义
    // 此处读取 observation接收到的点云信息
    bool current = true;
    std::vector<Observation> observations, clearing_observations;

    // get the marking observations
    current = current && getMarkingObservations(observations);

    // get the clearing observations
    current = current && getClearingObservations(clearing_observations);

    // update the global current status
    current_ = current;

    // 遍历observation 完成update

    
}
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