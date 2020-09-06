//
// Created by sun on 2020/9/3.
//

//subscriber and pc2 callback
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_3d_costmap_plugin/nav2_3d_buffer.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "pcl/io/pcd_io.h"
#include "tf2_ros/message_filter.h"

void PointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<buffer::nav23dBuffer> & buffer)
    {
        // buffer the point cloud
        buffer->lock();
        // 将点云信息预处理
        buffer->bufferCloud(*message);
        buffer->unlock();
    }

//similar with initialize
int main(int argc, char ** argv)
{
//    parameters pass to buffer
//    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
//    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = node_.lock();
    rclcpp::init(argc, argv);

//    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("_", options);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("nodesub");
    tf2_ros::Buffer * tf2_;
    std::string global_frame = "map";
    std::string topic_name = "pc2test";
    std::list<nav2_costmap_2d::Observation> observation_list;
    double observation_keep_time = 10;
    double expected_update_rate = 1;
    double min_obstacle_height = 0.08;
    double max_obstacle_height = 0.4;
    double obstacle_range = 2.5;
    double tf_tolerance = 0.3;

    // set QoS
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

// create nav2 3d buffer
    std::vector<std::shared_ptr<buffer::nav23dBuffer>> buffers_;
    rclcpp::Node::SharedPtr rclcpp_node_;
    buffers_.push_back(
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
    rclcpp::Node::SharedPtr subNode = rclcpp::Node::make_shared("subscriber");
    // subscribe data
    // 传入数据：command line node; topic, qos
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub(
            new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(subNode, topic_name, custom_qos_profile));
    // message filter :filter
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> filter(
            new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(*sub, *tf2_, global_frame, 50, subNode));

    // 预设值函数
    // _observation_buffers.back() 存储获取到的信息值 pc2 信息
    filter->registerCallback(
            std::bind(PointCloud2Callback, std::placeholders::_1, buffers_.back()));

    int nosense = 0;


}

//read pcd
