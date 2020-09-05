//
// Created by sun on 2020/9/3.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_3d_costmap_plugin/nav2_3d_buffer.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pcl/io/pcd_io.h"

using namespace std::chrono_literals;
using namespace buffer;

class pc2publisher: public rclcpp::Node
{
//    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
//    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = node_.lock();

public:
    pc2publisher(): Node("pc2_publisher"), count_(0)
{
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc2test", 10);
    timer_ = this->create_wall_timer( 500ms , std::bind( & pc2publisher::pc2_callback, this));
}

private:
void pc2_callback()
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCDReader reader;
    reader.read("table_scene_lms400.pcd", *cloud);
//    add buffer
    nav23dBuffer::bufferCloud(cloud);
    publisher_->publish(cloud);
}

rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
size_t count_;
};


int main(int argc, char ** argv)
{
//    parameters pass to buffer
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = node_.lock();
    tf2_ros::Buffer & tf2_;
//    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
    std::string global_frame;
    std::string topic_name = "pc2test";
    std::list<nav2_costmap_2d::Observation> observation_list;
    std::recursive_mutex lock;
    double observation_keep_time = 10;
    double expected_update_rate = 1;
    double min_obstacle_height = 0;
    double max_obstacle_height = 0;
    double obstacle_range = 0;
    double tf_tolerance = 0;


// create nav2 3d buffer
    std::vector<std::shared_ptr<nav23dBuffer>> buffers_;
    buffers_.push_back(
            std::shared_ptr<nav23dBuffer>(
                    new nav23dBuffer(
                            tf2_,node,global_frame,topic_name,observation_list,lock,observation_keep_time,
                            expected_update_rate,min_obstacle_height,max_obstacle_height,obstacle_range,tf_tolerance
                            )
                    )
            );
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("process");
//    node->create_publisher<sensor_msgs::msg::PointCloud2>("pc2test", 10);
//    rclcpp::spin(node->get_node_base_interface());
    rclcpp::spin(std::make_shared<pc2publisher>());
    rclcpp::shutdown();
    return 0;
}

//read pcd
