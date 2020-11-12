//
// Created by sun on 2020/11/3.
//

#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "gtest/gtest.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "nav2_3d_static_layer/nav2_3d_static_layer.hpp"

#include "../test_header.hpp"

using namespace std::chrono_literals;

//defined a rclcppfixture
class RclCppFixture
{
public:
    RclCppFixture() {rclcpp::init(0, nullptr);}
    ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

//class TestLifecycleNode : public nav2_util::LifecycleNode
//{
//public:
//    explicit TestLifecycleNode(const std::string & name)
//            : nav2_util::LifecycleNode(name)
//    {
//    }
//
//    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
//    {
//        return nav2_util::CallbackReturn::SUCCESS;
//    }
//
//    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
//    {
//        return nav2_util::CallbackReturn::SUCCESS;
//    }
//
//    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
//    {
//        return nav2_util::CallbackReturn::SUCCESS;
//    }
//
//    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
//    {
//        return nav2_util::CallbackReturn::SUCCESS;
//    }
//
//    nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
//    {
//        return nav2_util::CallbackReturn::SUCCESS;
//    }
//
//    nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
//    {
//        return nav2_util::CallbackReturn::SUCCESS;
//    }
//};
// create PC2Publisher for publishing 0.5s/pc2 info
class PC2Publisher : public rclcpp::Node
{
    public:
        PC2Publisher(): Node("pc2_publisher")
        {
            readPCD(_pc2);
            _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc2_info", 10);
            _timer = this->create_wall_timer( 500ms, std::bind(&PC2Publisher::timer_callback, this));
        }

        void CreatePC2Msg()
        {
            sensor_msgs::msg::PointCloud2 test_pc2;
        }

        void readPCD(sensor_msgs::msg::PointCloud2 pc2)
        {
            pcl::PCLPointCloud2 pclpc2;
            std::string filePath = "/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/nav2_3d_costmap_plugin/test/map/diagonalMap.pcd";
            pcl::PCDReader reader;
            reader.read(
                    filePath,
                    pclpc2
                    );
            pcl_conversions::fromPCL(pclpc2, pc2);
        }

    private:
        void timer_callback()
        {
            RCLCPP_INFO(
                    this->get_logger(),
                    "sending..."
                    );
            _publisher->publish(_pc2);
        }

        rclcpp::TimerBase::SharedPtr _timer;
        sensor_msgs::msg::PointCloud2 _pc2;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;

};

class MainNode :public nav2_util::LifecycleNode
{
public:
    MainNode():nav2_util::LifecycleNode("main_node")
    {
    }

    ~MainNode() {}

protected:
    void publishMap();
    void addLayer();
    void multiThread();

};

void MainNode::publishMap() {
    std::shared_ptr<PC2Publisher> map_pub = std::make_shared<PC2Publisher>();
    rclcpp::spin(map_pub);
}

void MainNode::addLayer(){
    std::shared_ptr<nav2_util::LifecycleNode> node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");

    node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");
    tf2_ros::Buffer tf(node_->get_clock());
    nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
    layers.resizeMap(10,10, 0.05, 0.0, 0.0);
    std::shared_ptr<nav2_3d_static_layer::StaticLayer3D> nlayer = std::make_shared<nav2_3d_static_layer::StaticLayer3D>();
    nlayer->initialize(&layers, "static3d", &tf, node_, nullptr, nullptr /*TODO*/);

//    layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(nlayer));
//    addObservation(nlayer, 0.0, 0.0, 1.0 / 2, 0, 0, 1.0 / 2);
//    layers.updateMap(0, 0, 0);
//    nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
//    printMap(*costmap);
}

class TestNode :public ::testing::Test
{
public:
    TestNode() {

        using SingleThreadedExecutor = rclcpp::executors::SingleThreadedExecutor;

//        SingleThreadedExecutor executor_pub;
//        std::shared_ptr<PC2Publisher> map_pub = std::make_shared<PC2Publisher>();
//        executor_pub.add_node(map_pub);
//        std::thread pub_thread(std::bind(&SingleThreadedExecutor::spin, &executor_pub));

        SingleThreadedExecutor executor_sub;
        std::shared_ptr<nav2_util::LifecycleNode> main_node = std::make_shared<nav2_util::LifecycleNode>("main_node");
        executor_sub.add_node(main_node->get_node_base_interface());

        std::thread sub_thread(std::bind(&SingleThreadedExecutor::spin, &executor_sub));

//        pub_thread.join();
        sub_thread.join();

        rclcpp::shutdown();
    }

    ~TestNode() {}


};


TEST_F(TestNode, testTemplate){


}




