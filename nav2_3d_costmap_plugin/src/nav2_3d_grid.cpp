//
// Created by sun on 2020/9/12.
//
#include <memory>
#include "nav2_3d_costmap_plugin/nav2_3d_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nav2_3d_grid
{
    vdbBasicGrid::vdbBasicGrid(
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
        const double &bg_value,
        const float &voxel_size):
        node_(node),
        bg_value_(bg_value),
        voxel_size_(voxel_size)
{
    this->onInitializeGrid();
}
    vdbBasicGrid::~vdbBasicGrid(){}

    void
    vdbBasicGrid::mark(sensor_msgs::msg::PointCloud2 & pc2)
    {
        sensor_msgs::PointCloud2ConstIterator<double> it_x(pc2, "x");
        sensor_msgs::PointCloud2ConstIterator<double> it_y(pc2, "y");
        sensor_msgs::PointCloud2ConstIterator<double> it_z(pc2, "z");

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
        {
            openvdb::Vec3d mark_grid(voxel_grid_->worldToIndex(openvdb::Vec3d(*it_x, *it_y, *it_z)));
            openvdb::DoubleGrid::Accessor MarkAccessor = voxel_grid_->getAccessor();
            openvdb::Coord MarkCoord(mark_grid[0],mark_grid[1],mark_grid[2]);
            MarkAccessor.setValue(MarkCoord, 9);
        }
    }

    void vdbBasicGrid::onInitializeGrid()
    {
        voxel_size_ = 0.05;
        openvdb::initialize();
        voxel_grid_ = openvdb::DoubleGrid::create(bg_value_);
        openvdb::Mat4d m = openvdb::Mat4d::identity();
        m.preScale(openvdb::Vec3d(voxel_size_, voxel_size_, voxel_size_));
        voxel_grid_->setTransform(openvdb::math::Transform::createLinearTransform(m));
        voxel_grid_->setName("Nav2TestVoxelGrid");
        voxel_grid_->insertMeta("Voxel Size", openvdb::FloatMetadata(voxel_size_));
        voxel_grid_->setGridClass(openvdb::GRID_LEVEL_SET);
    }

    void
    vdbBasicGrid::convert()
    {
//    create a fixed size 2d map (x, y) - value x[min_x, max_x] y[min_y, max_y]
//    map size is same as point cloud size? or static map
//    need to unify static map size and point cloud map size
//    how to unify: by landmarks or bbox ?

        map_2d_ = std::make_shared<std::map<openvdb::Vec2d, uint>>();
        openvdb::DoubleGrid::ValueOnCIter it_grid = voxel_grid_->cbeginValueOn();
        for (; it_grid.test() ; ++it_grid) {
            const openvdb::Coord point_coord(it_grid.getCoord());
            std::cout << point_coord <<std::endl;
            openvdb::Vec3d point_world = voxel_grid_->indexToWorld(point_coord);
//        x-point_world[0] y-point_world[1]
            std::map<openvdb::Vec2d, uint>::iterator it;
            openvdb::Vec2d current_point = openvdb::Vec2d (point_world[0], point_world[1]);
            it = map_2d_->find(current_point);
            if (it != map_2d_->end())
            {
                it->second++;
            }
            else
            {
                map_2d_->insert(std::make_pair(openvdb::Vec2d(point_world[0], point_world[1]), 1));
            }
        }

    }

}


