//
// Created by sun on 2020/9/7.
//

#include "voxelgrid.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
namespace grid
{
nav2_3d_grid::nav2_3d_grid(
        std::shared_ptr<rclcpp::Node> node,
        const double &bg_value,
        const float &voxel_size):
        node_(node),
        bg_value_(bg_value),
        voxel_size_(voxel_size)
{
    this->onInitializeGrid();
}

nav2_3d_grid::~nav2_3d_grid(void) {}

void nav2_3d_grid::mark(sensor_msgs::msg::PointCloud2 & pc2)
{
//       covert pc2 to VoxelGrid
//
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



void nav2_3d_grid::onInitializeGrid()
{
    voxel_size_ = 0.05;
    openvdb::initialize();
    voxel_grid_ = openvdb::DoubleGrid::create(bg_value_);
    openvdb::Mat4d m = openvdb::Mat4d::identity();
    m.preScale(openvdb::Vec3d(voxel_size_, voxel_size_, voxel_size_));
    m.preTranslate(openvdb::Vec3d(0,0,0));
    m.preRotate(openvdb::math::Z_AXIS, 0);

    voxel_grid_->setTransform(openvdb::math::Transform::createLinearTransform(m));
    voxel_grid_->setName("Nav2TestVoxelGrid");
    voxel_grid_->insertMeta("Voxel Size", openvdb::FloatMetadata(voxel_size_));
    voxel_grid_->setGridClass(openvdb::GRID_LEVEL_SET);

    int meaningless = 0;
}

sensor_msgs::msg::PointCloud2
nav2_3d_grid::convert()
{

}

}

