#include "nav2_3d_costmap_plugin/nav2_3d_costmap.hpp"

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
// read parameters
void
Nav23dStaticLayer::onInitialize()
{

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

void
Nav23dStaticLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y)
{
}
void
Nav23dStaticLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
{
}

void
Nav23dStaticLayer::matchSize()
{
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_3d_costmap_plugin::Nav23dStaticLayer, nav2_costmap_2d::CostmapLayer)