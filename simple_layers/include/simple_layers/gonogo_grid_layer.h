#ifndef GONOGO_GRID_LAYER_H_
#define GONOGO_GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32.h>

namespace simple_layer_namespace
{

  class GonogoGridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
  {
  public:
    GonogoGridLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
			      double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    void gonogoCallback(const std_msgs::Float32& prob);

    bool isDiscretized()
    {
      return true;
    }

    virtual void matchSize();

  private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  protected:
    ros::Subscriber gonogo_sub_;
    std_msgs::Float32 prob_;
    boost::recursive_mutex lock_;
    double mark_x_, mark_y_;
  };
}
#endif
