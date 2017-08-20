#include<simple_layers/gonogo_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GonogoLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace simple_layer_namespace
{

  GonogoLayer::GonogoLayer() {}

  void GonogoLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    gonogo_sub_ = nh.subscribe("/cnn_out2", 1, &GonogoLayer::gonogoCallback, this);

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
												&GonogoLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void GonogoLayer::gonogoCallback(const std_msgs::Float32& prob) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    prob_ = prob;
  }

  void GonogoLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void GonogoLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
				 double* min_y, double* max_x, double* max_y)
  {
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (!enabled_)
      return;

    mark_x_ = robot_x + 0.35*cos(robot_yaw);
    mark_y_ = robot_y + 0.35*sin(robot_yaw);

    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);
  }

  void GonogoLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
				int max_j)
  {
    if (!enabled_)
      return;
   
    unsigned int mx;
    unsigned int my;

    //std::cout << "Updating Costs" << std::endl;
    //std::cout << prob_.data << std::endl;

    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
      if(float(prob_.data)>0.5)
	master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      else
	master_grid.setCost(mx, my, FREE_SPACE);	
    }
  }

} // end namespace
