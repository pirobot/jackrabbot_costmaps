#include <simple_layers/gonogo_grid_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GonogoGridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

  GonogoGridLayer::GonogoGridLayer() {}

  void GonogoGridLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    default_value_ = NO_INFORMATION;

    matchSize();

    gonogo_sub_ = nh.subscribe("/cnn_out2", 1, &GonogoGridLayer::gonogoCallback, this);

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
												&GonogoGridLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void GonogoGridLayer::gonogoCallback(const std_msgs::Float32& prob) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    prob_ = prob;
  }

  void GonogoGridLayer::matchSize()
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
	      master->getOriginX(), master->getOriginY());
  }


  void GonogoGridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void GonogoGridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
			       double* min_y, double* max_x, double* max_y)
  {
    if (!enabled_)
      return;

    unsigned int mx;
    unsigned int my;

    mark_x_ = robot_x + 0.35*cos(robot_yaw);
    mark_y_ = robot_y + 0.35*sin(robot_yaw);

    if(worldToMap(mark_x_, mark_y_, mx, my)){
      if(float(prob_.data)>0.5)
    	  setCost(mx, my, LETHAL_OBSTACLE);
      else
    	  setCost(mx, my, FREE_SPACE);
    }
  
    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);
  }

  void GonogoGridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
			      int max_j)
  {
    if (!enabled_)
      return;

    for (int j = min_j; j < max_j; j++)
      {
	for (int i = min_i; i < max_i; i++)
	  {
	    int index = getIndex(i, j);
	    if (costmap_[index] == NO_INFORMATION)
	      continue;
	    master_grid.setCost(i, j, costmap_[index]); 
	  }
      }

  }

} // end namespace
