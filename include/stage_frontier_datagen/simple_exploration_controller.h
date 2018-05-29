//
// Created by rakesh on 29/05/18.
//

#ifndef STAGE_FRONTIER_DATAGEN_SIMPLE_EXPLORATION_CONTROLLER_H
#define STAGE_FRONTIER_DATAGEN_SIMPLE_EXPLORATION_CONTROLLER_H

#include <ros/ros.h>
#include <hector_exploration_planner/hector_exploration_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <hector_path_follower/hector_path_follower.h>
#include <nav_msgs/Path.h>

#include <boost/shared_ptr.hpp>

class SimpleExplorationController
{
public:
  SimpleExplorationController();

  /**
   * @brief runs hector_navigation_planner methods to get navigation path based on exploration transform optimization
   * @param path navigation path
   * @return status
   */
  bool getNavPath(nav_msgs::Path &path);

  /**
   * @brief timer callback function to run planner
   * @param e reference to ROS Timer Event that was used to invoke the method
   */
  void timerPlanExploration(const ros::TimerEvent& e);

  /**
   * @brief timer callback function to publish command velocity
   * @param e reference to ROS Timer Event that was used to invoke the method
   */
  void timerCmdVelGeneration(const ros::TimerEvent& e);


protected:
  boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_2d_ros_;
  boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> planner_;
  pose_follower::HectorPathFollower path_follower_;

  ros::Publisher vel_pub_;
  ros::Publisher exploration_plan_pub_;

  tf::TransformListener tfl_;

  ros::Timer exploration_plan_generation_timer_;
  ros::Timer cmd_vel_generator_timer_;
};

#endif //STAGE_FRONTIER_DATAGEN_SIMPLE_EXPLORATION_CONTROLLER_H
