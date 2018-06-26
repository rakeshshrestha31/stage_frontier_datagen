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
#include <boost/function.hpp>

namespace stage_frontier_datagen
{
class SimpleExplorationController
{
public:
  SimpleExplorationController(boost::function<void(geometry_msgs::Twist)> update_cmd_vel_functor=0);

  /**
   * @brief runs hector_navigation_planner methods to get navigation path based on exploration transform optimization
   * @return status
   */
  bool updatePlan();

  /**
   * @brief timer callback function to run planner
   * @param e reference to ROS Timer Event that was used to invoke the method
   */
  void timerPlanExploration(const ros::TimerEvent &e);

  /**
   * @brief timer callback function to publish command velocity
   * @param e reference to ROS Timer Event that was used to invoke the method
   */
  void timerCmdVelGeneration(const ros::TimerEvent &e);

  /**
   * @brief start the timer events for exploration
   */
  void startExploration();

  /**
   * @brief stop the timer events for exploration (+ sends 0 cmd_vel for safety once)
   */
  void stopExploration();

  /**
   * @brief updates path for the path follower (thread-safe)
   * @param path the path to update
   */
  void updatePath(nav_msgs::Path &path);

  /**
   *
   * @return a copy of current navigation path (thread-safe)
   */
  nav_msgs::Path getCurrentPath();


  /**
   *
   * @param resolution of the map
   * @return current map
   */
  cv::Mat getCurrentMap(double resolution);

  /**
   *
   * @param callback callback function to call when new plan received
   */
  void subscribeNewPlan(boost::function<void(const SimpleExplorationController&, bool)> callback)
  {
    plan_update_callback_ = callback;
  }

  /**
   *
   * @return whether planner is running
   */
  bool isPlannerRunning() { return (bool)is_planner_running_; }

  /**
   *
   * @return costmap being used for planning
   */
  const boost::shared_ptr<costmap_2d::Costmap2DROS> getCostmap2DROS() const { return costmap_2d_ros_; }

  /**
   *
   * @return shared pointer to hector planner instance
   */
  boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> getPlanner() const { return planner_; }

  /**
   * @brief updates update_cmd_vel_functor_ member
   * @param update_cmd_vel_functor
   */
  void updateCmdVelFunctor(boost::function<void(geometry_msgs::Twist)> update_cmd_vel_functor);
protected:
  boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_2d_ros_;
  boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> planner_;
  pose_follower::HectorPathFollower path_follower_;

  boost::mutex path_mutex_;
  boost::atomic_bool is_planner_running_;
  boost::function<void(const SimpleExplorationController&, bool)> plan_update_callback_;  ///< callback on plan update

  boost::mutex update_cmd_vel_functor_mutex_;
  boost::function<void(geometry_msgs::Twist)> update_cmd_vel_functor_;

  ros::Publisher vel_pub_;
  ros::Publisher exploration_plan_pub_;
  ros::NodeHandle nh_;
  tf::TransformListener tfl_;

  ros::Timer exploration_plan_generation_timer_;
  ros::Timer cmd_vel_generator_timer_;

  nav_msgs::Path current_path_;

  bool is_planner_initialized_;
  boost::atomic_bool planner_status_;
  boost::atomic_bool is_plan_update_callback_running_;

  // protected methods
  /**
   * clears the costmap (without unsubscribing to the required topics)
   */
  void clearCostmap();
};
} // namespace stage_frontier_datagen
#endif //STAGE_FRONTIER_DATAGEN_SIMPLE_EXPLORATION_CONTROLLER_H
