//
// Created by rakesh on 29/05/18.
//

//=================================================================================================
// Copyright (c) 2018, Rakesh Shrestha
// Acknowledgment: Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#define MAX_ROBOT_TO_FIRST_WAYPOINT_DISTANCE 2

#include <stage_frontier_datagen/simple_exploration_controller.h>
#include <thread>
#include <cmath>

namespace stage_frontier_datagen
{
SimpleExplorationController::SimpleExplorationController(
    boost::shared_ptr<StageInterface::AbstractStepWorld> stage_world,
    const boost::function<void(const geometry_msgs::Twist&)> update_cmd_vel_functor)
  : stage_world_(stage_world),
    update_cmd_vel_functor_(update_cmd_vel_functor),
    planner_(new hector_exploration_planner::HectorExplorationPlanner()),
    plan_update_callback_(0),
    plan_finished_callback_(0),
    is_planner_initialized_(false),
    is_planner_running_(false),
    is_plan_update_callback_running_(false),
    is_plan_finished_callback_running_(false),
    planner_status_(false),
    last_planner_status_(false),
    plan_number_(0)
{
  path_follower_.initialize(&tfl_);
  // TODO: update exploration plan a few waypoints before your previous plan will finish
//  exploration_plan_generation_timer_ = nh_.createTimer(ros::Duration(1.0),
//                                                       &SimpleExplorationController::timerPlanExploration, this, false);
//  cmd_vel_generator_timer_ = nh_.createTimer(ros::Duration(0.1), &SimpleExplorationController::timerCmdVelGeneration,
//                                            this, false);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  exploration_plan_pub_ = nh_.advertise<nav_msgs::Path>("exploration_path", 2);
}

void SimpleExplorationController::startExploration(const cv::Mat& ground_truth)
{
  clearData();
  if (!costmap_2d_ros_)
  {
    initializeCostmap();
//    costmap_2d_ros_.reset(new costmap_2d::Costmap2DROS("", tfl_));
    planner_->initialize(ros::this_node::getNamespace(), costmap_2d_ros_.get(), ground_truth);
  }

  exploration_plan_generation_timer_.start();
  cmd_vel_generator_timer_.start();
  is_planner_initialized_ = true;
}

void SimpleExplorationController::stopExploration()
{
  exploration_plan_generation_timer_.stop();
  cmd_vel_generator_timer_.stop();

  geometry_msgs::Twist empty_vel;
  vel_pub_.publish(empty_vel);
  {
    boost::mutex::scoped_lock lock(update_cmd_vel_functor_mutex_);
    if (update_cmd_vel_functor_)
    {
      update_cmd_vel_functor_(empty_vel);
    }
  }

  clearData();

  is_planner_initialized_ = false;

}

bool SimpleExplorationController::updatePlan()
{
  if (!is_planner_initialized_)
  {
    ROS_WARN_THROTTLE(1, "Planner not initialized");
    return false;
  }

  if (is_planner_running_)
  {
    return false;
  }

  geometry_msgs::PoseStamped pose;
  if(!this->getRobotPose(pose))
    return false;

  // record robot pose at the end pose of last plan
  getRobotPose(this->robot_pose_at_plan_end);

//  auto planner_thread = boost::thread([this, pose]() {

    nav_msgs::Path path;
    nav_msgs::Path the_other_path;

    is_planner_running_ = true;

    ROS_INFO("running planner...");
    std::chrono::steady_clock::time_point begin_t = std::chrono::steady_clock::now();
    planner_status_ = planner_->doExploration(pose, path.poses, the_other_path.poses);
    std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
    planner_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_t-begin_t).count();

    // make a copy of current costmap in planner for data recording
    last_costmap_ = new costmap_2d::Costmap2D(*(planner_->getCostMap()));

    is_planner_running_ = false;

    // update current plan
    this->plan = path.poses;
    this->the_other_plan = the_other_path.poses;

    // record last explored costmap and frontiers detected after planner
    if(last_planner_status_ && plan_finished_callback_)
    {
      if (!is_plan_finished_callback_running_) {
        is_plan_finished_callback_running_ = true;
        // since this is record the last explored area
        plan_finished_callback_(*this, this->plan_number_);
        is_plan_finished_callback_running_ = false;
      } else {
        ROS_WARN("Skipping plan_finished_callback because previous one hasn't finished");
      }
    }

    // clear last plan data
    clearPlanData();

    if (path.poses.empty()) {
      planner_status_ = false;
    }

    if (planner_status_) {
      if (std::hypot(path.poses[0].pose.position.x - pose.pose.position.x,
                     path.poses[0].pose.position.y - pose.pose.position.y) > MAX_ROBOT_TO_FIRST_WAYPOINT_DISTANCE) {
        planner_status_ = false;
        ROS_WARN("Invalid plan, first waypoint far away from robot");
      } else {
        // update current path
        updatePath(path);
        ROS_INFO("Generated exploration current_path_ with %u poses", (unsigned int) current_path_.poses.size());
        current_path_.header.frame_id = "map";
        current_path_.header.stamp = ros::Time::now();

        if (exploration_plan_pub_.getNumSubscribers() > 0) {
          exploration_plan_pub_.publish(current_path_);
        }
        plan_number_++;
      }
    } else {
      ROS_INFO("planner failed");
    }

    // make sure that the previous callback finished before going to next
    if (plan_update_callback_) {
      if (!is_plan_update_callback_running_) {
        is_plan_update_callback_running_ = true;
        plan_update_callback_(*this, static_cast<bool>(planner_status_));
        is_plan_update_callback_running_ = false;
      } else {
        ROS_WARN("Skipping plan update callback because previous one hasn't finished");
      }
    }

    last_planner_status_ = static_cast<bool>(planner_status_);

//    });
//
//  planner_thread.detach();

  return true;
}

void SimpleExplorationController::timerPlanExploration(const ros::TimerEvent &e)
{
  planExploration();
}

void SimpleExplorationController::planExploration()
{
  if (!is_planner_running_)
  {
    updatePlan();
  }
}

void SimpleExplorationController::timerCmdVelGeneration(const ros::TimerEvent &e)
{
  generateCmdVel();
}

bool SimpleExplorationController::ObstaclesInWayPoints()
{
  // check if current_plan_ is inside obstacle or not
  int current_way_point = path_follower_.get_current_waypoint();
  auto pose_size = current_path_.poses.size();
  for (int i = current_way_point; i < pose_size; i++)
  {
    unsigned int mx,my;
    auto costmap = costmap_2d_ros_->getCostmap();
    auto charmap = costmap->getCharMap();
    costmap->worldToMap(current_path_.poses[i].pose.position.x,current_path_.poses[i].pose.position.y,mx,my);
    auto index = costmap->getIndex(mx,my);
    if(charmap[index] == costmap_2d::LETHAL_OBSTACLE)
      return true;
  }

  return false;
}

void SimpleExplorationController::updateCmdVel(const geometry_msgs::Twist &cmd_vel)
{
  vel_pub_.publish(cmd_vel);
  {
    boost::mutex::scoped_lock lock(update_cmd_vel_functor_mutex_);
    if (update_cmd_vel_functor_)
    {
      update_cmd_vel_functor_(cmd_vel);
    }
  }
}

void SimpleExplorationController::updateStepMotionInfo()
{
  // get robot pose
  geometry_msgs::PoseStamped pose;
  // if cannot get current robot pose, skip...
  if(!this->getRobotPose(pose))
  {
    ROS_WARN("Cannot get robot pose from costmap!");
    return;
  }

  this->robot_poses_in_plan.push_back(pose);

  // get time stamp
  double cur_time = std::chrono::duration_cast< std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()
  ).count();
  this->system_time_in_plan.push_back(cur_time);

  // get current explored cells
  cv::Mat map = hector_exploration_planner::frontier_analysis::getMap(this->costmap_2d_ros_);
  cv::Mat channels[3]; // unknown, free, obstacle
  cv::split(map, channels);
  int cell_num_unknown = static_cast<int>(cv::sum(channels[0] / 255)[0]);
  int cells_num_explored = map.size().height * map.size().width - cell_num_unknown;
  double area = cells_num_explored * pow(this->costmap_2d_ros_->getCostmap()->getResolution(), 2);
  this->explored_area_in_plan.push_back(area);
  // get current simulation time
  this->simulation_time_in_plan.push_back(stage_world_->SimTimeNow() / 1e3 ); // ms
}

void SimpleExplorationController::generateCmdVel()
{
  // whether the callback is already running
  static boost::atomic_bool is_running(false);

  if (is_running)
  {
    return;
  }

  // check if new obstacles occurs in current plan's following way points
  if(ObstaclesInWayPoints())
  {
    ROS_WARN("Plan inside obstacle, re-planning!");
    updatePlan();

    // empty twist while planning
    geometry_msgs::Twist empty_vel;
    vel_pub_.publish(empty_vel);
    // TODO: save total planning time
    // block while planning
    while (is_planner_running_);
    return;
  }

  geometry_msgs::Twist twist;
  bool status = path_follower_.computeVelocityCommands(twist);

  if (!status || path_follower_.stopped() || path_follower_.isGoalReached() || !planner_status_)
  {
    is_running = true;

    // empty twist while planning
    geometry_msgs::Twist empty_vel;
    updateCmdVel(empty_vel);

    if (!is_planner_running_)
    {
      updatePlan();
      // TODO: save total planning time
      // block while planning
      while (is_planner_running_);
    }
  }
  else
  {
    updateCmdVel(twist);
    // only record one step motion info when everything is right
    updateStepMotionInfo();
  }

  is_running = false;
}

void SimpleExplorationController::updatePath(nav_msgs::Path &path)
{
  boost::mutex::scoped_lock lock(path_mutex_);
  current_path_ = path;
  path_follower_.setPlan(path.poses);
}

nav_msgs::Path SimpleExplorationController::getCurrentPath()
{
  boost::mutex::scoped_lock lock(path_mutex_);
  return current_path_;
}

void SimpleExplorationController::clearCostmap()
{
  if (!costmap_2d_ros_)
  {
    ROS_ERROR("SimpleExplorationController: tried to clear non-existant costmap");
    return;
  }

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_2d_ros_->getCostmap()->getMutex()));
  costmap_2d_ros_->resetLayers();
}

void SimpleExplorationController::clearPlanData()
{
  robot_poses_in_plan.clear();
  system_time_in_plan.clear();
  explored_area_in_plan.clear();
  simulation_time_in_plan.clear();
}

void SimpleExplorationController::clearData()
{
  this->plan_number_ = 0;
  this->last_planner_status_ = false;
  this->planner_status_ = false;
  clearPlanData();
  clearCostmap();
}

void SimpleExplorationController::updateCmdVelFunctor(const boost::function<void(const geometry_msgs::Twist&)> &update_cmd_vel_functor)
{
  boost::mutex::scoped_lock lock(update_cmd_vel_functor_mutex_);
  update_cmd_vel_functor_ = update_cmd_vel_functor;
}

void SimpleExplorationController::updateRobotOdom(const nav_msgs::OdometryConstPtr &odom)
{
  if (planner_ && costmap_2d_ros_)
  {
    costmap_2d_ros_->setRobotOdom(odom);
    path_follower_.setRobotOdom(odom);
  }
}

geometry_msgs::PoseStamped SimpleExplorationController::getRobotPoseAtPlanEnd() const
{
  return this->robot_pose_at_plan_end;
}

bool SimpleExplorationController::getRobotPose(geometry_msgs::PoseStamped &pose)
{
  tf::Stamped<tf::Pose> robot_pose_tf;
  bool robot_pose_status = costmap_2d_ros_->getRobotPose(robot_pose_tf);

  if (!robot_pose_status || robot_pose_tf.getRotation().length2() < 1e-5)
  {
    ROS_ERROR("Failed to get robot pose from costmap");
    return false;
  }

  tf::poseStampedTFToMsg(robot_pose_tf, pose);

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.pose.orientation, orientation);
  if (orientation.length2() < 1e-5)
  {
    ROS_ERROR("poseStampedTFToMsg incorrect");
  }
  return true;
}

void SimpleExplorationController::initializeCostmap()
{
  costmap_2d_ros_.reset(new hector_exploration_planner::CustomCostmap2DROS("global_costmap", tfl_, false));

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_2d_ros_->getCostmap()->getMutex()));
  for (const auto &layer: *(costmap_2d_ros_->getLayeredCostmap()->getPlugins()))
  {
    std::string layer_name = layer->getName();
    if (layer_name.find("ground_truth") != std::string::npos)
    {
      ground_truth_layer_ = boost::static_pointer_cast<ground_truth_layer::GroundTruthLayer>(layer);
      break;
    }
  }

  if (ground_truth_layer_)
  {
    // unsubscribe the topics. We want to updaet the data manually here
    ground_truth_layer_->deactivate();
  }
}

} // namespace stage_frontier_datagen

#include <costmap_2d/costmap_layer.h>
