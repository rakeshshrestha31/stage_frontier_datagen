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

// TODO: make this proper parameter
#include <stage_frontier_datagen/simple_exploration_controller.h>

namespace stage_frontier_datagen
{
SimpleExplorationController::SimpleExplorationController()
  : planner_(new hector_exploration_planner::HectorExplorationPlanner()),
    plan_update_callback_(0),
    is_planner_initialized_(false),
    is_planner_running_(false)
{
  path_follower_.initialize(&tfl_);

  exploration_plan_generation_timer_ = nh_.createTimer(ros::Duration(3.0),
                                                      &SimpleExplorationController::timerPlanExploration, this, false);
  cmd_vel_generator_timer_ = nh_.createTimer(ros::Duration(0.1), &SimpleExplorationController::timerCmdVelGeneration,
                                            this, false);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  exploration_plan_pub_ = nh_.advertise<nav_msgs::Path>("exploration_path", 2);
}

void SimpleExplorationController::startExploration()
{
  if (!costmap_2d_ros_)
  {
    costmap_2d_ros_.reset(new costmap_2d::Costmap2DROS("global_costmap", tfl_));
    planner_->initialize(ros::this_node::getNamespace(), costmap_2d_ros_.get());
    is_planner_initialized_ = true;
  }

  exploration_plan_generation_timer_.start();
  cmd_vel_generator_timer_.start();
}

void SimpleExplorationController::stopExploration()
{
  exploration_plan_generation_timer_.stop();
  cmd_vel_generator_timer_.stop();
  vel_pub_.publish(geometry_msgs::Twist());
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

  tf::Stamped<tf::Pose> robot_pose_tf;
  bool robot_pose_status = costmap_2d_ros_->getRobotPose(robot_pose_tf);
  if (!robot_pose_status || robot_pose_tf.getRotation().length2() < 1e-5)
  {
    ROS_ERROR("Failed to get robot pose from costmap");
    return false;
  }

  geometry_msgs::PoseStamped pose;
  tf::poseStampedTFToMsg(robot_pose_tf, pose);

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.pose.orientation, orientation);
  if (orientation.length2() < 1e-5)
  {
    ROS_ERROR("poseStampedTFToMsg incorrect");
  }

  auto planner_thread = boost::thread([this, pose]() {
    nav_msgs::Path path;
    is_planner_running_ = true;
    ROS_INFO("running planner...");
    bool status = planner_->doExploration(pose, path.poses);
    is_planner_running_ = false;

    if (status)
    {
      updatePath(path);
      ROS_INFO("Generated exploration current_path_ with %u poses", (unsigned int) current_path_.poses.size());
      current_path_.header.frame_id = "map";
      current_path_.header.stamp = ros::Time::now();

      if (exploration_plan_pub_.getNumSubscribers() > 0)
      {
        exploration_plan_pub_.publish(current_path_);
      }
    }
    else
    {
      ROS_INFO("planner failed");
    }

    if (plan_update_callback_)
    {
      plan_update_callback_(*this, status);
    }
  });

  planner_thread.detach();

  return true;
}

void SimpleExplorationController::timerPlanExploration(const ros::TimerEvent &e)
{
  if (!is_planner_running_)
  {
    updatePlan();
  }
}

void SimpleExplorationController::timerCmdVelGeneration(const ros::TimerEvent &e)
{
  // whether the callback is already running
  static boost::atomic_bool is_running(false);

  if (is_running)
  {
    return;
  }

  geometry_msgs::Twist twist;
  bool status = path_follower_.computeVelocityCommands(twist);

  if (!status || path_follower_.stopped() || path_follower_.isGoalReached())
  {
    is_running = true;

    // empty twist while planning
    vel_pub_.publish(twist);
    if (!is_planner_running_)
    {
      updatePlan();
    }
  }
  else
  {
    vel_pub_.publish(twist);
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

} // namespace stage_frontier_datagen
