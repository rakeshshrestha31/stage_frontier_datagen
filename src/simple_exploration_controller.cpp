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

#include <stage_frontier_datagen/simple_exploration_controller.h>

SimpleExplorationController::SimpleExplorationController()
  : costmap_2d_ros_(new costmap_2d::Costmap2DROS("global_costmap", tfl_)),
    planner_(new hector_exploration_planner::HectorExplorationPlanner())
{
  ros::NodeHandle nh;

  planner_->initialize("hector_exploration_planner",costmap_2d_ros_.get());
  path_follower_.initialize(&tfl_);

  exploration_plan_generation_timer_ = nh.createTimer(ros::Duration(15.0), &SimpleExplorationController::timerPlanExploration, this, false );
  cmd_vel_generator_timer_ = nh.createTimer(ros::Duration(0.1), &SimpleExplorationController::timerCmdVelGeneration, this, false );

  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  exploration_plan_pub_ = nh.advertise<nav_msgs::Path>("exploration_path",2);
}

bool SimpleExplorationController::getNavPath(nav_msgs::Path &path)
{
  ROS_INFO("Planner called");

  tf::Stamped<tf::Pose> robot_pose_tf;
  costmap_2d_ros_->getRobotPose(robot_pose_tf);

  geometry_msgs::PoseStamped pose;
  tf::poseStampedTFToMsg(robot_pose_tf, pose);
  planner_->doExploration(pose, path.poses);
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();

  if (exploration_plan_pub_.getNumSubscribers() > 0)
  {
    exploration_plan_pub_.publish(path);
  }

  return true;
}

void SimpleExplorationController::timerPlanExploration(const ros::TimerEvent& e)
{
  nav_msgs::Path path;

  if (getNavPath(path))
  {
    ROS_INFO("Generated exploration path with %u poses", (unsigned int)path.poses.size());
    path_follower_.setPlan(path.poses);
  }else{
    ROS_WARN("Planner failed");
  }
}

void SimpleExplorationController::timerCmdVelGeneration(const ros::TimerEvent& e)
{
  geometry_msgs::Twist twist;
  path_follower_.computeVelocityCommands(twist);
  vel_pub_.publish(twist);
}
