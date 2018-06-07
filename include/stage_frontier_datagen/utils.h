//
// Created by rakesh on 30/05/18.
//

#ifndef STAGE_FRONTIER_DATAGEN_UTILS_H
#define STAGE_FRONTIER_DATAGEN_UTILS_H

#include <unistd.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

namespace stage_frontier_datagen
{
namespace utils
{

/**
 *
 * @param command command to execute
 * @param outfp pointer of stdout to pipe to
 * @param infp pointer of stdin to pipe to
 * @return
 */
pid_t popen2(const char *command, int *outfp = nullptr, int *infp = nullptr);

/**
 *
 * @param odometry nav_msgs Odometry type
 * @return tf stamped pose
 */
tf::Stamped<tf::Pose> odometryMsgToTfStampedPose(const nav_msgs::Odometry &odometry);

} // namespace utils
} // namespace stage_frontier_datagen

#endif //STAGE_FRONTIER_DATAGEN_UTILS_H
