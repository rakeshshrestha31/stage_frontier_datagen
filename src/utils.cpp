//
// Created by rakesh on 30/05/18.
//

#include <stage_frontier_datagen/utils.h>

#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sys/wait.h>

namespace stage_frontier_datagen
{
namespace utils
{

#define READ 0
#define WRITE 1

pid_t
popen2(const char *command, int *outfp, int *infp)
{
  int p_stdin[2], p_stdout[2];
  pid_t pid;

  if (pipe(p_stdin) != 0 || pipe(p_stdout) != 0)
    return -1;

  pid = fork();

  if (pid < 0)
    return pid;
  else if (pid == 0)
  {
    close(p_stdin[WRITE]);
    dup2(p_stdin[READ], READ);
    close(p_stdout[READ]);
    dup2(p_stdout[WRITE], WRITE);

    execl("/bin/sh", "sh", "-c", command, NULL);
    perror("execl");
    exit(1);
  }

  if (infp == NULL)
    close(p_stdin[WRITE]);
  else
    *infp = p_stdin[WRITE];

  if (outfp == NULL)
    close(p_stdout[READ]);
  else
    *outfp = p_stdout[READ];

  return pid;
}

tf::Stamped<tf::Pose> odometryMsgToTfStampedPose(const nav_msgs::Odometry &odometry)
{
  tf::Pose groundtruth_tf_pose;
  tf::poseMsgToTF(odometry.pose.pose, groundtruth_tf_pose);
  return tf::Stamped<tf::Pose>(
    groundtruth_tf_pose, odometry.header.stamp, odometry.header.frame_id
  );
}

} // namespace utils
} // namespace stage_frontier_datagen