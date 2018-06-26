//
// Created by rakesh on 13/06/18.
//

#include <stage_frontier_datagen/stage_interface.h>

using namespace Stg;

StageInterface::StageInterface(int argc, char **argv,
                               const boost::shared_ptr<StepWorldGui> &stage_world, const std::string &worldfile,
                               const boost::function<int (const sensor_msgs::LaserScanConstPtr&, const nav_msgs::OdometryConstPtr)> &sensor_callback)
  : stage_world_(stage_world),
    sensor_callback_functor_(sensor_callback),
    laser_scan_msg_(new sensor_msgs::LaserScan()),
    odom_msg_(new nav_msgs::Odometry())
{
  stage_world_->Load(worldfile);

  robot_model_ = dynamic_cast<Stg::ModelPosition *>(stage_world_->GetModel("r0"));
  robot_model_->AddCallback(Model::CB_UPDATE, (model_callback_t)StageInterface::poseUpdateCallback, (void*)this);
  robot_model_->Subscribe();

  laser_model_ = dynamic_cast<Stg::ModelRanger *>(robot_model_->GetChild("ranger:0"));
  laser_model_->AddCallback(Model::CB_UPDATE, (model_callback_t)StageInterface::laserUpdateCallback, (void*)this);
  laser_model_->Subscribe();

  laser_scan_msg_->header.frame_id = "base_laser_link";
  odom_msg_->header.frame_id = "odom";

  geometry_msgs::TransformStamped base_laser_tf = getBaseLaserTf();
  static_tf_broadcaster_.sendTransform(base_laser_tf);

  // identity transform between base_link and base_footprint
  geometry_msgs::TransformStamped identity_transform_msg;
  tf::transformStampedTFToMsg(
    tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), "base_footprint", "base_link"),
    identity_transform_msg
  );
  static_tf_broadcaster_.sendTransform(identity_transform_msg);

  // TODO: no publishers
  ros::NodeHandle nh;
  laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("base_scan", 10);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
}

geometry_msgs::TransformStamped StageInterface::getBaseLaserTf()
{
  assert(robot_model_ & laser_model_);

  Stg::Pose lp = laser_model_->GetPose();
  tf::Quaternion laserQ;
  laserQ.setRPY(0.0, 0.0, lp.a);
  tf::Transform txLaser =  tf::Transform(laserQ, tf::Point(lp.x, lp.y, robot_model_->GetGeom().size.z + lp.z));

  geometry_msgs::TransformStamped stamped_tf_msg;
  tf::transformStampedTFToMsg(tf::StampedTransform(txLaser, ros::Time::now(), "base_link", "base_laser_link"), stamped_tf_msg);
  return stamped_tf_msg;
}

void StageInterface::updateCmdVel(const geometry_msgs::Twist &cmd_vel)
{
  boost::mutex::scoped_lock lock(cmd_vel_mutex_);
  if (!cmd_vel_msgs_)
  {
    cmd_vel_msgs_ = boost::make_shared<geometry_msgs::Twist>(cmd_vel);
  }
  else
  {
    *cmd_vel_msgs_ = cmd_vel;
  }
}

int StageInterface::poseUpdateCallback(Model *model, StageInterface *stage_interface)
{
  assert(stage_interface && model);

  boost::mutex::scoped_lock odom_lock(stage_interface->odom_mutex_);

  stage_interface->odom_msg_->header.stamp = ros::Time::now();

  stage_interface->odom_msg_->pose.pose.position.x = stage_interface->robot_model_->est_pose.x;
  stage_interface->odom_msg_->pose.pose.position.y = stage_interface->robot_model_->est_pose.y;
  stage_interface->odom_msg_->pose.pose.orientation = tf::createQuaternionMsgFromYaw(stage_interface->robot_model_->est_pose.a);

  auto velocity = stage_interface->robot_model_->GetVelocity();
  stage_interface->odom_msg_->twist.twist.linear.x = velocity.x;
  stage_interface->odom_msg_->twist.twist.linear.y = velocity.y;
  stage_interface->odom_msg_->twist.twist.angular.z = velocity.a;

  // TF broadcast
  tf::Quaternion odomQ;
  tf::quaternionMsgToTF(stage_interface->odom_msg_->pose.pose.orientation, odomQ);
  tf::Transform txOdom(odomQ, tf::Point(stage_interface->odom_msg_->pose.pose.position.x, stage_interface->odom_msg_->pose.pose.position.y, 0.0));
  stage_interface->tf_broadcaster_.sendTransform(tf::StampedTransform(
    txOdom,
    ros::Time::now(),
    "odom",
    "base_footprint"
  ));

  return 0;
}

int StageInterface::laserUpdateCallback(Model *model, StageInterface *stage_interface)
{
  assert(stage_interface && model);

  boost::mutex::scoped_lock laser_scan_lock(stage_interface->laser_scan_mutex_);
  stage_interface->laser_scan_msg_->header.stamp = ros::Time::now();
  const std::vector<Stg::ModelRanger::Sensor>& sensors = stage_interface->laser_model_->GetSensors();

  if( sensors.size() > 1 )
    ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

  const Stg::ModelRanger::Sensor& sensor = sensors[0];

  stage_interface->laser_scan_msg_->angle_min = -sensor.fov/2.0;
  stage_interface->laser_scan_msg_->angle_max = +sensor.fov/2.0;
  stage_interface->laser_scan_msg_->angle_increment = sensor.fov/(double)(sensor.sample_count-1);
  stage_interface->laser_scan_msg_->range_min = sensor.range.min;
  stage_interface->laser_scan_msg_->range_max = sensor.range.max;
  stage_interface->laser_scan_msg_->ranges.resize(sensor.ranges.size());
  stage_interface->laser_scan_msg_->intensities.resize(sensor.intensities.size());

  for(unsigned int i = 0; i < sensor.ranges.size(); i++)
  {
    stage_interface->laser_scan_msg_->ranges[i] = sensor.ranges[i];
    stage_interface->laser_scan_msg_->intensities[i] = sensor.intensities[i];
  }
  stage_interface->laser_pub_.publish(stage_interface->laser_scan_msg_);

  // sync odom message
  boost::mutex::scoped_lock odom_lock(stage_interface->odom_mutex_);
  stage_interface->odom_msg_->header.stamp = stage_interface->laser_scan_msg_->header.stamp;
  stage_interface->odom_pub_.publish(stage_interface->odom_msg_);

  // apply cmd_vel
  boost::mutex::scoped_lock cmd_vel_lock(stage_interface->cmd_vel_mutex_);
  if (stage_interface->cmd_vel_msgs_)
  {
    stage_interface->robot_model_->SetSpeed(
      stage_interface->cmd_vel_msgs_->linear.x,
      stage_interface->cmd_vel_msgs_->linear.y,
      stage_interface->cmd_vel_msgs_->angular.z
    );
  }

  if (stage_interface->sensor_callback_functor_)
  {
    return stage_interface->sensor_callback_functor_(stage_interface->laser_scan_msg_, stage_interface->odom_msg_);
  }
  else
  {
    return 0;
  }
}