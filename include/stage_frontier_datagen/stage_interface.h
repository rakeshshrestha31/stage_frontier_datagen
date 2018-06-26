//
// Created by rakesh on 13/06/18.
//

#ifndef STAGE_FRONTIER_DATAGEN_STAGE_INTERFACE_H
#define STAGE_FRONTIER_DATAGEN_STAGE_INTERFACE_H

#include <stage.hh>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

/**
 * @brief interface for stage simulation
 * @details adapted from https://github.com/Voidminded/DQNStageROS and https://github.com/payamn/DeepRLMapCoverage
 */
class StageInterface
{
public:
  /**
   * @brief Stage World GUI supporting stepped execution
   * @todo make it general to use without GUI
   */
  class StepWorldGui : public Stg::WorldGui
  {
  public:
    using WorldGui::WorldGui;

    void step()
    {
      Stop();
      Fl::lock();
      World::Update();
      Fl::unlock();
      Fl::awake();
    }
  };

  /**
   *
   * @param argc commandline argument argc
   * @param argv commandline argument argv
   * @param worldfile location of worldfile
   */
  StageInterface(int argc, char **argv,
                 boost::shared_ptr<StepWorldGui> stage_world, const std::string &worldfile,
                 const boost::function<int (Stg::Model*)> &pose_callback, const boost::function<int (Stg::Model*)> &laser_callback);

  /**
   * @brief adapted from stage_ros
   * @return tf from laser to base frame
   */
  geometry_msgs::TransformStamped getBaseLaserTf();

  /**
   * @brief step the world by iteration
   */
  void step() { stage_world_->step(); }

  /**
   * @brief stage callback for pose update (static because we need function pointer)
   * @param model stage pose model
   * @param stage_interface pointer to object of StageInterface
   * @return
   */
  static int poseUpdateCallback(Stg::Model *model, StageInterface *stage_interface);

  /**
   * @brief stage callback for laser update (static because we need function pointer)
   * @param model stage pose model
   * @param stage_interface pointer to object of StageInterface
   * @return
   */
  static int laserUpdateCallback(Stg::Model *model, StageInterface *stage_interface);

  boost::function<int (Stg::Model*)> laser_callback_functor_;
  boost::function<int (Stg::Model*)> pose_callback_functor_;
protected:
  // TODO: make the type of stage_world_ only World (to use without GUI)
  boost::shared_ptr<StepWorldGui> stage_world_;

  Stg::ModelRanger *laser_model_;
  Stg::ModelPosition *robot_model_;

  sensor_msgs::LaserScanPtr laser_scan_msg_;
  nav_msgs::OdometryPtr odom_msg_;

  boost::mutex laser_scan_mutex_;
  boost::mutex odom_mutex_;

  // TODO: don't use publishers
  ros::Publisher laser_pub_;
  ros::Publisher odom_pub_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

#endif //STAGE_FRONTIER_DATAGEN_STAGE_INTERFACE_H
