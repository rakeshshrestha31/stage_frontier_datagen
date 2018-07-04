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
#include <geometry_msgs/Twist.h>

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
   * @brief abstract interface for stepped world
   */
  class AbstractStepWorld
  {
  public:
    AbstractStepWorld() = default;
    virtual ~AbstractStepWorld() = default;
    /**
     * @brief method to step through the world
     */
    virtual void step() = 0;

    virtual bool Load(const std::string &worldfile_path) = 0;
    virtual void UnLoad() = 0;
    virtual Stg::Model *GetModel(const std::string &name) const = 0;
    virtual Stg::usec_t SimTimeNow(void) const = 0;
  };

  /**
   * @brief Stage World supporting stepped execution
   */
  class StepWorld : public AbstractStepWorld, public Stg::World
  {
  public:
    using World::World;
    virtual ~StepWorld() {};

    virtual bool Load(const std::string &worldfile_path) { World::Load(worldfile_path); };
    virtual void UnLoad() { World::UnLoad(); };
    virtual Stg::Model *GetModel(const std::string &name) const { World::GetModel(name); };
    virtual Stg::usec_t SimTimeNow(void) const { World::SimTimeNow(); };

    virtual void step()
    {
      World::Stop();
      World::Update();
    }
  };

  /**
   * @brief Stage World GUI supporting stepped execution
   * @todo make it general to use without GUI
   */
  class StepWorldGui : public AbstractStepWorld, public Stg::WorldGui
  {
  public:
    StepWorldGui(int width, int height, const char *caption) : Stg::WorldGui(width, height, caption) {}
    using WorldGui::WorldGui;

    virtual bool Load(const std::string &worldfile_path) { WorldGui::Load(worldfile_path); };
    virtual void UnLoad() { WorldGui::UnLoad(); };
    virtual Stg::Model *GetModel(const std::string &name) const { WorldGui::GetModel(name); };
    virtual Stg::usec_t SimTimeNow(void) const { WorldGui::SimTimeNow(); };

    void step()
    {
      WorldGui::Stop();
      Fl::lock();
      WorldGui::Update();
      Fl::unlock();
      Fl::awake();
    }
  };

  /**
   *
   * @param argc commandline argument argc
   * @param argv commandline argument argv
   * @param worldfile location of worldfile
   * @param sensor_callback callback functor on sensor update
   */
  StageInterface(int argc, char **argv,
                 const boost::shared_ptr<AbstractStepWorld> &stage_world, const std::string &worldfile,
                 const boost::function<int (const sensor_msgs::LaserScanConstPtr&, const nav_msgs::OdometryConstPtr)> &sensor_callback);

  /**
   * @brief reset the StageInterface with another new world file
   * @param worldfile  path of the new world file
   */
  void resetWorld(const std::string &worldfile);

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
   * @brief teleport robot to a pose
   * @param x x-coordinates
   * @param y y-coordinates
   * @param a orientation
   */
  void setRobotPose(double x, double y, double a);

  /**
   * @brief updates the velocity of the robot (takes effect in the next laser callback for synchronization)
   * @param cmd_vel
   */
  void updateCmdVel(const geometry_msgs::Twist &cmd_vel);

  /**
   * @brief stage callback for pose update (static because we need function pointer)
   * @param model stage pose model
   * @param stage_interface pointer to object of StageInterface
   * @return 0 (indicates we want the callback to be called again)
   */
  static int poseUpdateCallback(Stg::Model *model, StageInterface *stage_interface);

  /**
   * @brief stage callback for laser update (static because we need function pointer)
   * @param model stage pose model
   * @param stage_interface pointer to object of StageInterface
   * @return 0 (indicates we want the callback to be called again)
   */
  static int laserUpdateCallback(Stg::Model *model, StageInterface *stage_interface);

protected:
  /**
   * @brief update member variable/models based on new world
   */
  void updateModels();

  // TODO: make the type of stage_world_ only World (to use without GUI)
  boost::shared_ptr<AbstractStepWorld> stage_world_;

  /**
   * @brief callback functor on sensor data update
   */
  boost::function<int (const sensor_msgs::LaserScanConstPtr&, const nav_msgs::OdometryConstPtr&)> sensor_callback_functor_;

  Stg::ModelRanger *laser_model_;
  Stg::ModelPosition *robot_model_;

  boost::mutex laser_scan_mutex_;
  sensor_msgs::LaserScanPtr laser_scan_msg_;

  boost::mutex odom_mutex_;
  nav_msgs::OdometryPtr odom_msg_;

  boost::mutex cmd_vel_mutex_;
  geometry_msgs::TwistPtr cmd_vel_msgs_;

  // TODO: don't use publishers
  ros::Publisher laser_pub_;
  ros::Publisher odom_pub_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

#endif //STAGE_FRONTIER_DATAGEN_STAGE_INTERFACE_H
