//
// Created by rakesh on 13/06/18.
//

#ifndef STAGE_FRONTIER_DATAGEN_STAGE_INTERFACE_H
#define STAGE_FRONTIER_DATAGEN_STAGE_INTERFACE_H

/**
 * @brief interface for stage simulation
 * @details adapted from https://github.com/Voidminded/DQNStageROS
 */
#include <stage.hh>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

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
  StageInterface(int argc, char **argv, const std::string &worldfile,
                 const boost::function<int (Stg::Model*)> &pose_callback, const boost::function<int (Stg::Model*)> &laser_callback);

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

  boost::function<int (Stg::Model*)> laser_callback_functional_;
  boost::function<int (Stg::Model*)> pose_callback_functional_;
protected:
  // TODO: make the type of stage_world_ only World (to use without GUI)
  boost::shared_ptr<StepWorldGui> stage_world_;

};

#endif //STAGE_FRONTIER_DATAGEN_STAGE_INTERFACE_H
