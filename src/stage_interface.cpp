//
// Created by rakesh on 13/06/18.
//

#include <stage_frontier_datagen/stage_interface.h>

using namespace Stg;

StageInterface::StageInterface(int argc, char **argv, const std::string &worldfile,
                               const boost::function<int (Stg::Model*)> &pose_callback, const boost::function<int (Stg::Model*)> &laser_callback)
  : stage_world_(new StepWorldGui(800, 700, "Exploration Datagenerator")),
    pose_callback_functional_(pose_callback),
    laser_callback_functional_(laser_callback)
{
  stage_world_->Load(worldfile);

  auto robot_model = stage_world_->GetModel("r0");
  robot_model->AddCallback(Model::CB_UPDATE, (model_callback_t)StageInterface::poseUpdateCallback, this);

  auto laser_model = robot_model->GetChild("ranger:0");
  laser_model->AddCallback(Model::CB_UPDATE, (model_callback_t)StageInterface::laserUpdateCallback, this);
}

int StageInterface::poseUpdateCallback(Model *model, StageInterface *stage_interface)
{
  return stage_interface->pose_callback_functional_(model);
}

int StageInterface::laserUpdateCallback(Model *model, StageInterface *stage_interface)
{
  return stage_interface->laser_callback_functional_(model);
}