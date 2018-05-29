//
// Created by rakesh on 28/05/18.
//

#include <ros/ros.h>
#include <stage_frontier_datagen/kth_stage_loader.h>

using namespace stage_frontier_datagen;

class KTHStageNode
{
public:
  KTHStageNode() : private_nh_("~")
  {
    if (!private_nh_.getParam("dataset_dir", dataset_dir_))
    {
      ROS_ERROR("ROSPARAM dataset_dir not set");
      exit(1);
    }

    kth_stage_loader_.loadDirectory(dataset_dir_);

    run();
  }

  void run()
  {
    const std::vector<floorplanGraph> &floorplans = kth_stage_loader_.getFloorplans();
    for (const auto &floorplan: floorplans)
    {
      auto free_points = kth_stage_loader_.getRandomUnobstructedPoints(floorplan);
      cv::Mat map = floorplan::GraphFileOperations::getGraphLayout(floorplan, MAP_RESOLUTION, MAP_SIZE);
      // invert (so that only obstacle visible)
      map = 255 - map;
      cv::Mat free_points_map = cv::Mat::zeros(map.rows, map.cols, map.type());

      for (const auto &free_point: free_points)
      {
        free_points_map.at<uint8_t>(free_point) = 255;
      }

      std::vector<cv::Mat> channels(3);
      channels[2] = map;
      channels[1] = free_points_map;
      channels[0] = cv::Mat::zeros(map.rows, map.cols, map.type());

      cv::Mat rgb_image;
      cv::merge(channels, rgb_image);

      std::string floorplan_name = floorplan.m_property->floorname;
      auto extension_start = floorplan_name.find_last_of(".");
      if (extension_start != std::string::npos)
      {
        floorplan_name = floorplan_name.substr(0, extension_start);
      }

      std::string img_filename = std::string("/tmp/") + floorplan_name + ".png";
      cv::imwrite(img_filename, rgb_image);
      ROS_INFO("wrote map: %s", img_filename.c_str());
    }
  }

protected:
  ros::NodeHandle private_nh_;
  KTHStageLoader kth_stage_loader_;

  std::string dataset_dir_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kth_stage_node");
  KTHStageNode kth_stage_node;
  return 0;

}

