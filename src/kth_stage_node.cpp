//
// Created by rakesh on 28/05/18.
//

#define PACKAGE_NAME "stage_frontier_datagen"
#define TMP_FLOORPLAN_BITMAP "/tmp/floorplan.png"
#define TMP_WORLDFILE "tmp_floorplan.world"
#define STAGE_LOAD_SLEEP 5

// std includes
#include <fstream>
#include <regex>
#include <random>
#include <ctime>
#include <cstdlib>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// custom includes
#include <stage_frontier_datagen/kth_stage_loader.h>
#include <stage_frontier_datagen/simple_exploration_controller.h>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

using namespace stage_frontier_datagen;

class KTHStageNode
{
public:
  void newPlanCallback(const SimpleExplorationController &exploration_controller)
  {
    ROS_INFO("Received new plan");
  }

  KTHStageNode()
    : private_nh_("~"),
      exploration_controller_(new SimpleExplorationController())
  {
    if (!private_nh_.getParam("dataset_dir", dataset_dir_))
    {
      ROS_ERROR("ROSPARAM dataset_dir not set");
      exit(1);
    }
    last_path_.header.stamp = ros::Time(0);

    kth_stage_loader_.loadDirectory(dataset_dir_);
    exploration_controller_->subscribeNewPlan(boost::bind(&KTHStageNode::newPlanCallback, this, _1));

    run();
  }



  void runStageWorld(std::string worldfile)
  {
    char buffer[128];
    auto pipe = popen((std::string("roslaunch stage_frontier_datagen stage_gmapping.launch world_file:=") + worldfile).c_str(), "r");
    ros::Duration(STAGE_LOAD_SLEEP).sleep();
    exploration_controller_->startExploration();

    try
    {
      while (!feof(pipe))
      {
        if (fgets(buffer, 128, pipe) != NULL)
          std:cout << buffer << std::endl;
      }
    }
    catch (...)
    {
      pclose(pipe);
      ROS_ERROR("stage world died unexpectedly");
    }
    pclose(pipe);

//    int status;
//    status = std::system(
//      (std::string("rosrun stage_ros stageros ") + worldfile + " __name:=stageros").c_str()
//    );
//    status = std::system(
//      (std::string("rosnode kill /stageros")).c_str()
//    );
  }

  void run()
  {
    const std::vector<floorplanGraph> &floorplans = kth_stage_loader_.getFloorplans();
    for (const auto &floorplan: floorplans)
    {
      auto free_points = kth_stage_loader_.getUnobstructedPoints(floorplan);
      if (free_points.empty())
      {
        continue;
      }
      size_t random_index = std::rand() % free_points.size();
      Point2D random_point_meters = convertMapCoordsToMeters(
        Point2D(
          free_points.at(random_index).x,
          free_points.at(random_index).y
        ),
        MAP_SIZE,
        MAP_RESOLUTION
      );

      cv::Mat map = floorplan::GraphFileOperations::saveGraphLayoutToPNG(TMP_FLOORPLAN_BITMAP, floorplan, MAP_RESOLUTION, MAP_SIZE);

      // ------------------ debug map start ------------------ //
      cv::Mat free_points_map(map.rows, map.cols, map.type(), cv::Scalar(0));
      for (const auto &free_point: free_points)
      {
        free_points_map.at<uint8_t>(free_point) = 255;
      }

      std::vector<cv::Mat> channels(3);
      channels[2] = cv::Scalar(255) - map;
      channels[1] = free_points_map;
      channels[0] = cv::Mat::zeros(map.rows, map.cols, map.type());

      cv::Mat rgb_image;
      cv::merge(channels, rgb_image);

      cv::circle(rgb_image, free_points.at(random_index), 2, cv::Scalar(255, 255, 255), -1);

      std::string floorplan_name = floorplan.m_property->floorname;
      auto extension_start = floorplan_name.find_last_of('.');
      if (extension_start != std::string::npos)
      {
        floorplan_name = floorplan_name.substr(0, extension_start);
      }

      std::string img_filename = std::string("/tmp/") + floorplan_name + ".png";
      cv::imwrite(img_filename, rgb_image);
      ROS_INFO("wrote map: %s", img_filename.c_str());
      // ------------------ debug map ends ------------------ //

      cv::imwrite(TMP_FLOORPLAN_BITMAP, map);

      // create world file from template
      std::string package_path = ros::package::getPath(PACKAGE_NAME);
      std::string worldfile_template = package_path + "/worlds/template.world";

      std::ifstream file_stream(worldfile_template);
      std::string worldfile_content((std::istreambuf_iterator<char>(file_stream)),
                                    std::istreambuf_iterator<char>());

      worldfile_content = std::regex_replace(worldfile_content, std::regex("@bitmap_image@"), TMP_FLOORPLAN_BITMAP);
      worldfile_content = std::regex_replace(
        worldfile_content, std::regex("@size@"),
        std::to_string(
          (floorplan.m_property->maxx - floorplan.m_property->minx) * floorplan.m_property->real_distance / floorplan.m_property->pixel_distance
        )
        + " "
        + std::to_string(
          (floorplan.m_property->maxy - floorplan.m_property->miny) * floorplan.m_property->real_distance / floorplan.m_property->pixel_distance
        )
      );

      worldfile_content = std::regex_replace(
        worldfile_content, std::regex("@start_pose@"),
        std::to_string(random_point_meters.x) + " "
        + std::to_string(random_point_meters.y) + " "
        + "0 " + // z coord
        std::to_string(rand() % 360) // orientation (degree)
      );

      std::string tmp_worldfile = std::string(package_path) + "/worlds/" + TMP_WORLDFILE;
      std::ofstream tmp_worldfile_stream(tmp_worldfile);
      tmp_worldfile_stream << worldfile_content;
      tmp_worldfile_stream.close();

      runStageWorld(tmp_worldfile);
    }
  }

  static Point2D convertMapCoordsToMeters(Point2D map_coords, cv::Size map_size, double map_resolution)
  {
    Point2D map_centroid(
      (map_size.width) / 2.0,
      (map_size.height) / 2.0
    );

    Point2D metric_coords = (map_coords - map_centroid) * map_resolution;
    // the map coordinates frame is different than stage
    metric_coords.y = -metric_coords.y;

    return metric_coords;
  }

protected:
  ros::NodeHandle private_nh_;
  KTHStageLoader kth_stage_loader_;
  boost::shared_ptr<SimpleExplorationController> exploration_controller_;
  std::string dataset_dir_;

  nav_msgs::Path last_path_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kth_stage_node");
  srand(std::time(NULL));

  auto spin_thread = boost::thread(boost::bind(&ros::spin));
  spin_thread.detach();

  KTHStageNode kth_stage_node;
  return 0;

}

