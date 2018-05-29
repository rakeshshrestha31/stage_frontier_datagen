//
// Created by rakesh on 28/05/18.
//

#define PACKAGE_NAME "stage_frontier_datagen"
#define TMP_FLOORPLAN_BITMAP "/tmp/floorplan.png"
#define TMP_WORLDFILE "tmp_floorplan.world"

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
      std::cout << free_points.at(random_index) << std::endl;

      cv::Mat map = floorplan::GraphFileOperations::saveGraphLayoutToPNG(TMP_FLOORPLAN_BITMAP, floorplan, MAP_RESOLUTION, MAP_SIZE);

      // ------------------ debug map start ------------------ //
      cv::Mat free_points_map = cv::Mat::zeros(map.rows, map.cols, map.type());
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
      auto extension_start = floorplan_name.find_last_of(".");
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
      std::system(
        (std::string("rosrun stage_ros stageros ") + tmp_worldfile).c_str()
      );
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
  srand(std::time(NULL));
  KTHStageNode kth_stage_node;
  return 0;

}

