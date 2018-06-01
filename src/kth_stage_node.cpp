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
#include <signal.h>
#include <sys/wait.h>
#include <chrono>
#include <thread>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// custom includes
#include <stage_frontier_datagen/kth_stage_loader.h>
#include <stage_frontier_datagen/simple_exploration_controller.h>
#include <stage_frontier_datagen/utils.h>
#include <stage_frontier_datagen/frontier_analysis.h>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <opencv2/imgproc/imgproc.hpp>

using namespace stage_frontier_datagen;

class KTHStageNode
{
public:
  KTHStageNode()
    : private_nh_("~"),
      exploration_controller_(new SimpleExplorationController()),
      planner_status_(true),
      spin_thread_(&KTHStageNode::spin, this)
  {
    if (!private_nh_.getParam("dataset_dir", dataset_dir_))
    {
      ROS_ERROR("ROSPARAM dataset_dir not set");
      exit(1);
    }
    last_path_.header.stamp = ros::Time(0);

    kth_stage_loader_.loadDirectory(dataset_dir_);
    exploration_controller_->subscribeNewPlan(boost::bind(&KTHStageNode::newPlanCallback, this, _1, _2));

    run();
  }

  void newPlanCallback(const SimpleExplorationController &exploration_controller, bool planner_status)
  {
    ROS_INFO("Received new plan");
    planner_status_ = planner_status;

    if (planner_status)
    {
      auto costmap_2d_ros = exploration_controller.getCostmap2DROS();
      cv::Mat estimated_map = frontier_analysis::getMap(costmap_2d_ros, MAP_RESOLUTION);

      // TODO: move these processings to frontier_analysis
      // --------------------------- get frontiers bounding boxes --------------------------- //
      cv::Mat frontier_bounding_box_image = cv::Mat(
        current_groundtruth_map_.rows, current_groundtruth_map_.cols, current_groundtruth_map_.type(),
        cv::Scalar(0)
      );
      auto planner = exploration_controller.getPlanner();
      auto frontier_img = planner->getFrontierImg();
      auto clustered_frontier_world_points = planner->getClusteredFrontierPoints();

      cv::Mat original_costmap_image = frontier_analysis::getMap(costmap_2d_ros, costmap_2d_ros->getCostmap()->getResolution());
      for (const auto &frontier_world_points: clustered_frontier_world_points)
      {
        auto frontier_map_points = frontier_analysis::worldPointsToMapPoints(frontier_world_points, costmap_2d_ros);
        cv::Rect bounding_box_costmap = cv::boundingRect(frontier_map_points);
        cv::Rect bounding_box_map = frontier_analysis::resizeToDesiredResolution(
          bounding_box_costmap,
          costmap_2d_ros,
          MAP_RESOLUTION
        );
        frontier_bounding_box_image(bounding_box_map) = cv::Scalar(255);
        original_costmap_image(bounding_box_costmap) = cv::Scalar(127);
      }
      cv::imwrite("/tmp/costmap.png", original_costmap_image);

      {
        // flip vertically cuz the positive y in image is going down
        cv::Mat tmp_img;
        cv::flip(frontier_bounding_box_image, tmp_img, 0);
        frontier_bounding_box_image = tmp_img;
      }

      // --------------------------- clip to get groundtruth portion of the map --------------------------- //
      auto diff_size_rows = estimated_map.rows - current_groundtruth_map_.rows;
      auto diff_size_cols = estimated_map.cols - current_groundtruth_map_.cols;
      cv::Mat estimated_clipped_map = estimated_map
        .rowRange((int)std::floor(diff_size_rows/2.0), estimated_map.rows - (int)std::ceil(diff_size_rows/2.0))
        .colRange((int)std::floor(diff_size_cols/2.0), estimated_map.cols - (int)std::ceil(diff_size_cols/2.0));
      assert(estimated_clipped_map.size == current_groundtruth_map_.size);

      // --------------------------- multi-channeled image for visualization --------------------------- //
      std::vector<cv::Mat> channels(3);
      channels[0] = frontier_bounding_box_image;
      channels[1] = estimated_clipped_map;
      channels[2] = cv::Scalar(255) - current_groundtruth_map_;

      cv::Mat rgb_image;
      cv::merge(channels, rgb_image);
      cv::imwrite("/tmp/map.png", rgb_image);
    }
  }

  void runStageWorld(std::string worldfile)
  {
    int pipe;

    auto roslaunch_process = utils::popen2(
      (std::string("roslaunch stage_frontier_datagen stage_gmapping.launch world_file:=") + worldfile).c_str(), &pipe
    );

//    auto pipe = popen((std::string("roslaunch stage_frontier_datagen stage_gmapping.launch world_file:=") + worldfile).c_str(), "r");
    std::this_thread::sleep_for(std::chrono::seconds(STAGE_LOAD_SLEEP));

    std::cout << kill(roslaunch_process, 0) << std::endl;
    if (roslaunch_process <= 0 || kill(roslaunch_process, 0) < 0)
    {
      ROS_ERROR("Error launch simulation");
      return;
    }

    planner_status_ = true;
    exploration_controller_->startExploration();

    try
    {
      char buffer[128];
      int status_child;
      int ret;
      do
      {
        // TODO: find a way to read the piped output
        ret = waitpid(roslaunch_process, &status_child, WNOHANG);
      } while (!WIFEXITED(status_child) && planner_status_);
      ROS_INFO("simulation session ended successfully");

//      while (!feof(pipe) && planner_status_)
//      {
//        if (fgets(buffer, 128, pipe) != NULL)
//          std:cout << buffer << std::endl;
//      }
    }
    catch (...)
    {
      ROS_ERROR("stage world died unexpectedly");
      return;
    }

    while (exploration_controller_->isPlannerRunning());
    exploration_controller_->stopExploration();

    kill(roslaunch_process, SIGKILL);
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

      current_groundtruth_map_ = floorplan::GraphFileOperations::saveGraphLayoutToPNG(TMP_FLOORPLAN_BITMAP, floorplan, MAP_RESOLUTION, MAP_SIZE);
      cv::imwrite(TMP_FLOORPLAN_BITMAP, current_groundtruth_map_);

      writeDebugMap(floorplan, current_groundtruth_map_, free_points, random_index);

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

  /**
   * @brief visualize the point chosen for initialization
   * @param floorplan
   * @param map
   * @param free_points
   * @param random_index
   */
  void writeDebugMap(floorplanGraph floorplan, cv::Mat map, const std::vector<cv::Point> &free_points, size_t random_index)
  {
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

  void spin()
  {
    while (ros::ok())
    {
      if (planner_status_)
      {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

protected:
  ros::NodeHandle private_nh_;
  KTHStageLoader kth_stage_loader_;
  boost::shared_ptr<SimpleExplorationController> exploration_controller_;
  std::string dataset_dir_;

  nav_msgs::Path last_path_;
  cv::Mat current_groundtruth_map_;

  boost::thread spin_thread_;
  boost::atomic_bool planner_status_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kth_stage_node");
  srand(std::time(NULL));

//  auto spin_thread = boost::thread(boost::bind(&ros::spin));
//  spin_thread.detach();

  KTHStageNode kth_stage_node;
  return 0;

}

