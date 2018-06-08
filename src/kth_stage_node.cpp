//
// Created by rakesh on 28/05/18.
//

#define PACKAGE_NAME "stage_frontier_datagen"
#define TMP_FLOORPLAN_BITMAP "/tmp/floorplan.png"

#define STAGE_LOAD_SLEEP 5

// std includes
#include <random>
#include <ctime>
#include <cstdlib>
#include <csignal>
#include <sys/wait.h>
#include <chrono>
#include <thread>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>

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

    groundtruth_odom_subscriber_ = private_nh_.subscribe("/base_pose_ground_truth", 5, &KTHStageNode::groundtruthOdomCallback, this);
    run();
  }

  /**
   * @brief callback for groundtruth pose
   * @param groundtruth_odom_msg
   */
  void groundtruthOdomCallback(const nav_msgs::OdometryConstPtr &groundtruth_odom_msg)
  {
    boost::mutex::scoped_lock lock(groundtruth_odom_mutex_);
    groundtruth_odom_ = *groundtruth_odom_msg;
  }

  /**
   * @brief thread-safe get method for groundtruth_odom_ member. We call this method rather that accessing the member directly
   * @return groundtruth pose
   */
  nav_msgs::Odometry getGroundtruthOdom()
  {
    boost::mutex::scoped_lock lock(groundtruth_odom_mutex_);
    return groundtruth_odom_;
  }

  /**
   * @brief callback function for new plan from exploration_controller_ member
   * @param exploration_controller reference to the SimpleExploration object that called this callback function
   * @param planner_status status of the planner
   */
  void newPlanCallback(const SimpleExplorationController &exploration_controller, bool planner_status)
  {
    ROS_INFO("Received new plan");
    planner_status_ = planner_status;

    if (planner_status)
    {
      auto costmap_2d_ros = exploration_controller.getCostmap2DROS();
      auto planner = exploration_controller.getPlanner();
      auto frontier_img = planner->getFrontierImg();
      auto clustered_frontier_poses = planner->getClusteredFrontierPoints();

      auto groundtruth_odom = getGroundtruthOdom();
      auto transform_gt_est = frontier_analysis::getTransformGroundtruthEstimated(costmap_2d_ros, groundtruth_odom);

      auto costmap_image = frontier_analysis::getMap(
        costmap_2d_ros, costmap_2d_ros->getCostmap()->getResolution(), transform_gt_est
      );
      auto frontier_bounding_box_image = frontier_analysis::getBoundingBoxImage(
        costmap_2d_ros, clustered_frontier_poses, transform_gt_est
      );

      // resize to desired resolution
      auto resized_costmap_image = frontier_analysis::resizeToDesiredResolution(
        costmap_image, costmap_2d_ros, MAP_RESOLUTION
      );
      auto resized_frontier_bounding_box_image = frontier_analysis::resizeToDesiredResolution(
        frontier_bounding_box_image, costmap_2d_ros, MAP_RESOLUTION
      );

      // --------------------------- multi-channeled image for visualization --------------------------- //
      std::vector<cv::Mat> channels(3);
      cv::Mat rgb_image;

      channels[0] = cv::Mat(costmap_image.rows, costmap_image.cols, costmap_image.type(), cv::Scalar(0));
      channels[1] = cv::Scalar(255) - costmap_image;
      channels[2] = frontier_bounding_box_image;
      cv::merge(channels, rgb_image);
      cv::imwrite("/tmp/costmap.png", rgb_image);

      // --------------------------- clip to get groundtruth portion of the map --------------------------- //
      cv::Mat resized_clipped_costmap = frontier_analysis::convertToGroundtruthSize(
        resized_costmap_image, current_groundtruth_map_.size()
      );

      cv::Mat resized_clipped_frontier_bb_image = frontier_analysis::convertToGroundtruthSize(
        resized_frontier_bounding_box_image, current_groundtruth_map_.size()
      );

      channels[0] = cv::Scalar(255) - current_groundtruth_map_;
      channels[1] = cv::Scalar(255) - resized_clipped_costmap;
      channels[2] = resized_clipped_frontier_bb_image;
      cv::merge(channels, rgb_image);
      cv::imwrite("/tmp/map.png", rgb_image);

    }
  }

  /**
   * @brief run the stage with SLAM
   * @param worldfile world file for stage
   */
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

  /**
   * @brief iterate over all the floorplans and run simulations over them
   */
  void run()
  {
    std::string package_name = ros::package::getPath(PACKAGE_NAME);

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


      std::string worldfile_directory = package_name + "/worlds/";
      std::string tmp_worldfile_name = kth_stage_loader_.createWorldFile(
        floorplan, random_point_meters, worldfile_directory, std::string(TMP_FLOORPLAN_BITMAP)
      );
      runStageWorld(tmp_worldfile_name);
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

  /**
   * @brief function for ros spin (to be called as a thread). Stops spinning when planner isn't ready to avoid costmap updates
   */
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

  /**
   *
   * @param map_coords
   * @param map_size
   * @param map_resolution
   * @return
   */
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
  cv::Mat current_groundtruth_map_;

  nav_msgs::Odometry groundtruth_odom_;
  boost::mutex groundtruth_odom_mutex_;
  ros::Subscriber groundtruth_odom_subscriber_;

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

