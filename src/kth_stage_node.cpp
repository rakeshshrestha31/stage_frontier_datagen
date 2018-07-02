//
// Created by rakesh on 28/05/18.
//

#define PACKAGE_NAME "stage_frontier_datagen"
#define TMP_FLOORPLAN_BITMAP "/tmp/floorplan.png"

// number of meters to increase on either limit of map size (to accommodate mapping errors)
#define MAP_SIZE_CLEARANCE 2

#define STAGE_LOAD_SLEEP 3

#define PLANNER_FAILURE_TOLERANCE 15 // 15e5
// time interval to call planner (in simulation time)
#define PLANNER_CALL_INTERVAL 15

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
//#include <tf/tf.h>

// custom includes
#include <stage_frontier_datagen/kth_stage_loader.h>
#include <stage_frontier_datagen/simple_exploration_controller.h>
#include <stage_frontier_datagen/utils.h>
#include <stage_frontier_datagen/frontier_analysis.h>
#include <stage_frontier_datagen/stage_interface.h>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <X11/Xlib.h>

#include <opencv2/imgproc/imgproc.hpp>

using namespace stage_frontier_datagen;

class KTHStageNode
{
public:
  KTHStageNode(int argc, char **argv, boost::shared_ptr<StageInterface::AbstractStepWorld> stage_world)
    : argc_(argc), argv_(argv),
      stage_world_(stage_world),
      reset_stage_world_(false),
      is_latest_sensor_received_(false),
      private_nh_("~"),
      exploration_controller_(new SimpleExplorationController()),
      planner_status_(true),
      planner_failure_count_(0),
//      spin_thread_(&KTHStageNode::spin, this),
      kth_stage_loader_(new KTHStageLoader()),
      last_plan_time_(0)
  {
    if (!private_nh_.getParam("dataset_dir", dataset_dir_))
    {
      ROS_ERROR("ROSPARAM dataset_dir not set");
      exit(1);
    }
    package_path_ = ros::package::getPath(PACKAGE_NAME);
    last_path_.header.stamp = ros::Time(0);

    if (!kth_stage_loader_->loadDirectory(dataset_dir_))
    {
      ROS_ERROR("no valid floorplan");
    }

    exploration_controller_->subscribeNewPlan(boost::bind(&KTHStageNode::newPlanCallback, this, _1, _2));

    groundtruth_odom_subscriber_ = private_nh_.subscribe("/base_pose_ground_truth", 5, &KTHStageNode::groundtruthOdomCallback, this);
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
      planner_failure_count_ = 0;
      auto costmap_2d_ros = exploration_controller.getCostmap2DROS();
      auto planner = exploration_controller.getPlanner();
      auto frontier_img = planner->getFrontierImg();
      auto clustered_frontier_poses = planner->getClusteredFrontierPoints();

//      auto groundtruth_odom = getGroundtruthOdom();
//      auto transform_gt_est = frontier_analysis::getTransformGroundtruthEstimated(costmap_2d_ros, groundtruth_odom);

      auto transform_gt_est = tf::Transform::getIdentity();

      auto costmap_image = frontier_analysis::getMap(
        costmap_2d_ros, costmap_2d_ros->getCostmap()->getResolution(), transform_gt_est
      );
      auto frontier_bounding_box_image = frontier_analysis::getBoundingBoxImage(
        costmap_2d_ros, clustered_frontier_poses, transform_gt_est
      );

      if (frontier_bounding_box_image.size() != costmap_image.size())
      {
        ROS_ERROR("costmap size changed while generating frontier images!!!");
        return;
      }

      cv::imwrite("/tmp/original.png", costmap_image);

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
      channels[1] = costmap_image;
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
      channels[1] = resized_clipped_costmap;
      channels[2] = resized_clipped_frontier_bb_image;
      cv::merge(channels, rgb_image);
      cv::imwrite("/tmp/map.png", rgb_image);
    }
    else
    {
      planner_failure_count_++;
    }
  }

  void resetStageWorld()
  {
    ROS_INFO("Starting new stage world");
    stage_interface_ = boost::make_shared<StageInterface>(
      argc_, argv_, stage_world_, world_file_,
      boost::bind(&KTHStageNode::sensorsCallback, this, _1, _2)
    );

    reset_stage_world_ = false;

    if (exploration_controller_)
    {
      exploration_controller_->updateCmdVelFunctor(
        boost::bind(&StageInterface::updateCmdVel, stage_interface_, _1)
      );
    }
  }

  /**
   * @brief run the stage with SLAM
   * @param worldfile world file for stage
   */
  void runStageWorld(const std::string &worldfile, const floorplanGraph &floorplan)
  {
    int pipe;
    auto metric_ratio = floorplan.m_property->real_distance / floorplan.m_property->pixel_distance;
    auto metric_width = (floorplan.m_property->maxx - floorplan.m_property->minx) * metric_ratio;
    auto metric_height = (floorplan.m_property->maxy - floorplan.m_property->miny) * metric_ratio;
    auto half_width = metric_width / 2;
    auto half_height = metric_height / 2;

    private_nh_.setParam("global_costmap/ground_truth_layer/xmin", -half_width - MAP_SIZE_CLEARANCE);
    private_nh_.setParam("global_costmap/ground_truth_layer/ymin", -half_height - MAP_SIZE_CLEARANCE);
    private_nh_.setParam("global_costmap/ground_truth_layer/xmax", half_width + MAP_SIZE_CLEARANCE);
    private_nh_.setParam("global_costmap/ground_truth_layer/ymax", half_height + MAP_SIZE_CLEARANCE);

    world_file_ = worldfile;
    reset_stage_world_ = true;
    while (reset_stage_world_);
    std::this_thread::sleep_for(std::chrono::seconds(STAGE_LOAD_SLEEP));

    ROS_INFO_STREAM("Running floorplan: " << floorplan.m_property->floorname);
    // single step for initial messages
    stage_interface_->step();
    planner_status_ = true;
    planner_failure_count_ = 0;

    exploration_controller_->startExploration();

    ROS_INFO("Starting exploration");
    try
    {
      char buffer[128];
      int status_child;
      int ret;
      do
      {
        stage_interface_->step();
        exploration_controller_->generateCmdVel();
        updatePlan();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        while (!is_latest_sensor_received_);
        is_latest_sensor_received_ = false;
      } while (planner_failure_count_ < PLANNER_FAILURE_TOLERANCE && ros::ok());
      ROS_INFO("simulation session ended successfully");
    }
    catch (...)
    {
      ROS_ERROR("stage world died unexpectedly");
      return;
    }

    while (exploration_controller_->isPlannerRunning());
    exploration_controller_->stopExploration();
  }

  /**
   * @brief iterate over all the floorplans and run simulations over them
   */
  void run()
  {
    for (const KTHStageLoader::floorplan_t &floorplan: kth_stage_loader_->getFloorplans())
    {
      if (floorplan.unobstructed_points.empty())
      {
        continue;
      }

      size_t random_index = std::rand() % floorplan.unobstructed_points.size();
      Point2D random_point_meters = convertMapCoordsToMeters(
        Point2D(
          floorplan.unobstructed_points.at(random_index).x,
          floorplan.unobstructed_points.at(random_index).y
        ),
        MAP_SIZE,
        MAP_RESOLUTION
      );

      current_groundtruth_map_ = floorplan.map.clone();
      cv::imwrite(TMP_FLOORPLAN_BITMAP, current_groundtruth_map_);

      writeDebugMap(floorplan.graph, current_groundtruth_map_, floorplan.unobstructed_points, random_index);

      std::string worldfile_directory(std::string(package_path_) + "/worlds/");
      std::string tmp_worldfile_name = kth_stage_loader_->createWorldFile(
        floorplan.graph, random_point_meters, worldfile_directory, std::string(TMP_FLOORPLAN_BITMAP)
      );
      runStageWorld(tmp_worldfile_name, floorplan.graph);

      if (!ros::ok())
      {
        return;
      }
    }
  }

  /**
   * @brief visualize the point chosen for initialization
   * @param floorplan
   * @param map
   * @param free_points
   * @param random_index
   */
  void writeDebugMap(const floorplanGraph &floorplan, cv::Mat map, const std::vector<cv::Point> &free_points, size_t random_index)
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

  bool isResetStageWorld()
  {
    return reset_stage_world_;
  }

  bool clearResetStageWorld()
  {
    reset_stage_world_ = false;
  }

  int sensorsCallback(const sensor_msgs::LaserScanConstPtr &laser_scan, const nav_msgs::OdometryConstPtr &odom)
  {
    exploration_controller_->updateRobotOdom(odom);
    exploration_controller_->updateCostmap(laser_scan, odom);
    is_latest_sensor_received_ = true;

    return 0;
  }

  void updatePlan()
  {
    double current_sim_time_sec = stage_world_->SimTimeNow() / 1e6;
    if (current_sim_time_sec - last_plan_time_ > PLANNER_CALL_INTERVAL)
    {
      exploration_controller_->updatePlan();
      last_plan_time_ = current_sim_time_sec;
    }
    // TODO: check how far it's away from last waypoint to see if it needs replanning
  }

  /**
   * @brief function for ros spin (to be called as a thread). Stops spinning when planner isn't ready to avoid costmap updates
   */
  void spin()
  {
    while (ros::ok())
    {
      if (planner_failure_count_ < PLANNER_FAILURE_TOLERANCE)
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
  boost::shared_ptr<KTHStageLoader> kth_stage_loader_;
  boost::shared_ptr<SimpleExplorationController> exploration_controller_;
  std::string dataset_dir_;

  nav_msgs::Path last_path_;
  cv::Mat current_groundtruth_map_;

  nav_msgs::Odometry groundtruth_odom_;
  boost::mutex groundtruth_odom_mutex_;
  ros::Subscriber groundtruth_odom_subscriber_;

  boost::thread spin_thread_;
  boost::atomic_bool planner_status_;
  boost::atomic_uint planner_failure_count_;

  boost::shared_ptr<StageInterface> stage_interface_;
  boost::shared_ptr<StageInterface::AbstractStepWorld> stage_world_;
  boost::atomic_bool reset_stage_world_;
  std::string world_file_;
  int argc_;
  char **argv_;

  double last_plan_time_;

  /** @brief whether the latest sensor has been received (used for stepping stage world as soon as it's received */
  boost::atomic_bool is_latest_sensor_received_;

  std::string package_path_;
};

int main(int argc, char **argv)
{
  const char *window_title = "Exploration Data Generator";
  XInitThreads();
  Stg::Init(&argc, &argv);
  ros::init(argc, argv, "kth_stage_node");

  ros::NodeHandle nh("~");
  bool is_gui;
  nh.param("gui", is_gui, false);

//  boost::shared_ptr<StageInterface::StepWorldGui> stage_world(new StageInterface::StepWorldGui(800, 700, "Exploration Data Generator"));
  boost::shared_ptr<StageInterface::AbstractStepWorld> stage_world;

  if (is_gui)
  {
    stage_world.reset(static_cast<StageInterface::AbstractStepWorld*>(new StageInterface::StepWorldGui(800, 700, window_title)));
  }
  else
  {
    stage_world.reset(static_cast<StageInterface::AbstractStepWorld*>(new StageInterface::StepWorld(window_title)));
  }

  srand(std::time(NULL));

//  auto spin_thread = boost::thread(boost::bind(&ros::spin));
//  spin_thread.detach();

  KTHStageNode kth_stage_node(argc, argv, stage_world);
  boost::thread run_thread(boost::bind(&KTHStageNode::run, &kth_stage_node));
  run_thread.detach();

  while (ros::ok())
  {
    if (Fl::first_window() && !kth_stage_node.isResetStageWorld())
    {
      Fl::wait();
    }

    // resetting of stage world needs to be done in the main function
    if (kth_stage_node.isResetStageWorld())
    {
      kth_stage_node.resetStageWorld();
      std::this_thread::sleep_for(std::chrono::seconds(STAGE_LOAD_SLEEP));
    }

  }
  return 0;

}

