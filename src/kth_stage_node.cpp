//
// Created by rakesh on 28/05/18.
//

#define PACKAGE_NAME "stage_frontier_datagen"
#define TMP_FLOORPLAN_BITMAP "/tmp/floorplan.png"

// number of meters to increase on either limit of map size (to accommodate mapping errors)
#define MAP_SIZE_CLEARANCE 2

#define STAGE_LOAD_SLEEP 3

#define PLANNER_FAILURE_TOLERANCE 3 // 15e5
// time interval to call planner (in simulation time)
#define PLANNER_CALL_INTERVAL 15

#define NUM_RUNS_IN_ONE_MAP 50

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
//#include <stage_frontier_datagen/frontier_analysis.h>
#include <stage_frontier_datagen/stage_interface.h>
#include <stage_frontier_datagen/data_recorder.h>

#include <hector_exploration_planner/frontier_analysis.h>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <X11/Xlib.h>

#include <opencv2/imgproc/imgproc.hpp>

using namespace stage_frontier_datagen;
using namespace hector_exploration_planner;

class KTHStageNode
{
public:
  KTHStageNode(int argc, char **argv, bool is_gui)
    : argc_(argc), argv_(argv),
      is_gui_(is_gui),
      reset_stage_world_(false),
      is_latest_sensor_received_(false),
      private_nh_("~"),
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

    if (!private_nh_.getParam("data_record_dir", data_record_dir))
    {
      ROS_ERROR("ROSPARAM data_record_dir not set");
      exit(1);
    }

    package_path_ = ros::package::getPath(PACKAGE_NAME);
    last_path_.header.stamp = ros::Time(0);

    if (!kth_stage_loader_->loadDirectory(dataset_dir_))
    {
      ROS_ERROR("no valid floorplan");
    }

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

  void recordPlanInformation(const SimpleExplorationController &exploration_controller,
      int interation, int plan_number)
  {
    auto planner = exploration_controller.getPlanner();
    auto custom_costmap = exploration_controller.getCostmap2DROS();

    //---- get raw costmap and clusted_frontier_points, resolution is the same with original costmap ---
    // get costMap with three channels, unkown, free, obstacle
    cv::Mat rawCostMap = frontier_analysis::getRawMap(custom_costmap);
    cv::Mat costMap = frontier_analysis::splitRawMap(rawCostMap);

    // get robot pose
    Pose2D robotPose = exploration_controller.getRobotPose();

    // get plan related info
    std::vector<geometry_msgs::PoseStamped> plan_world_poses;
    std::vector<double> plan_ms_times, plan_explored_areas;
    double simu_time, planner_time;
    exploration_controller.getLastPlanInfo(plan_world_poses, plan_ms_times,
        plan_explored_areas, simu_time, planner_time);
    std::vector<Pose2D> plan_poses = frontier_analysis::worldPosesToMapPoses(plan_world_poses, custom_costmap);

    // get frontiers points with the same resolution of costmap
    std::vector<std::vector<Pose2D>> all_clusters_frontiers;
    frontier_analysis::getFrontierPoints(custom_costmap, planner, all_clusters_frontiers);

    // get frontiers cluster centers with the same resolution of costmap
    std::vector<Pose2D> frontier_cluster_centers;
    frontier_analysis::getFronierCenters(custom_costmap, planner, frontier_cluster_centers);

    //--- resize costmap and clusted_frontier_points to the same resolution of ground_truth map---
    double resize_ratio = custom_costmap->getCostmap()->getResolution() / MAP_RESOLUTION;

    cv::Mat costMap_resize;
    std::vector<std::vector<Pose2D>> frontiers_resize;
    std::vector<Pose2D> frontier_centers_resize;
    std::vector<Pose2D> plan_poses_resize;
    Pose2D robotPose_resize;

    // only resize when the ratio is not equal to 1
    if(std::abs(resize_ratio - 1.0) > 1e-6)
    {
      cv::resize(costMap, costMap_resize, cv::Size(), resize_ratio, resize_ratio);
      frontier_analysis::resizePoints(all_clusters_frontiers, frontiers_resize, resize_ratio);
      frontier_analysis::resizePoints(frontier_cluster_centers, frontier_centers_resize, resize_ratio);
      robotPose_resize = frontier_analysis::resizePoint(robotPose, resize_ratio);
      frontier_analysis::resizePoints(plan_poses, plan_poses_resize, resize_ratio);
    }
    else
    {
      costMap_resize = costMap;
      frontiers_resize = all_clusters_frontiers;
      frontier_centers_resize = frontier_cluster_centers;
      robotPose_resize = robotPose;
      plan_poses_resize = plan_poses;
    }

    //----- clip or padding costmap and frontier_points to adapt the size of ground truth----
    cv::Size currentSize = costMap_resize.size();
    cv::Size GTSize = current_groundtruth_map_.size();
    cv::Mat costMap_resize_clipped = frontier_analysis::convertToGTSizeFillUnknown(costMap_resize, GTSize);
    assert(costMap_resize_clipped.size() == GTSize);
    std::vector<std::vector<Pose2D>> frontiers_resize_clipped;
    std::vector<Pose2D> frontier_centers_clipped, plan_poses_clipped;
    frontier_analysis::convertToGroundTruthSize(frontiers_resize, frontiers_resize_clipped, currentSize, GTSize);
    frontier_analysis::convertToGroundTruthSize(frontier_centers_resize, frontier_centers_clipped, currentSize, GTSize);
    frontier_analysis::convertToGroundTruthSize(plan_poses_resize, plan_poses_clipped, currentSize, GTSize);
    Pose2D robotPose_resize_clipped = frontier_analysis::convertToGroundTruthSize(robotPose_resize, currentSize, GTSize);

    //-------- generate bounding box and boundingBox images for frontiers ---------
    std::vector<cv::Rect> boundingBoxes = frontier_analysis::generateBoundingBox(frontiers_resize_clipped);
    cv::Mat boundingBoxImg = frontier_analysis::generateBoundingBoxImage(boundingBoxes, GTSize);

    //-------- record map and related informations ------
    boost::filesystem::path floorplanName(getFloorplanName());
    string floorplan_baseName = floorplanName.stem().string(); // baseName without extension
    data_recorder::recordImage(data_record_dir, floorplan_baseName, iteration_, plan_number,
                               costMap_resize_clipped, boundingBoxImg);
    data_recorder::recordInfo(data_record_dir, floorplan_baseName, iteration_, plan_number,
                              frontiers_resize_clipped, frontier_centers_clipped, boundingBoxes, robotPose,
                              plan_poses_resize, plan_ms_times, plan_explored_areas, simu_time, planner_time);

    // generate verifyImage and record it: optional
    cv::Mat verifyImg = frontier_analysis::generateVerifyImage(costMap_resize_clipped, boundingBoxes,
                                                               frontiers_resize_clipped, robotPose_resize_clipped,
                                                               plan_poses_clipped);
    data_recorder::recordVerifyImage(data_record_dir, floorplan_baseName, iteration_, plan_number, verifyImg);
  }

  void planFinishedCallback(const SimpleExplorationController &exploration_controller, int plan_num)
  {
    recordPlanInformation(exploration_controller, iteration_, plan_num);
  }

  /**
   * @brief callback function for new plan from exploration_controller_ member
   * @param exploration_controller reference to the SimpleExploration object that called this callback function
   * @param planner_status status of the planner
   */
  void newPlanCallback(const SimpleExplorationController &exploration_controller, bool planner_status)
  {
//    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ROS_INFO("Received new plan");
    planner_status_ = planner_status;
    if (planner_status)
    {
      planner_failure_count_ = 0;
    }
    else
    {
      planner_failure_count_++;
    }
  }

  /**
   * @brief create new stage world for new world file
   */
  void createNewWorld()
  {
    ROS_INFO("Starting new stage world");

    const char *window_title = "Exploration Data Generator";
    if (is_gui_)
    {
      stage_world_.reset(new StageInterface::StepWorldGui(800, 700, window_title));
    }
    else
    {
      stage_world_.reset(new StageInterface::StepWorld(window_title));
    }

    stage_interface_.reset(new StageInterface(argc_, argv_, stage_world_, world_file_,
                                              boost::bind(&KTHStageNode::sensorsCallback, this, _1, _2)));
  }

  void resetStageWorld()
  {
    // create new stage world
    createNewWorld();

    reset_stage_world_ = false;

    // create new exploration_controller
    boost::shared_ptr<SimpleExplorationController> control_ptr(new SimpleExplorationController(this->stage_world_));
    this->exploration_controller_ = control_ptr;
    exploration_controller_->subscribeNewPlan(boost::bind(&KTHStageNode::newPlanCallback, this, _1, _2));
    exploration_controller_->subscribePlanFinished(boost::bind(&KTHStageNode::planFinishedCallback, this, _1, _2));

    if (exploration_controller_)
    {
      exploration_controller_->updateCmdVelFunctor(
          boost::bind(&StageInterface::updateCmdVel, stage_interface_, _1)
      );
    }
  }

  /**
   *
   * @param worldfile world file for stage
   * @param floorplan floorplan to load
   */
  void loadStageWorld(const std::string &worldfile, const floorplanGraph &floorplan)
  {
    auto metric_ratio = floorplan.m_property->real_distance / floorplan.m_property->pixel_distance;
    auto metric_width = (floorplan.m_property->maxx - floorplan.m_property->minx) * metric_ratio;
    auto metric_height = (floorplan.m_property->maxy - floorplan.m_property->miny) * metric_ratio;
    auto half_width = metric_width / 2;
    auto half_height = metric_height / 2;

    metric_map_size_ = cv::Size2f(metric_width + 2 * MAP_SIZE_CLEARANCE, metric_height + 2 * MAP_SIZE_CLEARANCE);
    metric_map_half_size_ = cv::Size2f(half_width + MAP_SIZE_CLEARANCE, half_height + MAP_SIZE_CLEARANCE);

    private_nh_.setParam("global_costmap/ground_truth_layer/xmin", -half_width - MAP_SIZE_CLEARANCE);
    private_nh_.setParam("global_costmap/ground_truth_layer/ymin", -half_height - MAP_SIZE_CLEARANCE);
    private_nh_.setParam("global_costmap/ground_truth_layer/xmax", half_width + MAP_SIZE_CLEARANCE);
    private_nh_.setParam("global_costmap/ground_truth_layer/ymax", half_height + MAP_SIZE_CLEARANCE);

    world_file_ = worldfile;

    //Single thread version
//    this->resetStageWorld();

    // Two thread version
    reset_stage_world_ = true;
    while (reset_stage_world_);
    std::this_thread::sleep_for(std::chrono::seconds(STAGE_LOAD_SLEEP));

    ROS_INFO_STREAM("Running floorplan: " << floorplan.m_property->floorname);
  }

  /**
   * @brief run the exploration in current stage world
   */
  void runStageWorld()
  {
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

        while (!is_latest_sensor_received_);
        is_latest_sensor_received_ = false;

        exploration_controller_->generateCmdVel();

        // don't update the plan in the middle of execution of another one. The planner takes a long time, hence the by
        // the time planner is done, the robot would have moved significant distance and the plan is no longer starting
        // from the robot's current position
//        updatePlan();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));



        if (!isRobotWithinMap())
        {
          ROS_WARN("The robot has strayed outside of the map, quitting");
          break;
        }

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
    string map_name = "";
    int map_num_t = 0, iteration_t = 0;
    if(data_recorder::readConfig(map_name, map_num_t, iteration_t))
    {
      map_num = map_num_t; //here we only restore map_num
    }
    else
    {
      map_num = 0;
    }

    map_num = 0;

    for (int i = map_num; i < kth_stage_loader_->getFloorplans().size(); i++, map_num ++)
    {
      const KTHStageLoader::floorplan_t &floorplan = kth_stage_loader_->getFloorplans()[i];
      iteration_ = 0;
      if (floorplan.unobstructed_points.empty())
      {
        continue;
      }

      // run in the same map a number of times
      for (size_t floorplan_run_idx = 0; floorplan_run_idx < NUM_RUNS_IN_ONE_MAP; floorplan_run_idx++)
      {
        iteration_ ++;

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

        boost::filesystem::path floorplanName(getFloorplanName());
        string baseName = floorplanName.stem().string(); // baseName without extension
        data_recorder::recordGTMapAndResolution(data_record_dir, baseName, current_groundtruth_map_, MAP_RESOLUTION);

        writeDebugMap(floorplan.graph, current_groundtruth_map_, floorplan.unobstructed_points, random_index);

        std::string worldfile_directory(std::string(package_path_) + "/worlds");
        std::string tmp_worldfile_name = kth_stage_loader_->createWorldFile(
          floorplan.graph, random_point_meters, worldfile_directory, std::string(TMP_FLOORPLAN_BITMAP)
        );

        if (floorplan_run_idx == 0)
        {
          loadStageWorld(tmp_worldfile_name, floorplan.graph);
        }
        else
        {
          stage_interface_->setRobotPose(
            random_point_meters.x,random_point_meters.y, (double)(rand() % 360)
          );
        }

        data_recorder::writeConfig(this->getFloorplanName(), map_num, iteration_);

        runStageWorld();

        if (!ros::ok())
        {
          return;
        }
      }
    }
  }

  string getFloorplanName()
  {
    return kth_stage_loader_->getFloorplans()[map_num].graph.m_property->floorname;
  }


  bool isRobotWithinMap()
  {
    Stg::Pose robot_pose;
    auto is_success = stage_interface_->getRobotPose(robot_pose);
    return is_success
      &&robot_pose.x >= -metric_map_half_size_.width
      && robot_pose.x <= metric_map_half_size_.width
      && robot_pose.y >= -metric_map_half_size_.height
      && robot_pose.y <= metric_map_half_size_.height;
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

  int sensorsCallback(const sensor_msgs::LaserScanConstPtr &laser_scan, const nav_msgs::OdometryConstPtr &odom)
  {
    boost::mutex::scoped_lock lock(sensor_callback_mutex_);
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
  std::string data_record_dir;

  nav_msgs::Path last_path_;
  cv::Mat current_groundtruth_map_;
  cv::Size2d metric_map_size_;
  cv::Size2d metric_map_half_size_;

  nav_msgs::Odometry groundtruth_odom_;
  boost::mutex groundtruth_odom_mutex_;
  ros::Subscriber groundtruth_odom_subscriber_;

  boost::thread spin_thread_;
  boost::atomic_bool planner_status_;
  boost::atomic_uint planner_failure_count_;

  boost::shared_ptr<StageInterface> stage_interface_;
  boost::shared_ptr<StageInterface::AbstractStepWorld> stage_world_;
  boost::atomic_bool reset_stage_world_;

//  bool reset_stage_world_;
  std::string world_file_;
  int argc_;
  char **argv_;

  double last_plan_time_;

  boost::mutex sensor_callback_mutex_;
  /** @brief whether the latest sensor has been received (used for stepping stage world as soon as it's received */
  boost::atomic_bool is_latest_sensor_received_;

  std::string package_path_;

  // current State: which map is running, the running iteration, the path planning number
  int map_num = 0;
  int iteration_ = 0;

  bool is_gui_;
};

int main(int argc, char **argv)
{
  XInitThreads();
  Stg::Init(&argc, &argv);
  ros::init(argc, argv, "kth_stage_node");

  ros::NodeHandle nh("~");
  bool is_gui;
  nh.param("gui", is_gui, false);

  srand(std::time(NULL));

//  auto spin_thread = boost::thread(boost::bind(&ros::spin));
//  spin_thread.detach();

  KTHStageNode kth_stage_node(argc, argv, is_gui);

  // Single thread version
//  kth_stage_node.run();

  // Two thread version
  boost::thread run_thread(boost::bind(&KTHStageNode::run, &kth_stage_node));
  run_thread.detach();

  while (ros::ok())
  {
    if (is_gui && Fl::first_window() && !kth_stage_node.isResetStageWorld())
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

