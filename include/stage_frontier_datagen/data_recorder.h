//
// Created by Fei-Peng Tian on 07/07/18.
//

#ifndef DATAGEN_DATA_RECORDER_H
#define DATAGEN_DATA_RECORDER_H

#include <json/json.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <hector_exploration_planner/frontier_analysis.h>

namespace stage_frontier_datagen {
  using hector_exploration_planner::frontier_analysis::Pose2D;

  namespace data_recorder {

    /**
     * @brief convert a 2D cv::Point to Json::Value
     * @param point cv::Point
     * @return Json::Value
     */
    Json::Value Point2Json(const cv::Point &point);

    /**
     * @brief Convert a 2D pose to Json::Value
     * @param pose 2D pose, position and orientation (yaw)
     * @return Json::Value
     */
    Json::Value Pose2Json(const Pose2D pose);

    /**
     * @brief convert a cv::Rect to Json::Value
     * @param point cv::Rect
     * @return Json::Value
     */
    Json::Value Rect2Json(const cv::Rect &rect);

    Json::Value FrontierClusters2Json(const std::vector<std::vector<Pose2D>> &cluster_frontiers);

    /**
     * convert all frontiers cluster to a Json::Value
     * @param cluster_frontiers frontier clusters, each frontier is represented by a cv::Point
     * @return Json value
     */
    Json::Value FrontierClusters2Json(const std::vector<std::vector<cv::Point>> &cluster_frontiers);

    /**
     * convert all BoundingBox (Rectangles) of frontiers to a Json::Value
     * @param rects BoundingBox of frontiers, each is represented by a cv::Rect
     * @return Json value
     */
    Json::Value Rects2Json(const std::vector<cv::Rect> &rects);

    /**
     * Record Ground Truth image and related resolution into directory base_dir/map_name
     * @param base_dir base directory for recording
     * @param map_name floorplan map name (better without extension)
     * @param groundTruth ground Truth images
     * @param resolution the resolution of the ground images, meters/pixel
     */
    void recordGTMapAndResolution(std::string base_dir, std::string map_name, cv::Mat groundTruth, double resolution);

    /**
     * @brief record a image into directory base_dir/map_name/iteration for once path-planning
     * @param base_dir base_dir base directory for recording
     * @param map_name floorplan map name (better without extension)
     * @param iteration the repeated iterations for this map
     * @param planning_num the number of path planning in one exploration
     * @param image image to be recorded
     * @param record__base_name base name for record the image
     */
    void recordImage(std::string base_dir, std::string map_name, int iteration, int planning_num,
                     const cv::Mat &image, std::string record__base_name);

    /**
     * @brief record costmap and bounding-box image into directory base_dir/map_name/iteration for once path-planning
     * @param base_dir base directory for recording
     * @param map_name floorplan map name (better without extension)
     * @param iteration the repeated iterations for this map
     * @param planning_num the number of path planning in one exploration
     * @param costmap costmap to be recorded
     * @param boundingBoxImg costmap to be recorded
     */
    void recordImage(std::string base_dir, std::string map_name, int iteration, int planning_num,
                     const cv::Mat &costmap, const cv::Mat &boundingBoxImg);

    /**
     * @brief record verify Image into directory base_dir/map_name/iteration for once path-planning
     * @param base_dir base directory for recording
     * @param map_name floorplan map name (better without extension)
     * @param iteration the repeated iterations for this map
     * @param planning_num to be recorded
     * @param verifyImg verifyImage to be recored
     */
    void recordVerifyImage(std::string base_dir, std::string map_name, int iteration, int planning_num,
                           cv::Mat verifyImg);

    /**
     * @brief record information into directory base_dir/map_name/iteration for once path-planning
     * @param base_dir base directory for recording
     * @param map_name floorplan map name (better without extension)
     * @param iteration the repeated iterations for this map
     * @param planning_num the number of path planning in one exploration
     * @param cluster_frontier all frontiers points in different frontier clusters
     * @param frontierBoundingBox all Bounding-Box for each cluster of frontiers
     * @param robotPose robot pose when last plan finished
     */
    void recordInfo(std::string base_dir, std::string map_name, int iteration, int planning_num,
                    const std::vector<std::vector<Pose2D>> &cluster_frontier,
                    const std::vector<cv::Rect> &frontierBoundingBox,
                    const Pose2D robotPose);
    /**
     * @brief write configuration files to record the current state in collecting data
     * @param map_name the floorplan map name
     * @param map_num the num of map in the dataset
     * @param iteration the repeated iterations for this map
     */
    void writeConfig(std::string map_name, int map_num, int iteration);

    /**
     * @brief read configuration files to restore the current state in collecting data
     * @param map_name the floorplan map name
     * @param map_num the num of map in the dataset
     * @param iteration the repeated iterations for this map
     * @return
     */
    bool readConfig(std::string& map_name, int &map_num, int &iteration);
  }
}

#endif //DATAGEN_DATA_RECORDER_H
