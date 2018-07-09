//
// Created by Fei-Peng Tian on 07/07/18.
//

#ifndef DATAGEN_DATA_RECORDER_H
#define DATAGEN_DATA_RECORDER_H

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

namespace stage_frontier_datagen {
  namespace data_recorder {
    /**
     * @brief convert a PoseStamped pose to YAML::Node
     * @param pose PoseStamped pose
     * @return YAML node
     */
    YAML::Node Pose2YAML(geometry_msgs::PoseStamped pose);

    /**
     * @brief convert a 2D cv::Point to YAML::Node
     * @param point cv::Point
     * @return YAML node
     */
    YAML::Node Point2YAML(cv::Point point);

    /**
     * convert all frontiers cluster to a YAML::node
     * @param cluster_frontiers frontier clusters, each frontier is represented by a PoseStamped pose
     * @return YAML node
     */
    YAML::Node FrontierClusters2Yaml(std::vector<std::vector<geometry_msgs::PoseStamped>> cluster_frontiers);

    /**
     * convert all frontiers cluster to a YAML::node
     * @param cluster_frontiers frontier clusters, each frontier is represented by a cv::Point
     * @return YAML node
     */
    YAML::Node FrontierClusters2Yaml(std::vector<std::vector<cv::Point>> cluster_frontiers);

    /**
     * Record Ground Truth image and related resolution into directory base_dir/map_name
     * @param base_dir base directory for recording
     * @param map_name floorplan map name (better without extension)
     * @param groundTruth ground Truth images
     * @param resolution the resolution of the ground images, meters/pixel
     */
    void recordGTMapAndResolution(std::string base_dir, std::string map_name, cv::Mat groundTruth, double resolution);

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
     */
    void recordInfo(std::string base_dir, std::string map_name, int iteration, int planning_num,
                    const std::vector<std::vector<cv::Point>> &cluster_frontier,
                    const std::vector<cv::Rect> &frontierBoundingBox);
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
