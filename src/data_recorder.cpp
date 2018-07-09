//
// Created by Fei-Peng Tian on 07/07/18.
//
#include "stage_frontier_datagen/data_recorder.h"
#include <fstream>
#include <boost/filesystem.hpp>

namespace fs=boost::filesystem;

namespace stage_frontier_datagen {
  namespace data_recorder {

#define CONFIG_FILENAME "config.yaml"

    YAML::Node Pose2YAML(geometry_msgs::PoseStamped pose) {
      YAML::Node position;
      position.push_back(pose.pose.position.x);
      position.push_back(pose.pose.position.y);
      position.push_back(pose.pose.position.z);

      YAML::Node orientation;
      orientation.push_back(pose.pose.orientation.x);
      orientation.push_back(pose.pose.orientation.y);
      orientation.push_back(pose.pose.orientation.z);
      orientation.push_back(pose.pose.orientation.w);

      YAML::Node node;
      node["position"] = position;
      node["orientation"] = orientation;
      return node;
    }

    YAML::Node Point2YAML(cv::Point point)
    {
      YAML::Node node;
      node["x"] = point.x;
      node["y"] = point.y;
      return node;
    }

    YAML::Node Rect2YAML(cv::Rect rect)
    {
      YAML::Node node;
      node["x"] = rect.x;
      node["y"] = rect.y;
      node["height"] = rect.height;
      node["width"] = rect.width;
      return node;
    }

    YAML::Node FrontierClusters2Yaml(std::vector <std::vector<geometry_msgs::PoseStamped>> cluster_frontiers) {
      YAML::Node all_clusters;
      for (auto cluster: cluster_frontiers) {
        YAML::Node a_cluster;
        for (auto frontier_pose: cluster) {
          YAML::Node point = Pose2YAML(frontier_pose);
          a_cluster.push_back(point);
        }
        all_clusters.push_back(a_cluster);
      }
      return all_clusters;
    }

    YAML::Node FrontierClusters2Yaml(std::vector<std::vector<cv::Point>> cluster_frontiers)
    {
      YAML::Node all_clusters;
      for (auto cluster: cluster_frontiers) {
        YAML::Node a_cluster;
        for (auto frontier_pose: cluster) {
          YAML::Node point = Point2YAML(frontier_pose);
          a_cluster.push_back(point);
        }
        all_clusters.push_back(a_cluster);
      }
      return all_clusters;
    }

    YAML::Node Rects2YAML(std::vector<cv::Rect> rects)
    {
      YAML::Node rect_nodes;
      for (auto rect: rects) {
        YAML::Node rect_node = Rect2YAML(rect);
        rect_nodes.push_back(rect_node);
      }
      return rect_nodes;
    }

    fs::path constructPath(std::string base_dir, std::string map_name, int iteration)
    {
      fs::path base_path(base_dir);
      fs::path map_path = base_path / map_name;
      fs::path iter_path = map_path / std::to_string(iteration);

      return iter_path;
    }

    void recordImage(std::string base_dir, std::string map_name, int iteration, int planning_num,
                     const cv::Mat &image, std::string record__base_name)
    {
      std::string ext = ".png";
      fs::path record_path = constructPath(base_dir, map_name, iteration);
      if(!fs::exists(record_path))
        fs::create_directories(record_path);

      fs::path image_path = record_path / (record__base_name + std::to_string(planning_num) + ext);
      cv::imwrite(image_path.string(), image);
    }

    void recordImage(std::string base_dir, std::string map_name, int iteration, int planning_num,
                     const cv::Mat &costmap, const cv::Mat &boundingBoxImg)
    {
      recordImage(base_dir, map_name, iteration, planning_num, costmap, "costmap");
      recordImage(base_dir, map_name, iteration, planning_num, boundingBoxImg, "boundingBox");
    }

    void recordVerifyImage(std::string base_dir, std::string map_name, int iteration, int planning_num,
                           cv::Mat verifyImg)
    {
      recordImage(base_dir, map_name, iteration, planning_num, verifyImg, "verify");
    }

    void recordInfo(std::string base_dir, std::string map_name, int iteration, int planning_num,
                    const std::vector<std::vector<cv::Point>> &cluster_frontiers,
                    const std::vector<cv::Rect> &frontierBoundingBox)
    {
      fs::path record_path = constructPath(base_dir, map_name, iteration);
      if(!fs::exists(record_path))
        fs::create_directories(record_path);

      fs::path info_path = record_path / ("info" + std::to_string(planning_num) + ".yaml");

      YAML::Node frontiers =  FrontierClusters2Yaml(cluster_frontiers);
      YAML::Node bb_nodes = Rects2YAML(frontierBoundingBox);

      YAML::Node root;
      root["Frontiers"] = frontiers;
      root["BoundingBoxes"] = bb_nodes;
      std::ofstream fout(info_path.string());
      fout << root;
      fout.close();
    }

    void recordGTMapAndResolution(std::string base_dir, std::string map_name, cv::Mat groundTruth, double resolution)
    {
      fs::path base_path(base_dir);
      fs::path map_path(map_name);
      fs::path filename("GT.bmp");
      fs::path middle_path = base_path / map_path;
      if(!fs::exists(middle_path))
        fs::create_directories(middle_path);

      fs::path image_path = middle_path / filename;

      cv::imwrite(image_path.string(), groundTruth);

      fs::path info_name("GT.yaml");
      fs::path info_path = base_path/ map_path / info_name;
      YAML::Node node;
      node["resolution"] = resolution;
      std::ofstream fout(info_path.string());
      fout << node;
      fout.close();
    }

    void test(std::vector<std::vector<geometry_msgs::PoseStamped>> cluster_frontier)
    {
      std::string filename = "/tmp/frontiers.yaml";
      YAML::Node node = FrontierClusters2Yaml(cluster_frontier);
      std::ofstream fout(filename);
      fout << node;
    }

    void writeConfig(std::string map_name, int map_num, int iteration)
    {
      YAML::Node config;
      config["map_name"] = map_name;
      config["map_num"] = map_num;
      config["iteration"] = iteration;
      std::ofstream fout(CONFIG_FILENAME);
      fout << config;
    }

    bool readConfig(std::string& map_name, int &map_num, int &iteration)
    {
      if(!fs::exists(CONFIG_FILENAME))
        return false;

      YAML::Node config = YAML::LoadFile(CONFIG_FILENAME);
      map_name = config["map_name"].as<std::string>();
      map_num = config["map_num"].as<int>();
      iteration = config["iteration"].as<int>();
      return true;
    }
  }
}