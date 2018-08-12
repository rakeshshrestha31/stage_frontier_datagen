//
// Created by Fei-Peng Tian on 07/07/18.
//
#include "stage_frontier_datagen/data_recorder.h"
#include <fstream>
#include <boost/filesystem.hpp>

namespace fs=boost::filesystem;

namespace stage_frontier_datagen {
  namespace data_recorder {

#define CONFIG_FILENAME "config.json"

    Json::Value Point2Json(const cv::Point &point)
    {
      Json::Value node;
      node["x"] = point.x;
      node["y"] = point.y;
      return node;
    }

    Json::Value Rect2Json(const cv::Rect &rect)
    {
      Json::Value node;
      node["x"] = rect.x;
      node["y"] = rect.y;
      node["height"] = rect.height;
      node["width"] = rect.width;
      return node;
    }

    Json::Value Pose2Json(const Pose2D &pose)
    {
      Json::Value node;
      node["x"] = pose.position.x;
      node["y"] = pose.position.y;
      node["yaw"] = pose.orientation;
      return node;
    }

    Json::Value Poses2Json(const std::vector<Pose2D> &poses)
    {
      Json::Value poses_json;
      for (const auto &pose: poses) {
        Json::Value pose_json = Pose2Json(pose);
        poses_json.append(pose_json);
      }
      return poses_json;
    }

    Json::Value Poses2Json(const std::vector<std::vector<Pose2D>> &poses_sets)
    {
      Json::Value poses_sets_json;
      for (const auto &poses: poses_sets) {
        Json::Value poses_json = Poses2Json(poses);
        poses_sets_json.append(poses_json);
      }
      return poses_sets_json;
    }

    Json::Value Rects2Json(const std::vector<cv::Rect> &rects)
    {
      Json::Value rect_jsons;
      for (const auto &rect: rects) {
        Json::Value point = Rect2Json(rect);
        rect_jsons.append(point);
      }
      return rect_jsons;
    }

    Json::Value nums2Json(const std::vector<double> &nums)
    {
      Json::Value jsons;
      for(const auto& num: nums)
      {
        jsons.append(num);
      }
      return jsons;
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
                    const std::vector<std::vector<Pose2D>> &frontier_clusters,
                    const std::vector<Pose2D> &frontier_cluster_centers,
                    const std::vector<cv::Rect> &frontierBoundingBox,
                    const Pose2D& robotPose,
                    const std::vector<Pose2D> &plan_poses,
                    const std::vector<double> &plan_ms_times,
                    const std::vector<double> &plan_explored_areas,
                    const std::vector<double> & simulation_times,
                    double planner_time)
    {
      fs::path record_path = constructPath(base_dir, map_name, iteration);
      if(!fs::exists(record_path))
        fs::create_directories(record_path);

      fs::path info_path = record_path / ("info" + std::to_string(planning_num) + ".json");

      Json::Value frontiers =  Poses2Json(frontier_clusters);
      Json::Value centers = Poses2Json(frontier_cluster_centers);
      Json::Value bb_nodes = Rects2Json(frontierBoundingBox);
      Json::Value robot_pose_node = Pose2Json(robotPose);

      Json::Value root;
      root["Frontiers"] = frontiers;
      root["Frontier_centers"] = centers;
      root["BoundingBoxes"] = bb_nodes;
      root["RobotPose"] = robot_pose_node;
      root["RobotPosesInPlan"] = Poses2Json(plan_poses);
      root["TimesInPlan"] = nums2Json(plan_ms_times);
      root["AreasExploredInPlan"] = nums2Json(plan_explored_areas);
      root["SimulationTime"] = nums2Json(simulation_times);
      root["PlannerTime"] = planner_time;

      Json::StyledWriter writer;
      std::string json_str = writer.write(root);
      std::ofstream fout(info_path.string());
      fout << json_str;
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

      fs::path info_name("GT.json");
      fs::path info_path = base_path/ map_path / info_name;
      Json::Value node;
      node["resolution"] = resolution;

      Json::StyledWriter writer;
      std::string json_str = writer.write(node);
      std::ofstream fout(info_path.string());
      fout << json_str;
      fout.close();
    }

    void writeConfig(std::string map_name, int map_num, int iteration)
    {
      Json::Value config;
      config["map_name"] = map_name;
      config["map_num"] = map_num;
      config["iteration"] = iteration;

      Json::StyledWriter writer;
      std::string json_str = writer.write(config);
      std::ofstream fout(CONFIG_FILENAME);
      fout << json_str;
      fout.close();
    }

    bool readConfig(std::string& map_name, int &map_num, int &iteration)
    {
      if(!fs::exists(CONFIG_FILENAME))
        return false;

      Json::Value config;
      std::ifstream file(CONFIG_FILENAME);
      file >> config;

      map_name = config["map_name"].asString();
      map_num = config["map_num"].asInt();
      iteration = config["iteration"].asInt();
      return true;
    }
  }
}