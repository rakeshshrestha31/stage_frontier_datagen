//
// Created by rakesh on 09/05/18.
//
#define NORMAL_LENGTH 50

#include <hector_exploration_planner/frontier_vis.h>
#include <hector_exploration_planner/frontier_analysis.h>

#include <sensor_msgs/Image.h>
#include <tf/tf.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <ctime>

namespace hector_exploration_planner
{

FrontierVis::FrontierVis(const std::string &topic_name):
  nh_("~"),
  image_transport_(nh_),
  image_pub_(image_transport_.advertise(topic_name, 1))
{

}

void FrontierVis::drawPose(cv::Mat img, cv::Point point, double yaw,
                           cv::Scalar point_color, cv::Scalar normal_color)
{
  drawPoint(img, point, point_color);
  cv::Point normal_end(
    point.x + std::cos(yaw) * NORMAL_LENGTH,
    point.y + std::sin(yaw) * NORMAL_LENGTH
  );

  cv::line(img, point, normal_end, normal_color);
}

void FrontierVis::drawPoint(cv::Mat &img, cv::Point point, cv::Scalar color)
{
  cv::circle(img, point, 3, color, -1);
}

void FrontierVis::publishVisOnDemand(cv::Mat frontiers_img,
                                     const std::vector<cv::Point> &clustered_frontiers,
                                     cv::Mat exploration_transform,
                                     const costmap_2d::Costmap2D& costmap,
                                     const costmap_2d::Costmap2DROS& costmap_ros)
{
  ROS_DEBUG_NAMED("state", "STARTED PUBLISHVIS");
  boost::lock_guard<boost::mutex> guard(mutex_);
  static cv::RNG rng(std::time(nullptr));
  cv::Mat map(costmap.getSizeInCellsY(), costmap.getSizeInCellsX(), CV_8UC3, cv::Scalar(0, 0, 0));

  cv::Mat preprocessed_frontier_img;
  frontier_analysis::preprocessFrontierImg(frontiers_img, preprocessed_frontier_img);
  auto grouped_frontiers = frontier_analysis::groupFrontiers(preprocessed_frontier_img, clustered_frontiers);
  // ------------------ raw frontiers ------------------//
//  if (frontiers_img.size == map.size) {
//    cv::Mat channels[3];
//    cv::split(map, channels);
//    channels[2] = preprocessed_frontier_img; // frontiers_img;
////    channels[1] = frontiers_img;
//    cv::merge(channels, 3, map);
//  }
//
//  // ------------------ clustered frontiers ------------------//
//  for (const auto &frontier: clustered_frontiers) {
////    drawPose(map, frontier, tf::getYaw(frontier.pose.orientation), cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0));
//    drawPoint(map, frontier, cv::Scalar(255, 0, 0));
//  }
//
//  // ------------------ exploration transform ------------------//
//  if (exploration_transform.cols == map.cols && exploration_transform.rows == map.rows) {
//    cv::Mat channels[3];
//    cv::split(map, channels);
//    channels[1] = exploration_transform;
//    cv::merge(channels, 3, map);
//  }

  // ------------------ groundtruth ------------------//
  // TODO: don't hard code these
  // {
  //   cv::Mat groundtruth_occupancy_map;
  //   frontier_analysis::loadStageWorld(
  //     // std::string("/opt/ros/indigo/share/stage/worlds/bitmaps/cave.png"), cv::Size2f(16, 16),
  //     "/opt/ros/indigo/share/stage/worlds/simple.world",
  //     costmap,
  //     groundtruth_occupancy_map
  //   );

  //   cv::Mat channels[3];
  //   cv::split(map, channels);
  //   channels[2] = groundtruth_occupancy_map;
  //   cv::merge(channels, 3, map);
  // }

  // ------------------ costmap ------------------//
  {
    auto raw_costmap = costmap.getCharMap();
    cv::Mat raw_costmap_img(costmap.getSizeInCellsY(), costmap.getSizeInCellsX(), CV_8UC1, (void*)raw_costmap);
    cv::Mat obstacle_costmap_img;
    cv::Mat free_costmap_img;

    cv::threshold(raw_costmap_img, obstacle_costmap_img, 170, 255, cv::THRESH_BINARY_INV);
//    cv::threshold(raw_costmap_img, free_costmap_img, 100, 255, cv::THRESH_BINARY_INV);
    cv::Mat costmap_certain_img = obstacle_costmap_img; // + (255 - free_costmap_img);

    cv::Mat channels[3];
    cv::split(map, channels);
    channels[1] = costmap_certain_img;
    cv::merge(channels, 3, map);
  }

  // ------------------ frontiers ------------------//
  auto frontier_colors = frontier_analysis::colorFrontiers(
    frontiers_img,
    grouped_frontiers,
    rng,
    map
  );

  // ------------------ floodfill ------------------//
  auto closest_unknowns = frontier_analysis::getClosestUnknowns(map, grouped_frontiers);
  assert(grouped_frontiers.size() == closest_unknowns.size());

  auto floodfilled_frontiers = frontier_analysis::expandUnknowns(map, closest_unknowns);
  assert(floodfilled_frontiers.size() == frontier_colors.size());

  for (size_t i = 0, len = closest_unknowns.size(); i < len; i++)
  {
    auto closest_unknown = closest_unknowns[i];
    if (closest_unknown.x >= 0 && closest_unknown.y >= 0)
    {
      drawPoint(map, closest_unknown, frontier_colors[i]);
    }
  }

  for (size_t i = 0, len = floodfilled_frontiers.size(); i < len; i++)
  {
    auto flooded_points = floodfilled_frontiers[i];
    for (const auto &point: flooded_points)
    {
//      map.at<cv::Vec3b>(point) = cv::Vec3b(frontier_colors[i][0], frontier_colors[i][1], frontier_colors[i][2]);
    }
  }


  // ------------------ robot pose ------------------//
  tf::Stamped<tf::Pose> robot_pose;
  costmap_ros.getRobotPose(robot_pose);
  unsigned int robot_map_x, robot_map_y;
  auto robot_position = robot_pose.getOrigin();
  costmap.worldToMap(robot_position.x(), robot_position.y(), robot_map_x, robot_map_y);
  drawPose(map, cv::Point(robot_map_x, robot_map_y), tf::getYaw(robot_pose.getRotation()), cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255));


  // flip vertically cuz the positive y in image is going down
  cv::Mat map_flipped;
  cv::flip(map, map_flipped, 0);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map_flipped).toImageMsg();
  image_pub_.publish(msg);

  ROS_DEBUG_NAMED("state", "ENDED PUBLISHVIS");
}
} // namespace hector_exploration_planner