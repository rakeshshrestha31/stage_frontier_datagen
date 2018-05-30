//
// Created by rakesh on 09/05/18.
//

#ifndef HECTOR_NAVIGATION_FRONTIER_VIS_H
#define HECTOR_NAVIGATION_FRONTIER_VIS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <opencv2/core/core.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_array.hpp>

namespace hector_exploration_planner
{
class FrontierVis
{
public:
  FrontierVis(const std::string &topic_name);
  void publishVisOnDemand(cv::Mat frontier_img,
                          const std::vector<cv::Point> &clustered_frontiers,
                          cv::Mat exploration_transform,
                          const costmap_2d::Costmap2D& costmap,
                          const costmap_2d::Costmap2DROS& costmap_ros);
  static void drawPose(cv::Mat img, cv::Point, double yaw,
                       cv::Scalar point_color, cv::Scalar normal_color);
  static void drawPoint(cv::Mat &img, cv::Point point, cv::Scalar color);



protected:
  boost::mutex mutex_;

  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher image_pub_;

}; // class FrontierVis
} // namespace hector_exploration_planner

#endif //HECTOR_NAVIGATION_FRONTIER_VIS_H
