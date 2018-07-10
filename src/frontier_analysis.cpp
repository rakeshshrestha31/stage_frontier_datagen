//
// Created by rakesh on 14/05/18.
//

#define MAX_TIME_OFFSET_ESTIMATED_GROUNDTRUTH 0.11
#define TRANSFORM_TOLERANCE 0.11

#include <numeric>

#include <stage_frontier_datagen/frontier_analysis.h>
#include <stage_frontier_datagen/utils.h>

#include <hector_exploration_planner/custom_costmap_2d_ros.h>
#include <hector_exploration_planner/hector_exploration_planner.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <stack>
#include <map>
#include <numeric>
#include <cmath>

#include <ros/ros.h>
#include <costmap_2d/static_layer.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace stage_frontier_datagen
{
namespace frontier_analysis
{
cv::Mat getRawMap(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros)
{
  cv::Mat map;
  boost::shared_ptr<costmap_2d::Costmap2D> static_layer;
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_2d_ros->getCostmap()->getMutex()));
  for (const auto &layer: *(costmap_2d_ros->getLayeredCostmap()->getPlugins()))
  {
    std::string layer_name = layer->getName();
    if (layer_name.find("ground_truth") != std::string::npos)  // find ground truth layer
    {
      static_layer = boost::dynamic_pointer_cast<costmap_2d::Costmap2D>(layer);
      break;
    }
  }

  // get raw map and flip it to cv::Mat, because the costmap origin starts from bottom left,
  // while opencv matrix origin start at top left
  if(static_layer)
  {
    auto raw_costmap = costmap_2d_ros->getCostmap()->getCharMap(); //static_layer->getCharMap();
    cv::Mat raw_costmap_img(static_layer->getSizeInCellsY(), static_layer->getSizeInCellsX(), CV_8UC1,
                            (void *) raw_costmap);

    cv::flip(raw_costmap_img, map, 0);
  }
  return map;
}

cv::Mat splitRawMap(const cv::Mat &rawMap)
{
  // split map into three channels: free, obstacle, unknown
  cv::Mat unknown, free, obstacle;
  thresholdCostmap(rawMap, unknown, free, obstacle);

  // merge three channels into one map
  std::vector<cv::Mat> channels(3);
  channels[0] = unknown;
  channels[1] = free;
  channels[2] = obstacle;
  cv::Mat rgb_image;
  cv::merge(channels, rgb_image);

  return rgb_image;
}

cv::Mat getMap(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros)
{
  cv::Mat map = getRawMap(costmap_2d_ros);
  return splitRawMap(map);
}

void getFrontierPoints(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros,
    boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> planner,
    std::vector<std::vector<cv::Point>>& all_clusters_cv)
{
  std::vector<std::vector<geometry_msgs::PoseStamped>> all_clusters = planner->getClusteredFrontierPoints();
  for(auto single_cluster: all_clusters)
  {
    std::vector<cv::Point> single_cluster_cv =
        frontier_analysis::worldPointsToMapPoints(single_cluster, costmap_2d_ros);
    all_clusters_cv.push_back(single_cluster_cv);
  }
}

void resizeFrontierPoints(std::vector<std::vector<cv::Point>> &inputPoints,
                          std::vector<std::vector<cv::Point>> &outputPoints,double ratio)
{
  for(auto single_cluster: inputPoints)
  {
    std::vector<cv::Point> single_cluster_cv;
    for(auto single_point: single_cluster)
    {
      cv::Point point(int(single_point.x * ratio), int(single_point.y * ratio));
      single_cluster_cv.push_back(point);
    }
    outputPoints.push_back(single_cluster_cv);
  }
}

void convertToGroundTruthSize(std::vector<std::vector<cv::Point>> &inputPoints,
                              std::vector<std::vector<cv::Point>> &outputPoints,
                              cv::Size orgSize, cv::Size groundtruth_size)
{
  int diff_height_half = (int)std::floor((groundtruth_size.height - orgSize.height) / 2.0);
  int diff_width_half = (int)std::floor((groundtruth_size.width - orgSize.width) / 2.0);

  for(auto single_cluster: inputPoints)
  {
    std::vector<cv::Point> single_cluster_cv;
    for(auto single_point: single_cluster)
    {
      cv::Point point(single_point.x + diff_width_half, single_point.y + diff_height_half);
      single_cluster_cv.push_back(point);
    }
    outputPoints.push_back(single_cluster_cv);
  }
}

std::vector<cv::Rect> generateBoundingBox(std::vector<std::vector<cv::Point>> &inputPoints)
{
  std::vector<cv::Rect> outputBoundingBox;
  for(auto single_cluster: inputPoints)
  {
    if(!single_cluster.empty())
    {
      cv::Rect bounding_box = cv::boundingRect(single_cluster);
      outputBoundingBox.push_back(bounding_box);
    }
  }
  return outputBoundingBox;
}

cv::Mat generateBoundingBoxImage(std::vector<cv::Rect> &inputRects, cv::Size size)
{
  cv::Mat img = cv::Mat::zeros(size.height, size.width, CV_8UC1);
  for(cv::Rect rect : inputRects)
  {
    img(rect).setTo(cv::Scalar(255));
  }
  return img;
}

cv::Mat generateVerifyImage(cv::Mat costmap, std::vector<cv::Rect> &boundingBoxes,
                         std::vector<std::vector<cv::Point>> cluster_frontiers)
{
  int height = costmap.size().height, width = costmap.size().width;
  // generate frontiers map
  cv::Mat frontier_points(height, width, CV_8UC1, cv::Scalar(0));
  for(std::vector<cv::Point> cluster: cluster_frontiers)
  {
    for(cv::Point frontier: cluster)
    {
      frontier_points.at<unsigned char>(frontier) = 255;
    }
  }

  // generate boundingBox map
  cv::Mat boundingBoxImage(height, width, CV_8UC1, cv::Scalar(0));
  for(auto rect: boundingBoxes)
  {
    cv::rectangle(boundingBoxImage, rect, cv::Scalar(255));
  }

  cv::Mat channels[3];
  cv::split(costmap, channels);

  std::vector<cv::Mat> new_channels(3);
  new_channels[0] = frontier_points;  // Frontier Points
  new_channels[1] = channels[1];      // Free Space
  new_channels[2] = boundingBoxImage; // Bounding Box Image

  cv::Mat rgb_image;
  cv::merge(new_channels, rgb_image);

  return rgb_image;
}


cv::Mat getMap(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros, double desired_resolution)
{
  cv::Mat map;
  boost::shared_ptr<costmap_2d::Costmap2D> static_layer;
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_2d_ros->getCostmap()->getMutex()));
    for (const auto &layer: *(costmap_2d_ros->getLayeredCostmap()->getPlugins()))
    {
      std::string layer_name = layer->getName();
      if (layer_name.find("ground_truth") != std::string::npos)
      {
        static_layer = boost::dynamic_pointer_cast<costmap_2d::Costmap2D>(layer);
        break;
      }
    }

    if (static_layer)
    {
//      auto raw_costmap = static_layer->getCharMap();
      auto raw_costmap = costmap_2d_ros->getCostmap()->getCharMap(); // static_layer->getCharMap();

      cv::Mat raw_costmap_img(static_layer->getSizeInCellsY(), static_layer->getSizeInCellsX(), CV_8UC1,
                              (void *) raw_costmap);

      // the costmap origin starts from bottom left, while opencv matrix origin start at top left
      cv::flip(raw_costmap_img, map, 0);
    }
  }

  if (static_layer)
  {
    auto costmap_resolution = static_layer->getResolution();
    if (!map.empty() && std::abs(desired_resolution - costmap_resolution) > 1e-2)
    {
      cv::Mat resized_map = resizeToDesiredResolution(map, costmap_2d_ros, desired_resolution);
      map = resized_map;
    }

    cv::Mat unknown, free, obstacle;
    thresholdCostmap(map, unknown, free, obstacle);
    map = free;
  }
  return map;
}

void thresholdCostmap(const cv::Mat &original, cv::Mat &unknown, cv::Mat &free, cv::Mat &obstacle)
{
  double thres_small = 10, thres_large =  254;
  // get unknown space
  cv::threshold(original, unknown, thres_large, 255, cv::THRESH_BINARY);
  // get free space
  cv::threshold(original, free, thres_small, 255, cv::THRESH_BINARY_INV);

  // reserve both obstacle and unknown with original values, set free to 0
  cv::Mat obstacle_unknown;
  cv::threshold(original, obstacle_unknown, thres_small, 255, cv::THRESH_BINARY);
  // remove unknown value by minus them
  cv::Mat obstacle_tmp = obstacle_unknown - unknown;
  obstacle_tmp.copyTo(obstacle);
}

cv::Mat thresholdCostmap(const cv::Mat &original_map)
{
  cv::Mat thresholded_map(original_map.size(), original_map.type(), cv::Scalar(0));
  cv::threshold(original_map, thresholded_map, 10, 255, cv::THRESH_BINARY_INV);
  return thresholded_map;
}

cv::Mat getBoundingBoxImage(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros,
                            const std::vector< std::vector<geometry_msgs::PoseStamped> > clustered_frontier_poses)
{
  costmap_2d::Costmap2D* costmap;
  int size_x;
  int size_y;
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_2d_ros->getCostmap()->getMutex()));
    costmap = costmap_2d_ros->getCostmap();

    size_x = costmap->getSizeInCellsX();
    size_y = costmap->getSizeInCellsY();
  }

  cv::Mat frontier_bounding_box_image(
    size_y, size_x, CV_8UC1, cv::Scalar(0)
  );

  for (const auto &frontier_world_points: clustered_frontier_poses)
  {
    auto frontier_map_points = frontier_analysis::worldPointsToMapPoints(frontier_world_points, costmap_2d_ros);
    if (!frontier_map_points.empty())
    {
      cv::Rect bounding_box_costmap = cv::boundingRect(frontier_map_points);
      frontier_bounding_box_image(bounding_box_costmap) = cv::Scalar(255);
    }
  }

  return frontier_bounding_box_image;
}

cv::Mat convertToGroundtruthSize(const cv::Mat &original_map, const cv::Size groundtruth_size)
{
  cv::Mat resized_clipped_map;

  auto diff_size_rows = original_map.rows - groundtruth_size.height;
  auto diff_size_cols = original_map.cols - groundtruth_size.width;


  if (diff_size_rows > 0)
  {
    cv::Range row_range((int)std::floor(diff_size_rows/2.0), original_map.rows - (int)std::ceil(diff_size_rows/2.0));
    resized_clipped_map = original_map.rowRange(row_range).clone();
  }
  else if (diff_size_rows < 0)
  {
    auto border_top = (int)std::floor(-diff_size_rows/2.0);
    auto border_bottom = (int)std::ceil(-diff_size_rows/2.0);
    cv::copyMakeBorder(original_map, resized_clipped_map, border_top, border_bottom, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
  }
  else
  {
    resized_clipped_map = original_map.clone();
  }

  if (diff_size_cols > 0)
  {
    cv::Range col_range((int)std::floor(diff_size_cols/2.0), original_map.cols - (int)std::ceil(diff_size_cols/2.0));
    resized_clipped_map = resized_clipped_map.colRange(col_range).clone();
  }
  else if (diff_size_cols < 0)
  {
    auto border_left = (int)std::floor(-diff_size_cols/2.0);
    auto border_right = (int)std::ceil(-diff_size_cols/2.0);
    cv::Mat tmp_img = resized_clipped_map.clone();
    cv::copyMakeBorder(tmp_img, resized_clipped_map, 0, 0, border_left, border_right, cv::BORDER_CONSTANT, cv::Scalar(0));
  }

  assert(resized_clipped_map.size() == groundtruth_size);

  return resized_clipped_map;
}

cv::Mat getMapCenteringAffineTransformation(const boost::shared_ptr<costmap_2d::Costmap2D> static_costmap)
{
  auto resolution = static_costmap->getResolution();

  auto costmap_origin_map_x = static_costmap->getOriginX() / resolution;
  auto costmap_origin_map_y = static_costmap->getOriginY() / resolution;

  auto groundtruth_origin_map_x = -static_costmap->getSizeInMetersX() / 2 / resolution;
  auto groundtruth_origin_map_y = -static_costmap->getSizeInMetersY() / 2 / resolution;

  auto translation_x = std::round(costmap_origin_map_x - groundtruth_origin_map_x);
  auto translation_y = std::round(costmap_origin_map_y - groundtruth_origin_map_y);

  // y is negated because y is pointing downwards in opencv coords but upwards in costmap coords
  return (cv::Mat_<double>(2, 3) <<   1.0, 0.0, translation_x,
                                      0.0, 1.0, -translation_y);
}

cv::Mat getMapGroundtruthAffineTransformation(const boost::shared_ptr<costmap_2d::Costmap2D> static_costmap,
                                              const tf::Transform &transform_gt_est)
{
  auto resolution = static_costmap->getResolution();

  auto translation_tf_vector = transform_gt_est.getOrigin();
  auto translation_x = translation_tf_vector.getX() / resolution;
  auto translation_y = translation_tf_vector.getY() / resolution;

  auto rotation_angle = tf::getYaw(transform_gt_est.getRotation());
  auto cos_rotation = std::cos(rotation_angle);
  auto sin_rotation = std::sin(rotation_angle);

  // y is negated because y is pointing downwards in opencv coords but upwards in costmap coords
  return (cv::Mat_<double>(2, 3) <<   cos_rotation, -sin_rotation, translation_x,
                                      sin_rotation, cos_rotation, -translation_y);
}

cv::Mat resizeToDesiredResolution(const cv::Mat &costmap_image,
                                  const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> costmap_2d_ros,
                                  double desired_resolution)
{
  auto costmap = costmap_2d_ros->getCostmap();
  assert(
    costmap_image.rows == costmap->getSizeInCellsY()
    && costmap_image.cols == costmap->getSizeInCellsX()
  );

  auto costmap_resolution = costmap->getResolution();
  auto resize_factor = costmap_resolution / desired_resolution;
  cv::Mat resized_image(costmap_image.size(), costmap_image.type(), cv::Scalar(0));
  cv::resize(costmap_image, resized_image, cv::Size(), resize_factor, resize_factor);

  return resized_image;
}

cv::Rect resizeToDesiredResolution(const cv::Rect &costmap_bounding_rect,
                                   const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros,
                                   double desired_resolution)
{
  auto costmap = costmap_2d_ros->getCostmap();
  assert(
    costmap_bounding_rect.height <= costmap->getSizeInCellsY()
    && costmap_bounding_rect.width <= costmap->getSizeInCellsX()
  );

  auto costmap_resolution = costmap->getResolution();
  auto resize_factor = costmap_resolution / desired_resolution;

  cv::Rect resized_bounding_rect(
    (int)((costmap_bounding_rect.x) * resize_factor),
    (int)((costmap_bounding_rect.y) * resize_factor),
    (int)(costmap_bounding_rect.width * resize_factor),
    (int)(costmap_bounding_rect.height * resize_factor)
  );
  return resized_bounding_rect;
}


std::vector<cv::Point> worldPointsToMapPoints(const std::vector<geometry_msgs::PoseStamped> &world_points,
                                              const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros)
{
  auto costmap = costmap_2d_ros->getCostmap();
  auto resolution = costmap->getResolution();

  auto cell_size_x = costmap->getSizeInCellsX();
  auto cell_size_y = costmap->getSizeInCellsY();

  std::vector<cv::Point> map_points;
  map_points.reserve(world_points.size());

  for (const auto &world_point: world_points)
  {
    unsigned int map_x;
    unsigned int map_y;

    tf::Vector3 estimated_position_vector(
      world_point.pose.position.x, world_point.pose.position.y, world_point.pose.position.z
    );
    auto groundtruth_position_vector = estimated_position_vector;
    map_x = static_cast<unsigned int>(groundtruth_position_vector.getX() / resolution + costmap->getSizeInCellsX() / 2);
    // y is negated because y is pointing downwards in opencv coords but upwards in costmap coords
    map_y = static_cast<unsigned int>(-groundtruth_position_vector.getY() / resolution + costmap->getSizeInCellsY() / 2);

//    costmap->worldToMap(groundtruth_position_vector.getX(), -groundtruth_position_vector.getY(), map_x, map_y);

    if
      (
      map_x >= 0
      && map_x < cell_size_x
      && map_y >= 0
      && map_y < cell_size_y
      )
    {
      map_points.emplace_back(
        map_x, map_y
      );
    }
  }

  return map_points;
}

void preprocessFrontierImg(cv::Mat &frontier_img_in, cv::Mat &frontier_img_out)
{
  cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                               cv::Size( 2*FRONTIER_OPENING_KERNEL + 1, 2*FRONTIER_OPENING_KERNEL+1 ),
                                               cv::Point( FRONTIER_OPENING_KERNEL, FRONTIER_OPENING_KERNEL ) );

  cv::Mat temp_img = frontier_img_in.clone();

  // closing (hole filling)
  cv::dilate(frontier_img_in, temp_img, element);
  cv::erode(temp_img, frontier_img_out, element);

// opening (cleaning)
//  element = cv::getStructuringElement( cv::MORPH_RECT,
//                                       cv::Size( 2*FRONTIER_HOLE_FILLING_KERNEL + 1, 2*FRONTIER_HOLE_FILLING_KERNEL+1 ),
//                                       cv::Point( FRONTIER_HOLE_FILLING_KERNEL, FRONTIER_HOLE_FILLING_KERNEL ) );
//  cv::dilate(frontier_img_out, temp_img, element);
//  cv::erode(temp_img, frontier_img_out, element);

}

std::vector< std::vector<cv::Point> > groupFrontiers(cv::Mat &frontier_img,
                                                     std::vector<cv::Point> clustered_frontier_points)
{
//  std::map<cv::Point, bool> already_grouped;
//
//  std::vector< std::vector<cv::Point> > grouped_frontiers;
//
//  for (const auto &frontier_point: clustered_frontier_points)
//  {
//    if (!frontier_img.at<uint8_t>(frontier_point))
//    {
//      std::cout << "CLUSTERED POINT NOT IN FRONTIER" << std::endl;
//      continue;
//    }
//    cv::Mat temp_img = frontier_img.clone();
//    cv::floodFill(temp_img, frontier_point, cv::Scalar(255), nullptr, cv::Scalar(0), cv::Scalar(0));
//    cv::Mat non_zero_coordinates;
//    cv::findNonZero(temp_img, non_zero_coordinates);
//    // TODO: check if they've already been grouped into some other group
//    std::vector<cv::Point> group;
//    for (size_t i = 0; i < non_zero_coordinates.total(); i++)
//    {
//      group.push_back(non_zero_coordinates.at<cv::Point>(i));
//    }
//    grouped_frontiers.push_back(group);
//  }
//
//  return grouped_frontiers;

  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(frontier_img, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  std::vector< std::vector<cv::Point> > filtered_contours;
  std::copy_if(
    contours.begin(), contours.end(),
    std::back_inserter(filtered_contours),
    [](std::vector<cv::Point> group) {
      return group.size() > MIN_FRONTIER_CLUSTER_SIZE;
    }
  );
  return filtered_contours;
}

std::vector<cv::Scalar> colorFrontiers(cv::Mat &frontier_img,
                                       std::vector< std::vector<cv::Point> > grouped_frontiers,
                                       cv::RNG &rng,
                                       cv::Mat &colored_frontier_img)
{
//  colored_frontier_img = cv::Mat(frontier_img.size(), CV_8UC3);
  std::vector<cv::Scalar> colors;
  colors.reserve(grouped_frontiers.size());

  for (size_t i = 0; i < grouped_frontiers.size(); i++)
  {
    cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::drawContours(colored_frontier_img, grouped_frontiers, i, color, 2);
    colors.push_back(color);
  }
  return colors;
}

void loadStageWorld(std::string world_file,
                    const costmap_2d::Costmap2D& costmap,
                    cv::Mat &output_img)
{
  std::string bitmap = "/opt/ros/indigo/share/stage/worlds/bitmaps/cave.png";
  cv::Size2f size(16, 16);

  cv::Mat resized_img;
  cv::Mat output_img_flipped;
  cv::Mat raw_img = cv::imread(bitmap, CV_LOAD_IMAGE_UNCHANGED);
  // fill borders
  raw_img.colRange(0, raw_img.cols).rowRange(0, 1) = cv::Scalar(0);
  raw_img.colRange(0, raw_img.cols).rowRange(raw_img.rows-1, raw_img.rows) = cv::Scalar(0);
  raw_img.rowRange(0, raw_img.rows).colRange(0, 1) = cv::Scalar(0);
  raw_img.rowRange(0, raw_img.rows).colRange(raw_img.cols-1, raw_img.cols) = cv::Scalar(0);

  auto resolution = costmap.getResolution();
  cv::Size output_size(
    static_cast<int>(std::round(size.width  / resolution)),
    static_cast<int>(std::round(size.height / resolution))
  );
  cv::resize(raw_img, resized_img, output_size);
  resized_img = 255 - resized_img;

  output_img_flipped = cv::Mat::zeros(cv::Size(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()), CV_8UC1);
  cv::Size costmap_center(std::round(costmap.getSizeInCellsX()/(float)2), std::round(costmap.getSizeInCellsY()/(float)2));
  cv::Size bitmap_center(std::round(resized_img.cols/(float)2), std::round(resized_img.rows/(float)2));

  // TODO: fix the funny offset business
  resized_img.copyTo(
    output_img_flipped
      .colRange(costmap_center.width - bitmap_center.width + 8, costmap_center.width + bitmap_center.width + 8)
      .rowRange(costmap_center.height - bitmap_center.height - 8, costmap_center.height + bitmap_center.height - 8)
  );

  cv::flip(output_img_flipped, output_img, 0);
}

std::vector<cv::Point> getClosestUnknowns(cv::Mat &occupancy_unknown_img,
                                          std::vector< std::vector<cv::Point> > grouped_frontiers)
{
  std::vector<cv::Point> closest_unknowns;
  closest_unknowns.reserve(grouped_frontiers.size());

  for (const auto &frontier_group: grouped_frontiers)
  {
    cv::Point closest_point(-1, -1);
    if (frontier_group.empty())
    {
      ROS_ERROR("Frontier group empty");
    }

    std::map<std::tuple<int, int>, bool> explored_points;
    std::stack<cv::Point> fringe;
    bool is_found = false;

    for (auto frontier_point: frontier_group)
    {
      fringe.push(frontier_point);
    }

    while (!fringe.empty())
    {
      auto point = fringe.top(); // front();
      fringe.pop();

      if (explored_points.find(pointToPair(point)) != explored_points.end()) continue;

      explored_points.insert(
        std::make_pair< std::tuple<int, int>, bool >(pointToPair(point), true)
      );

      for (int i = -1; i < 2; i++)
      {
        for (int j = -1; j < 2; j++)
        {
          cv::Point neighbour_point(point.x + i, point.y + j);

          if (
            explored_points.find(pointToPair(neighbour_point)) == explored_points.end()
            && neighbour_point.x >= 0
            && neighbour_point.x < occupancy_unknown_img.cols
            && neighbour_point.y >= 0
            && neighbour_point.y < occupancy_unknown_img.rows
            )
          {
            // TODO: softcode these numbers
            auto patch = occupancy_unknown_img.rowRange(std::max(0, neighbour_point.y - 3),
                                                        std::min(neighbour_point.y + 3, occupancy_unknown_img.rows - 1))
                                              .colRange(std::max(0, neighbour_point.x - 3),
                                                        std::min(neighbour_point.x + 3, occupancy_unknown_img.cols - 1));

            bool is_patch_zero = true;
            for (auto it = patch.begin<cv::Vec3b>(); it != patch.end<cv::Vec3b>(); it++)
            {
              if ((*it)[1] || (*it)[2])
              {
                is_patch_zero = false;
                break;
              }
            }

            auto color = occupancy_unknown_img.at<cv::Vec3b>(neighbour_point);
//            if (color[1] == 0 && color[2] == 0)
            if (is_patch_zero)
            {
              is_found = true;
              closest_point = neighbour_point;
              break;
            }
            // if not obstacle
            if (color[1] == 0 && color[2] == 0) {
              fringe.push(neighbour_point);
            }

            // debug
//            cv::Mat tmp_img = occupancy_unknown_img.clone();
//            cv::circle(tmp_img, neighbour_point, 3, cv::Scalar(255, 0, 0), -1);
//
//            auto patch_tmp = tmp_img.rowRange(std::max(0, neighbour_point.y - 3),
//                                           std::min(neighbour_point.y + 3, occupancy_unknown_img.rows - 1))
//              .colRange(std::max(0, neighbour_point.x - 3),
//                        std::min(neighbour_point.x + 3, occupancy_unknown_img.cols - 1)) = cv::Scalar(0, 0, 255);
//            patch_tmp = cv::Scalar(0, 0, 255);
//
//            cv::imwrite("/tmp/save_img.jpg", tmp_img);
          }
        }
        if (is_found) break;
      }
      if (is_found) break;
    }

    closest_unknowns.push_back(closest_point);
    if (!is_found)
    {
      ROS_ERROR("Could not find closest unknown!!!");
    }
  }

  return closest_unknowns;
}

std::vector< std::vector<cv::Point> > expandUnknowns(cv::Mat &occupancy_unknown_img,
                                                     std::vector<cv::Point> unknown_points)
{
  std::vector< std::vector<cv::Point> > unknowns_grouped;
  unknowns_grouped.reserve(unknown_points.size());

  for (const auto &unknown_point: unknown_points)
  {
    std::vector<cv::Point> unknown_group;
    if (unknown_point.x >= 0 && unknown_point.y >= 0) {
      std::map<std::tuple<int, int>, bool> explored_points;
      std::queue<cv::Point> queue;

      queue.push(unknown_point);
      unknown_group.push_back(unknown_point);

      while (!queue.empty()) {
        auto point = queue.front();
        queue.pop();

        if (explored_points.find(pointToPair(point)) != explored_points.end()) continue;
        explored_points.insert(
          std::make_pair<std::tuple<int, int>, bool>(pointToPair(point), true)
        );

        for (int i = -1; i < 2; i++) {
          for (int j = -1; j < 2; j++) {
            cv::Point neighbour_point(point.x + i, point.y + j);

            if (
              explored_points.find(pointToPair(neighbour_point)) == explored_points.end()
              && neighbour_point.x >= 0
              && neighbour_point.x < occupancy_unknown_img.cols
              && neighbour_point.y >= 0
              && neighbour_point.y < occupancy_unknown_img.rows
              ) {
              auto color = occupancy_unknown_img.at<cv::Vec3b>(neighbour_point);
              if (color[1] == 0 && color[2] == 0) {
                unknown_group.push_back(neighbour_point);
                queue.push(neighbour_point);
              }
            }
          }
        }
      }
    }
    unknowns_grouped.push_back(unknown_group);
  }
  return unknowns_grouped;
}

std::pair<int, int> pointToPair(cv::Point point)
{
  return std::make_pair(point.x, point.y);
}

} // namespace frontier_analysis
} // namespace hector_exploration_planner
