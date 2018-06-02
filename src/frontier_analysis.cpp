//
// Created by rakesh on 14/05/18.
//

#include <stage_frontier_datagen/frontier_analysis.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stack>
#include <map>
#include <numeric>

#include <ros/ros.h>
#include <costmap_2d/static_layer.h>

namespace stage_frontier_datagen
{
namespace frontier_analysis
{

cv::Mat getMap(const boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_2d_ros, double desired_resolution)
{
  cv::Mat map;
  boost::shared_ptr<costmap_2d::StaticLayer> static_layer;
  for (const auto &layer: *(costmap_2d_ros->getLayeredCostmap()->getPlugins()))
  {
    std::string layer_name = layer->getName();
    if (layer_name.find("static_layer") != std::string::npos)
    {
      static_layer = boost::dynamic_pointer_cast<costmap_2d::StaticLayer>(layer);
      break;
    }
  }

  if (static_layer)
  {
    auto raw_costmap = static_layer->getCharMap();
    cv::Mat raw_costmap_img(static_layer->getSizeInCellsY(), static_layer->getSizeInCellsX(), CV_8UC1, (void*)raw_costmap);

    // the costmap origin starts from bottom left, while opencv matrix origin start at top left
    cv::flip(raw_costmap_img, map, 0);

    // only intensity 0 to 100 are valid
//    cv::Mat validity_mask;
//    cv::Mat tmp_img;
//    cv::inRange(map, cv::Scalar(70), cv::Scalar(100), validity_mask);
    // masking doesn't work
//    map.copyTo(tmp_img, validity_mask);
//    map = tmp_img;
//    map = validity_mask;
  }

  if (!map.empty())
  {
    cv::Mat resized_map = resizeToDesiredResolution(map, costmap_2d_ros, desired_resolution);
    map = resized_map;
  }

  return map;
}

cv::Mat resizeToDesiredResolution(const cv::Mat &costmap_image,
                                  const boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_2d_ros,
                                  double desired_resolution)
{
  auto costmap = costmap_2d_ros->getCostmap();
  assert(
    costmap_image.rows == costmap->getSizeInCellsY()
    && costmap_image.cols == costmap->getSizeInCellsX()
  );

  auto costmap_resolution = costmap->getResolution();
  auto resize_factor = costmap_resolution / desired_resolution;
  cv::Mat resized_image;
  cv::resize(costmap_image, resized_image, cv::Size(), resize_factor, resize_factor);

  return resized_image;
}

cv::Rect resizeToDesiredResolution(const cv::Rect &costmap_bounding_rect,
                                   const boost::shared_ptr<costmap_2d::Costmap2DROS> &costmap_2d_ros,
                                   double desired_resolution)
{
  auto costmap = costmap_2d_ros->getCostmap();
  assert(
    costmap_bounding_rect.height <= costmap->getSizeInCellsY()
    && costmap_bounding_rect.width <= costmap->getSizeInCellsX()
  );

  auto costmap_resolution = costmap->getResolution();
  auto resize_factor = costmap_resolution / desired_resolution;

  unsigned int costmap_half_width = costmap->getSizeInCellsX() / 2;
  unsigned int costmap_half_height = costmap->getSizeInCellsY() / 2;
  cv::Rect resized_bounding_rect(
    (int)((costmap_bounding_rect.x - costmap_half_width) * resize_factor),
    (int)((costmap_bounding_rect.y - costmap_half_height) * resize_factor),
    (int)(costmap_bounding_rect.width * resize_factor),
    (int)(costmap_bounding_rect.height * resize_factor)
  );
  return resized_bounding_rect;
}

std::vector<cv::Point> worldPointsToMapPoints(const std::vector<geometry_msgs::PoseStamped> &world_points,
                                              const boost::shared_ptr<costmap_2d::Costmap2DROS> &costmap_2d_ros)
{
  auto costmap = costmap_2d_ros->getCostmap();

  std::vector<cv::Point> map_points;
  map_points.reserve(world_points.size());

  for (const auto &world_point: world_points)
  {
    unsigned int map_x;
    unsigned int map_y;
    // y is negated because y is pointing downwards in opencv coords but upwards in costmap coords
    costmap->worldToMap(world_point.pose.position.x, -world_point.pose.position.y, map_x, map_y);
    map_points.emplace_back(cv::Point(
      map_x, map_y
    ));
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
