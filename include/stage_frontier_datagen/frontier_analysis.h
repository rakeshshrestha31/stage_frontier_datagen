//
// Created by rakesh on 14/05/18.
//

#ifndef HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H
#define HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H

// preprocessing operations on frontiers
#define FRONTIER_HOLE_FILLING_KERNEL 1
#define FRONTIER_OPENING_KERNEL 1

#define MIN_FRONTIER_CLUSTER_SIZE 20

#include <costmap_2d/costmap_2d.h>
#include <opencv2/core/core.hpp>

#include <vector>

namespace hector_exploration_planner
{
namespace frontier_analysis
{

/**
 * @brief preprocessing operations (closing (hole filling) and opening)
 * @param frontier_img_in input image
 * @param frontier_img_out output image
 * @return
 */
void preprocessFrontierImg(cv::Mat &frontier_img_in, cv::Mat &frontier_img_out);

/**
 *
 * @brief group frontier points based on the clusters they belong to
 * @param frontier_img input binary image of frontier points
 * @param clustered_frontier_points clusters of frontiers
 * @return grouped frontiers
 */
std::vector< std::vector<cv::Point> > groupFrontiers(cv::Mat &frontier_img,
                                                     std::vector<cv::Point> clustered_frontier_points);

/**
 *
 * @brief color the frontier image based on clusters
 * @param frontier_img input frontier binary image (single channeled)
 * @param grouped_frontiers coordinates of the frontiers grouped into clusters
 * @param rng random number generator
 * @param colored_frontier_img output image (three channeled)
 * @return vector of colors for each cluster
 */
std::vector<cv::Scalar> colorFrontiers(cv::Mat &frontier_img,
                                       std::vector< std::vector<cv::Point> > grouped_frontiers,
                                       cv::RNG &rng,
                                       cv::Mat &colored_frontier_img);

/**
 * @brief load stage world into occupancy map
 * @param world file of stage
 * @param costmap ros costmap_2d of the map being built
 * @param output_image occupancy map
 */
void loadStageWorld(std::string world_file,
                    const costmap_2d::Costmap2D& costmap,
                    cv::Mat &output_img);

/**
 *
 * @param occupancy_unknown_img image containing occupied pixels and unknown unoccupied images
 * @param grouped_frontiers frontier points grouped into clusters
 * @return for each frontier group, get unknown points closest to it
 */
std::vector<cv::Point> getClosestUnknowns(cv::Mat &occupancy_unknown_img,
                                          std::vector< std::vector<cv::Point> > grouped_frontiers);


/**
 *
 * @param occupancy_unknown_img image containing occupied pixels and unknown unoccupied images
 * @param unknown_points unknown point from each frontier cluster
 * @return floodfills each frontier cluster
 */
std::vector< std::vector<cv::Point> > expandUnknowns(cv::Mat &occupancy_unknown_img,
                                                     std::vector<cv::Point> unknown_points);

std::pair<int, int> pointToPair(cv::Point point);

} // namespace frontier_analysis
} // namespace hector_exploration_planner

#endif //HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H
