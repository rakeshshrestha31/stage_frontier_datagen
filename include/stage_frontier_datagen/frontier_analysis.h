//
// Created by rakesh on 14/05/18.
//

#ifndef STAGE_FRONTIER_DATAGEN_FRONTIER_ANALYSIS_H
#define STAGE_FRONTIER_DATAGEN_FRONTIER_ANALYSIS_H

// preprocessing operations on frontiers
#define FRONTIER_HOLE_FILLING_KERNEL 1
#define FRONTIER_OPENING_KERNEL 1

#define MIN_FRONTIER_CLUSTER_SIZE 20

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/static_layer.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>

#include <vector>

namespace stage_frontier_datagen
{
namespace frontier_analysis
{

/**
 *
 * @param costmap_2d_ros
 * @param desired_resolution
 * @return
 */
cv::Mat getMap(const boost::shared_ptr<costmap_2d::Costmap2DROS> &costmap_2d_ros, double desired_resolution,
               const tf::Transform &transform_gt_est);


/**
 *
 * @param costmap_2d_ros
 * @param clustered_frontier_poses
 * @param transform_gt_est
 * @return
 */
cv::Mat getBoundingBoxImage(const boost::shared_ptr<costmap_2d::Costmap2DROS> &costmap_2d_ros,
                            const std::vector< std::vector<geometry_msgs::PoseStamped> > clustered_frontier_poses,
                            const tf::Transform &transform_gt_est);

/**
 * @brief clip and resize the original map to fit groundtruth
 * @param original_map
 * @param groundtruth_size
 * @return
 */
cv::Mat convertToGroundtruthSize(const cv::Mat &original_map, const cv::Size groundtruth_size);

/**
 *
 * @param original_map the original map with probabilities between 0 to 100 and -1 for invalid
 * @return map with only free space colored with max intensity
 */
cv::Mat thresholdCostmap(const cv::Mat &original_map);

/**
 *
 * @param costmap_2d_ros
 * @param odometry
 * @return transform from estimated coordinates to groundtruth coordinates
 */
tf::Transform getTransformGroundtruthEstimated(const boost::shared_ptr<costmap_2d::Costmap2DROS> &costmap_2d_ros,
                                               const nav_msgs::Odometry &odometry);

/**
 * @brief affine transformation to center the image to (0, 0) in world (metric) frame
 * @param static_costmap static costmap
 * @return affine transformation
 */
cv::Mat getMapCenteringAffineTransformation(const boost::shared_ptr<costmap_2d::StaticLayer> static_costmap);


/**
 * @brief affine transformation to align estimated map with groundtruth
 * @param static_costmap static costmap
 * @param transform_gt_est transform from estimated map coordinates to groundtruth (in meters)
 * @return affine transformation
 */
cv::Mat getMapGroundtruthAffineTransformation(const boost::shared_ptr<costmap_2d::StaticLayer> static_costmap,
                                              const tf::Transform &transform_gt_est);


/**
 * @brief Based on costmap_2d_ros getRobotPose method
 * @param source_pose
 * @param target_frame_id
 * @param target_pose
 * @return if the pose query was successful
 */
bool transformPose(tf::Stamped<tf::Pose> source_pose,
                  std::string target_frame_id,
                  tf::Stamped<tf::Pose> &target_pose);
/**
 * @brief get robot pose in specific frame_id.
 * @param costmap_2d_ros
 * @param frame_id frame id of pose
 * @param global_pose output pose
 * @return if the pose query was successful
 */
bool getRobotPose(const boost::shared_ptr<costmap_2d::Costmap2DROS> &costmap_2d_ros,
                  std::string target_frame_id,
                  tf::Stamped<tf::Pose> &global_pose);

/**
 *
 * @param costmap_image image in the resolution/size of costmap (e.g. frontier image)
 * @param costmap_2d_ros
 * @param desired_resolution
 * @return
 */
cv::Mat resizeToDesiredResolution(const cv::Mat &costmap_image,
                                  const boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_2d_ros,
                                  double desired_resolution);

/**
 *
 * @param costmap_bounding_rect bounding rectangle in the resolution/size of costmap (e.g. frontier ROI)
 * @param costmap_2d_ros
 * @param desired_resolution
 * @return
 */
cv::Rect resizeToDesiredResolution(const cv::Rect &costmap_bounding_rect,
                                   const boost::shared_ptr<costmap_2d::Costmap2DROS> &costmap_2d_ros,
                                   double desired_resolution);

/**
 *
 * @param world_points points in world (metric) coords
 * @param costmap_2d_ros
 * @param transform_gt_est transform from estimated coords to groundtruth coords
 * @return points in (cost)map coordinates
 */
std::vector<cv::Point> worldPointsToMapPoints(const std::vector<geometry_msgs::PoseStamped> &world_points,
                                              const boost::shared_ptr<costmap_2d::Costmap2DROS> &costmap_2d_ros,
                                              const tf::Transform &transform_gt_est);

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
