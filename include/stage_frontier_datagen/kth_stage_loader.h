//
// Created by rakesh on 28/05/18.
//

#ifndef STAGE_FRONTIER_DATAGEN_KTH_STAGE_LOADER_H
#define STAGE_FRONTIER_DATAGEN_KTH_STAGE_LOADER_H

// TODO: make these proper parameters
#define MAP_RESOLUTION 0.25         // meters/pixels
#define MAP_SIZE cv::Size(256, 128)
// the inflations are on the either side of the wall
#define OBSTACLE_INFLATION_SIZE 0.5   // meters

// std includes
#include <string>

// libfloorplan includes
#include <GraphDatabase.hpp>
#include <GraphStatistics.hpp>
#include <GraphFileOperations.hpp>

namespace stage_frontier_datagen
{

/**
 * @brief class that wraps KTH floorplans for stage simulation
 */
class KTHStageLoader
{
public:
  /**
   * @brief creates the floorplan data structures
   * @param dataset_dir dataset directory (containing .xml specifications of floorplans)
   */
  int loadDirectory(std::string dataset_dir);

  /**
   *
   * @return reference to floorplans in the dataset
   */
  const std::vector<floorplanGraph> &getFloorplans() const { return floorplan_graphs_; }



  /**
   * @brief gets a random unobstructed points in a floorplan. Ensures that the point is within known space (rooms and corridors)
   * @param graph floorplan graph
   * @return random free points
   */
  static std::vector<cv::Point> getUnobstructedPoints(const floorplanGraph &graph);

  /**
   * @brief create world file from template (replacing keywords starting from @)
   * @param floorplan floorplan to create worldplan from
   * @param robot_position position to place robot to
   * @param base_path directory of worldfile template
   * @return world filename
   */
  static std::string
  createWorldFile(const floorplanGraph &floorplan, const Point2D &robot_position,
                  const std::string &base_path, const std::string &bitmap_name);



protected:
  std::string dataset_dir_;                                 ///< dataset directory (containing .xml specifications of floorplans)
  std::vector<floorplanGraph> floorplan_graphs_; ///< vector of floorplan graphs
}; // class KTHStageLoader

} // namespace stage_frontier_datagen

#endif //STAGE_FRONTIER_DATAGEN_KTH_STAGE_LOADER_H
