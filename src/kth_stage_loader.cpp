//
// Created by rakesh on 28/05/18.
//


#include <stage_frontier_datagen/kth_stage_loader.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace stage_frontier_datagen;

int KTHStageLoader::loadDirectory(std::string dataset_dir)
{
  floorplan::GraphDatabase D;
  D.loadGraphs(dataset_dir, "floor");
  auto floorplan_graphs = D.getGraphs();
  floorplan_graphs_.clear();

  for (const auto &graph: floorplan_graphs)
  {
    if (!graph.m_vertices.empty()
        && !graph.m_edges.empty()
        && graph.m_property->floorname.find("conflicted") == std::string::npos
        && graph.m_property->centroid.x != -1
        && graph.m_property->centroid.y != -1)
    {
      floorplan_graphs_.push_back(graph);
    }
  }

  return floorplan_graphs_.size() > 0;
}

std::vector<cv::Point> KTHStageLoader::getUnobstructedPoints(const floorplanGraph &graph)
{
  cv::Mat floorplan_map = floorplan::GraphFileOperations::getGraphLayout(graph, MAP_RESOLUTION, MAP_SIZE);

  // inflate the map (so that the robot isn't very close to the obstacles)
  const int inflation_size = static_cast<int>(std::round(OBSTACLE_INFLATION_SIZE / MAP_RESOLUTION));
  cv::Mat inflation_kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                       cv::Size( 2*inflation_size + 1, 2*inflation_size+1 ),
                                                       cv::Point( inflation_size, inflation_size ) );
  cv::Mat inflated_map = floorplan_map.clone();
  cv::erode(floorplan_map, inflated_map, inflation_kernel);

  std::vector<cv::Point> unobstructed_points;

  BGL_FORALL_VERTICES(v, graph, floorplanGraph)
  {
    // space represents room, corridors etc
    auto space = graph[v];

    // transform to map coordinates
    auto min_point_map = floorplan::GraphFileOperations::transformToMapCoords(
      graph, Point2D(space.minx, space.miny), MAP_SIZE, MAP_RESOLUTION
    );

    auto max_point_map = floorplan::GraphFileOperations::transformToMapCoords(
      graph, Point2D(space.maxx, space.maxy), MAP_SIZE, MAP_RESOLUTION
    );

    cv::Mat space_mat = inflated_map
      .rowRange((int)min_point_map.y, (int)max_point_map.y)
      .colRange((int)min_point_map.x, (int)max_point_map.x);

    std::vector<cv::Point> unobstructed_points_in_space;
    // TODO: find a better way to find non-zero, without doing it twice
    if (cv::countNonZero(space_mat))
    {
      cv::findNonZero(space_mat, unobstructed_points_in_space);
    }

    // apply the offset to get in map coordinates
    for (auto &unobstructed_point: unobstructed_points_in_space)
    {
      unobstructed_point.x += min_point_map.x;
      unobstructed_point.y += min_point_map.y;
    }

    unobstructed_points.insert(
      unobstructed_points.end(),
      unobstructed_points_in_space.begin(), unobstructed_points_in_space.end()
    );
  }

  return unobstructed_points;
}