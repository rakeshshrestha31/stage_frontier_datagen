//
// Created by rakesh on 28/05/18.
//

#define TMP_WORLDFILE "tmp_floorplan.world"

#include <fstream>
#include <regex>

#include <stage_frontier_datagen/kth_stage_loader.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace stage_frontier_datagen;

int KTHStageLoader::loadDirectory(std::string dataset_dir)
{
  dataset_dir_ = dataset_dir;

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
      // The BGL_FORALL_VERTICES macro doesn't work inside a thread, so do these in constructor before threading
      cv::Mat map = floorplan::GraphFileOperations::getGraphLayout(graph, MAP_RESOLUTION, MAP_SIZE);
      floorplan_t floorplan_obj(
        graph,
        map,
        getUnobstructedPoints(graph)
      );
      floorplan_graphs_.push_back(floorplan_obj);
    }
  }

  return !floorplan_graphs.empty();
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

      if (space.roomLayout.size() <= 0)
        continue;

      // transform to map coordinates
      auto min_point_map = floorplan::GraphFileOperations::transformToMapCoords(
          graph, Point2D(space.minx, space.miny), MAP_SIZE, MAP_RESOLUTION
      );

      auto max_point_map = floorplan::GraphFileOperations::transformToMapCoords(
          graph, Point2D(space.maxx, space.maxy), MAP_SIZE, MAP_RESOLUTION
      );

      cv::Point min_point = cv::Point((int)min_point_map.x, (int)min_point_map.y);
      cv::Point max_point = cv::Point((int)max_point_map.x, (int)max_point_map.y);

      // judge if the max_point and min_point is valid
      if(max_point.x <= 0 || max_point.y <=0 || min_point.x >= inflated_map.size().width
         || min_point.y >= inflated_map.size().height)
        continue;

      // crop the range of max_point and min_point if it is valid
      min_point = cv::Point(std::max(0, min_point.x), std::max(0, min_point.y));
      max_point = cv::Point(std::min(inflated_map.size().width, max_point.x),
                            std::min(inflated_map.size().height, max_point.y));

      std::vector<cv::Point> unobstructed_points_in_space;

      cv::Mat space_mat = inflated_map
          .rowRange(min_point.y, max_point.y)
          .colRange(min_point.x, max_point.x);

      // TODO: find a better way to find non-zero, without doing it twice
      if (cv::countNonZero(space_mat)) {
        cv::findNonZero(space_mat, unobstructed_points_in_space);
      }

      // apply the offset to get in map coordinates
      for (auto &unobstructed_point: unobstructed_points_in_space)
      {
        unobstructed_point.x += min_point.x;
        unobstructed_point.y += min_point.y;
      }

      unobstructed_points.insert(
          unobstructed_points.end(),
          unobstructed_points_in_space.begin(), unobstructed_points_in_space.end()
      );
    }

  return unobstructed_points;
}

std::string
KTHStageLoader::createWorldFile(const floorplanGraph &floorplan, const Point2D &robot_position,
                                const std::string &base_path, const std::string &bitmap_name)
{
  std::string worldfile_template = base_path + "/template.world";

  std::ifstream file_stream(worldfile_template);
  std::string worldfile_content((std::istreambuf_iterator<char>(file_stream)),
                                std::istreambuf_iterator<char>());

  worldfile_content = std::regex_replace(worldfile_content, std::regex("@bitmap_image@"), bitmap_name);
  double size_x = (floorplan.m_property->maxx - floorplan.m_property->minx) * floorplan.m_property->real_distance / floorplan.m_property->pixel_distance;
  double size_y = (floorplan.m_property->maxy - floorplan.m_property->miny) * floorplan.m_property->real_distance / floorplan.m_property->pixel_distance;
  std::cout << "world size: " << size_x << "x" << size_y << std::endl;

  worldfile_content = std::regex_replace(
    worldfile_content, std::regex("@size@"),
    std::to_string(size_x)
    + " "
    + std::to_string(size_y)
  );

  worldfile_content = std::regex_replace(
    worldfile_content, std::regex("@start_pose@"),
    std::to_string(robot_position.x) + " "
    + std::to_string(robot_position.y) + " "
    + "0 " + // z coord
    std::to_string(rand() % 360) // orientation (degree)
  );

  std::string tmp_worldfile_name = std::string(base_path) + "/" + TMP_WORLDFILE;
  std::ofstream tmp_worldfile_stream(tmp_worldfile_name);
  tmp_worldfile_stream << worldfile_content;
  tmp_worldfile_stream.close();

  return tmp_worldfile_name;
}