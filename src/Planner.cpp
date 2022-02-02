/*
 * Planner.cpp
 *
 *  Created on: 2 Feb 2022
 *      Author: matzewolf
 */

#include <set>
#include <cmath>

#include <arp/Planner.hpp>

namespace arp {

  Planner::Planner(cv::Mat & Map,
                   double x_goal, double y_goal, double z_goal,
                   double x_start = 0, double y_start = 0, double z_start = 0)
  {
    wrappedMapData_ = &Map;
    MapCoordinates startCoordinates = {x_start, y_start, z_start};
    start_->idx = coordinatesToIndices(startCoordinates);
    start_->previous = start_;
    MapCoordinates goalCoordinates = {x_goal, y_goal, z_goal};
    goalCoordinates_ = goalCoordinates;
    goal_->idx = coordinatesToIndices(goalCoordinates);
    goal_->previous = goal_;
  }

  void a_star ()
  {
    std::set<Planner::Vertex> openSet;
    openSet.insert(start_);
  }

  double distanceEstimate (Planner::Vertex& vertex)
  {
    // TODO: why doesnt it know these functions/variables here?
    Planner::MapCoordinates vertexCoords = indicesToCoordinates(vertex.idx);
    return std::sqrt((vertexCoords.x-goalCoords_.x)*(vertexCoords.x-goalCoords_.x) +
                     (vertexCoords.y-goalCoords_.y)*(vertexCoords.y-goalCoords_.y) +
                     (vertexCoords.z-goalCoords_.z)*(vertexCoords.z-goalCoords_.z));
  }

  Planner::MapIndices coordinatesToIndices (Planner::MapCoordinates& coordinates)
  {
    return Planner::MapIndices{
      std::round(coordinates.x/0.1) + (wrappedMapData_.size[0]-1)/2,
      std::round(coordinates.y/0.1) + (wrappedMapData_.size[1]-1)/2,
      std::round(coordinates.z/0.1) + (wrappedMapData_.size[2]-1)/2
    };
  }

  Planner::MapCoordinates indicesToCoordinates (Planner::MapIndices& indices)
  {
    return Planner::MapCoordinates{
      (indices.x - (wrappedMapData_.size[0]-1)/2) * 10,
      (indices.y - (wrappedMapData_.size[1]-1)/2) * 10,
      (indices.z - (wrappedMapData_.size[2]-1)/2) * 10
    };
  }

}  // namespace arp