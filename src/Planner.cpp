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
    relIndex_t neighborIndices_[6] = {
      {-1, 0, 0},
      {+1, 0, 0},
      {0, -1, 0},
      {0, +1, 0},
      {0, 0, -1},
      {0, 0, +1}
    };

    MapCoordinates startCoordinates = {x_start, y_start, z_start};
    start_->idx = coordinatesToIndices(startCoordinates);
    start_->previous = start_;

    MapCoordinates goalCoordinates = {x_goal, y_goal, z_goal};
    goalCoordinates_ = goalCoordinates;
    goal_->idx = coordinatesToIndices(goalCoordinates);
    goal_->previous = goal_;
  }

  double Planner::aStar ()
  {
    std::set<Planner::Vertex> openSet;
    start_->distance = 0;
    start_->distanceEstimate = distanceEstimate(start_);
    openSet.insert(*start_);

    while (!openSet.empty()) {
      Planner::Vertex current = *openSet.begin();
      if (current == *goal_) {
        return current.distance;
      }
      // go through 6 neighboring vertices (*NOT* the 26 neighboring!)
      for (int i=0; i<6; i++) {
        relIndex_t neighborIndex = neighborIndices_[i];
        //TODO
      }
    }
  }

  double Planner::distanceEstimate (Planner::Vertex* vertex)
  {
    Planner::MapCoordinates vertexCoordinates = indicesToCoordinates(vertex->idx);
    return std::sqrt(
      (vertexCoordinates.x - goalCoordinates_.x) * (vertexCoordinates.x - goalCoordinates_.x) +
      (vertexCoordinates.y - goalCoordinates_.y) * (vertexCoordinates.y - goalCoordinates_.y) +
      (vertexCoordinates.z - goalCoordinates_.z) * (vertexCoordinates.z - goalCoordinates_.z)
    );
  }

  Planner::MapIndices Planner::coordinatesToIndices (Planner::MapCoordinates& coordinates)
  {
    return Planner::MapIndices{
      (int) std::round(coordinates.x/0.1) + (wrappedMapData_->size[0]-1)/2,
      (int) std::round(coordinates.y/0.1) + (wrappedMapData_->size[1]-1)/2,
      (int) std::round(coordinates.z/0.1) + (wrappedMapData_->size[2]-1)/2
    };
  }

  Planner::MapCoordinates Planner::indicesToCoordinates (Planner::MapIndices& indices)
  {
    return Planner::MapCoordinates{
      (double) (indices.x - (wrappedMapData_->size[0]-1)/2) * 10,
      (double) (indices.y - (wrappedMapData_->size[1]-1)/2) * 10,
      (double) (indices.z - (wrappedMapData_->size[2]-1)/2) * 10
    };
  }

}  // namespace arp
