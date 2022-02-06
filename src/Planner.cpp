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
    cv::Mat* wrappedMapData_ = &Map;
    cv::Mat distanceMatrix_(3, wrappedMapData_->size, CV_64F, cv::Scalar(-1));
    MapIndices neighborIndices_[6] = {
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
    distanceMatrix_.at<char>(start_->idx.x, start_->idx.y, start_->idx.z) = 0;
    start_->distanceEstimate = distanceEstimate(start_);
    openSet.insert(*start_);

    while (!openSet.empty()) {
      Planner::Vertex current = *openSet.begin();
      if (current == *goal_) {
        return current.distance;
      }
      // go through 6 neighboring vertices (*NOT* the 26 neighboring!)
      for (int i=0; i<6; i++) {
        MapIndices neighborIndex = neighborIndices_[i];
        // skip if index out of bounds
        if (current.idx.x + neighborIndex.x < 0 ||
            current.idx.x + neighborIndex.x >= wrappedMapData_->size[0] ||
            current.idx.y + neighborIndex.y < 0 ||
            current.idx.y + neighborIndex.y >= wrappedMapData_->size[1] ||
            current.idx.z + neighborIndex.z < 0 ||
            current.idx.z + neighborIndex.z >= wrappedMapData_->size[2]
        ) {
          continue;
        }
        // skip neighbor if occupied (i.e. log-odds > -5)
        if ((int)wrappedMapData_->at<char>(
          current.idx.x + neighborIndex.x,
          current.idx.y + neighborIndex.y,
          current.idx.z + neighborIndex.z
        ) > -5) {
          continue;
        }
        // hardcoded neighbor distance = 10 (not valid for 26 neighbors!)
        double alt = current.distance + 10;
        double neighborDistance = distanceMatrix_.at<char>(
          current.idx.x + neighborIndex.x,
          current.idx.y + neighborIndex.y,
          current.idx.z + neighborIndex.z
        );
        if (alt < neighborDistance || neighborDistance == -1) {
          distanceMatrix_.at<char>(
            current.idx.x + neighborIndex.x,
            current.idx.y + neighborIndex.y,
            current.idx.z + neighborIndex.z
          ) = alt;
          Vertex newVertex;
          newVertex.idx = {
            current.idx.x + neighborIndex.x,
            current.idx.y + neighborIndex.y,
            current.idx.z + neighborIndex.z
          };
          newVertex.distance = alt;
          newVertex.distanceEstimate = alt + distanceEstimate(&newVertex);
          newVertex.previous = &current;
          openSet.insert(newVertex);
        }
      }
    }
    return -1;
  }

  std::deque<Autopilot::Waypoint> Planner::getWaypoints() {
    std::deque<Autopilot::Waypoint> waypoints;
    Planner::Vertex* current = goal_;
    while (true) {
      if (*current == *start_) {
        return waypoints;
      }
      Planner::MapCoordinates coordinates = indicesToCoordinates(current->idx);
      Autopilot::Waypoint waypoint = {
        coordinates.x,
        coordinates.y,
        coordinates.z,
        0,  // yaw angle
        10  // tolerance
      };
      waypoints.push_front(waypoint);
      current = current->previous;
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
