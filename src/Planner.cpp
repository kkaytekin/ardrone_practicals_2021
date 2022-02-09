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
                   : wrappedMapData_{&Map}, goalCoordinates_{x_goal,y_goal,z_goal}, startCoordinates_{x_start, y_start, z_start}
  {
    start_.idx = coordinatesToIndices(startCoordinates_);
    goal_.idx = coordinatesToIndices(goalCoordinates_);
  }

  double Planner::aStar (std::deque<Autopilot::Waypoint>* waypoints)
  {
    // return -1 if goal occupied
    if (isOccupied(goal_.idx.x, goal_.idx.y, goal_.idx.z)) {
      std::cout << "Goal is in occupied space!" << std::endl;
      return -1;
    }

    bool foundGoal = false;
    double distance;
    std::set<Planner::Vertex> openSet;
    std::vector<Planner::Vertex> vertices(100000);
    start_.distance = 0;
    int size[3] = {wrappedMapData_->size[0], wrappedMapData_->size[1], wrappedMapData_->size[2]};
    cv::Mat distanceMatrix(3, size, CV_64F, std::numeric_limits<double>::infinity());
    distanceMatrix.at<double>(start_.idx.x, start_.idx.y, start_.idx.z) = 0;
    start_.distanceEstimate = distanceEstimate(start_);
    openSet.insert(start_);

    while (!openSet.empty()) {
      Planner::Vertex current = *openSet.begin();
      vertices.push_back(current);
      openSet.erase(openSet.begin());
      if (current == goal_) {
        distance = current.distance;
        break;
      }
      // go through 6 neighboring vertices (*NOT* the 26 neighboring!)
      for (int i=0; i<6; i++) {
        MapIndices neighborIndex = neighborIndices_[i];
        // skip if index out of bounds
        if (current.idx.x + neighborIndex.x < 0 ||
            current.idx.x + neighborIndex.x >= size[0] ||
            current.idx.y + neighborIndex.y < 0 ||
            current.idx.y + neighborIndex.y >= size[1] ||
            current.idx.z + neighborIndex.z < 0 ||
            current.idx.z + neighborIndex.z >= size[2]
        ) {
          continue;
        }
        // skip neighbor if occupied
        if (isOccupied(
          current.idx.x + neighborIndex.x,
          current.idx.y + neighborIndex.y,
          current.idx.z + neighborIndex.z
        )) {
          continue;
        }
        // hardcoded neighbor distance = 10 (not valid for 26 neighbors!)
        double alt = current.distance + 10;
        double neighborDistance = distanceMatrix.at<double>(
          current.idx.x + neighborIndex.x,
          current.idx.y + neighborIndex.y,
          current.idx.z + neighborIndex.z
        );
        if (alt < neighborDistance) {
          distanceMatrix.at<double>(
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
          if (newVertex == goal_) {
            foundGoal = true;
            goal_.distance = alt;
            goal_.distanceEstimate = alt;
            goal_.previous = &vertices.back();
            openSet.insert(goal_);
          } else {
            newVertex.distance = alt;
            newVertex.distanceEstimate = alt + distanceEstimate(newVertex);
            newVertex.previous = &vertices.back();
            openSet.insert(newVertex);
          }
        }
      }
    }

    if (foundGoal) {
      Planner::Vertex* current = &goal_;
      while (true) {
        if (*current == start_) {
          return distance;
        }
        Planner::MapCoordinates coordinates = indicesToCoordinates(current->idx);
        Autopilot::Waypoint waypoint = {
          coordinates.x,
          coordinates.y,
          coordinates.z,
          0,   // yaw angle
          0.1  // tolerance
        };
        waypoints->push_front(waypoint);
        current = current->previous;
      }
      return distance;
    }

    std::cout << "Could not find path to goal!" << std::endl;
    return -1;
  }

  std::deque<Autopilot::Waypoint> Planner::getWaypoints() {
    std::deque<Autopilot::Waypoint> waypoints;
    Planner::Vertex* current = &goal_;
    while (true) {
      if (current == &start_) {
        return waypoints;
      }
      Planner::MapCoordinates coordinates = indicesToCoordinates(current->idx);
      Autopilot::Waypoint waypoint = {
        coordinates.x,
        coordinates.y,
        coordinates.z,
        0,  // yaw angle
        0.1  // tolerance
      };
      waypoints.push_front(waypoint);
      current = current->previous;
    }
  }

  double Planner::distanceEstimate (Planner::Vertex vertex)
  {
    Planner::MapCoordinates vertexCoordinates = indicesToCoordinates(vertex.idx);
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
      (double) (indices.x - (wrappedMapData_->size[0]-1)/2) * 0.1,
      (double) (indices.y - (wrappedMapData_->size[1]-1)/2) * 0.1,
      (double) (indices.z - (wrappedMapData_->size[2]-1)/2) * 0.1
    };
  }

  // Inputs: i,j,k: Indices of a position
  bool Planner::isOccupied(const int& i, const int& j, const int& k) {
    if ((int)wrappedMapData_->at<char>(i,j,k) < 0 && (int)wrappedMapData_->at<char>(i,j,k) > -128)
      return false;
    else
      return true;
  }

}  // namespace arp
