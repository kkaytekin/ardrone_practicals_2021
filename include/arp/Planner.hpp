/*
 * Planner.hpp
 *
 *  Created on: 02 Feb 2022
 *      Author: matzewolf
 */

#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include <opencv2/core/mat.hpp>

#include <arp/Autopilot.hpp>

namespace arp {

class Planner
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Planner(cv::Mat& Map,
          double x_goal, double y_goal, double z_goal,
          double x_start , double y_start , double z_start );
  
  struct MapIndices {
    int x;
    int y;
    int z;
  };

  struct MapCoordinates {
    double x;
    double y;
    double z;
  };

  struct Vertex {
    MapIndices idx;
    Vertex* previous = 0;  // 0 means undefined
    double distance = -1;
    double distanceEstimate = -1;
    bool operator< (const Vertex& rhs) const {
      return distanceEstimate < rhs.distanceEstimate;
    }
    bool operator== (const Vertex& rhs) const {
      return (idx.x == rhs.idx.x) && (idx.y == rhs.idx.y) && (idx.z == rhs.idx.z);
    }
  };

  double aStar(std::deque<Autopilot::Waypoint>* waypoints);
  bool isOccupied(const int& i, const int& j, const int& k);

  MapIndices neighborIndices_[6] = {
    {-1, 0, 0},
    {+1, 0, 0},
    {0, -1, 0},
    {0, +1, 0},
    {0, 0, -1},
    {0, 0, +1}
  };

 protected:
  cv::Mat* wrappedMapData_;
  Vertex start_;
  Vertex goal_;
  MapCoordinates goalCoordinates_;
  MapCoordinates startCoordinates_;
  std::vector<Vertex> vertices_;

  MapIndices coordinatesToIndices (MapCoordinates& coordinates);
  MapCoordinates indicesToCoordinates (MapIndices& indices);
  double distanceEstimate (Vertex vertex);
};

}  // namespace arp

#endif /* PLANNER_HPP_ */
