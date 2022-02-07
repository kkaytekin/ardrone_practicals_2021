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
    bool operator< (const Vertex& rhs) {
      return distanceEstimate < rhs.distanceEstimate;
    }
    bool operator== (const Vertex& rhs) {
      return (idx.x == rhs.idx.x) && (idx.y == rhs.idx.y) && (idx.z == rhs.idx.z);
    }
  };

  double aStar();
  std::deque<arp::Autopilot::Waypoint> getWaypoints();

 protected:
  cv::Mat* wrappedMapData_;
  cv::Mat distanceMatrix_;
  Vertex* start_;
  Vertex* goal_;
  MapCoordinates goalCoordinates_;
  MapCoordinates startCoordinates_;
  MapIndices neighborIndices_[];

  MapIndices coordinatesToIndices (MapCoordinates& coordinates);
  MapCoordinates indicesToCoordinates (MapIndices& indices);
  double distanceEstimate (Vertex* vertex);

};

}  // namespace arp

#endif /* PLANNER_HPP_ */
