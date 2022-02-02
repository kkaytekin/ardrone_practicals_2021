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
  
  Planner(cv::Mat & Map,
          double x_goal, double y_goal, double z_goal,
          double x_start = 0, double y_start = 0, double z_start = 0);
  
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
    Vertex* previous;
    double distance = -1;
    double distanceEstimate = -1;
  };

  void a_star();

 protected:
  cv::Mat* wrappedMapData_;
  Vertex* start_;
  Vertex* goal_;

  MapIndices coordinatesToIndices (MapCoordinates&);
  MapCoordinates indicesToCoordinates (MapIndices&);

};

}  // namespace arp

#endif /* PLANNER_HPP_ */
