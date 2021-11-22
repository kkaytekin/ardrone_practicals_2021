// Bring in my package's API, which is what I'm testing
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>

// Test projection and backprojection
TEST(PinholeCamera, backprojection)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame
  Eigen::Vector3d point = pinholeCamera.createRandomVisiblePoint();

  // projection
  Eigen::Vector2d imagePoint;
  pinholeCamera.project(point, &imagePoint);

  // backprojection
  Eigen::Vector3d ray;
  pinholeCamera.backProject(imagePoint, &ray);
  
  // backprojection should align with original point
  EXPECT_TRUE(fabs(ray.normalized().transpose()*point.normalized()-1.0)<1.0e-10);
}

// Test projection Jacobian
TEST(PinholeCamera, jacobian)
{
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame
  Eigen::Vector3d point = pinholeCamera.createRandomVisiblePoint();

  // project and get Jacobian
  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> jacobian;
  pinholeCamera.project(point, &imagePoint, &jacobian);

  // central difference computation of the projection Jacobian
  double h = 1.0e-5;
  Eigen::Matrix<double, 2, 3> numJacobian;
  Eigen::Vector2d left;
  Eigen::Vector2d right;
  pinholeCamera.project(point + h * Eigen::Vector3d::UnitX(), &right);
  pinholeCamera.project(point - h * Eigen::Vector3d::UnitX(), &left);
  numJacobian.col(0) = (right-left) / (2*h);
  pinholeCamera.project(point + h * Eigen::Vector3d::UnitY(), &right);
  pinholeCamera.project(point - h * Eigen::Vector3d::UnitY(), &left);
  numJacobian.col(1) = (right-left) / (2*h);
  pinholeCamera.project(point + h * Eigen::Vector3d::UnitZ(), &right);
  pinholeCamera.project(point - h * Eigen::Vector3d::UnitZ(), &left);
  numJacobian.col(2) = (right-left) / (2*h);

  // use square of Frobenius norm of the difference between
  // finite difference Jacobian and computed Jacobian to compare
  EXPECT_TRUE(fabs((jacobian-numJacobian).squaredNorm())<1.0e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
