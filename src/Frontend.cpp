/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2020
 *      Author: sleutene
 */

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>

#include <brisk/brisk.h>

#include <arp/Frontend.hpp>

#ifndef CV_AA
#define CV_AA cv::LINE_AA // maintains backward compatibility with older OpenCV
#endif

#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY // maintains backward compatibility with older OpenCV
#endif

namespace arp {

Frontend::Frontend(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2) :
  camera_(imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV,
          arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2))
{
  camera_.initialiseUndistortMaps();

  // also save for OpenCV RANSAC later
  cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cameraMatrix_.at<double>(0,0) = focalLengthU;
  cameraMatrix_.at<double>(1,1) = focalLengthV;
  cameraMatrix_.at<double>(0,2) = imageCenterU;
  cameraMatrix_.at<double>(1,2) = imageCenterV;
  cameraMatrix_.at<double>(2,2) = 1.0;
  distCoeffs_ = cv::Mat::zeros(1, 4, CV_64FC1);
  distCoeffs_.at<double>(0) = k1;
  distCoeffs_.at<double>(1) = k2;
  distCoeffs_.at<double>(2) = p1;
  distCoeffs_.at<double>(3) = p2;
  
  // BRISK detector and descriptor
  detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(10, 0, 100, 2000));
  extractor_.reset(new brisk::BriskDescriptorExtractor(true, false));
  
#if 1
  // leverage camera-aware BRISK (caution: needs the *_new* maps...)
  cv::Mat rays = cv::Mat(imageHeight, imageWidth, CV_32FC3);
  cv::Mat imageJacobians = cv::Mat(imageHeight, imageWidth, CV_32FC(6));
  for (int v=0; v<imageHeight; ++v) {
    for (int u=0; u<imageWidth; ++u) {
      Eigen::Vector3d ray;
      Eigen::Matrix<double, 2, 3> jacobian;
      if(camera_.backProject(Eigen::Vector2d(u,v), &ray)) {
        ray.normalize();
      } else {
        ray.setZero();
      }
      rays.at<cv::Vec3f>(v,u) = cv::Vec3f(ray[0],ray[1],ray[2]);
      Eigen::Vector2d pt;
      if(camera_.project(ray, &pt, &jacobian)
         ==cameras::ProjectionStatus::Successful) {
        cv::Vec6f j;
        j[0]=jacobian(0,0);
        j[1]=jacobian(0,1);
        j[2]=jacobian(0,2);
        j[3]=jacobian(1,0);
        j[4]=jacobian(1,1);
        j[5]=jacobian(1,2);
        imageJacobians.at<cv::Vec6f>(v,u) = j;
      }
    }
  }
  std::static_pointer_cast<cv::BriskDescriptorExtractor>(extractor_)->setCameraProperties(rays, imageJacobians, 185.6909);
#endif   
}

bool  Frontend::loadMap(std::string path) {
  std::ifstream mapfile(path);
  if(!mapfile.good()) {
    return false;
  }
  
  // read each line
  std::string line;
  uint64_t id=1;
  while (std::getline(mapfile, line)) {
    Landmark landmark;

    // Convert to stringstream
    std::stringstream ss(line);

    // read 3d position
    for(int i=0; i<3; ++i) {
      std::string coordString;
      std::getline(ss, coordString, ',');
      double coord;
      std::stringstream(coordString) >> coord;
      landmark.point[i] = coord;
    }

    // Get each descriptor
    std::string descriptorstring;
    while(ss.good()){
      std::getline(ss, descriptorstring, ',');
      cv::Mat descriptor(1,48,CV_8UC1);
      for(int col=0; col<48; ++col) {
        uint32_t byte;
        std::stringstream(descriptorstring.substr(2*col,2)) >> std::hex >> byte;
        descriptor.at<uchar>(0,col) = byte;
      }
      landmark.descriptors.push_back(descriptor);
    }

    // store into map
    landmarks_[id] = landmark;
    id++;
  }
  std::cout << "loaded " << landmarks_.size() << " landmarks." << std::endl;
  return landmarks_.size() > 0;
}

int Frontend::detectAndDescribe(
    const cv::Mat& grayscaleImage, const Eigen::Vector3d& extractionDirection,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const {

  // run BRISK detector
  detector_->detect(grayscaleImage, keypoints);

  // run BRISK descriptor extractor
  // orient the keypoints according to the extraction direction:
  Eigen::Vector3d ep;
  Eigen::Vector2d reprojection;
  Eigen::Matrix<double, 2, 3> Jacobian;
  Eigen::Vector2d eg_projected;
  for (size_t k = 0; k < keypoints.size(); ++k) {
    cv::KeyPoint& ckp = keypoints[k];
    const Eigen::Vector2d kp(ckp.pt.x, ckp.pt.y);
    // project ray
    camera_.backProject(kp, &ep);
    // obtain image Jacobian
    camera_.project(ep+extractionDirection.normalized()*0.001, &reprojection);
    // multiply with gravity direction
    eg_projected = reprojection-kp;
    double angle = atan2(eg_projected[1], eg_projected[0]);
    // set
    ckp.angle = angle / M_PI * 180.0;
  }
  extractor_->compute(grayscaleImage, keypoints, descriptors);

  return keypoints.size();
}

bool Frontend::ransac(const std::vector<cv::Point3d>& worldPoints, 
                      const std::vector<cv::Point2d>& imagePoints, 
                      kinematics::Transformation & T_CW, std::vector<int>& inliers) const {
  if(worldPoints.size() != imagePoints.size()) {
    return false;
  }
  if(worldPoints.size()<5) {
    return false; // not reliable enough
  }

  inliers.clear();
  cv::Mat rvec, tvec;
  bool ransacSuccess = cv::solvePnPRansac(
      worldPoints, imagePoints, cameraMatrix_, distCoeffs_,
      rvec, tvec, false, 100, 5.0, 0.99, inliers, cv::SOLVEPNP_EPNP);	

  // set pose
  cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T_CW_mat = Eigen::Matrix4d::Identity();
  for(int i=0; i<3; i++) {
    T_CW_mat(i,3) = tvec.at<double>(i);
    for(int j=0; j<3; j++) {
      T_CW_mat(i,j) = R.at<double>(i,j);
    }
  }
  T_CW = kinematics::Transformation(T_CW_mat);

  return ransacSuccess && (double(inliers.size())/double(imagePoints.size()) > 0.7);
}

bool Frontend::detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection, 
                              DetectionVec & detections, kinematics::Transformation & T_CW, 
                              cv::Mat & visualisationImage, bool needsReInitialisation)
{
  detections.clear(); // make sure empty

  // to gray:
  cv::Mat grayScale;
  cv::cvtColor(image, grayScale, CV_BGR2GRAY);

  // run BRISK detector and descriptor extractor:
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detectAndDescribe(grayScale, extractionDirection, keypoints, descriptors);

  std::vector<cv::Point3d> worldPoints;
  std::vector<cv::Point2d> imagePoints;
  DetectionVec preliminaryDetections;
  std::vector<cv::Point2d> visibleLandmarks;

  // match to map
  for(auto & lm : landmarks_) { // go through all landmarks in the map
    // efficient matching: check if landmark is visible in the image
    if (!needsReInitialisation) {
      Eigen::Vector4d lm_hom;
      lm_hom << lm.second.point, 1;
      Eigen::Vector4d lm_cam_hom = T_CW * lm_hom;
      Eigen::Vector3d lm_cam(lm_cam_hom.x(), lm_cam_hom.y(), lm_cam_hom.z());
      Eigen::Vector2d lm_projected;
      cameras::ProjectionStatus status = camera_.project(lm_cam, &lm_projected);
      if (status != cameras::ProjectionStatus::Successful) {
        continue;
      } else {
        cv::Point2d visibleLandmark(lm_projected.x(), lm_projected.y());
        visibleLandmarks.push_back(visibleLandmark);
      }
    }
    // try to match landmark to all the keypoints
    for(size_t k = 0; k < keypoints.size(); ++k) { // go through all keypoints in the frame
      uchar* keypointDescriptor = descriptors.data + k*48; // descriptors are 48 bytes long
      for(auto lmDescriptor : lm.second.descriptors) { // check agains all available descriptors
        const float dist = brisk::Hamming::PopcntofXORed(
                keypointDescriptor, lmDescriptor.data, 3); // compute desc. distance: 3 for 3x128bit (=48 bytes)
        // check if a match and process accordingly
        if(dist < 60) {
          Eigen::Vector2d keypoint(keypoints[k].pt.x, keypoints[k].pt.y);
          Detection detection;
          detection.keypoint = keypoint;
          imagePoints.push_back(keypoints[k].pt);
          detection.landmark = lm.second.point;
          cv::Point3d worldPoint(lm.second.point.x(), lm.second.point.y(), lm.second.point.z());
          worldPoints.push_back(worldPoint);
          detection.landmarkId = lm.first;
          preliminaryDetections.push_back(detection);
        }
      }
    }
  }

  // run RANSAC (to remove outliers and get pose T_CW estimate)
  std::vector<int> inliers;
  bool ransacSuccess = ransac(worldPoints, imagePoints, T_CW, inliers);

  // set detections
  for (size_t k = 0; k < inliers.size(); ++k) {
    detections.push_back(preliminaryDetections[inliers[k]]);
  }

  // visualise by painting stuff into visualisationImage
  for (size_t k = 0; k < imagePoints.size(); ++k) {
    cv::Scalar color;
    if (std::find(inliers.begin(), inliers.end(), k) != inliers.end()) {
      color << 0, 255, 0;  // green: matched keypoints
    } else {
      color << 0, 0, 255;  // red: keypoints that could not be matched
    }
    cv::circle(visualisationImage, imagePoints[k], 5, color);
  }
  for (cv::Point2d visibleLandmark : visibleLandmarks) {
    cv::Scalar color(255, 0, 0);  // blue: visible landmarks
    cv::circle(visualisationImage, visibleLandmark, 5, color);
  }

  return ransacSuccess;
}

}  // namespace arp
