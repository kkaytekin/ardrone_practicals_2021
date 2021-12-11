#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>
#include <arp/Frontend.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/ViEkf.hpp>
#include <arp/VisualInertialTracker.hpp>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Subscriber(arp::VisualInertialTracker* tracker)
  {
    tracker_ = tracker;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    tracker_->addImage(timeMicroseconds, lastImage_);
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    const Eigen::Vector3d angularVelocity(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
    const Eigen::Vector3d acceleration(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
    std::lock_guard<std::mutex> l(imuMutex_);  // needed?
    tracker_->addImuMeasurement(timeMicroseconds, angularVelocity, acceleration);
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
  std::mutex imuMutex_;
  arp::VisualInertialTracker* tracker_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // set up autopilot
  arp::Autopilot autopilot(nh);

  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 360, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // get configs
  double fu, fv, cu, cv, k1, k2, p1, p2;
  nh.getParam("/arp_node/fu", fu);
  nh.getParam("/arp_node/fv", fv);
  nh.getParam("/arp_node/cu", cu);
  nh.getParam("/arp_node/cv", cv);
  nh.getParam("/arp_node/k1", k1);
  nh.getParam("/arp_node/k2", k2);
  nh.getParam("/arp_node/p1", p1);
  nh.getParam("/arp_node/p2", p2);

  // init camera and distortion model
  arp::cameras::RadialTangentialDistortion distortion =
      arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2);
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> camera =
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
          640, 360, fu, fv, cu, cv, distortion);
  camera.initialiseUndistortMaps();

  // set up frontend
  arp::Frontend frontend(640, 360, fu, fv, cu, cv, k1, k2, p1, p2);

  // load map
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFile;
  if(!nh.getParam("arp_node/map", mapFile))
    ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFile;
  if(!frontend.loadMap(mapPath))
    ROS_FATAL_STREAM("could not load map from " << mapPath << " !");

  // state publisher -- provided for rviz visualisation of drone pose:
  arp::StatePublisher pubState(nh);

  // set up EKF
  arp::ViEkf viEkf;
  Eigen::Matrix4d T_SC_mat;
  std::vector<double> T_SC_array;
  if(!nh.getParam("arp_node/T_SC", T_SC_array))
    ROS_FATAL("error loading parameter");
  T_SC_mat << T_SC_array[0], T_SC_array[1], T_SC_array[2], T_SC_array[3],
              T_SC_array[4], T_SC_array[5], T_SC_array[6], T_SC_array[7],
              T_SC_array[8], T_SC_array[9], T_SC_array[10], T_SC_array[11],
              T_SC_array[12], T_SC_array[13], T_SC_array[14], T_SC_array[15];
  arp::kinematics::Transformation T_SC(T_SC_mat);
  viEkf.setCameraExtrinsics(T_SC);
  viEkf.setCameraIntrinsics(frontend.camera());

  // set up visual-inertial tracking
  arp::VisualInertialTracker visualInertialTracker;
  visualInertialTracker.setFrontend(frontend);
  visualInertialTracker.setEstimator(viEkf);

  // set up visualisation: publish poses to topic ardrone/vi_ekf_pose
  visualInertialTracker.setVisualisationCallback(std::bind(
      &arp::StatePublisher::publish,
      &pubState,
      std::placeholders::_1,
      std::placeholders::_2
  ));

  // setup inputs
  Subscriber subscriber(&visualInertialTracker);
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe(
      "ardrone/imu", 50, &Subscriber::imuCallback, &subscriber);

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;

  cv::Mat originalImage;
  cv::Mat undistortImage;
  cv::Mat image;
  bool undistort = false;
  bool undistortSuccess = true;

  float forward{0};
  float left{0};
  float up{0};
  float rotateLeft{0};
  std::string droneStatusString;

  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }
    auto droneStatus = autopilot.droneStatus();
    switch (droneStatus)
    {
      case 0:
        droneStatusString = "Unknown"; break;
      case 1:
        droneStatusString = "Inited"; break;
      case 2:
        droneStatusString = "Landed"; break;
      case 3:
        droneStatusString = "Flying"; break;
      case 4:
        droneStatusString = "Hovering"; break;
      case 5:
        droneStatusString = "Test"; break;
      case 6:
        droneStatusString = "Taking Off"; break;
      case 7:
        droneStatusString = "Flying 2"; break;
      case 8:
        droneStatusString = "Landing"; break;
      case 9:
        droneStatusString = "Looping"; break;
      default:
        droneStatusString = "Unknown"; break;
    }
    auto batteryStatus = autopilot.batteryStatus();

    // render image, if there is a new one available
    if(subscriber.getLastImage(originalImage)) {

      // Undistort image optionally
      if(undistort) {
        undistortSuccess = camera.undistortImage(originalImage, undistortImage);
        if(undistortSuccess) {
          image = undistortImage;
        } else {
          undistort = false;
          std::cout << "Undistortion failed, showing original image." << std::endl;
          image = originalImage;
        }
      } else {
        image = originalImage;
      }

      // Print instructions
      cv::putText(image,
                  "Instructions: T - take off, L - land, ESC - motors off, Arrows - move horizontally, W - ascend, S - descend",
                  cv::Point(5,345),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.5,
                  cv::Scalar(255,255,255));
      cv::putText(image,
                  "A - yaw left, D - yaw right, O - original image, U - undistorted image",
                  cv::Point(5,355),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.5,
                  cv::Scalar(255,255,255));
      // Print Drone State
      cv::putText(image,
                  "Status: " + droneStatusString,
                  cv::Point(5,20),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  1,
                  cv::Scalar(255,255,255),
                  1.5);
      // Print Distortion State
      std::string distortionStatusString = undistort ? "on" : "off";
      cv::putText(image,
                  "Undistortion: " + distortionStatusString,
                  cv::Point(5,40),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  1,
                  cv::Scalar(255,255,255),
                  1.5);
      // Print Battery State
      cv::putText(image,
                  "Battery: " + std::to_string(static_cast<int>(batteryStatus)) + "%",
                  cv::Point(4,60),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  1,
                  cv::Scalar(255,255,255),
                  1.5);
      // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
      // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
      // remember to pick the right SDL_PIXELFORMAT_* !
      texture = SDL_CreateTexture(
          renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);
      SDL_UpdateTexture(texture, NULL, (void*)image.data, image.step1());
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, NULL, NULL);
      SDL_RenderPresent(renderer);
      // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
      SDL_DestroyTexture(texture);
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // check states!
    // auto droneStatus = autopilot.droneStatus();

    // command
    if (state[SDL_SCANCODE_ESCAPE]) {
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_U]) {
      std::cout << "Showing undistorted image...           status=" << droneStatus << std::endl;
      undistort = true;
    }
    if (state[SDL_SCANCODE_O]) {
      std::cout << "Showing original image...              status=" << droneStatus << std::endl;
      undistort = false;
    }

    // TODO: process moving commands when in state 3,4, or 7
    /*
     * SDL_SCANCODE_W = 26,
     * SDL_SCANCODE_A = 4,
     * SDL_SCANCODE_S = 22,
     * SDL_SCANCODE_D = 7,
     * SDL_SCANCODE_RIGHT = 79,
     * SDL_SCANCODE_LEFT = 80,
     * SDL_SCANCODE_DOWN = 81,
     * SDL_SCANCODE_UP = 82,
     */
    up = 0; rotateLeft = 0; forward = 0; left = 0;

    if (state[26] || state[4] || state[22] || state[7] ||
      state[79] || state[80] || state[81] || state[82] ) {
        if (state[SDL_SCANCODE_W]){
          std::cout << "Moving up...                           status=" << droneStatus;
          up = 1.0;
        }
        if (state[SDL_SCANCODE_S]){
          std::cout << "Moving down...                         status=" << droneStatus;
          up = -1.0;
        }
        if (state[SDL_SCANCODE_A]){
          std::cout << "Turning Left...                        status=" << droneStatus;
          rotateLeft = 1.0;
        }
        if (state[SDL_SCANCODE_D]){
          std::cout << "Turning Right...                       status=" << droneStatus;
          rotateLeft = -1.0;
        }
        if (state[SDL_SCANCODE_UP]){
          std::cout << "Moving Forward...                      status=" << droneStatus;
          forward = 1.0;
        }
        if (state[SDL_SCANCODE_DOWN]){
          std::cout << "Moving Backward...                     status=" << droneStatus;
          forward = -1.0;
        }
        if (state[SDL_SCANCODE_LEFT]){
          std::cout << "Moving Left...                         status=" << droneStatus;
          left = 1.0;
        }
        if (state[SDL_SCANCODE_RIGHT]){
          std::cout << "Moving Right...                        status=" << droneStatus;
          left = -1.0;
        }
        bool success = autopilot.manualMove(forward,left,up,rotateLeft);
        if (success) {
          std::cout << " [ OK ]" << std::endl;
        } else {
          std::cout << " [FAIL]" << std::endl;
        }
    } else {
    	autopilot.manualMove(0.0,0.0,0.0,0.0);
    }
  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
