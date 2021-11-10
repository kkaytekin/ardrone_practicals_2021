#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
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

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
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
    // -- for later use
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // setup inputs
  Subscriber subscriber;
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

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

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  cv::Mat image;
  /// OWN CODE
  float forward{0};
  float left{0};
  float up{0};
  float rotateLeft{0};
  std::string droneStatusString;
  /// END OF OWN CODE

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
    if(subscriber.getLastImage(image)) {

      // TODO: add overlays to the cv::Mat image, e.g. text
      // Print instructions
      cv::putText(image,
                  "Instructions: T - take off, L - land, ESC - motors off, Arrows - move horizontally",
                  cv::Point(5,345),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.5,
                  cv::Scalar(255,255,255));
      cv::putText(image,
                  "W - ascend, S - descend, A - yaw left, D - yaw right",
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
      // Print Battery State
      cv::putText(image,
                  "Battery: " + std::to_string(static_cast<int>(batteryStatus)) + "%",
                  cv::Point(5,40),
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
    if (state[26] || state[4]  || state[22] || state[7] ||
      state[79] || state[80] || state[81] || state[82] ) {
        if (state[SDL_SCANCODE_W]){
          std::cout << "Moving up... Power " << up*100 << " %  status=" << droneStatus;
          if (up < 1.0)
            up += 0.1;
        }
        if (state[SDL_SCANCODE_S]){
          std::cout << "Moving down... Power " << up*100 << " %  status=" << droneStatus;
          if (up > -1.0)
            up -= 0.1;
        }
        if (state[SDL_SCANCODE_A]){
          std::cout << "Turning Left... Power " << rotateLeft*100 << " %  status=" << droneStatus;
          if (rotateLeft < 1.0)
            rotateLeft += 0.1;
        }
        if (state[SDL_SCANCODE_D]){
          std::cout << "Turning Right... Power " << rotateLeft*100 << " %  status=" << droneStatus;
          if (rotateLeft > -1.0)
            rotateLeft -= 0.1;
        }
        if (state[SDL_SCANCODE_UP]){
          std::cout << "Moving Forward... Power " << forward*100 << " %  status=" << droneStatus;
          if (forward < 1.0)
            forward += 0.1;
        }
        if (state[SDL_SCANCODE_DOWN]){
          std::cout << "Moving Backward... Power " << forward*100 << " %  status=" << droneStatus;
          if (forward > -1.0)
            forward -= 0.1;
        }
        if (state[SDL_SCANCODE_LEFT]){
          std::cout << "Moving Left... Power " << left*100 << " %  status=" << droneStatus;
          if (left < 1.0)
            left += 0.1;
        }
        if (state[SDL_SCANCODE_RIGHT]){
          std::cout << "Moving Right... Power " << left*100 << " %  status=" << droneStatus;
          if (left > -1.0)
            left -= 0.1;
        }
        bool success = autopilot.manualMove(forward,left,up,rotateLeft);
        if (success) {
          std::cout << " [ OK ]" << std::endl;
        } else {
          std::cout << " [FAIL]" << std::endl;
        }
    }
    // Decay velocity
    forward    /= 1.1;
    left       /= 1.1;
    up         /= 1.1;
    rotateLeft /= 1.1;
    // Full stop if decayed enough
    if (std::abs(forward) < 0.001 && std::abs(left) < 0.001 && std::abs(up) < 0.001 && std::abs(rotateLeft) < 0.001)
      autopilot.manualMove(0.0,0.0,0.0,0.0);
  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

