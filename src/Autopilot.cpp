/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode  

  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);

  // Set PID parameters
  PidController::Parameters p;
  p.k_p=0.2; p.k_i=3.0; p.k_d=0.05;
  rollAngPid.setParameters(p);
  p.k_p=0.2; p.k_i=3.0; p.k_d=0.05;
  pitchAngPid.setParameters(p);
  p.k_p=0.06; p.k_i=0.01; p.k_d=0.0;
  yawPid.setParameters(p);
  p.k_p=0.5; p.k_i=0.2; p.k_d=0.1;
  zPid.setParameters(p);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Get battery percentage.
float Autopilot::batteryStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.batteryPercent);
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up, double rotateLeft)
{
  DroneStatus status = droneStatus();
  if (status == DroneStatus::Flying || status == DroneStatus::Hovering
      || status == DroneStatus::Flying2) {
    //std::cout << "mv x, y, z , w: " << forward << ' ' << left << ' ' << up << ' ' << rotateLeft  << '\n';
    geometry_msgs::Twist Msg;
    Msg.linear.x = forward;
    Msg.linear.y = left;
    Msg.linear.z = up;
    Msg.angular.z = rotateLeft;
    pubMove_.publish(Msg);
    return true;
  }
  return false;
}

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}
//
std::string Autopilot::getOccupancyMap() {
  std::string mapFile;
      if(!nh_->getParam("/arp_node/occupancymap", mapFile))
        ROS_FATAL("error loading parameter");
  return mapFile;
}

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{ // Debug
  currentRobotState = x.t_WS;
  // TODO: Ask - Would it be beneficial to use mutex for the whole callback? what would be the drawbacks?

  // only do anything here, if automatic
  const double yaw = kinematics::yawAngle(x.q_WS);
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    return;
  }
  // DONE: only enable when in flight
  DroneStatus status = droneStatus();
  // 3: Flying, 4: Hovering, 7: Flying2
  if (status == DroneStatus::Flying ||
      status == DroneStatus::Hovering ||
      status == DroneStatus::Flying2) {
    if (flightChallenge_) {
      kinematics::Transformation T_WS(x.t_WS, x.q_WS);
      Eigen::Matrix3d R_SW = T_WS.R().transpose();
      Eigen::Vector3d posRef;
      // TODO: Ask: why the following if statement should be within lock_guard block?
      //   in our version, if the distance to waypoint is still greater than posTolerance,
      //   we just keep setting the same position as our reference. no harm done even if the mutex is not locked.
      if(!waypoints_.empty() && startToGoal_) {
        if (waypoints_.size() < 2)
          setPoseReference(waypoints_[0].x,
                         waypoints_[0].y,
                         0.3,
                         atan2(waypoints_[0].y, waypoints_[0].x));
        else
          setPoseReference(waypoints_[0].x,
                           waypoints_[0].y,
                           1.9,
                           atan2(waypoints_[0].y, waypoints_[0].x));
      } else if(!waypoints_.empty() && goalToStart_) {
        if (waypoints_.size() < 2)
          setPoseReference(waypoints_.back().x,
                         waypoints_.back().y,
                         0.3,
                         atan2(-waypoints_.back().y, waypoints_.back().x));
        else
          setPoseReference(waypoints_.back().x,
                           waypoints_.back().y,
                           1.9,
                           atan2(-waypoints_.back().y, waypoints_.back().x));
      }
      posRef << ref_x_, ref_y_ , ref_z_;
      // std::cout << "position ref: " << ref_x_<<' '<< ref_y_ <<' ' << ref_z_ << '\n';
      Eigen::Vector3d posError = R_SW * (posRef - x.t_WS);
      // std::cout << "pos. error: " << posError.norm() << '\n';
      if (!waypoints_.empty()) {
        std::lock_guard<std::mutex> l(waypointMutex_);
        if (startToGoal_ && posError.norm() < waypoints_[0].posTolerance) {
          waypoints_.pop_front();
          //lastPosTolerance_ = waypoints_[0].posTolerance;
        } else if (goalToStart_ && posError.norm() < waypoints_.back().posTolerance) {
          waypoints_.pop_back();
          //lastPosTolerance_ = waypoints_.back().posTolerance;
        }
      }
      if (waypoints_.empty()) {
        std::lock_guard<std::mutex> l(waypointMutex_);
        //if (posError.norm() <lastPosTolerance_) {
          m_objectReached = true;
        //}
      }
      //std::cout << "Waypoints empty: " << waypoints_.empty() << '\n';
      //std::cout << "last pos. tolerance: " << lastPosTolerance_ << '\n';
      //std::cout << "Object reached: " << m_objectReached << '\n';
      double yawError = (ref_yaw_ - yaw);
      // Ensure yawError \in [-pi,pi]
      yawError = std::fmod(yawError + M_PI , 2*M_PI);
      if (yawError < 0.0) yawError += 2*M_PI;
      yawError -= M_PI;
      // Calculate error derivatives
      Eigen::Vector3d dPosError = - R_SW * x.v_W;
      double dYawError = 0.0;
      // Set limits for controller output
      if(!nh_->getParam("/ardrone_driver/euler_angle_max",euler_angle_max_))
        std::cout << "Warning: Couldn't get controller boundary value: max angle\n";
      if(!nh_->getParam("/ardrone_driver/control_vz_max",control_vz_max_))
        std::cout << "Warning: Couldn't get controller boundary value: max velocity\n";
      if(!nh_->getParam("/ardrone_driver/control_yaw",control_yaw_max_))
        std::cout << "Warning: Couldn't get controller boundary value: max yaw\n";
      // Set controllers with these limits:
      rollAngPid.setOutputLimits(-euler_angle_max_,euler_angle_max_); // roll angle pid
      pitchAngPid.setOutputLimits(-euler_angle_max_,euler_angle_max_);
      yawPid.setOutputLimits(-control_yaw_max_,control_yaw_max_);
      control_vz_max_ /= 1000; // Convert from mm/s to m/s
      zPid.setOutputLimits(-control_vz_max_,control_vz_max_);
      // DONE: compute control output
      double u_y   = rollAngPid.control(timeMicroseconds,posError[1],dPosError[1]);
      double u_x   = pitchAngPid.control(timeMicroseconds,posError[0],dPosError[0]);
      double u_z   = zPid.control(timeMicroseconds,posError[2],dPosError[2]);
      double u_yaw = yawPid.control(timeMicroseconds,yawError,dYawError);
      // std::cout << "u_ x, y, z , w: " << u_x << ' ' << u_y << ' ' << u_z << ' ' << u_yaw  << '\n';
      // DONE: send to move
      if(!move(u_x,u_y,u_z,u_yaw)) std::cout << "Automatic movement: [FAIL]\n";
    }
  } // keep going
  else return; // else terminate
}

// Some functions for debugging
// Print reference values for debugging
void Autopilot::printRefVals(){
    std::cout << "X , Y , Z , yaw: " << ref_x_ << ' ' << ref_y_ << ' ' << ref_z_ << ' '
            << ref_yaw_ << '\n';
}

}  // namespace arp
