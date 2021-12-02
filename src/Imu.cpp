/*
 * Imu.cpp
 *
 *  Created on: 8 Feb 2017
 *      Author: sleutene
 */

#include <arp/kinematics/Imu.hpp>
#include <iostream>

namespace arp {
namespace kinematics {

// helper function that computes dt*f_c(.)
RobotState dx(const RobotState & x,
              const ImuMeasurement & z,
              const double dt)
{
  kinematics::Transformation T_WS(x.t_WS, x.q_WS);
  Eigen::Matrix3d R_WS = T_WS.R();

  Eigen::Vector4d helper;
  helper << z.omega_S - x.b_g, 0;

  Eigen::Vector3d g_W;
  g_W << 0, 0, -9.81;

  RobotState dx;
  dx.t_WS = dt * x.v_W;
  dx.q_WS = (dt/2) * kinematics::plus(x.q_WS) * helper;
  dx.v_W = dt * (R_WS * (z.acc_S - x.b_a) + g_W);
  dx.b_g = Eigen::Vector3d::Zero();
  dx.b_a = Eigen::Vector3d::Zero();

  return dx;
}

// helper function that computes Jacobian F_c
Eigen::Matrix<double,15,15> F_c(const RobotState & x,
                                const ImuMeasurement & z)
{
  kinematics::Transformation T_WS(x.t_WS, x.q_WS);
  Eigen::Matrix3d R_WS = T_WS.R();

  Eigen::Matrix<double,15,15> F_c = Eigen::Matrix<double,15,15>::Zero();
  F_c.block<3,3>(0,6) = Eigen::Matrix3d::Identity();
  F_c.block<3,3>(3,9) = -R_WS;
  F_c.block<3,3>(6,3) = -kinematics::crossMx(R_WS * (z.acc_S - x.b_a));
  F_c.block<3,3>(6,12) = -R_WS;

  return F_c;
}

bool Imu::stateTransition(const RobotState & state_k_minus_1,
                          const ImuMeasurement & z_k_minus_1,
                          const ImuMeasurement & z_k, RobotState & state_k,
                          ImuKinematicsJacobian* jacobian)
{
  // get the time delta
  const double dt = double(
      z_k.timestampMicroseconds - z_k_minus_1.timestampMicroseconds) * 1.0e-6;
  if (dt < 1.0e-12 || dt > 0.05) {
    // for safety, we assign reasonable values here.
    state_k = state_k_minus_1;
    if(jacobian) {
      jacobian->setIdentity();
    }
    return false;  // negative, no or too large time increments not permitted
  }

  // trapezoidal integration

  RobotState dx1 = dx(state_k_minus_1, z_k_minus_1, dt);

  RobotState x_h;
  x_h.t_WS = state_k_minus_1.t_WS + dx1.t_WS;
  x_h.q_WS.coeffs() = state_k_minus_1.q_WS.coeffs() + dx1.q_WS.coeffs();
  x_h.q_WS.normalize();
  x_h.v_W = state_k_minus_1.v_W + dx1.v_W;
  x_h.b_g = state_k_minus_1.b_g + dx1.b_g;
  x_h.b_a = state_k_minus_1.b_a + dx1.b_a;

  RobotState dx2 = dx(x_h, z_k, dt);

  state_k.t_WS = state_k_minus_1.t_WS + (dx1.t_WS + dx2.t_WS)/2;
  state_k.q_WS.coeffs() = state_k_minus_1.q_WS.coeffs() + (dx1.q_WS.coeffs() + dx2.q_WS.coeffs())/2;
  state_k.q_WS.normalize();
  state_k.v_W = state_k_minus_1.v_W + (dx1.v_W + dx2.v_W)/2;
  state_k.b_g = state_k_minus_1.b_g + (dx1.b_g + dx2.b_g)/2;
  state_k.b_a = state_k_minus_1.b_a + (dx1.b_a + dx2.b_a)/2;

  if (jacobian) {
    // jacobian of trapezoidal integration with chain rule
    Eigen::Matrix<double,15,15> F_c1 = F_c(state_k_minus_1, z_k_minus_1);
    Eigen::Matrix<double,15,15> F_c2 = F_c(x_h, z_k);
    Eigen::Matrix<double,15,15> I_15 = Eigen::Matrix<double,15,15>::Identity();
    *jacobian = I_15 + (dt/2)*F_c1 + (dt/2) * (F_c2 * (I_15 + dt*F_c1));
  }
  return true;
}

}
}  // namespace arp
