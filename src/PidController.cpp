/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <stdexcept>
#include <iostream>

namespace arp {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  // TODO: implement...
  // Calculate deltaT and update last timestamp
  uint64_t deltaT = (timestampMicroseconds - lastTimestampMicroseconds_);
  // Limit deltaT
  if (deltaT > maxDeltaT) {
      deltaT = maxDeltaT;
  }
  // Update time stamp
  lastTimestampMicroseconds_ = timestampMicroseconds;
  // Controller output
  //std::cout << "error: "<< e << " edot: " << e_dot <<'\n';
//  std::cout << "miO, maO, p, i, d, eint: "<< minOutput_ << ' ' << maxOutput_ << ' ' << parameters_.k_p << ' '
//      << parameters_.k_i << ' ' << parameters_.k_d << ' ' << integratedError_ <<'\n';
  double output = parameters_.k_p * e
                + parameters_.k_i * integratedError_
                + parameters_.k_d * e_dot;
  // Anti-reset windup and int. err. update
  if (output < minOutput_) {
      output = minOutput_;
  } else if (output > maxOutput_) {
      output = maxOutput_;
  } else {
      integratedError_ += e * (double)deltaT * 1e-6;
  }
  //std::cout << "output is: " << output << '\n';
  if(maxOutput_==minOutput_) std::cout << "Invalid output boundaries. \n";
  //TODO: I do not understand why we even care about minOutput_ and maxOutput.
  // I can always output a value normalized between [-1,1] without knowing them.
  // Since we use Autopilot::move(...), that is the only important thing...
  double controllergain{.1}; // Set 2.0 to allow full gas. Warning: goes crazy fast.
  //TODO: Ask - How do you find this strategy?

  if (e > 1.0 || e < -1.0) controllergain = 2;
  else if (e > 0.1 || e < -0.1) controllergain = 0.1;
  else if (e > 0.01 || e < -0.01) controllergain = 0.01;
  return controllergain * output / (maxOutput_ - minOutput_) ;
}

// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
