#pragma once

#include <UnicycleMotionControl/UnicycleConfiguration.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {

class UnicycleTrajectory {
 public:
  virtual labrob::Pose2D eval(double time) const = 0;
  virtual labrob::Pose2DDerivative eval_dt(double time) const = 0;
  virtual labrob::Pose2DSecondDerivative eval_ddt(double time) const = 0;

}; // end class UnicycleTrajectory

} // end namespace labrob