#pragma once

#include <UnicycleMotionControl/UnicycleConfiguration.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {

class UnicycleTrajectory {
 public:
  virtual labrob::Position2D eval(double time) const = 0;
  virtual labrob::Velocity2D eval_dt(double time) const = 0;

}; // end class UnicycleTrajectory

} // end namespace labrob