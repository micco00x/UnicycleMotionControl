#pragma once

#include <UnicycleMotionControl/UnicycleTrajectory.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {

class EightShapedTrajectory : public UnicycleTrajectory {
 public:
  EightShapedTrajectory(double desired_steering_velocity);

  labrob::Pose2D eval(double time) const override;
  labrob::Pose2DDerivative eval_dt(double time) const override;

 protected:
  const double R1_ = 3.0;
  const double R2_ = 3.0;

  double desired_steering_velocity_;
}; // end class CircularTrajectory

} // end namespace labrob