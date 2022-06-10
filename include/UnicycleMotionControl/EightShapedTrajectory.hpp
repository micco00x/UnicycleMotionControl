#pragma once

#include <UnicycleMotionControl/UnicycleTrajectory.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {

class EightShapedTrajectory : public UnicycleTrajectory {
 public:
  EightShapedTrajectory(
      const labrob::Pose2D& starting_pose,
      double desired_steering_velocity
  );

  labrob::Pose2D eval(double time) const override;
  labrob::Pose2DDerivative eval_dt(double time) const override;
  labrob::Pose2DSecondDerivative eval_ddt(double time) const override;

  double getDesiredDrivingVelocity(double time) const override;

 protected:
  const double R1_ = 3.0;
  const double R2_ = 3.0;

  labrob::Pose2D starting_pose_;
  double desired_steering_velocity_;
}; // end class CircularTrajectory

} // end namespace labrob