#pragma once

#include <UnicycleMotionControl/UnicycleTrajectory.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {

class LinearTrajectoryWithConstantDrivingVelocity : public UnicycleTrajectory {
 public:
  LinearTrajectoryWithConstantDrivingVelocity(
      const labrob::Pose2D& starting_pose,
      double desired_driving_velocity
  );

  labrob::Pose2D eval(double time) const override;
  labrob::Pose2DDerivative eval_dt(double time) const override;
  labrob::Pose2DSecondDerivative eval_ddt(double time) const override;

  double getDesiredDrivingVelocity(double time) const override;

 protected:
  labrob::Pose2D starting_pose_;
  double desired_driving_velocity_;
}; // end class LinearTrajectoryWithConstantDrivingVelocity

} // end namespace labrob