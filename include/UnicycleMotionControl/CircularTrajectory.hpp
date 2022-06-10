#pragma once

#include <UnicycleMotionControl/UnicycleTrajectory.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {

class CircularTrajectory : public UnicycleTrajectory {
 public:
  CircularTrajectory(
      const labrob::Position2D& center,
      double radius,
      double desired_driving_velocity,
      double phi
  );

  labrob::Pose2D eval(double time) const override;
  labrob::Pose2DDerivative eval_dt(double time) const override;
  labrob::Pose2DSecondDerivative eval_ddt(double time) const override;

  double getDesiredDrivingVelocity(double time) const override;

 protected:
  labrob::Position2D center_;
  double radius_;
  double desired_driving_velocity_;
  double desired_steering_velocity_;
  double phi_;
}; // end class CircularTrajectory

} // end namespace labrob