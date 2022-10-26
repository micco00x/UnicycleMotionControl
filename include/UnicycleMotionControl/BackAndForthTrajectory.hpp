#pragma once

#include <UnicycleMotionControl/UnicycleTrajectory.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {

class BackAndForthTrajectory : public UnicycleTrajectory {
 public:
  BackAndForthTrajectory(
      const labrob::Position2D& starting_position,
      double length,
      double desired_driving_velocity
  );

  labrob::Pose2D eval(double time) const override;
  labrob::Pose2DDerivative eval_dt(double time) const override;
  labrob::Pose2DSecondDerivative eval_ddt(double time) const override;

  double getDesiredDrivingVelocity(double time) const override;

  double getDuration() const override;

 protected:
  labrob::Position2D starting_position_;
  double length_;
  double desired_driving_velocity_;
}; // end class BackAndForthTrajectory

} // end namespace labrob