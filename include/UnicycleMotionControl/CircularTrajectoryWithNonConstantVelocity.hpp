#pragma once

#include <UnicycleMotionControl/UnicycleTrajectory.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {

class CircularTrajectoryWithNonConstantVelocity: public UnicycleTrajectory {
 public:
  CircularTrajectoryWithNonConstantVelocity(
      const labrob::Position2D& center,
      double radius,
      double omega_bar,
      double phi
  );

  labrob::Pose2D eval(double time) const override;
  labrob::Pose2DDerivative eval_dt(double time) const override;
  labrob::Pose2DSecondDerivative eval_ddt(double time) const override;

  double getDesiredDrivingVelocity(double time) const override;

  double getDuration() const override;

 protected:
  labrob::Position2D center_;
  double radius_;
  double omega_bar_;
  double phi_;
}; // end class CircularTrajectory

} // end namespace labrob