#pragma once

#include <UnicycleMotionControl/UnicycleTrajectory.hpp>
#include <UnicycleMotionControl/UnicycleConfiguration.hpp>

namespace labrob {

class SquaredTrajectoryWithConstantDrivingVelocity : public UnicycleTrajectory {
 public:
  SquaredTrajectoryWithConstantDrivingVelocity(
      double t0,
      const labrob::Pose2D& square_frame,
      double desired_driving_velocity,
      double square_length
  );

  labrob::Pose2D eval(double time) const override;
  labrob::Pose2DDerivative eval_dt(double time) const override;
  labrob::Pose2DSecondDerivative eval_ddt(double time) const override;

  double getDesiredDrivingVelocity(double time) const override;

  double getDuration() const override;

 protected:
  double vd_;
  double x0_, y0_, theta0_;
  double x1_, y1_, theta1_;
  double x2_, y2_, theta2_;
  double x3_, y3_, theta3_;
  double t0_, t1_, t2_, t3_, t4_;
};

} // end namespace labrob