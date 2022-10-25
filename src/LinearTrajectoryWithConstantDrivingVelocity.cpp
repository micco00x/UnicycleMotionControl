#include <UnicycleMotionControl/LinearTrajectoryWithConstantDrivingVelocity.hpp>

#include <cmath>

namespace labrob {

LinearTrajectoryWithConstantDrivingVelocity::LinearTrajectoryWithConstantDrivingVelocity(
      const labrob::Pose2D& starting_pose,
      double desired_driving_velocity
) : starting_pose_(starting_pose),
    desired_driving_velocity_(desired_driving_velocity) {

}

labrob::Pose2D
LinearTrajectoryWithConstantDrivingVelocity::eval(double time) const {

  labrob::Pose2DDerivative qd_dot = eval_dt(time);

  double xd = starting_pose_.x() + desired_driving_velocity_ * std::cos(starting_pose_.theta()) * time;
  double yd = starting_pose_.y() + desired_driving_velocity_ * std::sin(starting_pose_.theta()) * time;

  double xd_dot = desired_driving_velocity_ * std::cos(starting_pose_.theta());
  double yd_dot = desired_driving_velocity_ * std::sin(starting_pose_.theta());
  // Use flat outputs:
  double thetad = std::atan2(yd_dot, xd_dot);

  return labrob::Pose2D(xd, yd, thetad);
}

labrob::Pose2DDerivative
LinearTrajectoryWithConstantDrivingVelocity::eval_dt(double time) const {

  double xd_dot = desired_driving_velocity_ * std::cos(starting_pose_.theta());
  double yd_dot = desired_driving_velocity_ * std::sin(starting_pose_.theta());

  double xd_ddot = 0.0;
  double yd_ddot = 0.0;
  // Use flat outputs:
  double thetad_dot = (yd_ddot * xd_dot - xd_ddot * yd_dot) / (std::pow(xd_dot, 2.0) + std::pow(yd_dot, 2.0));

  return Pose2DDerivative(xd_dot, yd_dot, thetad_dot);
}

labrob::Pose2DSecondDerivative
LinearTrajectoryWithConstantDrivingVelocity::eval_ddt(double time) const {
  return Pose2DSecondDerivative(0.0, 0.0, 0.0);
}

double
LinearTrajectoryWithConstantDrivingVelocity::getDesiredDrivingVelocity(double time) const {
  return desired_driving_velocity_;
}

double
LinearTrajectoryWithConstantDrivingVelocity::getDuration() const {
  return 10.0;
}


} // end namespace labrob