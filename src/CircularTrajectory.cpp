#include <UnicycleMotionControl/CircularTrajectory.hpp>

#include <cmath>

namespace labrob {

CircularTrajectory::CircularTrajectory(
    const labrob::Position2D& center,
    double radius,
    double desired_driving_velocity,
    double phi
) : center_(center),
    radius_(radius),
    desired_driving_velocity_(desired_driving_velocity),
    desired_steering_velocity_(desired_driving_velocity / radius),
    phi_(phi) { }

labrob::Pose2D
CircularTrajectory::eval(double time) const {

  labrob::Pose2DDerivative qd_dot = eval_dt(time);

  double xd = center_.x() + radius_ * std::cos(phi_ + desired_steering_velocity_ * time);
  double yd = center_.y() + radius_ * std::sin(phi_ + desired_steering_velocity_ * time);
  // Use flat outputs:
  double thetad = std::atan2(qd_dot.y_dot(), qd_dot.x_dot());

  return labrob::Pose2D(xd, yd, thetad);
}

labrob::Pose2DDerivative
CircularTrajectory::eval_dt(double time) const {

  double xd_dot = -radius_ * std::sin(phi_ + desired_steering_velocity_ * time) * desired_steering_velocity_;
  double yd_dot =  radius_ * std::cos(phi_ + desired_steering_velocity_ * time) * desired_steering_velocity_;

  double xd_ddot = -radius_ * std::cos(phi_ + desired_steering_velocity_ * time) * std::pow(desired_steering_velocity_, 2.0);
  double yd_ddot = -radius_ * std::sin(phi_ + desired_steering_velocity_ * time) * std::pow(desired_steering_velocity_, 2.0);
  // Use flat outputs:
  double thetad_dot = (yd_ddot * xd_dot - xd_ddot * yd_dot) / (std::pow(xd_dot, 2.0) + std::pow(yd_dot, 2.0));

  return Pose2DDerivative(xd_dot, yd_dot, thetad_dot);
}

labrob::Pose2DSecondDerivative
CircularTrajectory::eval_ddt(double time) const {
  double xd_dot = -radius_ * std::sin(phi_ + desired_steering_velocity_ * time) * desired_steering_velocity_;
  double yd_dot =  radius_ * std::cos(phi_ + desired_steering_velocity_ * time) * desired_steering_velocity_;

  double xd_ddot = -radius_ * std::cos(phi_ + desired_steering_velocity_ * time) * std::pow(desired_steering_velocity_, 2.0);
  double yd_ddot = -radius_ * std::sin(phi_ + desired_steering_velocity_ * time) * std::pow(desired_steering_velocity_, 2.0);

  // NOTE: thetad_ddot is zero because thetad_dot is const.
  return Pose2DSecondDerivative(xd_ddot, yd_ddot, 0.0);
}

double
CircularTrajectory::getDesiredDrivingVelocity(double time) const {
  return desired_driving_velocity_;
}

} // end namespace labrob