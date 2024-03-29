#include <UnicycleMotionControl/EightShapedTrajectory.hpp>

#include <cmath>

namespace labrob {

EightShapedTrajectory::EightShapedTrajectory(
    const labrob::Pose2D& starting_pose,
    double desired_steering_velocity
) : starting_pose_(starting_pose),
    desired_steering_velocity_(desired_steering_velocity) {

}

labrob::Pose2D
EightShapedTrajectory::eval(double time) const {

  labrob::Pose2DDerivative qd_dot = eval_dt(time);

  double xd = starting_pose_.x() + R1_ * std::sin(desired_steering_velocity_ * time);
  double yd = starting_pose_.y() + R2_ + R2_ * std::cos(M_PI + 0.5 * desired_steering_velocity_ * time);
  // Use flat outputs:
  double thetad = std::atan2(qd_dot.y_dot(), qd_dot.x_dot());

  return labrob::Pose2D(xd, yd, thetad);
}

labrob::Pose2DDerivative
EightShapedTrajectory::eval_dt(double time) const {

  double xd_dot = R1_ * desired_steering_velocity_ * std::cos(desired_steering_velocity_ * time);
  double yd_dot = -0.5 * R2_ * desired_steering_velocity_ * std::sin(M_PI + 0.5 * desired_steering_velocity_ * time);

  double xd_ddot = -R1_ * std::pow(desired_steering_velocity_, 2.0) * std::sin(desired_steering_velocity_ * time);
  double yd_ddot = -0.25 * R2_ * std::pow(desired_steering_velocity_, 2.0) * std::cos(M_PI + 0.5 * desired_steering_velocity_ * time);

  // Use flat outputs:
  double thetad_dot = (yd_ddot * xd_dot - xd_ddot * yd_dot) / (std::pow(xd_dot, 2.0) + std::pow(yd_dot, 2.0));

  return Pose2DDerivative(xd_dot, yd_dot, thetad_dot);
}

labrob::Pose2DSecondDerivative
EightShapedTrajectory::eval_ddt(double time) const {

  double xd_ddot = -R1_ * std::pow(desired_steering_velocity_, 2.0) * std::sin(desired_steering_velocity_ * time);
  double yd_ddot = -0.25 * R2_ * std::pow(desired_steering_velocity_, 2.0) * std::cos(M_PI + 0.5 * desired_steering_velocity_ * time);

  // TODO: compute thetad_ddot
  double thetad_ddot = 0.0;

  return Pose2DSecondDerivative(xd_ddot, yd_ddot, thetad_ddot);
}

double
EightShapedTrajectory::getDesiredDrivingVelocity(double time) const {
  auto q_dot = eval_dt(time);
  // NOTE: safe to do this because assuming the robot is only moving forward.
  return std::sqrt(std::pow(q_dot.x_dot(), 2.0) + std::pow(q_dot.y_dot(), 2.0));
}

double
EightShapedTrajectory::getDuration() const {
  // NOTE: desired steering velocity in constant during the whole trajectory.
  return 4.0 * M_PI / desired_steering_velocity_;
}

} // end namespace labrob