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
    duration_(2.0 * M_PI * radius / desired_driving_velocity),
    phi_(phi) { }

labrob::Pose2D
CircularTrajectory::eval(double time) const {

  labrob::Pose2DDerivative qd_dot = eval_dt(time);

  double xd = center_.x() + radius_ * std::cos(phi_ + time / duration_);
  double yd = center_.y() + radius_ * std::sin(phi_ + time / duration_);
  // Use flat outputs:
  double thetad = std::atan2(qd_dot.y_dot(), qd_dot.x_dot());

  return labrob::Pose2D(xd, yd, thetad);
}

labrob::Pose2DDerivative
CircularTrajectory::eval_dt(double time) const {

  double xd_dot = -radius_ * std::sin(phi_ + time / duration_) / duration_;
  double yd_dot =  radius_ * std::cos(phi_ + time / duration_) / duration_;

  double xd_ddot = -radius_ * std::cos(phi_ + time / duration_) / std::pow(duration_, 2.0);
  double yd_ddot = -radius_ * std::sin(phi_ + time / duration_) / std::pow(duration_, 2.0);
  // Use flat outputs:
  double thetad_dot = (yd_ddot * xd_dot - xd_ddot * yd_dot) / (xd_dot + yd_dot);

  return Pose2DDerivative(xd_dot, yd_dot, thetad_dot);
}

} // end namespace labrob