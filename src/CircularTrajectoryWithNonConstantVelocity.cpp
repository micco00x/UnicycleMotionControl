#include <UnicycleMotionControl/CircularTrajectoryWithNonConstantVelocity.hpp>
#include <UnicycleMotionControl/types.hpp>

#include <cmath>
#include <iostream>

namespace labrob {

CircularTrajectoryWithNonConstantVelocity::CircularTrajectoryWithNonConstantVelocity(
    const labrob::Position2D& center,
    double radius,
    double omega_bar,
    double phi
) : center_(center),
    radius_(radius),
    omega_bar_(omega_bar),
    phi_(phi) { }

labrob::Pose2D
CircularTrajectoryWithNonConstantVelocity::eval(double time) const {
  labrob::Pose2DDerivative qd_dot = eval_dt(time);
  time += 0.001;

  double xd = center_.x() + radius_ * std::cos(phi_ + omega_bar_ * std::pow(time, 2.0));
  double yd = center_.y() + radius_ * std::sin(phi_ + omega_bar_ * std::pow(time, 2.0));
  // Use flat outputs:
  double thetad = std::atan2(qd_dot.y_dot(), qd_dot.x_dot());

  std::cerr << "xd=" << xd << ", yd=" << yd << "thetad=" << thetad << std::endl;

  return labrob::Pose2D(xd, yd, thetad);
}

labrob::Pose2DDerivative
CircularTrajectoryWithNonConstantVelocity::eval_dt(double time) const {
  time += 0.001;

  double xd_dot = -radius_ * std::sin(phi_ + omega_bar_ * std::pow(time, 2.0)) * (2.0 * omega_bar_ * time);
  double yd_dot =  radius_ * std::cos(phi_ + omega_bar_ * std::pow(time, 2.0)) * (2.0 * omega_bar_ * time);

  double xd_ddot = -radius_ * ( std::cos(phi_ + omega_bar_ * std::pow(time, 2.0)) * std::pow(2.0 * omega_bar_ * time, 2.0) + std::sin(phi_ + omega_bar_ * std::pow(time, 2.0)) * (2 * omega_bar_));
  double yd_ddot =  radius_ * (-std::sin(phi_ + omega_bar_ * std::pow(time, 2.0)) * std::pow(2.0 * omega_bar_ * time, 2.0) + std::cos(phi_ + omega_bar_ * std::pow(time, 2.0)) * (2 * omega_bar_));
  // Use flat outputs:
  double thetad_dot = (yd_ddot * xd_dot - xd_ddot * yd_dot) / (std::pow(xd_dot, 2.0) + std::pow(yd_dot, 2.0));

  std::cerr << "xddot=" << xd_dot << ", yddot=" << yd_dot << "thetaddot=" << thetad_dot << std::endl;
  std::cerr << "xdddot=" << xd_ddot << ", ydddot=" << yd_ddot << std::endl;


  return Pose2DDerivative(xd_dot, yd_dot, thetad_dot);
}

labrob::Pose2DSecondDerivative
CircularTrajectoryWithNonConstantVelocity::eval_ddt(double time) const {
  time += 0.001;

  double xd_dot = -radius_ * std::sin(phi_ + omega_bar_ * std::pow(time, 2.0)) * (2.0 * omega_bar_ * time);
  double yd_dot =  radius_ * std::cos(phi_ + omega_bar_ * std::pow(time, 2.0)) * (2.0 * omega_bar_ * time);

  double xd_ddot = -radius_ * ( std::cos(phi_ + omega_bar_ * std::pow(time, 2.0)) * std::pow(2.0 * omega_bar_ * time, 2.0) + std::sin(phi_ + omega_bar_ * std::pow(time, 2.0)) * (2 * omega_bar_));
  double yd_ddot =  radius_ * (-std::sin(phi_ + omega_bar_ * std::pow(time, 2.0)) * std::pow(2.0 * omega_bar_ * time, 2.0) + std::cos(phi_ + omega_bar_ * std::pow(time, 2.0)) * (2 * omega_bar_));

  // WARNING: thetad_ddot needed in DFL.
  return Pose2DSecondDerivative(xd_ddot, yd_ddot, 0.0);
}

double
CircularTrajectoryWithNonConstantVelocity::getDesiredDrivingVelocity(double time) const {
  labrob::Pose2DDerivative qd_dot = eval_dt(time);
  double xd_dot = qd_dot.x_dot();
  double yd_dot = qd_dot.y_dot();
  return std::sqrt(std::pow(xd_dot, 2.0) + std::pow(yd_dot, 2.0));
}

double CircularTrajectoryWithNonConstantVelocity::getDuration() const {
  return 100.0;
}

} // end namespace labrob