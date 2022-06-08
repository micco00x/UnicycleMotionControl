#include <UnicycleMotionControl/SquaredTrajectoryWithConstantDrivingVelocity.hpp>

#include <cmath>

namespace labrob {

SquaredTrajectoryWithConstantDrivingVelocity::SquaredTrajectoryWithConstantDrivingVelocity(
    double t0,
    const labrob::Pose2D& square_frame,
    double desired_driving_velocity,
    double square_length) {

  vd_ = desired_driving_velocity;

  double l = square_length;

  x0_ = square_frame.x();
  y0_ = square_frame.y();

  theta0_ = square_frame.theta();
  theta1_ = theta0_ + M_PI * 0.5;
  theta2_ = theta0_ + M_PI;
  theta3_ = theta0_ + M_PI * 1.5;

  x1_ = x0_ + l * std::cos(theta0_);
  y1_ = y0_ + l * std::sin(theta0_);
  x2_ = x1_ + l * std::cos(theta1_);
  y2_ = y1_ + l * std::sin(theta1_);
  x3_ = x2_ + l * std::cos(theta2_);
  y3_ = y2_ + l * std::sin(theta2_);

  t0_ = t0;
  t1_ = t0_ + l / vd_;
  double side_to_side_time = t1_ - t0_;
  t2_ = t1_ + side_to_side_time;
  t3_ = t2_ + side_to_side_time;
  t4_ = t3_ + side_to_side_time;
}

labrob::Pose2D
SquaredTrajectoryWithConstantDrivingVelocity::eval(double time) const {
  auto qd_dot = eval_dt(time);

  double xd;
  double yd;
  // Using flat outputs:
  double thetad = std::atan2(qd_dot.y_dot(), qd_dot.x_dot());

  if (time < t0_) {
    xd = x0_;
    yd = y0_;
  } else if (t0_ <= time && time < t1_) {
    xd = x0_ + vd_ * std::cos(theta0_) * (time - t0_);
    yd = y0_ + vd_ * std::sin(theta0_) * (time - t0_);
  } else if (t1_ <= time && time < t2_) {
    xd = x1_ + vd_ * std::cos(theta1_) * (time - t1_);
    yd = y1_ + vd_ * std::sin(theta1_) * (time - t1_);
  } else if (t2_ <= time && time < t3_) {
    xd = x2_ + vd_ * std::cos(theta2_) * (time - t2_);
    yd = y2_ + vd_ * std::sin(theta2_) * (time - t2_);
  } else if (t3_ <= time && time < t4_) {
    xd = x3_ + vd_ * std::cos(theta3_) * (time - t3_);
    yd = y3_ + vd_ * std::sin(theta3_) * (time - t3_);
  } else {
    xd = x0_;
    yd = y0_;
    thetad = theta3_;
  }

  return labrob::Pose2D(xd, yd, thetad);
}

labrob::Pose2DDerivative
SquaredTrajectoryWithConstantDrivingVelocity::eval_dt(double time) const {
  double xd_dot;
  double yd_dot;
  // Using flat outputs (NOTE: it is zero because \ddot{x}, \ddot{y} = 0):
  double thetad_dot = 0.0; // 

  if (time < t0_) {
    xd_dot = 0.0;
    yd_dot = 0.0;
  } else if (t0_ <= time && time < t1_) {
    xd_dot = vd_ * std::cos(theta0_);
    yd_dot = vd_ * std::sin(theta0_);
  } else if (t1_ <= time && time < t2_) {
    xd_dot = vd_ * std::cos(theta1_);
    yd_dot = vd_ * std::sin(theta1_);
  } else if (t2_ <= time && time < t3_) {
    xd_dot = vd_ * std::cos(theta2_);
    yd_dot = vd_ * std::sin(theta2_);
  } else if (t3_ <= time && time < t4_) {
    xd_dot = vd_ * std::cos(theta3_);
    yd_dot = vd_ * std::sin(theta3_);
  } else {
    xd_dot = 0.0;
    yd_dot = 0.0;
  }

  return labrob::Pose2DDerivative(xd_dot, yd_dot, thetad_dot);
}

} // end namespace labrob