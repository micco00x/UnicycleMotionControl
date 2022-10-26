#include <cmath>

#include <UnicycleMotionControl/BackAndForthTrajectory.hpp>
#include <UnicycleMotionControl/UnicycleTrajectory.hpp>
#include <UnicycleMotionControl/types.hpp>

namespace labrob {


BackAndForthTrajectory::BackAndForthTrajectory(
    const labrob::Position2D& starting_position,
    double length,
    double desired_driving_velocity
) : starting_position_(starting_position),
    length_(length),
    desired_driving_velocity_(desired_driving_velocity) { }

labrob::Pose2D
BackAndForthTrajectory::eval(double time) const {
  
  labrob::Pose2DDerivative qd_dot = eval_dt(time);

  double xd, yd = starting_position_.y(), thetad = 0.0;
  double z = desired_driving_velocity_ * time;

  double T = length_ / desired_driving_velocity_;

  int m = static_cast<int>(z / length_);
  if (m % 2 == 0) {
    // Moving forward
    xd = starting_position_.x() + desired_driving_velocity_ * (time - T * m);
  } else {
    // Moving backward
    xd = starting_position_.x() + length_ - desired_driving_velocity_ * (time - T * m);
  }

  return labrob::Pose2D(xd, yd, thetad);
}

labrob::Pose2DDerivative
BackAndForthTrajectory::eval_dt(double time) const {
  double xd_dot;
  double yd_dot = 0.0;
  double thetad_dot = 0.0;

  double z = desired_driving_velocity_ * time;
  int m = static_cast<int>(z / length_);
  if (m % 2 == 0) {
    // Moving forward
    xd_dot = desired_driving_velocity_;
  } else {
    // Moving backward
    xd_dot = -desired_driving_velocity_;
  }

  return Pose2DDerivative(xd_dot, yd_dot, thetad_dot);
}

labrob::Pose2DSecondDerivative
BackAndForthTrajectory::eval_ddt(double time) const {
  return Pose2DSecondDerivative(0.0, 0.0, 0.0);
}

double
BackAndForthTrajectory::getDesiredDrivingVelocity(double time) const {
  double xd_dot;
  double z = desired_driving_velocity_ * time;
  int m = static_cast<int>(z / length_);
  if (m % 2 == 0) {
    // Moving forward
    xd_dot = desired_driving_velocity_;
  } else {
    // Moving backward
    xd_dot = -desired_driving_velocity_;
  }
  return xd_dot;
}

double
BackAndForthTrajectory::getDuration() const {
  return 20.0;
}



} // end namespace labrob