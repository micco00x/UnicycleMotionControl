#include <UnicycleMotionControl/DifferentialWheeledRobotCommand.hpp>

namespace labrob {

DifferentialWheeledRobotCommand::DifferentialWheeledRobotCommand(
    double wheel_radius,
    double dist_wheel_to_wheel)
  : wheel_radius_(wheel_radius),
    dist_wheel_to_wheel_(dist_wheel_to_wheel),
    left_motor_velocity_(0.0),
    right_motor_velocity_(0.0) {

}

void
DifferentialWheeledRobotCommand::setLeftMotorVelocity(double left_motor_velocity) {
  left_motor_velocity_ = left_motor_velocity;
}

void
DifferentialWheeledRobotCommand::setRightMotorVelocity(double right_motor_velocity) {
  right_motor_velocity_ = right_motor_velocity;
}

void
DifferentialWheeledRobotCommand::setVelocitiesFromUnicycleCommand(
    labrob::UnicycleCommand unicycle_command) {
  double linear_velocity = unicycle_command.getLinearVelocity();
  double angular_velocity = unicycle_command.getAngularVelocity();
  left_motor_velocity_  = linear_velocity / dist_wheel_to_wheel_ - dist_wheel_to_wheel_ * angular_velocity / (2.0 * wheel_radius_);
  right_motor_velocity_ = linear_velocity / dist_wheel_to_wheel_ + dist_wheel_to_wheel_ * angular_velocity / (2.0 * wheel_radius_);
}

double
DifferentialWheeledRobotCommand::getLeftMotorVelocity() const {
  return left_motor_velocity_;
}

double
DifferentialWheeledRobotCommand::getRightMotorVelocity() const {
  return right_motor_velocity_;
}

} // end namespace labrob
