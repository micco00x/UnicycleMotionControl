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
  double driving_velocity = unicycle_command.getDrivingVelocity();
  double steering_velocity = unicycle_command.getSteeringVelocity();
  left_motor_velocity_  = driving_velocity / dist_wheel_to_wheel_ - dist_wheel_to_wheel_ * steering_velocity / (2.0 * wheel_radius_);
  right_motor_velocity_ = driving_velocity / dist_wheel_to_wheel_ + dist_wheel_to_wheel_ * steering_velocity / (2.0 * wheel_radius_);
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
