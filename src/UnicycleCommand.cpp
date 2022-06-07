#include <UnicycleMotionControl/UnicycleCommand.hpp>

namespace labrob {

UnicycleCommand::UnicycleCommand()
  : driving_velocity_(0.0),
    steering_velocity_(0.0) {

}

UnicycleCommand::UnicycleCommand(
    double driving_velocity,
    double steering_velocity)
  : driving_velocity_(driving_velocity),
    steering_velocity_(steering_velocity) {

}

void
UnicycleCommand::setDrivingVelocity(double driving_velocity) {
  driving_velocity_ = driving_velocity;
}

void
UnicycleCommand::setSteeringVelocity(double steering_velocity) {
  steering_velocity_ = steering_velocity;
}

double
UnicycleCommand::getDrivingVelocity() const {
  return driving_velocity_;
}

double
UnicycleCommand::getSteeringVelocity() const {
  return steering_velocity_;
}

} // end namespace labrob