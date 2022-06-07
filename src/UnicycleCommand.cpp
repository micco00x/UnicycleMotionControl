#include <UnicycleMotionControl/UnicycleCommand.hpp>

namespace labrob {

UnicycleCommand::UnicycleCommand()
  : linear_velocity_(0.0),
    angular_velocity_(0.0) {

}

UnicycleCommand::UnicycleCommand(
    double linear_velocity,
    double angular_velocity)
  : linear_velocity_(linear_velocity),
    angular_velocity_(angular_velocity) {

}

void
UnicycleCommand::setLinearVelocity(double linear_velocity) {
  linear_velocity_ = linear_velocity;
}

void
UnicycleCommand::setAngularVelocity(double angular_velocity) {
  angular_velocity_ = angular_velocity;
}

double
UnicycleCommand::getLinearVelocity() const {
  return linear_velocity_;
}

double
UnicycleCommand::getAngularVelocity() const {
  return angular_velocity_;
}

} // end namespace labrob