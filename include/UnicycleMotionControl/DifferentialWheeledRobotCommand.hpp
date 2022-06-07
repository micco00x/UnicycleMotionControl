#pragma once

namespace labrob {

class DifferentialWheeledRobotCommand {
 public:
  DifferentialWheeledRobotCommand(double wheel_radius, double dist_wheel_to_wheel);

  void setLeftMotorVelocity(double left_motor_velocity);
  void setRightMotorVelocity(double right_motor_velocity);

  void setVelocitiesFromUnicycleCommand(
      double linear_velocity,
      double angular_velocity
  );

  double getLeftMotorVelocity() const;
  double getRightMotorVelocity() const;

 protected:
  const double wheel_radius_;
  const double dist_wheel_to_wheel_;

  double left_motor_velocity_;
  double right_motor_velocity_;

}; // end class DifferentialWheeledRobotCommand

} // end namespace labrob