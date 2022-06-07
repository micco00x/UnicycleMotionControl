#pragma once

namespace labrob {

class UnicycleCommand {
 public:
  UnicycleCommand();
  UnicycleCommand(double linar_velocity, double angular_velocity);


  void setLinearVelocity(double linear_velocity);
  void setAngularVelocity(double angular_velocity);

  double getLinearVelocity() const;
  double getAngularVelocity() const;

 protected:
  double linear_velocity_;
  double angular_velocity_;

}; // end class UnicycleCommand

} // end namespace labrob