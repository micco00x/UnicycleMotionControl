#pragma once

namespace labrob {

class UnicycleCommand {
 public:
  UnicycleCommand();
  UnicycleCommand(double driving_velocity, double steering_velocity);


  void setDrivingVelocity(double driving_velocity);
  void setSteeringVelocity(double steering_velocity);

  double getDrivingVelocity() const;
  double getSteeringVelocity() const;

 protected:
  double driving_velocity_;
  double steering_velocity_;

}; // end class UnicycleCommand

} // end namespace labrob