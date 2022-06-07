#include <UnicycleMotionControl/CoppeliaSimP3DXController.hpp>
#include <UnicycleMotionControl/UnicycleConfiguration.hpp>

#include <iostream>


namespace labrob {

CoppeliaSimP3DXController::CoppeliaSimP3DXController()
  : p3dx_robot_cmd_(p3dx_wheel_radius_, p3dx_dist_wheel_to_wheel_) {

}

void
CoppeliaSimP3DXController::init() {

}

void
CoppeliaSimP3DXController::update() {
  const simChar* p3dx_object_path = "/PioneerP3DX";
  const simChar* left_motor_object_path = "/PioneerP3DX/leftMotor";
  const simChar* right_motor_object_path = "/PioneerP3DX/rightMotor";

  simInt p3dx_handle = simGetObject(p3dx_object_path, -1, -1, 0);
  simInt left_motor_handle = simGetObject(left_motor_object_path, -1, -1, 0);
  simInt right_motor_handle = simGetObject(right_motor_object_path, -1, -1, 0);

  if (p3dx_handle == -1) {
    std::cerr << "Could not get object with object path " << p3dx_handle << std::endl;
  }

  if (left_motor_handle == -1) {
    std::cerr << "Could not get object with object path " << left_motor_object_path << std::endl;
  }

  if (right_motor_handle == -1) {
    std::cerr << "Could not get object with object path " << right_motor_object_path << std::endl;
  }

  // Retrieve configuration of P3DX:
  simFloat p3dx_position[3];
  simFloat p3dx_orientation[3];
  simGetObjectPosition(p3dx_handle, -1, p3dx_position);
  simGetObjectOrientation(p3dx_handle, -1, p3dx_orientation);

  labrob::UnicycleConfiguration p3dx_configuration(
      p3dx_position[0], p3dx_position[1], p3dx_orientation[2]
  );

  std::cerr << "P3DX at ("
      << p3dx_configuration.getX() << ", "
      << p3dx_configuration.getY() << ", "
      << p3dx_configuration.getTheta() << std::endl;

  double driving_velocity = 0.1;
  double steering_velocity = 0.0;
  labrob::UnicycleCommand unicycle_cmd(driving_velocity, steering_velocity);

  p3dx_robot_cmd_.setVelocitiesFromUnicycleCommand(unicycle_cmd);

  // Send commands to robot:
  simSetJointTargetVelocity(left_motor_handle, p3dx_robot_cmd_.getLeftMotorVelocity());
  simSetJointTargetVelocity(right_motor_handle, p3dx_robot_cmd_.getRightMotorVelocity());
}

} // end namespace labrob