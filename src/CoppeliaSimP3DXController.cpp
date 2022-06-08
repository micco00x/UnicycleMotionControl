#include <UnicycleMotionControl/CoppeliaSimP3DXController.hpp>
#include <UnicycleMotionControl/SquaredTrajectoryWithConstantDrivingVelocity.hpp>
#include <UnicycleMotionControl/UnicycleConfiguration.hpp>
#include <UnicycleMotionControl/unicycle_controllers.hpp>

#include <filesystem>
#include <iostream>


namespace labrob {

CoppeliaSimP3DXController::CoppeliaSimP3DXController()
  : p3dx_robot_cmd_(p3dx_wheel_radius_, p3dx_dist_wheel_to_wheel_) {

  std::filesystem::path root_folder = "/tmp/UnicycleMotionControl";

  std::filesystem::create_directories(root_folder);


  if (std::filesystem::exists(root_folder)) {
    std::filesystem::path time_log_file_path = root_folder / "time_log.txt";
    std::filesystem::path unicycle_cmd_log_file_path = root_folder / "unicycle_cmd_log.txt";
    std::filesystem::path unicycle_configuration_log_file_path = root_folder / "unicycle_configuration_log.txt";
    std::filesystem::path unicycle_desired_position_log_file_path = root_folder / "unicycle_desired_position_log.txt";
    time_log_file_.open(time_log_file_path);
    unicycle_cmd_log_file_.open(unicycle_cmd_log_file_path);
    unicycle_configuration_log_file_.open(unicycle_configuration_log_file_path);
    unicycle_desired_position_log_file_.open(unicycle_desired_position_log_file_path);
  } else {
    std::cerr << "Cannot create directory " << root_folder << std::endl;
  }

}

void
CoppeliaSimP3DXController::init() {
  // Retrieve handles:
  const simChar* p3dx_object_path = "/PioneerP3DX";
  const simChar* left_motor_object_path = "/PioneerP3DX/leftMotor";
  const simChar* right_motor_object_path = "/PioneerP3DX/rightMotor";

  p3dx_handle_ = simGetObject(p3dx_object_path, -1, -1, 0);
  left_motor_handle_ = simGetObject(left_motor_object_path, -1, -1, 0);
  right_motor_handle_ = simGetObject(right_motor_object_path, -1, -1, 0);

  if (p3dx_handle_ == -1) {
    std::cerr << "Could not get object with object path " << p3dx_object_path << std::endl;
  }

  if (left_motor_handle_ == -1) {
    std::cerr << "Could not get object with object path " << left_motor_object_path << std::endl;
  }

  if (right_motor_handle_ == -1) {
    std::cerr << "Could not get object with object path " << right_motor_object_path << std::endl;
  }

  // Setup data for liner controller using input-outpu linearization via static feedback:
  labrob::UnicycleConfiguration initial_configuration = retrieveP3DXConfiguration();

  double desired_driving_velocity = 1.0;
  double square_length = 4.0;

  desired_trajectory_ptr_ =
      std::make_unique<labrob::SquaredTrajectoryWithConstantDrivingVelocity>(
          static_cast<double>(simGetSimulationTime()),
          initial_configuration,
          desired_driving_velocity,
          square_length
      );

  static_feedback_linearization_hparams_.b = 0.75;
  static_feedback_linearization_hparams_.k1 = 2.0;
  static_feedback_linearization_hparams_.k2 = 2.0;
}

void
CoppeliaSimP3DXController::update() {
  double time = static_cast<double>(simGetSimulationTime());

  labrob::UnicycleConfiguration p3dx_configuration = retrieveP3DXConfiguration();

  labrob::UnicycleCommand unicycle_cmd;

  labrob::staticFeedbackLinearizationControl(
      time,
      static_feedback_linearization_hparams_,
      p3dx_configuration,
      *desired_trajectory_ptr_,
      unicycle_cmd
  );

  p3dx_robot_cmd_.setVelocitiesFromUnicycleCommand(unicycle_cmd);

  // Send commands to robot:
  simSetJointTargetVelocity(left_motor_handle_, p3dx_robot_cmd_.getLeftMotorVelocity());
  simSetJointTargetVelocity(right_motor_handle_, p3dx_robot_cmd_.getRightMotorVelocity());

  // Log:
  if (time_log_file_.is_open()) {
    time_log_file_ << time << std::endl;
  }
  if (unicycle_cmd_log_file_.is_open()) {
    unicycle_cmd_log_file_
        << unicycle_cmd.getDrivingVelocity() << " "
        << unicycle_cmd.getSteeringVelocity() << std::endl;
  }
  if (unicycle_configuration_log_file_.is_open()) {
    unicycle_configuration_log_file_
        << p3dx_configuration.x() << " "
        << p3dx_configuration.y() << " "
        << p3dx_configuration.theta() << std::endl;
  }
  if (unicycle_desired_position_log_file_.is_open()) {
    labrob::Position2D desired_position = desired_trajectory_ptr_->eval(time);
    unicycle_desired_position_log_file_
        << desired_position.x() << " "
        << desired_position.y() << std::endl;
  }
}

labrob::UnicycleConfiguration
CoppeliaSimP3DXController::retrieveP3DXConfiguration() {
  simFloat p3dx_position[3];
  simFloat p3dx_orientation[3];
  simGetObjectPosition(p3dx_handle_, -1, p3dx_position);
  simGetObjectOrientation(p3dx_handle_, -1, p3dx_orientation);

  return labrob::UnicycleConfiguration(
      p3dx_position[0], p3dx_position[1], p3dx_orientation[2]
  );
}


} // end namespace labrob