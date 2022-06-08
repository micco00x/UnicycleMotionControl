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
    std::filesystem::path unicycle_desired_pose_log_file_path = root_folder / "unicycle_desired_pose_log.txt";
    std::filesystem::path unicycle_measured_velocity_log_file_path = root_folder / "unicycle_measured_velocity_log.txt";
    std::filesystem::path unicycle_desired_velocity_log_file_path = root_folder / "unicycle_desired_velocity_log.txt";
    time_log_file_.open(time_log_file_path);
    unicycle_cmd_log_file_.open(unicycle_cmd_log_file_path);
    unicycle_configuration_log_file_.open(unicycle_configuration_log_file_path);
    unicycle_desired_pose_log_file_.open(unicycle_desired_pose_log_file_path);
    unicycle_measured_velocity_log_file_.open(unicycle_measured_velocity_log_file_path);
    unicycle_desired_velocity_log_file_.open(unicycle_desired_velocity_log_file_path);
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

  double desired_driving_velocity = 0.4;
  double square_length = 4.0;

  desired_trajectory_ptr_ =
      std::make_unique<labrob::SquaredTrajectoryWithConstantDrivingVelocity>(
          static_cast<double>(simGetSimulationTime()),
          labrob::Pose2D(1.0, 0.5, 0.0),
          desired_driving_velocity,
          square_length
      );

  static_feedback_linearization_hparams_.b = 0.75;
  static_feedback_linearization_hparams_.k1 = 1.0;
  static_feedback_linearization_hparams_.k2 = 1.0;
}

void
CoppeliaSimP3DXController::update() {
  double time = static_cast<double>(simGetSimulationTime());

  labrob::UnicycleConfiguration p3dx_configuration = retrieveP3DXCorrespondingUnicycle();

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
  if (unicycle_desired_pose_log_file_.is_open()) {
    labrob::Pose2D desired_pose = desired_trajectory_ptr_->eval(time);
    unicycle_desired_pose_log_file_
        << desired_pose.x() << " "
        << desired_pose.y() << " "
        << desired_pose.theta() << std::endl;
  }
  if (unicycle_measured_velocity_log_file_.is_open()) {
    labrob::Pose2DDerivative measured_velocity = retrieveP3DXVelocity();
    unicycle_measured_velocity_log_file_
        << measured_velocity.x_dot() << " "
        << measured_velocity.y_dot() << " "
        << measured_velocity.theta_dot() << std::endl;
  }
  if (unicycle_desired_velocity_log_file_.is_open()) {
    labrob::Pose2DDerivative desired_velocity = desired_trajectory_ptr_->eval_dt(time);
    unicycle_desired_velocity_log_file_
        << desired_velocity.x_dot() << " "
        << desired_velocity.y_dot() << " "
        << desired_velocity.theta_dot() << std::endl;
  }
}

labrob::UnicycleConfiguration
CoppeliaSimP3DXController::retrieveP3DXCorrespondingUnicycle() {
  simFloat p3dx_orientation[3];
  simGetObjectOrientation(p3dx_handle_, -1, p3dx_orientation);

  // NOTE: center of the P3DX is not the center of the unicycle: it is
  //       the center between the two wheels; the orientation is instead the
  //       same so it is safe to use the orientation of the robot.
  simFloat left_motor_position[3];
  simFloat right_motor_position[3];
  simGetObjectPosition(left_motor_handle_, -1, left_motor_position);
  simGetObjectPosition(right_motor_handle_, -1, right_motor_position);

  return labrob::UnicycleConfiguration(
      (left_motor_position[0] + right_motor_position[0]) / 2.0,
      (left_motor_position[1] + right_motor_position[1]) / 2.0,
      p3dx_orientation[2]
  );
}

labrob::Pose2DDerivative
CoppeliaSimP3DXController::retrieveP3DXVelocity() {
  // NOTE: assume angular velocity of unicycle is the same as the center
  //       of the robot.
  simFloat p3dx_linear_velocity[3];
  simFloat p3dx_angular_velocity[3];
  simGetObjectVelocity(p3dx_handle_, p3dx_linear_velocity, p3dx_angular_velocity);

  simFloat left_motor_linear_velocity[3];
  simFloat right_motor_linear_velocity[3];
  simGetObjectVelocity(left_motor_handle_, left_motor_linear_velocity, nullptr);
  simGetObjectVelocity(right_motor_handle_, right_motor_linear_velocity, nullptr);

  return labrob::Pose2DDerivative(
      (left_motor_linear_velocity[0] + right_motor_linear_velocity[0]) / 2.0,
      (left_motor_linear_velocity[1] + right_motor_linear_velocity[1]) / 2.0,
      p3dx_angular_velocity[2]
  );
}


} // end namespace labrob