#include <UnicycleMotionControl/CoppeliaSimP3DXController.hpp>
#include <UnicycleMotionControl/CircularTrajectory.hpp>
#include <UnicycleMotionControl/EightShapedTrajectory.hpp>
#include <UnicycleMotionControl/LinearTrajectoryWithConstantDrivingVelocity.hpp>
#include <UnicycleMotionControl/SquaredTrajectoryWithConstantDrivingVelocity.hpp>
#include <UnicycleMotionControl/UnicycleConfiguration.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>


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
  // Enable/disable dynamics:
  dynamics_enabled_ = false;
  simSetBooleanParameter(
      sim_boolparam_dynamics_handling_enabled,
      dynamics_enabled_
  );

  // Retrieve handles:
  const simChar* p3dx_object_path = "/PioneerP3DX";
  const simChar* p3dx_unicycle_object_path = "/PioneerP3DX/unicycle_rf";
  const simChar* left_motor_object_path = "/PioneerP3DX/leftMotor";
  const simChar* right_motor_object_path = "/PioneerP3DX/rightMotor";

  p3dx_handle_ = simGetObject(p3dx_object_path, -1, -1, 0);
  p3dx_unicycle_handle_ = simGetObject(p3dx_unicycle_object_path, -1, -1, 0);
  left_motor_handle_ = simGetObject(left_motor_object_path, -1, -1, 0);
  right_motor_handle_ = simGetObject(right_motor_object_path, -1, -1, 0);

  if (p3dx_unicycle_handle_ == -1) {
    std::cerr << "Could not get object with object path " << p3dx_unicycle_object_path << std::endl;
  }

  if (left_motor_handle_ == -1) {
    std::cerr << "Could not get object with object path " << left_motor_object_path << std::endl;
  }

  if (right_motor_handle_ == -1) {
    std::cerr << "Could not get object with object path " << right_motor_object_path << std::endl;
  }

  // Init const z (used only in kinematic simulation):
  simFloat p3dx_unicycle_position[3];
  simGetObjectPosition(p3dx_unicycle_handle_, -1, p3dx_unicycle_position);
  z_unicycle_ = static_cast<double>(p3dx_unicycle_position[2]);

  TrajectoryType trajectory_type = TrajectoryType::Squared;
  desired_trajectory_ptr_ = generateDesiredTrajectory(trajectory_type);

  controller_type_ = ControllerType::StaticFeedbackLinearization;

  if (controller_type_ == ControllerType::ApproximateLinearization) {
    // Setup hparams for approximate linearization and linear controller:
    labrob::ApproximateLinearizationHparams approximate_linearization_hparams;
    approximate_linearization_hparams.zeta = 0.7;
    approximate_linearization_hparams.a = 1.0;
    approximate_linearization_controller_ptr_=
        std::make_unique<labrob::ApproximateLinearizationController>(
            approximate_linearization_hparams
        );
  } else if (controller_type_ == ControllerType::DynamicFeedbackLinearization) {
    // Setup hparams for dynamic feedback linearization and PD controller:
    labrob::DynamicFeedbackLinearizationHparams dynamic_feedback_linearization_hparams;
    dynamic_feedback_linearization_hparams.kp1 = 4.0;
    dynamic_feedback_linearization_hparams.kp2 = 4.0;
    dynamic_feedback_linearization_hparams.kd1 = 4.0;
    dynamic_feedback_linearization_hparams.kd2 = 4.0;
    double xi_0 = 1.0;
    dynamic_feedback_linearization_controller_ptr_ =
        std::make_unique<labrob::DynamicFeedbackLinearizationController>(
            dynamic_feedback_linearization_hparams,
            simGetSimulationTimeStep(),
            xi_0
        );
  } else if (controller_type_ == ControllerType::StaticFeedbackLinearization) {
    // Setup hparams for static feedback linearization and linear controller:
    labrob::StaticFeedbackLinearizationHParams static_feedback_linearization_hparams;
    static_feedback_linearization_hparams.b = 0.75;
    static_feedback_linearization_hparams.k1 = 2.0;
    static_feedback_linearization_hparams.k2 = 2.0;
    static_feedback_linearization_controller_ptr_ =
        std::make_unique<labrob::StaticFeedbackLinearizationController>(
            static_feedback_linearization_hparams
        );
  }

  // Add drawing objects to show during simulation:
  simFloat drawing_point_size = 2.0f;
  simFloat black_color[3] = {0.0f, 0.0f, 0.0f};
  simFloat green_color[3] = {0.0f, 1.0f, 0.0f};
  simFloat red_color[3]   = {1.0f, 0.0f, 0.0f};
  p3dx_unicycle_drawing_object_handle_= simAddDrawingObject(
      sim_drawing_points,
      drawing_point_size, 0.0f, -1, std::numeric_limits<simInt>::max(),
      black_color, nullptr, nullptr, nullptr
  );
  desired_trajectory_drawing_object_handle_ = simAddDrawingObject(
      sim_drawing_points,
      drawing_point_size, 0.0f, -1, std::numeric_limits<simInt>::max(),
      red_color, nullptr, nullptr, nullptr
  );
}

void
CoppeliaSimP3DXController::update() {
  double time = static_cast<double>(simGetSimulationTime());

  labrob::UnicycleConfiguration unicycle_configuration = retrieveP3DXUnicycleConfiguration();
  labrob::Pose2DDerivative unicycle_velocity = retrieveP3DXUnicycleVelocity();

  labrob::UnicycleCommand unicycle_cmd;

  if (controller_type_ == ControllerType::ApproximateLinearization) {
    approximate_linearization_controller_ptr_->cmd(
        time,
        unicycle_configuration,
        *desired_trajectory_ptr_,
        unicycle_cmd
    );
  } else if (controller_type_ == ControllerType::DynamicFeedbackLinearization) {
    dynamic_feedback_linearization_controller_ptr_->cmd(
        time,
        unicycle_configuration,
        unicycle_velocity,
        *desired_trajectory_ptr_,
        unicycle_cmd
      );
  } else if (controller_type_ == ControllerType::StaticFeedbackLinearization) {
    static_feedback_linearization_controller_ptr_->cmd(
        time,
        unicycle_configuration,
        *desired_trajectory_ptr_,
        unicycle_cmd
    );
  }

  p3dx_robot_cmd_.setVelocitiesFromUnicycleCommand(unicycle_cmd);

  // Send commands to robot:
  if (dynamics_enabled_) {
    simSetJointTargetVelocity(left_motor_handle_, p3dx_robot_cmd_.getLeftMotorVelocity());
    simSetJointTargetVelocity(right_motor_handle_, p3dx_robot_cmd_.getRightMotorVelocity());
  } else {
    // Integrate using exact integration, use 2nd order Runge-Kutta integration
    // in case omega_k is around zero to avoid numerical instabilities:
    double x_k = unicycle_configuration.x();
    double y_k = unicycle_configuration.y();
    double theta_k = unicycle_configuration.theta();
    double v_k = unicycle_cmd.getDrivingVelocity();
    double omega_k = unicycle_cmd.getSteeringVelocity();
    double dt = static_cast<double>(simGetSimulationTimeStep());
    labrob::UnicycleConfiguration next_unicycle_configuration;
    if (std::abs(omega_k) < 1e-3) {
      // 2nd order Runge-Kutta integration:
      next_unicycle_configuration = labrob::UnicycleConfiguration(
          x_k + v_k * dt * std::cos(theta_k + omega_k * dt / 2.0),
          y_k + v_k * dt * std::sin(theta_k + omega_k * dt / 2.0),
          theta_k + omega_k * dt
      );
    } else {
      // Exact integration:
      double theta_next = theta_k + omega_k * dt;
      next_unicycle_configuration = labrob::UnicycleConfiguration(
          x_k + v_k / omega_k * (std::sin(theta_next) - std::sin(theta_k)),
          y_k - v_k / omega_k * (std::cos(theta_next) - std::cos(theta_k)),
          theta_next
      );
    }

    simFloat position[3] = {
        static_cast<simFloat>(next_unicycle_configuration.x()),
        static_cast<simFloat>(next_unicycle_configuration.y()),
        static_cast<simFloat>(z_unicycle_)
    };
    simFloat orientation[3] = {
        0.0f,
        0.0f,
        static_cast<simFloat>(next_unicycle_configuration.theta())
    };
    // Retrieve offset p3dx unicycle (const. through time).
    simFloat offset_position[3];
    simGetObjectPosition(p3dx_handle_, p3dx_unicycle_handle_, offset_position);
    // NOTE: assuming P3DX is moving on x-y plane (const z).
    simFloat next_p3dx_position[3] = {
        offset_position[0] * std::cos(orientation[2]) - offset_position[1] * std::sin(orientation[2]) + position[0],
        offset_position[0] * std::sin(orientation[2]) + offset_position[1] * std::cos(orientation[2]) + position[1],
        offset_position[2] + position[2]
    };
    simFloat next_p3dx_orientation[3] = {
        orientation[0],
        orientation[1],
        orientation[2]
    };
    simSetObjectPosition(p3dx_handle_, -1, next_p3dx_position);
    simSetObjectOrientation(p3dx_handle_, -1, next_p3dx_orientation);
  }

  // Draw P3DX unicycle configuration and desired trajectory:
  if (draw_trajectories_) {
    simFloat p3dx_unicycle_position[3];
    simGetObjectPosition(p3dx_unicycle_handle_, -1, p3dx_unicycle_position);
    simAddDrawingObjectItem(
        p3dx_unicycle_drawing_object_handle_,
        p3dx_unicycle_position
    );
    labrob::Pose2D desired_trajectory = desired_trajectory_ptr_->eval(time);
    simFloat p3dx_unicycle_desired_position[3] = {
        static_cast<float>(desired_trajectory.x()),
        static_cast<float>(desired_trajectory.y()),
        p3dx_unicycle_position[2]
    };
    simAddDrawingObjectItem(
        desired_trajectory_drawing_object_handle_,
        p3dx_unicycle_desired_position
    );
  }

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
        << unicycle_configuration.x() << " "
        << unicycle_configuration.y() << " "
        << unicycle_configuration.theta() << std::endl;
  }
  if (unicycle_desired_pose_log_file_.is_open()) {
    labrob::Pose2D desired_pose = desired_trajectory_ptr_->eval(time);
    unicycle_desired_pose_log_file_
        << desired_pose.x() << " "
        << desired_pose.y() << " "
        << desired_pose.theta() << std::endl;
  }
  if (unicycle_measured_velocity_log_file_.is_open()) {
    labrob::Pose2DDerivative measured_velocity = retrieveP3DXUnicycleVelocity();
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
CoppeliaSimP3DXController::retrieveP3DXUnicycleConfiguration() {
  simFloat p3dx_unicycle_position[3];
  simFloat p3dx_unicycle_orientation[3];
  simGetObjectPosition(p3dx_unicycle_handle_, -1, p3dx_unicycle_position);
  simGetObjectOrientation(p3dx_unicycle_handle_, -1, p3dx_unicycle_orientation);

  return labrob::UnicycleConfiguration(
      p3dx_unicycle_position[0],
      p3dx_unicycle_position[1],
      p3dx_unicycle_orientation[2]
  );
}

labrob::Pose2DDerivative
CoppeliaSimP3DXController::retrieveP3DXUnicycleVelocity() {
  simFloat p3dx_unicycle_linear_velocity[3];
  simFloat p3dx_unicycle_angular_velocity[3];
  simGetObjectVelocity(p3dx_unicycle_handle_, p3dx_unicycle_linear_velocity, p3dx_unicycle_angular_velocity);

  return labrob::Pose2DDerivative(
      p3dx_unicycle_linear_velocity[0],
      p3dx_unicycle_linear_velocity[1],
      p3dx_unicycle_angular_velocity[2]
  );
}

std::unique_ptr<labrob::UnicycleTrajectory>
CoppeliaSimP3DXController::generateDesiredTrajectory(
    TrajectoryType trajectory_type
) {
  if (trajectory_type == TrajectoryType::Circular) {
    double desired_driving_velocity = 1.0;
    return std::make_unique<labrob::CircularTrajectory>(
        labrob::Position2D(4.0, 3.0),
        3.0,
        desired_driving_velocity,
        -M_PI / 2.0
      );
  } else if (trajectory_type == TrajectoryType::EightShaped) {
    double desired_steering_velocity = 0.06;

    return std::make_unique<labrob::EightShapedTrajectory>(
        labrob::Pose2D(4.0, 1.0, 0.0),
        desired_steering_velocity
    );
  } else if (trajectory_type == TrajectoryType::Linear) {
    return std::make_unique<labrob::LinearTrajectoryWithConstantDrivingVelocity>(
        labrob::Pose2D(1.0, 0.0, 0.0),
        1.0
    );
  } else if (trajectory_type == TrajectoryType::Squared) {
    double desired_driving_velocity = 1.0;
    double square_length = 4.0;
    return
        std::make_unique<labrob::SquaredTrajectoryWithConstantDrivingVelocity>(
            static_cast<double>(simGetSimulationTime()),
            labrob::Pose2D(1.0, 0.5, 0.0),
            desired_driving_velocity,
            square_length
        );
  }
  return nullptr;
}


} // end namespace labrob