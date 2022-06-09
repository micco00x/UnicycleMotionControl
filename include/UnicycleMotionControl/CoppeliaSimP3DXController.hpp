#pragma once

#include <fstream>
#include <memory>

#include <UnicycleMotionControl/CoppeliaSimController.hpp>
#include <UnicycleMotionControl/DifferentialWheeledRobotCommand.hpp>
#include <UnicycleMotionControl/DynamicFeedbackLinearizationController.hpp>
#include <UnicycleMotionControl/hparams.hpp>
#include <UnicycleMotionControl/StaticFeedbackLinearizationController.hpp>
#include <UnicycleMotionControl/UnicycleTrajectory.hpp>

namespace labrob {

class CoppeliaSimP3DXController : public CoppeliaSimController {
 public:
  CoppeliaSimP3DXController();

  void init() override;
  void update() override;

 protected:
  labrob::UnicycleConfiguration retrieveP3DXCorrespondingUnicycle();
  labrob::Pose2DDerivative retrieveP3DXVelocity();

  simInt p3dx_handle_;
  simInt left_motor_handle_;
  simInt right_motor_handle_;

  const double p3dx_wheel_radius_ = 0.09765;
  const double p3dx_dist_wheel_to_wheel_ = 0.330;

  labrob::DifferentialWheeledRobotCommand p3dx_robot_cmd_;

  std::unique_ptr<labrob::UnicycleTrajectory> desired_trajectory_ptr_;

  // TODO: add mother class and use polymorphism here.
  std::unique_ptr<labrob::DynamicFeedbackLinearizationController> dynamic_feedback_linearization_controller_ptr_;
  std::unique_ptr<labrob::StaticFeedbackLinearizationController> static_feedback_linearization_controller_ptr_;

 private:
  std::ofstream time_log_file_;
  std::ofstream unicycle_cmd_log_file_;
  std::ofstream unicycle_configuration_log_file_;
  std::ofstream unicycle_desired_pose_log_file_;
  std::ofstream unicycle_measured_velocity_log_file_;
  std::ofstream unicycle_desired_velocity_log_file_;

  enum class TrajectoryType { Circular, EightShaped, Squared };
  enum class ControllerType { DynamicFeedbackLinearization, StaticFeedbackLinearization };

  ControllerType controller_type_;

  std::unique_ptr<labrob::UnicycleTrajectory> generateDesiredTrajectory(
      TrajectoryType trajectory_type
  );

}; // end class CoppeliaSimP3DXController

} // end namespace labrob