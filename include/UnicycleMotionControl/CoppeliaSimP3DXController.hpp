#pragma once

#include <fstream>
#include <memory>

#include <UnicycleMotionControl/CoppeliaSimController.hpp>
#include <UnicycleMotionControl/DifferentialWheeledRobotCommand.hpp>
#include <UnicycleMotionControl/hparams.hpp>
#include <UnicycleMotionControl/UnicycleTrajectory.hpp>

namespace labrob {

class CoppeliaSimP3DXController : public CoppeliaSimController {
 public:
  CoppeliaSimP3DXController();

  void init() override;
  void update() override;

 protected:
  labrob::UnicycleConfiguration retrieveP3DXConfiguration();

  simInt p3dx_handle_;
  simInt left_motor_handle_;
  simInt right_motor_handle_;

  const double p3dx_wheel_radius_ = 0.09765;
  const double p3dx_dist_wheel_to_wheel_ = 0.330;

  labrob::DifferentialWheeledRobotCommand p3dx_robot_cmd_;

  std::unique_ptr<labrob::UnicycleTrajectory> desired_trajectory_ptr_;

  labrob::StaticFeedbackLinearizationHParams static_feedback_linearization_hparams_;

 private:
  std::ofstream time_log_file_;
  std::ofstream unicycle_cmd_log_file_;
  std::ofstream unicycle_configuration_log_file_;
  std::ofstream unicycle_desired_position_log_file_;

}; // end class CoppeliaSimP3DXController

} // end namespace labrob