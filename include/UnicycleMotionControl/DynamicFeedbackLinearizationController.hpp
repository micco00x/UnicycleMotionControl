#pragma once

#include <UnicycleMotionControl/hparams.hpp>
#include <UnicycleMotionControl/types.hpp>
#include <UnicycleMotionControl/UnicycleCommand.hpp>
#include <UnicycleMotionControl/UnicycleTrajectory.hpp>


namespace labrob {

class DynamicFeedbackLinearizationController {
 public:
  DynamicFeedbackLinearizationController(
      const labrob::DynamicFeedbackLinearizationHparams& hparams,
      double dt,
      double xi_0
  );

  void cmd(
      double time,
      const labrob::UnicycleConfiguration& unicycle_configuration,
      const labrob::Pose2DDerivative& unicycle_velocity,
      const labrob::UnicycleTrajectory& desired_trajectory,
      labrob::UnicycleCommand& command
  );

 protected:
  labrob::DynamicFeedbackLinearizationHparams hparams_;
  double dt_;

 private:
  double xi_prev_;

}; // end class DynamicFeedbackLinearizationController

} // end namespace labrob