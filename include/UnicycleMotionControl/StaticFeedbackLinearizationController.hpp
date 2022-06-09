#pragma once

#include <UnicycleMotionControl/hparams.hpp>
#include <UnicycleMotionControl/UnicycleCommand.hpp>
#include <UnicycleMotionControl/UnicycleConfiguration.hpp>
#include <UnicycleMotionControl/UnicycleTrajectory.hpp>

namespace labrob {

class StaticFeedbackLinearizationController {
 public:
  StaticFeedbackLinearizationController(
      const labrob::StaticFeedbackLinearizationHParams& hparams
  );

  void cmd(
    double time,
    const labrob::UnicycleConfiguration& configuration,
    const labrob::UnicycleTrajectory& desired_trajectory,
    labrob::UnicycleCommand& command
  );

 protected:
  labrob::StaticFeedbackLinearizationHParams hparams_;

}; // end class StaticFeedbackLinearizationController

} // end namespace labrob