#pragma once

#include <UnicycleMotionControl/hparams.hpp>
#include <UnicycleMotionControl/UnicycleCommand.hpp>
#include <UnicycleMotionControl/UnicycleConfiguration.hpp>
#include <UnicycleMotionControl/UnicycleTrajectory.hpp>


namespace labrob {

class ApproximateLinearizationController {
 public:
  ApproximateLinearizationController(
      const labrob::ApproximateLinearizationHparams& hparams
  );

  void cmd(
      double time,
      const labrob::UnicycleConfiguration& unicycle_configuration,
      const labrob::UnicycleTrajectory& desired_trajectory,
      labrob::UnicycleCommand& command
  );

 protected:
  labrob::ApproximateLinearizationHparams hparams_;
}; // end class ApproximateLinearizationController

} // end namespace labrob