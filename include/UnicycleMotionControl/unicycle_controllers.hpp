#pragma once

#include <UnicycleMotionControl/hparams.hpp>
#include <UnicycleMotionControl/UnicycleCommand.hpp>
#include <UnicycleMotionControl/UnicycleConfiguration.hpp>
#include <UnicycleMotionControl/UnicycleTrajectory.hpp>

namespace labrob {

void staticFeedbackLinearizationControl(
    double time,
    const labrob::StaticFeedbackLinearizationHParams& hparams,
    const labrob::UnicycleConfiguration& configuration,
    const labrob::UnicycleTrajectory& desired_trajectory,
    labrob::UnicycleCommand& command
);

} // end namespace labrob
