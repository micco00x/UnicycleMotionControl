#include <UnicycleMotionControl/StaticFeedbackLinearizationController.hpp>

#include <cmath>


namespace labrob {

StaticFeedbackLinearizationController::StaticFeedbackLinearizationController(
    const labrob::StaticFeedbackLinearizationHParams& hparams
) : hparams_(hparams) {

}

void
StaticFeedbackLinearizationController::cmd(
  double time,
  const labrob::UnicycleConfiguration& configuration,
  const labrob::UnicycleTrajectory& desired_trajectory,
  labrob::UnicycleCommand& command
) {
  labrob::Pose2D desired_pose = desired_trajectory.eval(time);
  labrob::Pose2DDerivative desired_velocity = desired_trajectory.eval_dt(time);

  double xd = desired_pose.x();
  double yd = desired_pose.y();
  double xd_dot = desired_velocity.x_dot();
  double yd_dot = desired_velocity.y_dot();

  double r1d = xd;
  double r2d = yd;
  double r1d_dot = xd_dot;
  double r2d_dot = yd_dot;

  double x = configuration.x();
  double y = configuration.y();
  double theta = configuration.theta();

  double r1 = x + hparams_.b * std::cos(theta);
  double r2 = y + hparams_.b * std::sin(theta);

  double u1 = r1d_dot + hparams_.k1 * (r1d - r1);
  double u2 = r2d_dot + hparams_.k2 * (r2d - r2);

  double driving_velocity = std::cos(theta) * u1 + std::sin(theta) * u2;
  double steering_velocity = -std::sin(theta) * u1 / hparams_.b + std::cos(theta) * u2 / hparams_.b;

  command.setDrivingVelocity(driving_velocity);
  command.setSteeringVelocity(steering_velocity);
}

} // end namespace labrob