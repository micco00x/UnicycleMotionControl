#include <UnicycleMotionControl/unicycle_controllers.hpp>

#include <cmath>
#include <iostream>

namespace labrob {

void staticFeedbackLinearizationControl(
    double time,
    const labrob::StaticFeedbackLinearizationHParams& hparams,
    const labrob::UnicycleConfiguration& configuration,
    const labrob::UnicycleTrajectory& desired_trajectory,
    labrob::UnicycleCommand& command
) {

  labrob::Pose2D desired_pose = desired_trajectory.eval(time);
  labrob::Pose2DDerivative desired_velocity = desired_trajectory.eval_dt(time);

  double xd = desired_pose.x();
  double yd = desired_pose.y();
  double thetad = desired_pose.theta();
  double xd_dot = desired_velocity.x_dot();
  double yd_dot = desired_velocity.y_dot();
  double thetad_dot = desired_velocity.theta_dot();

  double r1d = xd + hparams.b * std::cos(thetad);
  double r2d = yd + hparams.b * std::sin(thetad);
  double r1d_dot = xd_dot - hparams.b * thetad_dot * std::sin(thetad);
  double r2d_dot = yd_dot + hparams.b * thetad_dot * std::cos(thetad);

  double x = configuration.x();
  double y = configuration.y();
  double theta = configuration.theta();

  double r1 = x + hparams.b * std::cos(theta);
  double r2 = y + hparams.b * std::sin(theta);

  double u1 = r1d_dot + hparams.k1 * (r1d - r1);
  double u2 = r2d_dot + hparams.k2 * (r2d - r2);

  double driving_velocity = std::cos(theta) * u1 + std::sin(theta) * u2;
  double steering_velocity = -std::sin(theta) * u1 / hparams.b + std::cos(theta) * u2 / hparams.b;

  command.setDrivingVelocity(driving_velocity);
  command.setSteeringVelocity(steering_velocity);
}

} // end namespace labrob
