#include <UnicycleMotionControl/DynamicFeedbackLinearizationController.hpp>

#include <cmath>
#include <iostream>

namespace labrob {

DynamicFeedbackLinearizationController::DynamicFeedbackLinearizationController(
    const labrob::DynamicFeedbackLinearizationHparams& hparams,
    double dt,
    double xi_0
) : hparams_(hparams),
    dt_(dt),
    xi_prev_(xi_0) {

}

void
DynamicFeedbackLinearizationController::cmd(
    double time,
    const labrob::UnicycleConfiguration& unicycle_configuration,
    const labrob::Pose2DDerivative& unicycle_velocity,
    const labrob::UnicycleTrajectory& desired_trajectory,
    labrob::UnicycleCommand& command
) {
  
  double x = unicycle_configuration.x();
  double y = unicycle_configuration.y();
  double theta = unicycle_configuration.theta();

  double x_dot = unicycle_velocity.x_dot();
  double y_dot = unicycle_velocity.y_dot();

  labrob::Pose2D desired_pose = desired_trajectory.eval(time);
  labrob::Pose2DDerivative desired_velocity = desired_trajectory.eval_dt(time);
  labrob::Pose2DSecondDerivative desired_acceleration = desired_trajectory.eval_ddt(time);

  double xd = desired_pose.x();
  double yd = desired_pose.y();
  double xd_dot = desired_velocity.x_dot();
  double yd_dot = desired_velocity.y_dot();
  double xd_ddot = desired_acceleration.x_ddot();
  double yd_ddot = desired_acceleration.y_ddot();

  // PD controller:
  double u1 = xd_ddot + hparams_.kp1 * (xd - x) + hparams_.kd1 * (xd_dot - x_dot);
  double u2 = yd_ddot + hparams_.kp2 * (yd - y) + hparams_.kd2 * (yd_dot - y_dot);

  // Dynamic compensator:
  double xi_dot = u1 * std::cos(theta) + u2 * std::sin(theta);
  double xi = xi_prev_ + xi_dot * dt_; // integrator
  double v = xi;
  xi_prev_ = xi;
  double omega = (u2 * std::cos(theta) - u1 * std::sin(theta)) / xi;

  command.setDrivingVelocity(v);
  command.setSteeringVelocity(omega);
}

} // end namespace labrob