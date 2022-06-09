#include <UnicycleMotionControl/ApproximateLinearizationController.hpp>

#include <cmath>

namespace labrob {

ApproximateLinearizationController::ApproximateLinearizationController(
    const labrob::ApproximateLinearizationHparams& hparams
) : hparams_(hparams) {

}

void
ApproximateLinearizationController::cmd(
    double time,
    const labrob::UnicycleConfiguration& unicycle_configuration,
    const labrob::UnicycleTrajectory& desired_trajectory,
    labrob::UnicycleCommand& command
) {
  double x = unicycle_configuration.x();
  double y = unicycle_configuration.y();
  double theta = unicycle_configuration.theta();

  labrob::Pose2D desired_pose = desired_trajectory.eval(time);
  labrob::Pose2DDerivative desired_velocity = desired_trajectory.eval_dt(time);

  double xd = desired_pose.x();
  double yd = desired_pose.y();
  double thetad = desired_pose.theta();

  double xd_dot = desired_velocity.x_dot();
  double yd_dot = desired_velocity.y_dot();
  double thetad_dot = desired_velocity.theta_dot();

  // NOTE: controller is stable iff vd and omegad are const.
  double vd = std::sqrt(std::pow(xd_dot, 2.0) + std::pow(yd_dot, 2.0));
  double omegad = thetad_dot;

  double deltax = xd - x;
  double deltay = yd - y;
  double deltatheta = std::atan2(std::sin(thetad - theta), std::cos(thetad - theta));

  double e1 =  std::cos(theta) * deltax + std::sin(theta) * deltay;
  double e2 = -std::sin(theta) * deltax + std::cos(theta) * deltay;
  double e3 = deltatheta;

  double k1 = 2.0 * hparams_.zeta * hparams_.a;
  double k2 = (std::pow(hparams_.a, 2.0) - std::pow(omegad, 2.0)) / vd;
  double k3 = k1;

  double u1 = -k1 * e1;
  double u2 = -k2 * e2 - k3 * e3;

  double v = vd * std::cos(e3) - u1;
  double omega = omegad - u2;

  command.setDrivingVelocity(v);
  command.setSteeringVelocity(omega);
}

} // end namespace labrob