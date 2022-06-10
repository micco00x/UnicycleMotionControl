#include <UnicycleMotionControl/UnicycleTrajectory.hpp>

namespace labrob {

double
UnicycleTrajectory::getDesiredSteeringVelocity(double time) const {
  return eval_dt(time).theta_dot();
}

} // end namespace labrob