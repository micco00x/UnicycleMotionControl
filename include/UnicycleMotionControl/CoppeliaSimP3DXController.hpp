#pragma once

#include <UnicycleMotionControl/CoppeliaSimController.hpp>
#include <UnicycleMotionControl/DifferentialWheeledRobotCommand.hpp>

namespace labrob {

class CoppeliaSimP3DXController : public CoppeliaSimController {
 public:
  CoppeliaSimP3DXController();

  void init() override;
  void update() override;

 protected:
  const double p3dx_wheel_radius_ = 0.09765;
  const double p3dx_dist_wheel_to_wheel_ = 0.330;

  labrob::DifferentialWheeledRobotCommand p3dx_robot_cmd_;

}; // end class CoppeliaSimP3DXController

} // end namespace labrob