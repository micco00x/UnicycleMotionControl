#pragma once

#include <UnicycleMotionControl/simLib.h>

namespace labrob {

class CoppeliaSimController {
 public:
  virtual void init() = 0;
  virtual void update() = 0;
}; // end class CoppeliaSimController

} // end namespace labrob