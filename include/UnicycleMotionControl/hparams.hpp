#pragma once

namespace labrob {

struct StaticFeedbackLinearizationHParams {
  double b, k1, k2;
};

struct DynamicFeedbackLinearizationHparams {
  double kp1, kp2, kd1, kd2;
};

} // end namespace labrob