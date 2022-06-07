#include <UnicycleMotionControl/UnicycleConfiguration.hpp>

namespace labrob {

UnicycleConfiguration::UnicycleConfiguration() 
  : x_(0.0), y_(0.0), theta_(0.0) {

}

UnicycleConfiguration::UnicycleConfiguration(
    double x, double y, double theta)
  : x_(x), y_(y), theta_(theta) {

}

double UnicycleConfiguration::x() const { return x_; }
double& UnicycleConfiguration::x() { return x_; }

double UnicycleConfiguration::y() const { return y_; }
double& UnicycleConfiguration::y() { return y_; }

double UnicycleConfiguration::theta() const { return theta_; }
double& UnicycleConfiguration::theta() { return theta_; }


} // end namespace labrob