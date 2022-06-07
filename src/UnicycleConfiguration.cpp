#include <UnicycleMotionControl/UnicycleConfiguration.hpp>

namespace labrob {

UnicycleConfiguration::UnicycleConfiguration() 
  : x_(0.0), y_(0.0), theta_(0.0) {

}

UnicycleConfiguration::UnicycleConfiguration(
    double x, double y, double theta)
  : x_(x), y_(y), theta_(theta) {

}

void
UnicycleConfiguration::setX(double x) {
  x_ = x;
}

void
UnicycleConfiguration::setY(double y) {
  y_ = y;
}

void
UnicycleConfiguration::setTheta(double theta) {
  theta_ = theta;
}

double
UnicycleConfiguration::getX() const {
  return x_;
}

double
UnicycleConfiguration::getY() const {
  return y_;
}

double
UnicycleConfiguration::getTheta() const {
  return theta_;
}

} // end namespace labrob