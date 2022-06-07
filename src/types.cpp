#include <UnicycleMotionControl/types.hpp>

namespace labrob {

Position2D::Position2D() : x_(0.0), y_(0.0) { }
Position2D::Position2D(double x, double y) : x_(x), y_(y) { }

double Position2D::x() const { return x_; }
double& Position2D::x() { return x_; }

double Position2D::y() const { return y_; }
double& Position2D::y() { return y_; }

Velocity2D::Velocity2D() : x_(0.0), y_(0.0) { }
Velocity2D::Velocity2D(double x, double y) : x_(x), y_(y) { }

double Velocity2D::x() const { return x_; }
double& Velocity2D::x() { return x_; }

double Velocity2D::y() const { return y_; }
double& Velocity2D::y() { return y_; }

} // end namespace labrob