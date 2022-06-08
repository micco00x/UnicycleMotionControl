#include <UnicycleMotionControl/types.hpp>

namespace labrob {

// Position2D
Position2D::Position2D() : x_(0.0), y_(0.0) { }
Position2D::Position2D(double x, double y) : x_(x), y_(y) { }

double Position2D::x() const { return x_; }
double& Position2D::x() { return x_; }

double Position2D::y() const { return y_; }
double& Position2D::y() { return y_; }

// Pose2D
Pose2D::Pose2D() : x_(0.0), y_(0.0), theta_(0.0) { }
Pose2D::Pose2D(double x, double y, double theta)
  : x_(x), y_(y), theta_(theta) {}

double Pose2D::x() const { return x_; }
double& Pose2D::x() { return x_; }

double Pose2D::y() const { return y_; }
double& Pose2D::y() { return y_; }

double Pose2D::theta() const { return theta_; }
double& Pose2D::theta() { return theta_; }

// Pose2DDerivative
Pose2DDerivative::Pose2DDerivative()
  : x_dot_(0.0), y_dot_(0.0), theta_dot_(0.0) { }
Pose2DDerivative::Pose2DDerivative(double x_dot, double y_dot, double theta_dot)
  : x_dot_(x_dot), y_dot_(y_dot), theta_dot_(theta_dot) {}

double Pose2DDerivative::x_dot() const { return x_dot_; }
double& Pose2DDerivative::x_dot() { return x_dot_; }

double Pose2DDerivative::y_dot() const { return y_dot_; }
double& Pose2DDerivative::y_dot() { return y_dot_; }

double Pose2DDerivative::theta_dot() const { return theta_dot_; }
double& Pose2DDerivative::theta_dot() { return theta_dot_; }

// Velocity2D
Velocity2D::Velocity2D() : x_(0.0), y_(0.0) { }
Velocity2D::Velocity2D(double x, double y) : x_(x), y_(y) { }

double Velocity2D::x() const { return x_; }
double& Velocity2D::x() { return x_; }

double Velocity2D::y() const { return y_; }
double& Velocity2D::y() { return y_; }

} // end namespace labrob