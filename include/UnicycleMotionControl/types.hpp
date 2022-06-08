#pragma once

namespace labrob {

class Position2D {
 public:
  Position2D();
  Position2D(double x, double y);

  double x() const;
  double& x();

  double y() const;
  double& y();

 protected:
  double x_, y_;
}; // end Position2D

class Pose2D {
 public:
  Pose2D();
  Pose2D(double x, double y, double theta);

  double x() const;
  double& x();

  double y() const;
  double& y();

  double theta() const;
  double& theta();

 protected:
  double x_, y_, theta_;
}; // end class Pose2D

class Pose2DDerivative {
 public:
  Pose2DDerivative();
  Pose2DDerivative(double x_dot, double y_dot, double theta_dot);

  double x_dot() const;
  double& x_dot();

  double y_dot() const;
  double& y_dot();

  double theta_dot() const;
  double& theta_dot();

 protected:
  double x_dot_, y_dot_, theta_dot_;
}; // end class Pose2DDerivative

class Velocity2D {
 public:
  Velocity2D();
  Velocity2D(double x, double y);

  double x() const;
  double& x();

  double y() const;
  double& y();

 protected:
  double x_, y_;
}; // end Velocity2D

} // end namespace labrob