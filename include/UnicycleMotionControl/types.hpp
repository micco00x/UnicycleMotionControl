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