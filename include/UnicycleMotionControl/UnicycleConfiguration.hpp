#pragma once

namespace labrob {

class UnicycleConfiguration {
 public:
  UnicycleConfiguration();
  UnicycleConfiguration(double x, double y, double theta);

  void setX(double x);
  void setY(double y);
  void setTheta(double theta);

  double getX() const;
  double getY() const;
  double getTheta() const;
  
 protected:
  double x_;
  double y_;
  double theta_;

}; // end class UnicycleConfiguration

} // end namespace labrob